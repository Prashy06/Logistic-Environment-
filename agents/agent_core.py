import sys
import argparse
import time
import math
import threading
import yaml

import gz.transport
from gz.msgs.twist_pb2 import Twist
from gz.msgs.odometry_pb2 import Odometry
from gz.msgs.laserscan_pb2 import LaserScan

from peer_table import PeerTable
from comms import FleetComms
from table import ReservationTable
from wfg import WFG, check_admission
from avoidance import OrcaLite
from ml_traffic import MLTrafficController
from router import ManhattanRouter, AStarRouter


class DeterministicController:
    def __init__(self, config_path, robot_id):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        agent_cfg = config['agents'][f'robot_{robot_id}']
        self.v_max = agent_cfg['v_max']
        self.a_max = agent_cfg['a_max']
        self.decel = agent_cfg['decel']
        self.omega_max = agent_cfg['omega_max']
        self.current_v = 0.0
        self.current_omega = 0.0
        self.lookahead_dist = 0.8  # Pure pursuit lookahead distance
        
    def step(self, pose, goal, dt, speed_limit=None, is_final_goal=False):
        x, y, yaw = pose
        gx, gy = goal
        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)
        
        # --- Pure Pursuit Steering ---
        # Transform goal into robot's local frame
        local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        
        # Calculate curvature (gamma) for pure pursuit arc
        # gamma = 2 * local_y / (lookahead^2)
        L_sq = max(dist**2, 0.1) # prevent division by zero
        curvature = (2.0 * local_y) / L_sq
        
        # Desired angular velocity
        target_omega = self.current_v * curvature
        
        # Cap angular velocity
        self.current_omega = max(-self.omega_max, min(self.omega_max, target_omega))
        
        # If the goal is behind us (e.g. sharp turn recovery), turn in place
        if local_x < 0 and dist > 0.3:
            self.current_v = max(0.0, self.current_v - self.decel * dt)
            self.current_omega = self.omega_max if local_y > 0 else -self.omega_max
            return self.current_v, self.current_omega
            
        # --- Longitudinal Velocity Control ---
        if is_final_goal and dist < 0.1:
            self.current_v = max(0.0, self.current_v - self.decel * dt)
            return self.current_v, 0.0
            
        stop_dist = (self.current_v ** 2) / (2 * self.decel)
        v_target = self.v_max
        
        if speed_limit is not None and speed_limit >= 0.0:
            v_target = min(v_target, speed_limit)
            
        # Slow down for sharp turns (if curvature is high)
        turn_slowdown = max(0.2, 1.0 - abs(curvature) * 0.3)
        v_target *= turn_slowdown
            
        if is_final_goal and dist <= stop_dist + 0.05:
            self.current_v = max(0.0, self.current_v - self.decel * dt)
        elif self.current_v < v_target:
            self.current_v = min(v_target, self.current_v + self.a_max * dt)
        elif self.current_v > v_target:
            self.current_v = max(v_target, self.current_v - self.decel * dt)
        else:
            self.current_v = v_target
            
        return self.current_v, self.current_omega

class Agent:
    def __init__(self, robot_id, priority=1):
        self.robot_id = robot_id
        self.priority = priority
        self.node = gz.transport.Node()
        
        self.cmd_vel_pub = self.node.advertise(
            f"/model/robot_{self.robot_id}/cmd_vel",
            Twist
        )
        
        self.pose = None
        self.vel = (0.0, 0.0)
        self.node.subscribe(
            Odometry,
            f"/model/robot_{self.robot_id}/odometry",
            self.odom_callback
        )
        
        self.lidar_ranges = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_step = 0.0
        self.node.subscribe(
            LaserScan,
            f"/model/robot_{self.robot_id}/lidar",
            self.lidar_callback
        )
        
        self.controller = DeterministicController('/home/ubuntu/fleet_sim/config/agent.yaml', self.robot_id)
        self.peer_table = PeerTable()
        self.comms = FleetComms(self.robot_id, self.peer_table, lambda: self.pose)
        self.res_table = ReservationTable(self.comms, self.robot_id)
        self.ml_traffic = MLTrafficController()
        
        # Try to use AStarRouter if labyrinth graph exists, else fallback to Manhattan
        try:
            self.router = AStarRouter('/home/ubuntu/warehouse_sim_ext/config/labyrinth_graph.yaml')
            self.maze_nodes = list(self.router.nodes.keys())
        except FileNotFoundError:
            self.router = ManhattanRouter(drop_zone=(8.0, -4.0))
            self.maze_nodes = []
        
        self.state = "PICKING"
        self.current_goal = 0
        self.goals = []
        self.assign_new_task()
        self.running = True
        self.stuck_time = 0.0
        
    def assign_new_task(self):
        # Use a deterministic sequence of tasks based on robot ID to create a "set path"
        if not hasattr(self, 'task_idx'):
            self.task_idx = 0
            
        if self.maze_nodes:
            # Deterministic hash to pick a sequence of nodes for this robot
            node_idx = (self.robot_id * 17 + self.task_idx * 31) % len(self.maze_nodes)
            target_node = self.maze_nodes[node_idx]
            gx, gy = self.router.nodes[target_node]['pos']
        else:
            gx = (self.robot_id * 2.0) % 12.0 - 6.0
            gy = (self.task_idx * 4.0) % 12.0
            
        if self.state == "PICKING":
            self.goals = self.router.generate_route(self.pose[:2] if self.pose else (0,0), (gx, gy))
        else:
            self.goals = self.router.generate_route(self.pose[:2] if self.pose else (0,0), (8.0, -4.0))
            
        if not self.goals:
            self.goals = [{"id": "FINAL", "pos": (gx, gy)}]
        self.current_goal = 0
        self.task_idx += 1
        
    def lidar_callback(self, msg):
        self.lidar_ranges = list(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_step = msg.angle_step
        
    def odom_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (x, y, yaw)
        
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.vel = (vx, vy)
        
    def play_load_animation(self, dt):
        print(f"Agent {self.robot_id} executing load/unload animation...")
        for _ in range(int(2.0 / dt)):
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 3.14159  # 180 deg/s spin
            self.cmd_vel_pub.publish(msg)
            time.sleep(dt)
        self.cmd_vel_pub.publish(Twist()) # stop
        
    def control_loop(self):
        rate_hz = 20
        dt = 1.0 / rate_hz
        
        while self.pose is None:
            time.sleep(0.1)
            
        print(f"Agent {self.robot_id} starting preemptive traffic planner.")
        
        while self.running:
            if self.current_goal >= len(self.goals):
                if self.state == "PICKING":
                    print(f"Agent {self.robot_id} finished picking. Routing to drop zone.")
                    self.state = "DROPPING"
                    self.play_load_animation(dt)
                else:
                    print(f"Agent {self.robot_id} finished dropping. Routing to new pick task.")
                    self.state = "PICKING"
                    self.play_load_animation(dt)
                self.assign_new_task()
                continue
                
            goal_node = self.goals[self.current_goal]
            gx, gy = goal_node["pos"]
            x, y, yaw = self.pose
            dist = math.hypot(gx - x, gy - y)
            
            # 1. Broad intent and Strict Comm Layer (Intersection Node Locking)
            if dist > 0.15:
                t_now = time.time()
                t_arr_min = t_now + (dist / self.controller.v_max)
                
                # Lock the intersection parent node!
                intersection_id = goal_node.get("parent", f"START_{self.robot_id}")
                lock_id = f"NODE_{intersection_id}"
                
                intents = [(lock_id, optimal_t_arr, optimal_t_arr + 2.0)]
                
                # Create a "tail lock" on the previous intersection to ensure our physical body clears it!
                prev_intersection_id = self.goals[self.current_goal - 1].get("parent") if self.current_goal > 0 else None
                if prev_intersection_id and prev_intersection_id != intersection_id:
                    intents.append((f"NODE_{prev_intersection_id}", t_now, t_now + 2.0))
                
                while not self.res_table.is_free(lock_id, optimal_t_arr, optimal_t_arr + 2.0, self.priority):
                    optimal_t_arr += 0.5
                    # Update intents with the delayed arrival time
                    intents[0] = (lock_id, optimal_t_arr, optimal_t_arr + 2.0)
                    
                self.res_table.broadcast_intent(self.priority, intents)
                
                if optimal_t_arr - t_now > 0.5:
                    modulated_speed = 0.0
                else:
                    time_to_target = optimal_t_arr - t_now
                    modulated_speed = dist / time_to_target if time_to_target > 0 else self.controller.v_max
            else:
                modulated_speed = self.controller.v_max
                
            # 2. Sentient Lidar: Lane Keep Assist, Zipper Merging & ACC
            lidar_correction_w = 0.0
            if hasattr(self, 'lidar_ranges') and self.lidar_ranges:
                left_dist = 5.0
                right_dist = 5.0
                front_dist = 5.0
                side_front_dist = 5.0
                
                num_rays = len(self.lidar_ranges)
                for i, r in enumerate(self.lidar_ranges):
                    if r > 4.0 or math.isinf(r) or math.isnan(r):
                        continue
                    angle = self.lidar_angle_min + i * self.lidar_angle_step
                    
                    # Wide front check for Adaptive Cruise Control (ACC) & Zipper Merges
                    if abs(angle) < 0.6:
                        front_dist = min(front_dist, r)
                    # Check diagonal blind spots for side-humping / merging peers
                    elif 0.6 <= abs(angle) < 1.2:
                        side_front_dist = min(side_front_dist, r)
                        
                    # Gentle wall check for extreme drifts
                    if 1.2 < angle < 2.0:
                        left_dist = min(left_dist, r)
                    elif -2.0 < angle < -1.2:
                        right_dist = min(right_dist, r)
                        
                # Adaptive Cruise Control (platooning in the lane)
                # Harder braking if someone is directly in front
                if front_dist < 2.5:
                    acc_speed = max(0.0, (front_dist - 0.8))
                    modulated_speed = min(modulated_speed, acc_speed)
                    
                # Yield to peers merging from the side (prevents side-humping)
                if side_front_dist < 1.2:
                    modulated_speed = min(modulated_speed, 0.2) # slow down to let them merge
                    
                # Lane Keep Assist (prevent hitting wall if pushed)
                if right_dist < 0.5:
                    lidar_correction_w += 1.0 # push left
                if left_dist < 0.5:
                    lidar_correction_w -= 1.0 # push right
            
            is_final_goal = (self.current_goal == len(self.goals) - 1)
            acceptance_radius = 0.15 if is_final_goal else 0.4 
            
            if dist < acceptance_radius:
                self.current_goal += 1
                self.stuck_time = 0.0
                continue
                
            # Step the Pure Pursuit controller
            v, w = self.controller.step(self.pose, (gx, gy), dt, speed_limit=modulated_speed, is_final_goal=is_final_goal)
            
            # Apply sentient centering correction
            w = max(-self.controller.omega_max, min(self.controller.omega_max, w + lidar_correction_w))
            
            # Simple stuck replanner
            if v < 0.05 and dist > 0.5 and modulated_speed > 0.1:
                self.stuck_time += dt
                if self.stuck_time > 8.0:
                    print(f"Agent {self.robot_id} is physically stuck. Recalculating path.")
                    gx_final, gy_final = self.goals[-1]["pos"]
                    new_route = self.router.generate_route(self.pose[:2], (gx_final, gy_final))
                    if new_route:
                        self.goals = new_route
                        self.current_goal = 0
                    self.stuck_time = 0.0
                    continue
            else:
                self.stuck_time = 0.0
            
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            self.cmd_vel_pub.publish(msg)
            
            time.sleep(dt)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--priority", type=int, default=1)
    args = parser.parse_args()
    
    agent = Agent(args.id, args.priority)
    agent.control_loop()
