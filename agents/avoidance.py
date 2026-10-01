import math

class OrcaLite:
    def __init__(self, agent_radius=0.3):
        self.agent_radius = agent_radius
        
    def compute_velocity(self, my_pose, my_pref_vel, peers):
        vx, vy = my_pref_vel
        x, y, yaw = my_pose
        
        for peer in peers:
            px, py, _ = peer.pose
            dist = math.hypot(px - x, py - y)
            
            safe_dist = self.agent_radius + peer.get_safety_radius(0, base_radius=self.agent_radius)
            
            if dist < safe_dist + 1.0: # Interaction range
                # Simple repulsive force if too close
                if dist < safe_dist:
                    fx = (x - px) / (dist + 0.001)
                    fy = (y - py) / (dist + 0.001)
                    vx += fx * 0.5
                    vy += fy * 0.5
                else:
                    # Velocity obstacle approximation
                    rel_vx = vx - peer.vel[0]
                    rel_vy = vy - peer.vel[1]
                    
                    time_to_col = dist / (math.hypot(rel_vx, rel_vy) + 0.001)
                    if time_to_col < 2.0:
                        # steer away
                        vx -= rel_vx * 0.5
                        vy -= rel_vy * 0.5
                        
        # Cap velocity
        speed = math.hypot(vx, vy)
        max_speed = 0.5
        if speed > max_speed:
            vx = (vx / speed) * max_speed
            vy = (vy / speed) * max_speed
            
        return vx, vy
