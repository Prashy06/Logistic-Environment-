#!/usr/bin/env python3
"""
3 AMRs spawn in front of the shipping container (-4.85, 9.5)
and immediately turn LEFT (east, +X direction), following a
rectangular loop between the yellow lines for at least 30 seconds.

Waypoints (clockwise loop, starting from container exit):
  A (-4.0,  9.5)  → spawn/start, facing south  → TURN LEFT means go EAST
  B ( 3.5,  9.5)  right-top corner
  C ( 3.5, -3.0)  right-bottom corner
  D (-4.0, -3.0)  left-bottom corner
  back to A       (loop forever)
"""

import argparse
import math
import sys
import time
import threading

try:
    from gz.transport13 import Node
    from gz.msgs10.twist_pb2 import Twist
    from gz.msgs10.odometry_pb2 import Odometry
except ImportError:
    print("ERROR: gz-python bindings not found.")
    sys.exit(1)

# ── Hardcoded rectangular loop between the yellow lines ──────────────────────
# The centerlines between the yellow pathways are mathematically:
# Left Lane: X = -4.11, Right Lane: X = 4.25
# Top Start: Y = 9.5, Bottom Lane: Y = -3.4
WAYPOINTS = [
    ( 4.25,  9.5),   # Top-Right    (Turn left/east from container)
    ( 4.25, -3.4),   # Bottom-Right (Turn left/south down the right lane)
    (-4.11, -3.4),   # Bottom-Left  (Turn left/west across the bottom)
    (-4.11,  9.5),   # Top-Left     (Turn left/north back to start)
]

V_MAX     = 0.8
V_SLOW    = 0.3
OMEGA_MAX = 1.5
ACCEPT_R  = 0.4    # m, waypoint acceptance radius

class YellowFollower:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.pose     = None
        self.running  = True
        self.lock     = threading.Lock()

        # All robots start on the left lane, so the next waypoint for all is Top-Right (index 0)
        self.wp_idx = 0

        self.node = Node()
        model = f"robot_{robot_id}"

        self.node.subscribe(Odometry,
                            f"/model/{model}/odometry",
                            self._odom_cb)
        self.pub = self.node.advertise(f"/model/{model}/cmd_vel", Twist)

    def _odom_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw  = math.atan2(siny, cosy)
        with self.lock:
            self.pose = (p.x, p.y, yaw)

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _cmd(self, v, w):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def run(self):
        dt = 0.05
        print(f"[robot_{self.robot_id}] waiting for odometry…")
        while self.pose is None:
            time.sleep(0.1)
        print(f"[robot_{self.robot_id}] online at {self.pose[:2]}, starting on waypoint {self.wp_idx}")

        while self.running:
            with self.lock:
                x, y, yaw = self.pose

            wx, wy = WAYPOINTS[self.wp_idx % len(WAYPOINTS)]
            dx, dy = wx - x, wy - y
            dist   = math.hypot(dx, dy)

            # Reached waypoint → advance
            if dist < ACCEPT_R:
                prev = self.wp_idx % len(WAYPOINTS)
                self.wp_idx += 1
                nxt = self.wp_idx % len(WAYPOINTS)
                print(f"[robot_{self.robot_id}] wp {prev}→{nxt}: {WAYPOINTS[nxt]}")
                time.sleep(dt)
                continue

            desired = math.atan2(dy, dx)
            err     = self._wrap(desired - yaw)
            w       = max(-OMEGA_MAX, min(OMEGA_MAX, 2.5 * err))

            # Slow down for sharp turns
            if abs(err) > 0.45:
                v = V_SLOW
            elif dist < 0.8:
                v = V_SLOW
            else:
                v = V_MAX

            self._cmd(v, w)
            time.sleep(dt)

        self._cmd(0, 0)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--id", type=int, required=True)
    args = p.parse_args()
    f = YellowFollower(args.id)
    try:
        f.run()
    except KeyboardInterrupt:
        f._cmd(0, 0)

if __name__ == "__main__":
    main()
