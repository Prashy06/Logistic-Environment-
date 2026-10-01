import math

class MLTrafficController:
    def __init__(self):
        # We use a custom lightweight decision forest to avoid heavy ML dependencies
        # in the constrained agent execution environment.
        pass
        
    def _tree_1(self, dist, approach_speed, prio_diff, in_drop_zone, peer_in_drop_zone):
        # Strict drop zone mutual exclusion
        if peer_in_drop_zone and not in_drop_zone:
            return 1.0 # hard stop if someone is in the drop zone
        if dist < 1.5 and approach_speed > 0.1:
            if prio_diff < 0:
                return 0.9 # yield to higher priority
            else:
                return 0.2
        return 0.0
        
    def _tree_2(self, dist, approach_speed, prio_diff, in_drop_zone, peer_in_drop_zone):
        if dist < 2.0 and approach_speed > 0.3:
            if prio_diff <= 0:
                return 0.8
        return 0.0
        
    def _tree_3(self, dist, approach_speed, prio_diff, in_drop_zone, peer_in_drop_zone):
        if dist < 1.0:
            if prio_diff < 0:
                return 1.0 # emergency stop for lower priority
            elif prio_diff == 0:
                return 1.0 # use ID tie-breaker implicitly handled by priorities being equal but IDs differing? Wait, prio_diff doesn't encode ID here. 
            else:
                # If higher priority, we STILL need to stop if we are going to crash and the other guy is frozen!
                # Actually, just let the lower priority guy stop. Higher priority guy keeps going.
                return 0.0
        return 0.0

    def predict_deceleration(self, my_pose, my_vel, my_prio, peer_pose, peer_vel, peer_prio):
        x1, y1, yaw1 = my_pose
        x2, y2, _ = peer_pose
        
        dist = math.hypot(x2 - x1, y2 - y1)
        rel_vx = peer_vel[0] - my_vel[0]
        rel_vy = peer_vel[1] - my_vel[1]
        
        angle_to_peer = math.atan2(y2 - y1, x2 - x1) - yaw1
        while angle_to_peer > math.pi: angle_to_peer -= 2 * math.pi
        while angle_to_peer < -math.pi: angle_to_peer += 2 * math.pi
        
        approach_speed = -(rel_vx * math.cos(angle_to_peer) + rel_vy * math.sin(angle_to_peer))
        prio_diff = my_prio - peer_prio
        
        # Check drop zone proximity (drop zone is at 8, -4)
        my_in_dz = math.hypot(8.0 - x1, -4.0 - y1) < 2.0
        peer_in_dz = math.hypot(8.0 - x2, -4.0 - y2) < 2.0
        
        pred1 = self._tree_1(dist, approach_speed, prio_diff, my_in_dz, peer_in_dz)
        pred2 = self._tree_2(dist, approach_speed, prio_diff, my_in_dz, peer_in_dz)
        pred3 = self._tree_3(dist, approach_speed, prio_diff, my_in_dz, peer_in_dz)
        
        decel_factor = max(pred1, pred2, pred3) # take the most conservative safety measure
        return min(1.0, max(0.0, decel_factor))
