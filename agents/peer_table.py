import time

class Peer:
    def __init__(self, peer_id, pose, vel):
        self.peer_id = peer_id
        self.pose = pose # (x, y, yaw)
        self.vel = vel   # (vx, vy)
        self.last_seen = time.time()
        self.is_dead = False
        
    def update(self, pose, vel):
        self.pose = pose
        self.vel = vel
        self.last_seen = time.time()
        self.is_dead = False

    def predict_pose(self, current_time):
        dt = current_time - self.last_seen
        # Constant velocity extrapolation
        x = self.pose[0] + self.vel[0] * dt
        y = self.pose[1] + self.vel[1] * dt
        return (x, y, self.pose[2])
        
    def get_staleness(self, current_time):
        return current_time - self.last_seen
        
    def get_safety_radius(self, current_time, base_radius=0.4):
        staleness = self.get_staleness(current_time)
        # Inflate radius by 0.5m per second of staleness (up to a cap)
        inflation = min(2.0, 0.5 * staleness)
        return base_radius + inflation

class PeerTable:
    def __init__(self):
        self.peers = {}
        self.timeout_sec = 3.0
        
    def update_peer(self, peer_id, pose, vel):
        if peer_id not in self.peers:
            self.peers[peer_id] = Peer(peer_id, pose, vel)
            print(f"[PeerTable] Discovered new peer: {peer_id}")
        else:
            self.peers[peer_id].update(pose, vel)
            
    def get_active_peers(self):
        current_time = time.time()
        active = []
        for pid, peer in self.peers.items():
            if current_time - peer.last_seen > self.timeout_sec:
                if not peer.is_dead:
                    peer.is_dead = True
                    print(f"[PeerTable] Peer {pid} declared dead (timeout).")
            else:
                active.append(peer)
        return active
