import json
import time
import random
import threading
import math
import gz.transport
from gz.msgs.stringmsg_pb2 import StringMsg

class SimulatedNetwork:
    """
    Simulates a degraded wireless network (packet loss, latency, range limits, dead zones).
    """
    def __init__(self, config_path=None):
        self.comm_radius = 8.0 # meters
        self.latency_min = 0.005 # seconds
        self.latency_max = 0.050 # seconds
        self.packet_loss = 0.0 # 0.0 to 1.0
        self.dead_zones = [] # list of dicts with min_x, max_x, min_y, max_y
        
        # Load from config if provided
        if config_path:
            self._load_config(config_path)

    def _load_config(self, path):
        # Placeholder for loading yaml config
        pass

    def in_dead_zone(self, x, y):
        for dz in self.dead_zones:
            if dz['min_x'] <= x <= dz['max_x'] and dz['min_y'] <= y <= dz['max_y']:
                return True
        return False

    def should_drop(self, my_pose, peer_pose):
        if my_pose is None or peer_pose is None:
            return True
            
        x1, y1 = my_pose[0], my_pose[1]
        x2, y2 = peer_pose[0], peer_pose[1]
        
        if self.in_dead_zone(x1, y1) or self.in_dead_zone(x2, y2):
            return True
            
        dist = math.hypot(x1 - x2, y1 - y2)
        if dist > self.comm_radius:
            return True
            
        if random.random() < self.packet_loss:
            return True
            
        return False
        
    def get_latency(self):
        return random.uniform(self.latency_min, self.latency_max)

class FleetComms:
    def __init__(self, robot_id, peer_table, get_pose_cb):
        self.robot_id = robot_id
        self.peer_table = peer_table
        self.get_pose_cb = get_pose_cb
        self.node = gz.transport.Node()
        
        self.network = SimulatedNetwork()
        self.callbacks = {}
        
        # Topics
        self.topics = [
            "/fleet/state",
            "/fleet/heartbeat",
            "/fleet/map_delta",
            "/fleet/negotiation",
            "/fleet/auction",
            "/fleet/intent"
        ]
        
        self.pubs = {}
        for t in self.topics:
            self.pubs[t] = self.node.advertise(t, StringMsg)
            # Subscribe to same topic to receive from peers
            self.node.subscribe(StringMsg, t, lambda msg, topic=t: self._raw_rx_callback(msg, topic))

    def set_network_params(self, loss, latency_max, dead_zones=None):
        self.network.packet_loss = loss
        self.network.latency_max = latency_max
        if dead_zones:
            self.network.dead_zones = dead_zones

    def register_callback(self, topic, cb):
        if topic not in self.callbacks:
            self.callbacks[topic] = []
        self.callbacks[topic].append(cb)

    def _raw_rx_callback(self, msg, topic):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
            
        sender_id = payload.get("sender_id")
        if sender_id == self.robot_id:
            return # Ignore own messages
            
        sender_pose = payload.get("sender_pose")
        my_pose = self.get_pose_cb()
        
        # Apply simulated network rules
        if self.network.should_drop(my_pose, sender_pose):
            return
            
        # Simulate latency
        latency = self.network.get_latency()
        threading.Timer(latency, self._deliver_msg, args=(topic, payload)).start()
        
    def _deliver_msg(self, topic, payload):
        # Update peer table on any message
        sender_id = payload.get("sender_id")
        sender_pose = payload.get("sender_pose")
        if sender_id is not None and sender_pose is not None:
            self.peer_table.update_peer(sender_id, sender_pose, payload.get("sender_vel", (0,0)))
            
        if topic in self.callbacks:
            for cb in self.callbacks[topic]:
                cb(payload)

    def publish(self, topic, data):
        data["sender_id"] = self.robot_id
        data["sender_pose"] = self.get_pose_cb()
        # Add lamport clock or similar if needed here
        
        msg = StringMsg()
        msg.data = json.dumps(data)
        self.pubs[topic].publish(msg)
