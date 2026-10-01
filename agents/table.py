import time
import json
import threading

class ReservationTable:
    def __init__(self, comms, robot_id):
        self.comms = comms
        self.robot_id = robot_id
        
        # node -> list of (t_in, t_out, priority, robot_id)
        self.table = {}
        self.lock = threading.Lock()
        
        self.comms.register_callback("/fleet/intent", self.on_intent)
        
    def add_intent(self, robot_id, priority, route_windows):
        with self.lock:
            # Clear old reservations for this robot
            for node, res_list in self.table.items():
                self.table[node] = [r for r in res_list if r[3] != robot_id]
                
            # Add new ones
            for node, t_in, t_out in route_windows:
                if node not in self.table:
                    self.table[node] = []
                self.table[node].append((t_in, t_out, priority, robot_id))
                self.table[node].sort(key=lambda x: x[0])
                
    def on_intent(self, msg):
        sender = msg['sender_id']
        priority = msg['priority']
        windows = msg['windows']
        self.add_intent(sender, priority, windows)
        
    def broadcast_intent(self, priority, route_windows):
        self.add_intent(self.robot_id, priority, route_windows)
        self.comms.publish("/fleet/intent", {
            "type": "INTENT",
            "priority": priority,
            "windows": route_windows
        })
        
    def is_free(self, node, t_in, t_out, my_priority):
        with self.lock:
            if node not in self.table:
                return True
            for r_t_in, r_t_out, r_prio, r_id in self.table[node]:
                # Overlap check
                if t_in < r_t_out and t_out > r_t_in:
                    # Deterministic priority merge rule
                    if r_prio > my_priority:
                        return False
                    elif r_prio == my_priority and r_id > self.robot_id:
                        return False
            return True
