import time
import threading

class DistributedLock:
    def __init__(self, robot_id, comms, peer_table):
        self.robot_id = robot_id
        self.comms = comms
        self.peer_table = peer_table
        
        self.clock = 0
        self.request_queue = []
        self.replies_needed = 0
        self.replies_received = 0
        
        self.state = "RELEASED" # RELEASED, WANTED, HELD
        self.current_resource = None
        
        self.comms.register_callback("/fleet/negotiation", self.on_message)
        
    def acquire(self, resource):
        self.state = "WANTED"
        self.current_resource = resource
        self.clock += 1
        
        active_peers = self.peer_table.get_active_peers()
        self.replies_needed = len(active_peers)
        self.replies_received = 0
        
        if self.replies_needed == 0:
            self.state = "HELD"
            return True
            
        self.comms.publish("/fleet/negotiation", {
            "type": "REQUEST",
            "resource": resource,
            "clock": self.clock,
            "robot_id": self.robot_id
        })
        
        # Wait for replies
        start_wait = time.time()
        while self.replies_received < self.replies_needed:
            if time.time() - start_wait > 3.0:
                print(f"[Lock] Timeout waiting for lock on {resource}, assuming held for dead peers.")
                break
            time.sleep(0.05)
            
        self.state = "HELD"
        return True
        
    def release(self):
        self.state = "RELEASED"
        for req in self.request_queue:
            self.comms.publish("/fleet/negotiation", {
                "type": "REPLY",
                "resource": self.current_resource,
                "target_id": req["robot_id"]
            })
        self.request_queue.clear()
        self.current_resource = None
        
    def on_message(self, msg):
        msg_type = msg.get("type")
        if msg_type == "REQUEST":
            sender_id = msg["robot_id"]
            sender_clock = msg["clock"]
            resource = msg["resource"]
            
            self.clock = max(self.clock, sender_clock) + 1
            
            if self.state == "HELD" or (self.state == "WANTED" and (self.clock < sender_clock or (self.clock == sender_clock and self.robot_id < sender_id))):
                if resource == self.current_resource:
                    self.request_queue.append(msg)
                else:
                    self.comms.publish("/fleet/negotiation", {
                        "type": "REPLY",
                        "resource": resource,
                        "target_id": sender_id
                    })
            else:
                self.comms.publish("/fleet/negotiation", {
                    "type": "REPLY",
                    "resource": resource,
                    "target_id": sender_id
                })
        elif msg_type == "REPLY":
            target = msg.get("target_id")
            if target == self.robot_id and msg.get("resource") == self.current_resource:
                self.replies_received += 1
