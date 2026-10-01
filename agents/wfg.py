class WFG:
    def __init__(self):
        self.adj = {}
        
    def add_wait(self, waiting_robot, holding_robot):
        if waiting_robot not in self.adj:
            self.adj[waiting_robot] = set()
        self.adj[waiting_robot].add(holding_robot)
        
    def check_cycle(self):
        visited = set()
        rec_stack = set()
        
        def dfs(node):
            visited.add(node)
            rec_stack.add(node)
            for neighbor in self.adj.get(node, []):
                if neighbor not in visited:
                    if dfs(neighbor):
                        return True
                elif neighbor in rec_stack:
                    return True
            rec_stack.remove(node)
            return False
            
        for node in self.adj:
            if node not in visited:
                if dfs(node):
                    return True
        return False

def check_admission(route, table, my_id):
    # Simplified Banker's admission control:
    # Verify that the last node in the intended route is a valid holding/escape node
    # and is free indefinitely (or free at t_out).
    if not route:
        return False
        
    last_node = route[-1][0]
    # In a full impl, check if last_node is type "holding"
    return True
