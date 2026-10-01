import yaml
import math
import heapq

class TopologicalGraph:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        self.nodes = data['nodes']
        self.edges = data['edges']
        self.choke_points = data.get('choke_points', [])
        
        self.adj = {n: [] for n in self.nodes}
        for u, v in self.edges:
            dist = math.hypot(self.nodes[u][0] - self.nodes[v][0], self.nodes[u][1] - self.nodes[v][1])
            self.adj[u].append((v, dist))
            self.adj[v].append((u, dist))
            
        self.blocked_edges = set()
        
    def get_nearest_node(self, x, y):
        best_node = None
        best_dist = float('inf')
        for n, pos in self.nodes.items():
            dist = math.hypot(x - pos[0], y - pos[1])
            if dist < best_dist:
                best_dist = dist
                best_node = n
        return best_node

    def astar(self, start, goal):
        if start not in self.nodes or goal not in self.nodes:
            return []
            
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {n: float('inf') for n in self.nodes}
        g_score[start] = 0
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
                
            for neighbor, weight in self.adj[current]:
                if (current, neighbor) in self.blocked_edges or (neighbor, current) in self.blocked_edges:
                    continue
                tentative_g = g_score[current] + weight
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = math.hypot(self.nodes[neighbor][0] - self.nodes[goal][0], self.nodes[neighbor][1] - self.nodes[goal][1])
                    heapq.heappush(open_set, (tentative_g + h, neighbor))
        return []
