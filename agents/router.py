import math
import yaml
from queue import PriorityQueue

class ManhattanRouter:
    def __init__(self, drop_zone=(8.0, -4.0)):
        self.drop_zone = drop_zone
        
    def generate_route(self, start, end, step=2.0):
        # Generate a path of waypoints using Manhattan routing (grid aligned)
        sx, sy = start
        ex, ey = end
        
        route = []
        cx, cy = sx, sy
        
        # Move along X first, then Y
        idx = 0
        while abs(cx - ex) > 0.1:
            dx = math.copysign(min(step, abs(ex - cx)), ex - cx)
            cx += dx
            route.append({"id": f"M_{idx}", "pos": (cx, cy)})
            idx += 1
            
        while abs(cy - ey) > 0.1:
            dy = math.copysign(min(step, abs(ey - cy)), ey - cy)
            cy += dy
            route.append({"id": f"M_{idx}", "pos": (cx, cy)})
            idx += 1
            
        return route

class AStarRouter:
    def __init__(self, graph_path):
        with open(graph_path, 'r') as f:
            data = yaml.safe_load(f)
            
        self.nodes = data['nodes']
        self.adj = {n: [] for n in self.nodes}
        for u, v in data['edges']:
            self.adj[u].append(v)
            self.adj[v].append(u)
            
    def _dist(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
        
    def get_nearest_node(self, pos):
        best_node = None
        best_d = float('inf')
        for n_id, n_data in self.nodes.items():
            d = self._dist(pos, n_data['pos'])
            if d < best_d:
                best_d = d
                best_node = n_id
        return best_node

    def generate_route(self, start_pos, end_pos):
        start_node = self.get_nearest_node(start_pos)
        end_node = self.get_nearest_node(end_pos)
        
        if not start_node or not end_node:
            return []
            
        pq = PriorityQueue()
        pq.put((0, start_node))
        came_from = {}
        cost_so_far = {start_node: 0.0}
        
        while not pq.empty():
            _, current = pq.get()
            
            if current == end_node:
                break
                
            curr_pos = self.nodes[current]['pos']
            
            for next_node in self.adj[current]:
                next_pos = self.nodes[next_node]['pos']
                new_cost = cost_so_far[current] + self._dist(curr_pos, next_pos)
                
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self._dist(next_pos, self.nodes[end_node]['pos'])
                    pq.put((priority, next_node))
                    came_from[next_node] = current
                    
        # Reconstruct path
        path = []
        curr = end_node
        while curr != start_node:
            if curr not in came_from:
                # No path found!
                return [end_pos]
            path.append(curr)
            curr = came_from[curr]
            
        path.append(start_node)
        path.reverse()
        # Convert nodes to dicts (with optional lane offset)
        route = []
        import os
        LANE_OFFSET = float(os.environ.get('LANE_OFFSET', 0.0))
        
        if LANE_OFFSET == 0.0:
            for n in path:
                route.append({"id": n, "pos": tuple(self.nodes[n]['pos']), "parent": n})
            if self._dist(route[-1]['pos'], end_pos) > 0.1:
                route.append({"id": "FINAL", "pos": (end_pos[0], end_pos[1]), "parent": "FINAL"})
            return route
            
        # Right-hand lane logic
            curr_id = path[i]
            next_id = path[i+1]
            
            cx, cy = self.nodes[curr_id]['pos']
            nx, ny = self.nodes[next_id]['pos']
            
            # Calculate direction vector
            dx = nx - cx
            dy = ny - cy
            dist = math.hypot(dx, dy)
            if dist == 0:
                continue
                
            # Right-hand normal vector
            norm_x = dy / dist
            norm_y = -dx / dist
            
            # Shifted points for this segment
            shift_cx = cx + norm_x * LANE_OFFSET
            shift_cy = cy + norm_y * LANE_OFFSET
            shift_nx = nx + norm_x * LANE_OFFSET
            shift_ny = ny + norm_y * LANE_OFFSET
            
            # Add entry point for this lane segment
            route.append({"id": f"{curr_id}_OUT", "pos": (shift_cx, shift_cy), "parent": curr_id})
            # Add exit point for this lane segment
            route.append({"id": f"{next_id}_IN", "pos": (shift_nx, shift_ny), "parent": next_id})
        
        # Finally, append the exact end position (shifted to its right if possible, or just exact)
        if len(route) >= 2:
            last_dx = end_pos[0] - route[-1]['pos'][0]
            last_dy = end_pos[1] - route[-1]['pos'][1]
            last_dist = math.hypot(last_dx, last_dy)
            if last_dist > 0.1:
                norm_x = last_dy / last_dist
                norm_y = -last_dx / last_dist
                route.append({"id": "FINAL", "pos": (end_pos[0] + norm_x * LANE_OFFSET, end_pos[1] + norm_y * LANE_OFFSET), "parent": "FINAL"})
        elif self._dist(self.nodes[path[-1]]['pos'], end_pos) > 0.1:
            route.append({"id": "FINAL", "pos": (end_pos[0], end_pos[1]), "parent": "FINAL"})
            
        return route
