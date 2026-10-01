import os
import yaml
import random

def generate_labyrinth(base_dir):
    world_path = os.path.join(base_dir, 'worlds', 'labyrinth.sdf')
    graph_path = os.path.join(base_dir, 'config', 'labyrinth_graph.yaml')
    
    # 10x10 maze grid
    width = 8
    height = 8
    spacing = 4.0
    
    # Simple randomized Kruskal's for maze generation to ensure connectivity
    edges = []
    for x in range(width):
        for y in range(height):
            if x < width - 1:
                edges.append(((x,y), (x+1,y)))
            if y < height - 1:
                edges.append(((x,y), (x,y+1)))
                
    random.seed(42)
    random.shuffle(edges)
    
    parent = { (x,y): (x,y) for x in range(width) for y in range(height) }
    def find(i):
        if parent[i] == i:
            return i
        parent[i] = find(parent[i])
        return parent[i]
        
    def union(i, j):
        root_i = find(i)
        root_j = find(j)
        parent[root_i] = root_j

    maze_edges = []
    for u, v in edges:
        if find(u) != find(v):
            union(u, v)
            maze_edges.append((u, v))
            
    # Now we have a spanning tree (maze). 
    # Let's add some loops to make it a labyrinth (multiple paths)
    for u, v in edges[len(maze_edges):]:
        if random.random() < 0.2:
            maze_edges.append((u, v))
            
    # Center offset
    offset_x = (width * spacing) / 2.0
    offset_y = (height * spacing) / 2.0

    # Build Graph YAML
    nodes = {}
    graph_edges = []
    
    for x in range(width):
        for y in range(height):
            n_id = f"N_{x}_{y}"
            nodes[n_id] = {"pos": [x * spacing - offset_x, y * spacing - offset_y], "type": "junction"}
            
    for (u, v) in maze_edges:
        u_id = f"N_{u[0]}_{u[1]}"
        v_id = f"N_{v[0]}_{v[1]}"
        graph_edges.append([u_id, v_id])
        
    graph_data = {
        "nodes": nodes,
        "edges": graph_edges,
    }
    with open(graph_path, 'w') as f:
        yaml.dump(graph_data, f)
        
    # Build SDF
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="labyrinth">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    
    <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose></light>
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry><material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>
"""
    # Create walls where there are NO edges
    all_edges_set = set(edges)
    maze_edges_set = set(maze_edges)
    wall_edges = all_edges_set - maze_edges_set
    
    wall_idx = 0
    for u, v in wall_edges:
        # A wall between u and v
        cx = ((u[0] + v[0]) / 2.0 * spacing) - offset_x
        cy = ((u[1] + v[1]) / 2.0 * spacing) - offset_y
        
        if u[0] == v[0]: # horizontal wall blocking vertical path
            size_x, size_y, size_z = spacing, 0.5, 2.0
        else: # vertical wall blocking horizontal path
            size_x, size_y, size_z = 0.5, spacing, 2.0
            
        sdf_content += f"""
    <model name="wall_{wall_idx}">
      <static>true</static>
      <pose>{cx} {cy} 1 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry></collision>
        <!-- Brown solid core -->
        <visual name="vis_core">
          <geometry><box><size>{size_x*0.99} {size_y*0.99} {size_z*0.99}</size></box></geometry>
          <material><ambient>0.4 0.2 0.1 1</ambient><diffuse>0.4 0.2 0.1 1</diffuse></material>
        </visual>
        <!-- Grey horizontal grid lines -->
        <visual name="vis_h1">
          <pose>0 0 -0.5 0 0 0</pose>
          <geometry><box><size>{size_x*1.01} {size_y*1.01} 0.05</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
        <visual name="vis_h2">
          <pose>0 0 0.5 0 0 0</pose>
          <geometry><box><size>{size_x*1.01} {size_y*1.01} 0.05</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
        <!-- Grey vertical grid lines -->
        <visual name="vis_v1">
          <pose>{(size_x/2)*0.95} 0 0 0 0 0</pose>
          <geometry><box><size>0.1 {size_y*1.01} {size_z*1.01}</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
        <visual name="vis_v2">
          <pose>{-(size_x/2)*0.95} 0 0 0 0 0</pose>
          <geometry><box><size>0.1 {size_y*1.01} {size_z*1.01}</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>
"""
        wall_idx += 1
        
    sdf_content += """  </world>\n</sdf>\n"""
    with open(world_path, 'w') as f:
        f.write(sdf_content)
        
if __name__ == "__main__":
    generate_labyrinth("/home/ubuntu/warehouse_sim_ext")
