import os
import yaml

def generate_warehouse(base_dir):
    world_path = os.path.join(base_dir, 'worlds', 'warehouse.sdf')
    graph_path = os.path.join(base_dir, 'config', 'graph.yaml')
    
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    os.makedirs(os.path.dirname(graph_path), exist_ok=True)

    # Grid parameters
    num_x = 5
    num_y = 4
    spacing = 4.0 # meters between aisles
    
    nodes = {}
    edges = []
    junctions = []
    
    node_id = 0
    
    # Create junctions and their holding nodes
    # For a 5x4 lattice, we have 20 junctions
    j_map = {}
    for i in range(num_x):
        for j in range(num_y):
            x = i * spacing
            y = j * spacing
            j_id = f"J_{i}_{j}"
            nodes[j_id] = {"pos": [x, y], "type": "junction"}
            junctions.append(j_id)
            j_map[(i, j)] = j_id
            
            # Holding nodes (0.5m away)
            h_n = f"H_{i}_{j}_N"
            h_s = f"H_{i}_{j}_S"
            h_e = f"H_{i}_{j}_E"
            h_w = f"H_{i}_{j}_W"
            
            nodes[h_n] = {"pos": [x, y + 0.5], "type": "holding", "junction": j_id}
            nodes[h_s] = {"pos": [x, y - 0.5], "type": "holding", "junction": j_id}
            nodes[h_e] = {"pos": [x + 0.5, y], "type": "holding", "junction": j_id}
            nodes[h_w] = {"pos": [x - 0.5, y], "type": "holding", "junction": j_id}
            
            edges.append([h_n, j_id])
            edges.append([h_s, j_id])
            edges.append([h_e, j_id])
            edges.append([h_w, j_id])
            
    # Connect holding nodes with aisle segments
    for i in range(num_x):
        for j in range(num_y):
            if i < num_x - 1: # Connect E of (i,j) to W of (i+1,j)
                edges.append([f"H_{i}_{j}_E", f"H_{i+1}_{j}_W"])
            if j < num_y - 1: # Connect N of (i,j) to S of (i,j+1)
                edges.append([f"H_{i}_{j}_N", f"H_{i}_{j+1}_S"])
                
    # Save graph
    graph_data = {
        "nodes": nodes,
        "edges": edges,
        "junctions": junctions,
        "aisle_mode": "bidirectional"
    }
    
    with open(graph_path, 'w') as f:
        yaml.dump(graph_data, f)
        
    # Generate SDF (simplified)
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="warehouse">
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
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry><material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>
"""
    # Add racks (brown with grey grid)
    for i in range(num_x - 1):
        for j in range(num_y - 1):
            cx = i * spacing + spacing / 2.0
            cy = j * spacing + spacing / 2.0
            sdf_content += f"""
    <model name="rack_{i}_{j}">
      <static>true</static>
      <pose>{cx} {cy} 1 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>2.0 2.0 2.0</size></box></geometry></collision>
        <!-- Brown solid core -->
        <visual name="vis_core">
          <geometry><box><size>1.98 1.98 1.98</size></box></geometry>
          <material><ambient>0.4 0.2 0.1 1</ambient><diffuse>0.4 0.2 0.1 1</diffuse></material>
        </visual>
        <!-- Grey horizontal grid lines (shelves) -->
        <visual name="vis_h1">
          <pose>0 0 -0.5 0 0 0</pose>
          <geometry><box><size>2.01 2.01 0.05</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
        <visual name="vis_h2">
          <pose>0 0 0.5 0 0 0</pose>
          <geometry><box><size>2.01 2.01 0.05</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
        <!-- Grey vertical grid lines (struts) -->
        <visual name="vis_v1">
          <pose>0.95 0 0 0 0 0</pose>
          <geometry><box><size>0.1 2.01 2.01</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
        <visual name="vis_v2">
          <pose>-0.95 0 0 0 0 0</pose>
          <geometry><box><size>0.1 2.01 2.01</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>
"""
    sdf_content += """  </world>\n</sdf>\n"""
    with open(world_path, 'w') as f:
        f.write(sdf_content)
        
    print("Generated warehouse SDF and graph YAML.")

if __name__ == "__main__":
    generate_warehouse("/home/ubuntu/fleet_sim")
