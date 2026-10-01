#!/bin/bash
echo "=== Professional 7-AMR Warehouse Drop-Zone Demo ==="
export GZ_SIM_RESOURCE_PATH=/home/ubuntu/warehouse_sim_ext/models

# Use the warehouse world that I updated with grey floor and brown/grey shelves
gz sim -r /home/ubuntu/warehouse_sim_ext/worlds/warehouse.world &
GZ_PID=$!

echo "Waiting 8 seconds for Gazebo GUI to load..."
sleep 8

echo "Spawning exactly 3 AMRs coming out of the warehouse..."
for i in {1..3}; do
  y=$(echo "11.0 - $i * 1.5" | bc)
  gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req "sdf_filename: \"/home/ubuntu/warehouse_sim_ext/models/warehouse_robot_ariac/model.sdf\", name: \"robot_$i\", pose: {position: {x: -4.0, y: $y, z: 0.1}, orientation: {z: -0.707, w: 0.707}}" >/dev/null
done
sleep 2

echo "Starting autonomous agents on the left-turn only path..."
cd /home/ubuntu/warehouse_sim_ext/agents
export LANE_OFFSET=0.0
for i in {1..3}; do
  python3 -u agent_core.py --id $i --priority $i --graph ../config/left_turn_graph.yaml > /tmp/agent_$i.log 2>&1 &
  pids[${i}]=$!
done

echo "========================================================="
echo "Watch the Gazebo GUI: The 3 AMRs are coming out of the"
echo "warehouse and looping continuously using ONLY left turns!"
echo "The new ML Decision Tree strictly enforces right-of-way"
echo "and mutual exclusion when an AMR is in the Drop Zone!"
echo "========================================================="
echo "Press [CTRL+C] in this terminal to stop the simulation."

trap "echo 'Stopping...'; kill ${pids[*]} $GZ_PID; exit" INT
wait $GZ_PID
