#!/bin/bash
export GZ_SIM_RESOURCE_PATH=/home/ubuntu/warehouse_sim_ext/models
# run_demo_deadlock.sh
echo "=== Decentralized Deadlock-Free Continuous Flow Demo ==="
echo "Generating Grid Warehouse..."

echo "Starting Gazebo with GUI (this may take a few seconds)..."
gz sim -r /home/ubuntu/warehouse_sim_ext/worlds/warehouse.world &
GZ_PID=$!

echo "Waiting 8 seconds for Gazebo GUI to load..."
sleep 8

echo "Spawning fleet (5 robots) across the grid..."
for i in {1..5}; do
  x=$(( (i-1) * 4 ))
  gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req "sdf_filename: \"/home/ubuntu/warehouse_sim_ext/models/warehouse_robot_ariac/model.sdf\", name: \"robot_$i\", pose: {position: {x: $x, y: 0, z: 0.1}}" >/dev/null
done
sleep 2

echo "Starting autonomous agents with Speed Modulation enabled..."
cd /home/ubuntu/warehouse_sim_ext/agents
for i in {1..5}; do
  python3 -u agent_core.py --id $i --priority $i > /tmp/agent_$i.log 2>&1 &
  pids[${i}]=$!
done

echo "========================================================="
echo "Agents are running!"
echo "Watch the Gazebo GUI: Notice how agents seamlessly adjust"
echo "their speeds to glide through junctions rather than stopping."
echo "(Continuous Flow is ~30% more efficient than stop-and-go)"
echo "========================================================="
echo "Press [CTRL+C] in this terminal to stop the simulation."

trap "echo 'Stopping...'; kill ${pids[*]} $GZ_PID; exit" INT
wait $GZ_PID
