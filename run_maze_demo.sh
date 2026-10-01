#!/bin/bash
echo "=== Complete Labyrinth & Avoidance Demo ==="
echo "Generating Wall Labyrinth..."
python3 /home/ubuntu/warehouse_sim_ext/tools/gen_labyrinth.py > /dev/null

echo "Starting Gazebo with GUI..."
export GZ_SIM_RESOURCE_PATH=/home/ubuntu/warehouse_sim_ext/models
gz sim -r /home/ubuntu/warehouse_sim_ext/worlds/labyrinth.sdf &
GZ_PID=$!

echo "Waiting 8 seconds for Gazebo GUI to load..."
sleep 8

echo "Spawning fleet inside the maze..."
for i in {1..7}; do
  x=$((i * 4 - 16))
  gz service -s /world/labyrinth/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req "sdf_filename: \"/home/ubuntu/warehouse_sim_ext/models/warehouse_robot_ariac/model.sdf\", name: \"robot_$i\", pose: {position: {x: $x, y: 0, z: 0.1}}" >/dev/null
done
sleep 2

echo "Starting autonomous agents with Lidar Wall Avoidance & ML..."
cd /home/ubuntu/warehouse_sim_ext/agents
for i in {1..7}; do
  python3 -u agent_core.py --id $i --priority $i > /tmp/agent_$i.log 2>&1 &
  pids[${i}]=$!
done

echo "========================================================="
echo "Watch the Gazebo GUI: Notice how agents actively repulse"
echo "from the maze walls using Lidar Artificial Potential Fields,"
echo "while avoiding each other using the ML Traffic layer."
echo "========================================================="
echo "Press [CTRL+C] in this terminal to stop the simulation."

trap "echo 'Stopping...'; kill ${pids[*]} $GZ_PID; exit" INT
wait $GZ_PID
