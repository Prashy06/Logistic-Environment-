#!/bin/bash
echo "=== 3-AMR Yellow Lane Demo (Agent Core) ==="
export GZ_SIM_RESOURCE_PATH=/home/ubuntu/warehouse_sim_ext/models

# Launch warehouse world
gz sim -r /home/ubuntu/warehouse_sim_ext/worlds/warehouse.world &
GZ_PID=$!

echo "Waiting for Gazebo to load..."
sleep 8

echo "Spawning 3 AMRs on the yellow lines..."
# Spawn Robot 2 at the front, Robot 1 in middle, Robot 3 at back.
# We spawn them on the left lane (X=-4.11).
# In yellow_graph.yaml, this is the NW to SW edge.
for i in 2 1 3; do
  PX=-4.11
  if [ "$i" -eq 2 ]; then
    PY=2.5
  elif [ "$i" -eq 1 ]; then
    PY=0.5
  else
    PY=-1.5
  fi
  
  echo "  robot_$i  →  ($PX, $PY)"
  gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    --req "sdf_filename: \"/home/ubuntu/warehouse_sim_ext/models/warehouse_robot_ariac/model.sdf\", \
name: \"robot_$i\", \
pose: {position: {x: $PX, y: $PY, z: 0.1}, orientation: {z: -0.7071, w: 0.7071}}" \
    >/dev/null
done

sleep 2

echo "Starting autonomous agents routing strictly on the yellow paths..."
cd /home/ubuntu/warehouse_sim_ext/agents
export LANE_OFFSET=0.0
for i in 2 1 3; do
  python3 -u agent_core.py --id $i --priority $i --graph ../config/yellow_graph.yaml > /tmp/agent_$i.log 2>&1 &
  PIDS[$i]=$!
  echo "  robot_$i agent PID=${PIDS[$i]}"
done

echo ""
echo "========================================================="
echo "Watch the Gazebo GUI: The 3 AMRs are using the A* router"
echo "to elegantly follow the yellow lines with collision avoidance!"
echo "Press [CTRL+C] to stop."
echo "========================================================="

sleep 60 &
TIMER_PID=$!
trap "echo 'Stopping...'; kill ${PIDS[*]} $GZ_PID $TIMER_PID 2>/dev/null; exit" INT
wait $GZ_PID
