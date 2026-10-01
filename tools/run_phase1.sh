#!/bin/bash
echo "=== Phase 1: Deterministic Motion ==="
gz sim -s -r /home/ubuntu/fleet_sim/worlds/warehouse.sdf > /tmp/gz.log 2>&1 &
GZ_PID=$!
sleep 3

gz service -s /world/warehouse/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req 'sdf_filename: "/home/ubuntu/fleet_sim/models/amr_robot/model.sdf", name: "robot_1", pose: {position: {x: 0, y: 0, z: 0.1}}' >/dev/null

sleep 1
python3 -u /home/ubuntu/fleet_sim/agents/agent_core.py --id 1 > /tmp/agent_1.log

kill $GZ_PID
cat /tmp/agent_1.log
echo "Done."
