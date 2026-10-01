#!/bin/bash
# Spawn a single robot for Phase 1
gz service -s /world/warehouse/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req 'sdf_filename: "/home/ubuntu/fleet_sim/models/amr_robot/model.sdf", name: "robot_1", pose: {position: {x: 0, y: 0, z: 0.1}}'
