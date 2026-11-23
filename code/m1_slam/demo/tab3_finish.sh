#!/bin/bash
# Tab 3: wait for the flight to finish, then save map, commit and tag m1
sleep 15
source "$(dirname "$0")/env.sh"
echo "== waiting for the waypoint flight to start =="
until ros2 node list 2>/dev/null | grep -q waypoint_flight; do sleep 3; done
echo "== flight running, waiting for it to land =="
while ros2 node list 2>/dev/null | grep -q waypoint_flight; do sleep 3; done
sleep 3
echo "== saving map =="
mkdir -p ~/slam_drone/maps && cd ~/slam_drone/maps
ros2 run nav2_map_server map_saver_cli -f building_m1 --ros-args -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/slam_drone/maps/building_m1'}"
ros2 service call /map_3d/save std_srvs/srv/Trigger || true
mkdir -p "$AUTO/media/m1_map" && cp building_m1.* "$AUTO/media/m1_map/"; cp $(ls -t map3d_*.ply 2>/dev/null | head -1) "$AUTO/media/m1_map/" 2>/dev/null
cd ~/slam_drone && git add . && git commit -q -m "M1 demo run" ; git tag -f m1
echo
echo "=== M1 DONE: map saved, committed, tagged m1. Stop the screen recording now. ==="
git log --oneline -3
exec bash
