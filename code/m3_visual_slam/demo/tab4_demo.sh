#!/bin/bash
# Tab 4: exploration while the camera maps, then land and save everything for the report
source "$(dirname "$0")/env.sh"
sleep 5
until [ -f /tmp/m3v_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"     # again, so the freshly built drone_vslam is found
goal() {   # goal x y label
  echo; echo "== $3: ($1, $2) =="
  timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}}" \
    | grep -E "SUCCEEDED|ABORTED|CANCELED|rejected"
}
spin() {   # one slow full turn in place so the camera sees every wall of the room
  echo "   spinning 360 deg for the camera"
  timeout 12 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.6}}" > /dev/null
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null
  sleep 2
}
sleep 40
echo "== waiting for RTAB-Map to publish its map =="
until ros2 topic list 2>/dev/null | grep -q '^/rtabmap/cloud_map$'; do sleep 3; done
sleep 10
echo
echo "############################################################"
echo "  Camera + LiDAR running. START THE RECORDING NOW (Win+Alt+R)."
echo "  Exploration starts in 15 s. Watch the coloured camera map grow."
echo "############################################################"
sleep 15
rm -f /tmp/m3v_explore.log
ros2 launch drone_explore explore.launch.py 2>&1 | tee /tmp/m3v_explore.log &
T0=$(date +%s)
until grep -q "Exploration stopped" /tmp/m3v_explore.log 2>/dev/null; do
  timeout 3 ros2 topic echo /camera_guard/status --once --field data 2>/dev/null | head -1
  if [ $(( $(date +%s) - T0 )) -gt 360 ]; then echo "== exploration time limit (6 min) reached =="; break; fi
  sleep 20
done
pkill -f "explore_lite/explore"; pkill -f "explore.launch.py"
echo
echo "== LiDAR map complete. Camera tour: every room, one 360 deg turn in each =="
goal -6.0 -4.5 "south-west room";            spin
goal -2.5 -6.0 "south-west room, far side";  spin
goal -8.5  2.0 "west side";                  spin
goal -6.0  6.2 "north corridor, west";       spin
goal  1.5  6.0 "north corridor, middle";     spin
goal  8.5  5.0 "north-east corner";          spin
goal  6.5 -4.0 "south-east room";            spin
goal  2.5 -6.0 "south-east room, far side";  spin
goal  0.0  0.0 "back to the start"
echo "== tour done, landing =="
ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"
sleep 10
echo "== saving maps and logs =="
OUT="$AUTO/media/m3v"; mkdir -p "$OUT"
(cd ~/slam_drone/maps 2>/dev/null || mkdir -p ~/slam_drone/maps && cd ~/slam_drone/maps
 ros2 run nav2_map_server map_saver_cli -f m3v_building --ros-args -p use_sim_time:=true \
   -p save_map_timeout:=10.0 && cp m3v_building.* "$OUT/")
pkill -INT -f "vslam.launch.py"; sleep 8           # RTAB-Map writes its database on shutdown
cp ~/.ros/rtabmap.db "$OUT/m3v_rtabmap.db" 2>/dev/null && echo "RTAB-Map database saved"
cp $(ls -t ~/slam_drone/logs/camera_mem_*.csv 2>/dev/null | head -1) "$OUT/" 2>/dev/null && echo "camera memory log saved"
echo
grep -E "CAMERA OFF|growth|stopped" "$OUT/m3v_vslam.log" | tail -5
echo
echo "=== M3v DONE. Stop the recording. Files: automotive\\media\\m3v ==="
exec bash
