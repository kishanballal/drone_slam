#!/bin/bash
# Tab 4: exploration, then a tour past every room so the 3D LiDAR sees the floor everywhere,
# then save the 3D map + victim list, land, print the results.
source "$(dirname "$0")/env.sh"
sleep 5
until [ -f /tmp/m3_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"
goal() {   # goal x y label
  echo; echo "== $3: ($1, $2) =="
  timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}}" \
    | grep -E "SUCCEEDED|ABORTED|CANCELED|rejected"
  sleep 3
}
sleep 40
echo "== waiting for the 3D map =="
until ros2 topic list 2>/dev/null | grep -q '^/map_3d$'; do sleep 3; done
sleep 5
echo
echo "############################################################"
echo "  3D map + victim detector running. START THE RECORDING NOW (Win+Alt+R)."
echo "  Exploration starts in 15 s. Red pins = victims found."
echo "############################################################"
sleep 15
rm -f /tmp/m3_explore.log
ros2 launch drone_explore explore.launch.py 2>&1 | tee /tmp/m3_explore.log &
T0=$(date +%s)
until grep -q "Exploration stopped" /tmp/m3_explore.log 2>/dev/null; do
  if [ $(( $(date +%s) - T0 )) -gt 240 ]; then echo "== exploration time limit (4 min) reached =="; break; fi
  grep -h "VICTIM .* FOUND" "$OUT/m3_run.log" 2>/dev/null | tail -1
  sleep 20
done
pkill -f "explore_lite/explore"; pkill -f "explore.launch.py"
echo
echo "== 2D map complete. Search tour: one stop in every room =="
goal -6.0 -4.5 "south-west room"
goal -2.5 -5.0 "south-west room, east side"
goal -8.5  2.0 "west side"
goal -5.0  6.0 "north corridor, west"
goal  1.5  6.0 "north corridor, middle"
goal  6.5  5.5 "north-east corner"
goal  6.0 -4.0 "south-east room"
goal  1.5 -3.0 "south-east room, west side"
goal  0.0  0.0 "back to the start"
sleep 5
echo "== saving the 3D map =="
ros2 service call /map_3d/save std_srvs/srv/Trigger "{}" | grep -E "success|message"
echo "== landing =="
ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"
sleep 10
echo "== saving maps and results to media\\m3 =="
mkdir -p "$OUT"; cd ~/slam_drone/maps 2>/dev/null || { mkdir -p ~/slam_drone/maps; cd ~/slam_drone/maps; }
ros2 run nav2_map_server map_saver_cli -f m3_building --ros-args -p use_sim_time:=true \
  -p save_map_timeout:=10.0 && cp m3_building.* "$OUT/"
cp victims.csv "$OUT/" 2>/dev/null && echo "victims.csv saved"
P=$(ls -t map3d_*.ply 2>/dev/null | head -1); [ -n "$P" ] && cp "$P" "$OUT/" && echo "3D map $P saved"
echo
echo "================= VICTIMS FOUND ================="
grep -h "VICTIM .* FOUND" "$OUT/m3_run.log" | sed 's/.*VICTIM/VICTIM/'
echo "(4 victims are hidden in the building)"
echo
echo "=== M3 DONE. Stop the recording. Files: automotive\\media\\m3 ==="
exec bash
