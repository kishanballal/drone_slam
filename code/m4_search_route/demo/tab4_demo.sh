#!/bin/bash
# Tab 4 (M4): exploration builds the map, then the drone PLANS ITS OWN SEARCH ROUTE from that map:
# splits it into rooms, picks the fewest stops that let the 3D LiDAR see the floor it has not
# seen yet, and flies them once each in the shortest order, then home. Replaces M3's 9 fixed stops.
source "$(dirname "$0")/env.sh"
sleep 5
until [ -f /tmp/m4_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"
sleep 40
echo "== waiting for the 3D map =="
until ros2 topic list 2>/dev/null | grep -q '^/map_3d$'; do sleep 3; done
sleep 5
echo "== starting the search-route planner (it records the exploration path) =="
rm -f /tmp/m4_route.log
ros2 launch drone_search search_route.launch.py 2>&1 | tee /tmp/m4_route.log | \
  grep --line-buffered -E "SEARCH ROUTE|stop [0-9]|STOP|== |WARN|ERROR|error" &
sleep 5
echo
echo "############################################################"
echo "  M4: self-planned search route. START THE RECORDING NOW (Win+Alt+R)."
echo "  Exploration starts in 15 s. Orange balls = planned stops, blue line = route,"
echo "  stops turn green when reached. Red pins = victims found."
echo "############################################################"
sleep 15
rm -f /tmp/m4_explore.log
T_START=$(date +%s)
ros2 launch drone_explore explore.launch.py 2>&1 | tee /tmp/m4_explore.log > /dev/null &
until grep -q "Exploration stopped" /tmp/m4_explore.log 2>/dev/null; do
  if [ $(( $(date +%s) - T_START )) -gt 240 ]; then echo "== exploration time limit (4 min) reached =="; break; fi
  grep -h "VICTIM .* FOUND" "$OUT/m4_victims.log" 2>/dev/null | tail -1
  sleep 10
done
pkill -f "explore_lite/explore"; pkill -f "explore.launch.py"
T_EXPLORED=$(date +%s)
echo
echo "== 2D map complete after $((T_EXPLORED - T_START)) s. Planning the search route from it =="
ros2 topic pub -t 3 -r 1 /search_route/start std_msgs/msg/Empty "{}" > /dev/null   # 3x in case the first is missed; extra ones are ignored
until grep -q "SEARCH ROUTE DONE" /tmp/m4_route.log 2>/dev/null; do
  if [ $(( $(date +%s) - T_EXPLORED )) -gt 900 ]; then echo "== route time limit (15 min) reached =="; break; fi
  sleep 5
done
T_DONE=$(date +%s)
sleep 3
echo "== saving the 3D map =="
ros2 service call /map_3d/save std_srvs/srv/Trigger "{}" | grep -E "success|message"
echo "== landing =="
ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"
sleep 10
pkill -f "search_route.launch.py"; pkill -f "drone_search/search_route"
echo "== saving maps and results to media\\m4 =="
mkdir -p "$OUT"; cd ~/slam_drone/maps
ros2 run nav2_map_server map_saver_cli -f m4_building --ros-args -p use_sim_time:=true \
  -p save_map_timeout:=10.0 && cp m4_building.* "$OUT/"
cp victims.csv search_route.txt search_route.png "$OUT/" 2>/dev/null && echo "victims.csv + search_route.txt/.png saved"
P=$(ls -t map3d_*.ply 2>/dev/null | head -1); [ -n "$P" ] && cp "$P" "$OUT/" && echo "3D map $P saved"
cp /tmp/m4_route.log "$OUT/m4_route.log"
{
  echo "exploration: $((T_EXPLORED - T_START)) s"
  echo "search route: $((T_DONE - T_EXPLORED)) s"
  echo "total mission: $((T_DONE - T_START)) s"
} | tee "$OUT/m4_times.txt"
echo
echo "================= SEARCH ROUTE ================="
grep -h -E "SEARCH ROUTE PLAN|SEARCH ROUTE DONE" /tmp/m4_route.log | sed 's/.*SEARCH/SEARCH/'
echo
echo "================= VICTIMS FOUND ================="
grep -h "VICTIM .* FOUND" "$OUT/m4_victims.log" | sed 's/.*VICTIM/VICTIM/'
echo "(4 victims are hidden in the building)"
echo
echo "=== M4 DONE. Stop the recording. Files: automotive\\media\\m4 ==="
exec bash
