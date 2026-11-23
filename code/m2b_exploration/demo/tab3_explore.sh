#!/bin/bash
# Tab 3: wait until the drone hovers at 1.5 m, start exploration, then land and save the map
source "$(dirname "$0")/env.sh"
sleep 30
echo "== waiting for the drone to reach 1.5 m =="
while true; do
  z=$(timeout 5 ros2 topic echo /fmu/out/vehicle_local_position_v1 --once --field z 2>/dev/null | head -1)
  [[ "$z" =~ ^-?[0-9.]+$ ]] && awk "BEGIN{exit !($z < -1.3)}" && break
  sleep 3
done
echo
echo "############################################################"
echo "  Drone is hovering. START THE RECORDING NOW (Win+Alt+R)."
echo "  Exploration starts in 15 s."
echo "############################################################"
sleep 15
rm -f /tmp/m2b_explore.log
ros2 launch drone_explore explore.launch.py 2>&1 | tee /tmp/m2b_explore.log &
until grep -q "Exploration stopped" /tmp/m2b_explore.log 2>/dev/null; do sleep 3; done
echo "== exploration finished, hovering 5 s then landing =="
sleep 5
ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"
pkill -f "explore_lite/explore" ; pkill -f "explore.launch.py"
echo "== saving map =="
mkdir -p ~/slam_drone/maps && cd ~/slam_drone/maps
ros2 run nav2_map_server map_saver_cli -f m2b_building --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
mkdir -p "$AUTO/media/m2b_map" && cp m2b_building.* "$AUTO/media/m2b_map/"
echo
echo "=== M2b DONE: map saved to automotive\\media\\m2b_map. Stop the recording after it lands. ==="
exec bash
