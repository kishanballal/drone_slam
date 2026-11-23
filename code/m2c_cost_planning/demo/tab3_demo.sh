#!/bin/bash
# Tab 3: the demo script.
#  Part 1: autonomous exploration; every path comes from the cost planner. Map saved.
#  Part 2: two moving walkers appear; the drone flies goals through their corridors and re-plans.
#  Part 3: back to the start, land.
source "$(dirname "$0")/env.sh"
goal() {   # goal x y label
  echo
  echo "== GOAL $3: ($1, $2) =="
  timeout 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}}" \
    | grep -E "accepted|rejected|status|SUCCEEDED|ABORTED|CANCELED"
}
sleep 30
echo "== waiting for the drone to reach 1.5 m =="
while true; do
  z=$(timeout 5 ros2 topic echo /fmu/out/vehicle_local_position_v1 --once --field z 2>/dev/null | head -1)
  [[ "$z" =~ ^-?[0-9.]+$ ]] && awk "BEGIN{exit !($z < -1.3)}" && break
  sleep 3
done
until ros2 action list 2>/dev/null | grep -q compute_path_to_pose_cost; do
  echo "waiting for cost_planner ..."; sleep 3; done
sleep 10
echo
echo "############################################################"
echo "  Drone is hovering. START THE RECORDING NOW (Win+Alt+R)."
echo "  Part 1 (exploration with the cost planner) starts in 15 s."
echo "############################################################"
sleep 15
rm -f /tmp/m2c_explore.log
ros2 launch drone_explore explore.launch.py 2>&1 | tee /tmp/m2c_explore.log &
until grep -q "Exploration stopped" /tmp/m2c_explore.log 2>/dev/null; do sleep 3; done
pkill -f "explore_lite/explore"; pkill -f "explore.launch.py"
echo "== Part 1 done: saving the map =="
mkdir -p ~/slam_drone/maps "$AUTO/media/m2c_map"
(cd ~/slam_drone/maps && ros2 run nav2_map_server map_saver_cli -f m2c_building \
   --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 && cp m2c_building.* "$AUTO/media/m2c_map/")

echo
echo "== Part 2: moving obstacles =="
# park the drone at the start first, so no walker is spawned on top of it
goal 0.0 0.0 "park at the start before the walkers appear"
if ! gz service -l 2>/dev/null | grep -q "/world/building/create"; then
  echo "!! Gazebo has no /world/building/create service: walkers cannot spawn. Send Claude a photo."
fi
ros2 launch drone_planning obstacles.launch.py 2>&1 | tee "$AUTO/media/m2c_obstacles.log" | grep --line-buffered "DYNAMIC OBSTACLE" &
sleep 12
goal -8.5  6.6 "A: west end of north corridor (meets walker_1)"
goal  6.0  6.8 "B: east part of north corridor (walker_1 again)"
goal  8.0 -0.3 "C: down the east side past walker_2"
goal -8.5  6.6 "D: all the way back to the north-west corner"
echo
echo "== Part 3: back to the start and land =="
goal 0.0 0.0 "HOME"
sleep 3
ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"
pkill -f obstacle_mover; pkill -f dynamic_tracker
echo
grep -E "REPLAN|NEW GOAL" "$AUTO/media/m2c_run.log" | tail -15
grep -E "DYNAMIC OBSTACLE #[0-9]+ detected" "$AUTO/media/m2c_obstacles.log"
echo
echo "=== M2c DONE. Stop the recording after it lands. ==="
echo "Log for the report: automotive\\media\\m2c_run.log   Map: automotive\\media\\m2c_map"
exec bash
