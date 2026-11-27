#!/bin/bash
# Tab 3: 3D height map + victim detector (from M3) once the drone hovers. Log saved to media\m4\
source "$(dirname "$0")/env.sh"
sleep 5
until [ -f /tmp/m4_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"     # again, so the freshly built packages are found
sleep 30
echo "== waiting for the drone to reach 1.5 m =="
while true; do
  z=$(timeout 5 ros2 topic echo /fmu/out/vehicle_local_position_v1 --once --field z 2>/dev/null | head -1)
  [[ "$z" =~ ^-?[0-9.]+$ ]] && awk "BEGIN{exit !($z < -1.3)}" && break
  sleep 3
done
echo "== starting 3D map + victim detector =="
ros2 launch slam_drone_m3 m3.launch.py rviz:=false 2>&1 | tee "$OUT/m4_victims.log" | \
  grep --line-buffered -E "VICTIM|error|Error|saved|PLY|WARN"
exec bash
