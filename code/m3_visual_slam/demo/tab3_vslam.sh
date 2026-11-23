#!/bin/bash
# Tab 3: camera visual SLAM (camera_guard + RTAB-Map) once the drone hovers. Output saved to media\m3v\
source "$(dirname "$0")/env.sh"
sleep 5
until [ -f /tmp/m3v_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"     # again, so the freshly built drone_vslam is found
sleep 30
echo "== waiting for the drone to reach 1.5 m =="
while true; do
  z=$(timeout 5 ros2 topic echo /fmu/out/vehicle_local_position_v1 --once --field z 2>/dev/null | head -1)
  [[ "$z" =~ ^-?[0-9.]+$ ]] && awk "BEGIN{exit !($z < -1.3)}" && break
  sleep 3
done
if ! gz topic -l 2>/dev/null | grep -q '^/camera/image$'; then
  echo "!! Gazebo has no /camera/image topic. Send Claude a photo of:  gz topic -l | grep -i cam"
fi
mkdir -p "$AUTO/media/m3v"
echo "== starting camera + RTAB-Map =="
ros2 launch drone_vslam vslam.launch.py 2>&1 | tee "$AUTO/media/m3v/m3v_vslam.log" | \
  grep --line-buffered -E "camera_guard|CAMERA|rtabmap.*(Error|error|WARN|Rate)|Odom" 
exec bash
