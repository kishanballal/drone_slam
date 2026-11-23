#!/bin/bash
# Tab 2: wait for the sim, then the M2b stack (GPS-off LiDAR SLAM, Nav2, bridge, RViz)
echo "== waiting for setup/build + Gazebo (first run: PX4 rebuild takes a few minutes) =="
sleep 5
until [ -f /tmp/m3_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"
until gz topic -l 2>/dev/null | grep -q '/scan$'; do sleep 2; done
echo "== Gazebo up, giving PX4 20 s to get ready =="
sleep 20
ros2 launch drone_explore m2b.launch.py 2>&1 | grep --line-buffered -v rf2o
exec bash
