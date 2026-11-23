#!/bin/bash
# Tab 2: wait for the sim, then start M2b (rf2o odometry, slam_toolbox, Nav2, bridge, RViz)
echo "== waiting for build + Gazebo =="
sleep 5
until [ -f /tmp/m2b_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"
until gz topic -l 2>/dev/null | grep -q '/scan$'; do sleep 2; done
echo "== Gazebo up, giving PX4 20 s to get ready =="
sleep 20
ros2 launch drone_explore m2b.launch.py 2>&1 | grep -v rf2o
exec bash
