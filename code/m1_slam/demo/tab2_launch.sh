#!/bin/bash
# Tab 2: wait for the sim, then launch M1 with the autonomous loop
rm -f /tmp/m1_build_done
echo "== waiting for build + Gazebo =="
until [ -f /tmp/m1_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"
until gz topic -l 2>/dev/null | grep -q '/scan$'; do sleep 2; done
echo "== Gazebo up, giving PX4 20 s to get ready =="
sleep 20
ros2 launch slam_drone m1.launch.py fly:=true map3d:=true
exec bash
