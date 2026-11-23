#!/bin/bash
# Tab 2: wait for the sim, then start M2c (GPS-off stack + bridge + cost_planner + Nav2 with the cost tree).
# Everything except rf2o spam is also saved to automotive\media\m2c_run.log (for the report).
echo "== waiting for build + Gazebo =="
sleep 5
until [ -f /tmp/m2c_build_done ]; do sleep 2; done
source "$(dirname "$0")/env.sh"
until gz topic -l 2>/dev/null | grep -q '/scan$'; do sleep 2; done
echo "== Gazebo up, giving PX4 20 s to get ready =="
sleep 20
ros2 launch drone_planning m2c.launch.py 2>&1 | grep --line-buffered -v rf2o | tee "$AUTO/media/m2c_run.log"
exec bash
