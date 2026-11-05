#!/bin/bash
# Tab 1: update waypoint node, build, start PX4 + Gazebo (with GUI for the video)
source "$(dirname "$0")/env.sh"
pkill -f "gz sim"; pkill -x px4; pkill -f "make px4_sitl"; pkill -x MicroXRCEAgent; pkill -x rviz2; sleep 2
echo "== copying latest waypoint_flight.py and building =="
cp "$AUTO/code/m1_waypoints/waypoint_flight.py" ~/slam_drone/src/slam_drone/slam_drone/
cp "$AUTO/code/m1_slam/src/slam_drone/launch/m1.launch.py" ~/slam_drone/src/slam_drone/launch/
cp "$AUTO/code/m1_slam/src/slam_drone/config/m1.rviz" ~/slam_drone/src/slam_drone/config/
rm -rf ~/slam_drone/src/slam_drone_m3 && cp -r "$AUTO/code/m3_victims/slam_drone_m3" ~/slam_drone/src/
cd ~/slam_drone && colcon build --symlink-install
touch /tmp/m1_build_done
echo "== starting PX4 + Gazebo =="
cd ~/PX4-Autopilot
PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d
exec bash
