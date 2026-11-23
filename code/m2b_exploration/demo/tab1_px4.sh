#!/bin/bash
# Tab 1: copy latest code + world, build, start PX4 + Gazebo with GPS OFF (airframe 4097).
# Headless by default. For the Gazebo window too: run_m2b_demo.bat gui
source "$(dirname "$0")/env.sh"
rm -f /tmp/m2b_build_done
pkill -f "gz sim"; pkill -x px4; pkill -f "make px4_sitl"; pkill -x MicroXRCEAgent; pkill -x rviz2
pkill -f "ros2 launch"; sleep 2
echo "== copying latest drone_explore + world, building =="
rm -rf ~/slam_drone/src/drone_explore && cp -r "$AUTO/code/m2b_exploration/drone_explore" ~/slam_drone/src/
cp "$AUTO/code/m0_setup/building.sdf" ~/PX4-Autopilot/Tools/simulation/gz/worlds/building.sdf
# RViz layout with goal tool, cost planner path, replan marker, dynamic obstacle markers
R="$AUTO/code/m1_slam/src/slam_drone/config/m2.rviz"
cp "$R" ~/slam_drone/src/slam_drone/config/m1.rviz
cp "$R" ~/slam_drone/install/slam_drone/share/slam_drone/config/m1.rviz 2>/dev/null
cd ~/slam_drone && colcon build --symlink-install --packages-select drone_explore --allow-overriding drone_explore
touch /tmp/m2b_build_done
echo "== starting PX4 + Gazebo (GPS off) =="
cd ~/PX4-Autopilot
if [ "$1" = "gui" ]; then
  PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev
else
  HEADLESS=1 PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev
fi
exec bash
