#!/bin/bash
# Tab 1: one-time setup (RTAB-Map, camera airframe + model), copy latest code, build, start PX4 + Gazebo.
# The first run asks for your sudo password (apt) and rebuilds PX4 for the new airframe (a few minutes).
source "$(dirname "$0")/env.sh"
rm -f /tmp/m3v_build_done
pkill -f "gz sim"; pkill -x px4; pkill -f "make px4_sitl"; pkill -x MicroXRCEAgent; pkill -x rviz2
pkill -f "ros2 launch"; sleep 2
if ! ros2 pkg list 2>/dev/null | grep -q '^rtabmap_slam$'; then
  echo "== installing RTAB-Map (one time, needs your password) =="
  sudo apt update && sudo apt install -y ros-jazzy-rtabmap-ros
  source /opt/ros/jazzy/setup.bash
fi
echo "== camera airframe + model =="
AF=~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
cp "$M3V/px4/airframes/4096_gz_x500_lidar_3d_ev_cam" $AF/
rm -rf ~/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_3d_ev_cam
cp -r "$M3V/px4/models/x500_lidar_3d_ev_cam" ~/PX4-Autopilot/Tools/simulation/gz/models/
grep -q 4096_gz_x500_lidar_3d_ev_cam $AF/CMakeLists.txt || \
  sed -i 's/^\(\s*\)4097_gz_x500_lidar_3d_ev$/&\n\14096_gz_x500_lidar_3d_ev_cam/' $AF/CMakeLists.txt
if ! grep -q 4096_gz_x500_lidar_3d_ev_cam $AF/CMakeLists.txt; then
  echo "!! Could not add 4096 to $AF/CMakeLists.txt. Send Claude a photo of: grep -n 409 $AF/CMakeLists.txt"
  exec bash
fi
echo "== copying latest code + world + RViz layout, building =="
rm -rf ~/slam_drone/src/drone_explore ~/slam_drone/src/drone_vslam
cp -r "$AUTO/code/m2b_exploration/drone_explore" ~/slam_drone/src/
cp -r "$M3V/drone_vslam" ~/slam_drone/src/
cp "$AUTO/code/m0_setup/building.sdf" ~/PX4-Autopilot/Tools/simulation/gz/worlds/building.sdf
R="$AUTO/code/m1_slam/src/slam_drone/config/m2.rviz"
cp "$R" ~/slam_drone/src/slam_drone/config/m1.rviz
cp "$R" ~/slam_drone/install/slam_drone/share/slam_drone/config/m1.rviz 2>/dev/null
cd ~/slam_drone && colcon build --symlink-install --packages-select drone_explore drone_vslam \
  --allow-overriding drone_explore drone_vslam || { echo "!! BUILD FAILED: send Claude a photo of the lines above"; exec bash; }
source install/setup.bash
ros2 pkg list | grep -q '^drone_vslam$' || { echo "!! drone_vslam not built: send Claude a photo"; exec bash; }
touch /tmp/m3v_build_done
echo "== starting PX4 + Gazebo with the camera drone (GPS off) =="
cd ~/PX4-Autopilot
if [ "$1" = "gui" ]; then
  PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev_cam
else
  HEADLESS=1 PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev_cam
fi
exec bash
