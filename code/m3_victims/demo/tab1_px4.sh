#!/bin/bash
# Tab 1: victims world + models, 3D LiDAR tilt, copy latest code, build, start PX4 + Gazebo (GPS off).
source "$(dirname "$0")/env.sh"
rm -f /tmp/m3_build_done
pkill -f "gz sim"; pkill -x px4; pkill -f "make px4_sitl"; pkill -x MicroXRCEAgent; pkill -x rviz2
pkill -f "ros2 launch"; sleep 2
GZ=~/PX4-Autopilot/Tools/simulation/gz
echo "== world with victims (wide doors) + victim models =="
cp "$AUTO/code/m0_setup/building.sdf" $GZ/worlds/building.sdf
rm -f $GZ/worlds/building_victims.sdf
python3 "$M3/slam_drone_m3/tools/make_victims_world.py" $GZ/worlds/building.sdf || { echo "!! world failed: send Claude a photo"; exec bash; }
for m in victim_lying victim_sitting; do rm -rf $GZ/models/$m; cp -r "$M3/slam_drone_m3/models/$m" $GZ/models/; done
echo "== 3D LiDAR: look further down =="
python3 "$(dirname "$0")/lidar_tilt.py" $GZ/models/x500_lidar_3d/model.sdf
echo "== copying latest code + RViz layout, building =="
rm -rf ~/slam_drone/src/drone_explore ~/slam_drone/src/slam_drone_m3
cp -r "$AUTO/code/m2b_exploration/drone_explore" ~/slam_drone/src/
cp -r "$M3/slam_drone_m3" ~/slam_drone/src/
R="$AUTO/code/m1_slam/src/slam_drone/config/m2.rviz"
cp "$R" ~/slam_drone/src/slam_drone/config/m1.rviz
cp "$R" ~/slam_drone/install/slam_drone/share/slam_drone/config/m1.rviz 2>/dev/null
cd ~/slam_drone && colcon build --symlink-install --packages-select drone_explore slam_drone_m3 \
  --allow-overriding drone_explore slam_drone_m3 || { echo "!! BUILD FAILED: send Claude a photo of the lines above"; exec bash; }
source install/setup.bash
ros2 pkg list | grep -q '^slam_drone_m3$' || { echo "!! slam_drone_m3 not built: send Claude a photo"; exec bash; }
(cd src/slam_drone_m3 && python3 -m pytest -q test 2>&1 | tail -2)
mkdir -p "$OUT"; rm -f ~/slam_drone/maps/victims.csv
touch /tmp/m3_build_done
echo "== starting PX4 + Gazebo in building_victims (GPS off) =="
cd ~/PX4-Autopilot
if [ "$1" = "gui" ]; then
  PX4_GZ_WORLD=building_victims make px4_sitl gz_x500_lidar_3d_ev
else
  HEADLESS=1 PX4_GZ_WORLD=building_victims make px4_sitl gz_x500_lidar_3d_ev
fi
exec bash
