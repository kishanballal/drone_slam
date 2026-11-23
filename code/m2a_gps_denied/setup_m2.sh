#!/bin/bash
# One-time setup for M2a + M2b + M2c. Safe to run again (it skips what is done).
#   bash /mnt/c/Users/kisha/PROJECTS/automotive/code/m2a_gps_denied/setup_m2.sh
# Takes 10-20 min the first time (rf2o + Nav2 + explore_lite + a PX4 rebuild later).
set -e
AUTO=/mnt/c/Users/kisha/PROJECTS/automotive/code
M2A=$AUTO/m2a_gps_denied
S=~/slam_drone/src/slam_drone
ok()   { echo -e "\e[32m[OK]\e[0m $*"; }
warn() { echo -e "\e[33m[WARN]\e[0m $*"; }
die()  { echo -e "\e[31m[ERROR]\e[0m $*"; exit 1; }

source /opt/ros/jazzy/setup.bash
source ~/ws_px4/install/setup.bash

echo "=== 1/6  rf2o laser odometry (~/ws_ext) ==="
if [ ! -d ~/ws_ext/src/rf2o_laser_odometry ]; then
  mkdir -p ~/ws_ext/src
  git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git ~/ws_ext/src/rf2o_laser_odometry
fi
cd ~/ws_ext
rosdep install --from-paths src --ignore-src -r -y || warn "rosdep had complaints (usually fine)"
colcon build --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release
grep -q "ws_ext/install" ~/.bashrc || echo "source ~/ws_ext/install/local_setup.bash" >> ~/.bashrc
source ~/ws_ext/install/local_setup.bash
ros2 pkg executables rf2o_laser_odometry | grep -q rf2o_laser_odometry_node || die "rf2o did not build: send a photo of the first red error"
ok "rf2o built"

echo "=== 2/6  PX4 GPS-denied airframe 4097 + model ==="
AF=~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
cp $M2A/px4/airframes/4097_gz_x500_lidar_3d_ev $AF/
mkdir -p ~/PX4-Autopilot/Tools/simulation/gz/models
cp -r $M2A/px4/models/x500_lidar_3d_ev ~/PX4-Autopilot/Tools/simulation/gz/models/
if ! grep -q "4097_gz_x500_lidar_3d_ev" $AF/CMakeLists.txt; then
  sed -i 's/^\(\s*\)4098_gz_x500_lidar_3d$/&\n\14097_gz_x500_lidar_3d_ev/' $AF/CMakeLists.txt
fi
grep -q "4097_gz_x500_lidar_3d_ev" $AF/CMakeLists.txt || die "add the line 4097_gz_x500_lidar_3d_ev under 4098_gz_x500_lidar_3d in $AF/CMakeLists.txt by hand, then run me again"
ok "airframe 4097 registered"

echo "=== 3/6  slam_drone: M2a nodes, configs, launch ==="
cp $M2A/slam_drone/slam_drone/laser_odom_to_px4.py $M2A/slam_drone/slam_drone/odom_compare.py $S/slam_drone/
cp $M2A/slam_drone/config/rf2o.yaml $M2A/slam_drone/config/rf2o_bench.yaml $S/config/
cp $M2A/slam_drone/launch/m2a.launch.py $S/launch/
python3 - "$S/setup.py" <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
add = ["'laser_odom_to_px4 = slam_drone.laser_odom_to_px4:main',",
       "'odom_compare = slam_drone.odom_compare:main',"]
key = "'console_scripts': ["
if key not in s:
    sys.exit('setup.py has no console_scripts list')
for a in add:
    if a.split('=')[0] not in s:
        s = s.replace(key, key + "\n            " + a, 1)
open(p, 'w').write(s)
PY
ok "slam_drone updated"

echo "=== 4/6  Nav2 (apt) ==="
if ! ros2 pkg list | grep -q nav2_bt_navigator; then
  sudo apt update && sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
fi
ok "Nav2 installed ($(ros2 pkg list | grep -c nav2) nav2 packages)"

echo "=== 5/6  explore_lite + drone_explore (M2b) + drone_planning (M2c) ==="
cd ~/slam_drone/src
[ -d m-explore-ros2 ] || git clone https://github.com/robo-friends/m-explore-ros2.git
touch m-explore-ros2/map_merge/COLCON_IGNORE
rm -rf drone_explore drone_planning
cp -r $AUTO/m2b_exploration/drone_explore .
cp -r $AUTO/m2c_cost_planning/drone_planning .
find drone_explore drone_planning -name __pycache__ -prune -exec rm -rf {} +
cd ~/slam_drone
grep -qx "src/m-explore-ros2/" .gitignore 2>/dev/null || echo "src/m-explore-ros2/" >> .gitignore
rosdep install --from-paths src/m-explore-ros2 --ignore-src -r -y || warn "rosdep had complaints (usually fine)"

echo "=== 6/6  build ==="
set +e
colcon build --symlink-install --packages-select slam_drone drone_explore drone_planning
R1=$?
colcon build --symlink-install --packages-select explore_lite
R2=$?
set -e
source install/setup.bash
[ $R1 -eq 0 ] && ok "slam_drone, drone_explore, drone_planning built" || die "build failed: send a photo of the first red error"
[ $R2 -eq 0 ] && ok "explore_lite built" || warn "explore_lite failed: fine for now (plan B exists); send a photo later"
python3 -m pytest -q src/drone_explore/test src/drone_planning/test 2>/dev/null | tail -1 || true
ros2 pkg executables slam_drone

echo
ok "SETUP DONE. Close this tab and open new tabs (so ~/.bashrc is reloaded)."
