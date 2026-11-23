#!/bin/bash
# M2a Tab 1: PX4 + Gazebo with GPS OFF (airframe 4097). First run rebuilds PX4 (~3-5 min).
# Add "gui" as argument to see the Gazebo window (slower):  bash m2a_tab1_px4.sh gui
pkill -f "gz sim"; pkill -x px4; pkill -x MicroXRCEAgent; pkill -x rviz2; sleep 2
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
cd ~/PX4-Autopilot
if [ "$1" = "gui" ]; then
  PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev
else
  HEADLESS=1 PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev
fi
