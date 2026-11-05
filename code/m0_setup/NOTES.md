# M0 – setup (done)
- Win11 + WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.17.0 (~/PX4-Autopilot), XRCE agent v2.4.3, ~/ws_px4 (px4_msgs release/1.17)
- GPU: export GALLIUM_DRIVER=d3d12 (RTF ~100%)
- Model x500_lidar_3d (2D + 3D LiDAR), airframe 4098, world building.sdf – files in WSL ~/slam_drone/m0
- Launch: PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d
- PX4 out topics have _v1 suffix. No cameras (Gazebo cameras leak memory under WSL).
