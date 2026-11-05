# shared environment for the M1 demo scripts
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
source /opt/ros/jazzy/setup.bash
source ~/ws_px4/install/setup.bash
[ -f ~/slam_drone/install/setup.bash ] && source ~/slam_drone/install/setup.bash
AUTO=/mnt/c/Users/kisha/PROJECTS/automotive
