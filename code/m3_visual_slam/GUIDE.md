# M3v: visual SLAM with one memory-guarded camera

Covers the professor's "visual SLAM" item. LiDAR stays the main sensor for the flight
(GPS-denied odometry, slam_toolbox, Nav2). The camera adds RTAB-Map RGB-D visual SLAM: a
coloured 3D map and visual loop closures. There's also an experimental pure-camera mode,
which is the cheap-hardware story: a camera costs far less than a LiDAR.

**Time box: Wed 18 Nov, one evening.** If Gazebo memory still runs away with every
saver on, stop, keep the memory log as evidence, and present LiDAR SLAM (that is the
fallback we agreed).

## How the memory is managed

| Saver | Effect |
|---|---|
| 320 x 240 instead of 640 x 480 | 4x fewer pixels per frame |
| 5 Hz instead of 30 Hz | 6x fewer frames |
| One `rgbd_camera` sensor | colour + depth from one sensor, no separate camera |
| Gazebo renders a camera only while something subscribes to it | no cost when it isn't used |
| `camera_guard` owns the camera bridge | it starts the bridge, then **stops** it (the camera stops rendering) when Gazebo grows by 1.5 GB, Gazebo passes 7 GB, free RAM drops under 2.5 GB, or after 10 minutes. The flight and LiDAR SLAM keep going. |
| RTAB-Map at 1 Hz, half-size images stored | small database, low CPU |
| Memory log | `~/slam_drone/logs/camera_mem_*.csv` (for the report: memory with camera on vs off) |

You can also cap WSL itself so a leak can never freeze Windows. Put this in
`C:\Users\kisha\.wslconfig`, then run `wsl --shutdown` in PowerShell:

```
[wsl2]
memory=16GB
swap=8GB
```

## Step 1: Install RTAB-Map (any tab, once)

```bash
sudo apt update
sudo apt install -y ros-jazzy-rtabmap-ros
ros2 pkg list | grep -E "rtabmap_(odom|slam)"     # both listed
```

## Step 2: Add the camera drone to PX4 (any tab, once)

```bash
A=/mnt/c/Users/kisha/PROJECTS/automotive/code/m3_visual_slam
cp $A/px4/airframes/4096_gz_x500_lidar_3d_ev_cam ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
cp -r $A/px4/models/x500_lidar_3d_ev_cam ~/PX4-Autopilot/Tools/simulation/gz/models/
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
grep -q 4096_gz_x500_lidar_3d_ev_cam CMakeLists.txt || \
  sed -i 's/^\(\s*\)4097_gz_x500_lidar_3d_ev$/&\n\14096_gz_x500_lidar_3d_ev_cam/' CMakeLists.txt
grep -n "409[678]_" CMakeLists.txt                  # all three listed
```

If the last command doesn't show `4096_gz_x500_lidar_3d_ev_cam`, add that line by hand
under the 4097 line (same indentation).

## Step 3: Build the package (any tab, once)

```bash
cp -r /mnt/c/Users/kisha/PROJECTS/automotive/code/m3_visual_slam/drone_vslam ~/slam_drone/src/
cd ~/slam_drone && colcon build --symlink-install --packages-select drone_vslam
source install/setup.bash
```

## Step 4: Run it

| Tab | Command |
|---|---|
| **Tab 1** | `cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev_cam` |
| **Tab 2** | agent, if you use a separate tab |
| **Tab 3** | `ros2 launch drone_planning m2c.launch.py` (or `m2a.launch.py waypoints:=true` for the simple loop flight) |
| **Tab 4** | `ros2 launch drone_vslam vslam.launch.py` |
| **Tab 5** | `ros2 launch drone_explore explore.launch.py`, or click goals |
| **Tab 6** | `ros2 topic echo /camera_guard/status` (memory every 5 s) |

The first run of Tab 1 builds PX4 again for the new airframe, which takes a few minutes.

Checks (Tab 6):

```bash
ros2 topic hz /camera/image            # about 5 Hz
ros2 topic hz /rtabmap/cloud_map       # about 1 Hz once the drone moves
```

RViz: add **PointCloud2 `/rtabmap/cloud_map`** (Color Transformer RGB8) and
**Path `/rtabmap/mapPath`**.

✅ Pass when the coloured RTAB-Map cloud lines up with the slam_toolbox map, and
`/camera_guard/status` shows `growth_mb` levelling off rather than climbing steadily
for 10 minutes.

## Step 5 (experiment): pure camera odometry

```bash
ros2 launch drone_vslam vslam.launch.py visual_odom:=true
```

RViz: add **Odometry `/vo/odom`** next to `/odom` (rf2o). Plain grey Gazebo walls have few
visual features, so camera odometry may get lost (RTAB-Map prints `Odom: quality=0`).
That is a real result for the report: in a textureless building a camera alone is not
enough, which is why the LiDAR leads. If you have time, Yathivandith can give the walls
a texture, and you can run it again.

## Step 6: Freeze

```bash
cd ~/slam_drone
git add src/drone_vslam && git commit -m "M3v: RTAB-Map visual SLAM with memory-guarded camera" && git tag m3v
```

Copy `~/slam_drone/logs/camera_mem_*.csv` to `automotive\media\` for the report.

## When something breaks

| Symptom | Check |
|---|---|
| `make` says unknown target `gz_x500_lidar_3d_ev_cam` | Step 2's CMakeLists line is missing. |
| No `/camera/image` | `gz topic -l \| grep camera` must list `/camera/image`. If the names differ, send me the list. |
| `CAMERA OFF: Gazebo memory grew ...` | The guard did its job. Restart the camera when you want: `ros2 service call /camera_guard/start std_srvs/srv/Trigger`. If it trips within 2 minutes, set `<update_rate>2</update_rate>` in the model. |
| RTAB-Map: `Did not receive data since 5 seconds` | Check `ros2 topic hz /camera/depth_image`, and `ros2 run tf2_ros tf2_echo base_link camera_optical`. |
| RTAB-Map: `TF of received image ... is not set` | The camera frame is not `camera_optical`. Run `ros2 topic echo /camera/image --field header.frame_id --once` and send me the output. |
