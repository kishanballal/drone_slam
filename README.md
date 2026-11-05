# SLAM Drone (GPS-denied indoor search & rescue) – project archive

Team: Kishan Ballal (PX4 side), Yathivandith JP (perception). Deadline: Nov 21, 2026.

**This folder is the organised archive / reference copy.**
The live code you actually build and run is in WSL at `~/slam_drone` (git repo + colcon workspace).
Copy files from `code/` into `~/slam_drone/src/...` as each guide says.

## Status (Nov 5, 2026)
- **M0** setup + indoor world: done
- **M1** slam_toolbox 2D map + waypoint flight + 3D height map: done (git tag `m1`)
- **M2a / M2b / M3**: drafts, not yet tested end to end

Demo videos are too big for git (over 100 MB), so they are attached to GitHub Releases instead.

## Layout
| Folder | What's inside |
|---|---|
| `docs/` | Milestone build plan, project handoff (source of truth since Nov 3), work log (PDFs) |
| `media/` | Recordings (e.g. M0 RViz sensor demo, Oct 22) |
| `code/m0_setup/` | M0 notes – model `x500_lidar_3d`, airframe 4098, world `building.sdf` (live files in `~/slam_drone/m0`) |
| `code/m1_slam/` | M1: `slam_drone` ROS 2 package (odom_tf, m1.launch.py, slam_toolbox config, RViz) + `M1_guide.md` |
| `code/m1_waypoints/` | M1: PX4 offboard waypoint script `waypoint_flight.py` + `GUIDE.md` |
| `code/m2a_gps_denied/` | M2a: rf2o laser odometry -> PX4 EKF2, airframe 4097 `_ev`, `m2a.launch.py` |
| `code/m2b_exploration/` | M2b: `drone_explore` package (Nav2 cmd_vel bridge, explore_lite, plan-B frontier explorer) + `GUIDE.md` |
| `code/m3_victims/` | M3: `slam_drone_m3` package (3D height map, LiDAR victim detection, victim world) |

## Launch quick reference
- Sim: `PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d` (M2a+: `gz_x500_lidar_3d_ev`)
- M1: `ros2 launch slam_drone m1.launch.py fly:=true`
- M2a: `ros2 launch slam_drone m2a.launch.py`
- M2b: `ros2 launch drone_explore m2b.launch.py`
- M3: `ros2 launch slam_drone_m3 m3.launch.py`

Each milestone: freeze with a git tag + recording (save videos in `media/`).
