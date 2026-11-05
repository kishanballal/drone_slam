# SLAM Drone (GPS-denied indoor search & rescue) – project archive

Team: Kishan Ballal (PX4 side), Yathivandith JP (perception). Deadline: Nov 21, 2026.

Simulated x500 drone (PX4 SITL + ROS 2 Jazzy + Gazebo Harmonic) that maps an unknown indoor building with LiDAR SLAM.

## Status (Nov 5, 2026)
- **M0** setup + indoor world: done
- **M1** slam_toolbox 2D map + waypoint flight + 3D height map: done (git tag `m1`)
- Next: M2 (GPS-denied flight + autonomous exploration) and M3 (victim detection). Added here once they're complete.

Demo videos are too big for git (over 100 MB), so they are attached to GitHub Releases instead.

## Layout
| Folder | What's inside |
|---|---|
| `docs/` | Milestone build plan, project handoff (source of truth since Nov 3), work log (PDFs) |
| `media/m1_map/` | M1 results: 2D SLAM map, 3D point-cloud maps (.ply), previews |
| `code/m0_setup/` | M0 notes – model `x500_lidar_3d`, airframe 4098, world `building.sdf` (live files in `~/slam_drone/m0`) |
| `code/m1_slam/` | M1: `slam_drone` ROS 2 package (odom_tf, m1.launch.py, slam_toolbox config, RViz) + `M1_guide.md` |
| `code/m1_waypoints/` | M1: PX4 offboard waypoint script `waypoint_flight.py` + `GUIDE.md` |

## Launch quick reference
- M1: `ros2 launch slam_drone m1.launch.py fly:=true`

Each milestone: freeze with a git tag + recording (save videos in `media/`).
