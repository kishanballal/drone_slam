# SLAM Drone: GPS-denied indoor search and rescue (simulation)

A simulated PX4 x500 drone enters an unknown building with **no GPS, no prior map and no remote pilot**,
maps it with LiDAR SLAM, explores it on its own and marks where the victims are.

Team: Kishan Ballal (PX4 side), Yathivandith JP (perception). ECE, B section.
Stack: Ubuntu 24.04 (WSL2) · ROS 2 Jazzy · Gazebo Harmonic · PX4 v1.17 SITL · Micro-XRCE-DDS · slam_toolbox · rf2o · Nav2 · explore_lite · RTAB-Map.

## Milestones (all complete)
| Milestone | What it shows | Code | One-click demo |
|---|---|---|---|
| M0 | Indoor world (rooms, corridor loop) + x500 with 2D and 3D LiDAR | `code/m0_setup` | — |
| M1 | PX4 offboard waypoint flight, slam_toolbox 2D map + 3D LiDAR map growing live | `code/m1_slam`, `code/m1_waypoints` | `code/m1_slam/demo/run_m1_demo.bat` |
| M2a | GPS off: rf2o laser odometry fed into PX4 EKF2 as external vision | `code/m2a_gps_denied` | (used by M2b/M2c/M3) |
| M2b | Autonomous exploration with Nav2 + explore_lite, GPS off | `code/m2b_exploration` | `code/m2b_exploration/demo/run_m2b_demo.bat` |
| M2c | Cost-based planner (distance + collision risk + battery + path complexity), moving people, live re-planning | `code/m2c_cost_planning` | `code/m2c_cost_planning/demo/run_m2c_demo.bat` |
| M3v | Camera visual SLAM (RTAB-Map RGB-D) with a memory guard | `code/m3_visual_slam` | `code/m3_visual_slam/demo/run_m3v_demo.bat` |
| M3 | Search and rescue: explore, build a 3D LiDAR map, detect victims (lying / sitting), drop pins on the map | `code/m3_victims` | `code/m3_victims/demo/run_m3_demo.bat gui` |

## M3 result (final run, Nov 23, 2026)
4 victims hidden in the building (2 lying, 2 sitting). GPS off, no prior map.

| Victim | Type | Found at (m) | Error |
|---|---|---|---|
| 1 | lying | (3.06, -4.38) | 0.13 m |
| 2 | lying | (8.55, 5.44) | 0.08 m |
| 3 | sitting | (-2.97, 6.52) | 0.08 m |
| 4 | sitting | (-7.91, -6.34) | 0.19 m |

All 4 found. One extra false pin appeared 1.2 m from victim 3 (a duplicate from map drift); known limitation.
Files: `media/m3/` (victims.csv, 2D map, 3D map `.ply`, run log, video frames).

How it works: the 3D LiDAR cloud is placed in the SLAM map using x/y/yaw from slam_toolbox and height + roll/pitch
from PX4, each scan is snapped so the floor is at z = 0, points go into a 0.1 m voxel map, and low person-sized
clusters away from walls are classified as lying or sitting (furniture is rejected by its flat top).
A victim is only pinned after it is seen in 4 map updates in a row.

## Videos
Demo videos are larger than GitHub's 100 MB limit, so they are attached to the GitHub Releases of this repo.

## Run it
Each milestone folder has a `GUIDE.md` or `demo/` folder. The `.bat` files open Windows Terminal tabs in WSL,
copy the code into `~/slam_drone`, build, start PX4 + Gazebo and run the demo end to end.
