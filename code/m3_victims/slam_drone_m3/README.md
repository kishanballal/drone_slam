# slam_drone_m3 — Milestone 3 (3D map + victim detection)

Add-on package. It only adds nodes on top of the M2 demo, so earlier milestones keep working.

| Piece | What it does |
|---|---|
| `height_map_node` | Accumulates `/lidar_3d/points` in the `map` frame into a 0.1 m voxel map, coloured by height, on `/map_3d`. Services `/map_3d/save` (PLY file) and `/map_3d/clear`. |
| `victim_detector_node` | Finds person-sized low clusters in `/map_3d` (lying or sitting), confirms after 3 updates, publishes red pins on `/victims/markers`, writes `~/slam_drone/maps/victims.csv`. |
| `models/victim_*` | Simple person-shaped Gazebo models (orange clothes). |
| `tools/make_victims_world.py` | Makes `building_victims.sdf` (building + 4 victims) without touching `building.sdf`. |
| `test/test_core_logic.py` | Offline test on a synthetic copy of the building: `python3 test/test_core_logic.py`. |

Launch: `ros2 launch slam_drone_m3 m3.launch.py` (args `rviz:=false`, `victims:=false`).
The full step-by-step plan is in the project's "M3 plan" doc.
