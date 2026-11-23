# M2c: cost-aware path planning + moving obstacles + real-time replanning

This milestone covers the professor's item:

> path planning with a cost function containing distance + collision risk + battery
> consumption + path complexity. Add dynamic obstacles and require the drone to re-plan
> its path in real time.

**Build it on top of M2b.** It needs the M2a stack (GPS off) and the M2b `drone_explore`
package (cmd_vel bridge + Nav2 params). Nothing in M1, M2a or M2b is edited.

## Schedule from today (Thu 12 Nov) to the deadline (Sat 21 Nov)

| Day | Work | Must pass |
|---|---|---|
| Thu 12 to Fri 13 | **M2a**: GPS off, rf2o into EKF2 (code in `code\m2a_gps_denied`, steps in the M2a doc) | M1 waypoint loop flies with GPS off. Tag `m2a`. |
| Sat 14 to Sun 15 | **M2b**: Nav2 + exploration (`code\m2b_exploration\GUIDE.md`, tests A, B and C) | Test B (3 clicked goals). Tag `m2b`. |
| Mon 16 to Tue 17 | **M2c** (this guide): cost planner + moving obstacles | Steps 4 to 6 below. Tag `m2c`. |
| Wed 18 | **M3v** visual SLAM camera (`code\m3_visual_slam\GUIDE.md`), one evening max | RTAB-Map map grows, memory stays flat |
| Thu 19 | **M3** 3D map + victims (`code\m3_victims`) | Victim pins on the map |
| Fri 20 to Sat 21 | Final demo run, recordings, report | |

**Deadline risk:** M2a has not run yet, and M2a + M2b + M2c are now in 5 days. The new
plan-B cutoff for Nav2 is **Sun 15 Nov evening**. If Nav2 Test B is not passing by then,
skip explore_lite and click goals by hand for the M2c demo (the cost planner and the
replanning still work, because they need Nav2's controller only, not explore_lite).

---

## What the planner does (for the report)

For one path P, `cost_planner` minimises

```
J(P) = w_d * D  +  w_r * R  +  w_b(soc) * E / e_ref  +  w_k * K
```

| Term | Meaning | Where it comes from |
|---|---|---|
| D, distance | metres flown | grid A*, 8 directions |
| R, collision risk | sum along the path of the costmap risk (0 to 1) x metres | Nav2 global costmap: the LiDAR obstacle layer + inflation (1 = touching a wall). Unmapped cells count as 0.35. |
| E, battery energy | Wh: hover power x flight time (slower near walls), plus yaw time and braking time for each turn | x500 model: 180 W hover, 0.5 m/s, 0.8 rad/s yaw |
| K, path complexity | total heading change in radians + 0.3 per turn | from the path shape |
| w_b(soc) | `w_b x (1 + 2 x (1 - battery))`. As the battery drains, energy matters more. | PX4 `battery_status` |

Real-time replanning: our behaviour tree (`behavior_trees/nav_to_pose_cost.xml`) asks
`cost_planner` for a new path **twice a second** on the live costmap. When a moving
obstacle blocks the current path, the next plan avoids it or takes the other way round the
corridor loop. The node logs `REPLAN #n: obstacle on the old path at (x, y) ...` and shows a
red sphere at that spot in RViz.

Obstacle detection: the 2D LiDAR `/scan` marks obstacles in Nav2's costmaps (5 to 8 Hz).
The RPP controller also checks the next 1 s of motion for collisions and stops if needed.

Offline check I ran: 6 unit tests pass (planning round a 2 m ring corridor, rerouting when
a walker blocks it, staying off walls as w_risk rises, saving energy at low battery, turn
counting). Planning takes about 0.1 s on a 30 x 20 m map. Gazebo, PX4 and Nav2 were not
part of that test.

---

## Terminal tabs

| Tab | Runs |
|---|---|
| **Tab 1** | PX4 + Gazebo: `cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev` |
| **Tab 2** | `MicroXRCEAgent udp4 -p 8888` (only if your M2a/M2b runs use a separate agent tab) |
| **Tab 3** | `ros2 launch drone_planning m2c.launch.py` |
| **Tab 4** | `ros2 launch drone_planning obstacles.launch.py` (the moving obstacles) |
| **Tab 5** | `ros2 launch drone_explore explore.launch.py` (exploration), or click goals in RViz |
| **Tab 6** | Checks and one-off commands |

---

## Step 1: Copy and build (Tab 6, once)

```bash
cp -r /mnt/c/Users/kisha/PROJECTS/automotive/code/m2c_cost_planning/drone_planning ~/slam_drone/src/
cd ~/slam_drone
colcon build --symlink-install --packages-select drone_planning
source install/setup.bash
python3 -m pytest -q src/drone_planning/test          # expect: 6 passed
ros2 pkg executables drone_planning                   # cost_planner, obstacle_mover
```

If `pytest` is missing: `sudo apt install -y python3-pytest`.

## Step 2: Let the PX4 battery drain slowly (Tab 1, in `pxh>`, once)

The SITL battery normally empties in 60 s and then sits at 50 %. These settings make it
drain over 15 minutes, down to 20 %, without triggering a low-battery failsafe.

```
param set SIM_BAT_DRAIN 900
param set SIM_BAT_MIN_PCT 20
param save
```

Check the topic name (Tab 6) after starting Tab 3:
`ros2 topic list | grep battery`. If it is not `/fmu/out/battery_status_v1`, put the name you
see into `battery_topic` in `src/drone_planning/config/cost_planner.yaml`. If no battery
topic is listed, the planner still works and assumes 100 %.

## Step 3: Check that Gazebo can spawn models (Tab 6, sim running)

```bash
gz service -l | grep -E "/world/building/(create|set_pose)"
```

You should see `/world/building/create`. If nothing is printed, send me a photo, because the
world is missing the UserCommands system.

## Step 4: Test A: the planner flies clicked goals

1. Tab 1: start the sim. Wait for `Ready for takeoff`.
2. Tab 2 (if you use it): the agent.
3. Tab 3: `ros2 launch drone_planning m2c.launch.py`
   Wait for `cost_planner up`, `READY at 1.5 m` and `Managed nodes are active`.
4. RViz: add **Path `/cost_planner/path`** (set the colour to green) and
   **Marker `/cost_planner/blocked`**. Keep the M2b displays. Ctrl+S to save it.
5. Click **2D Goal Pose** about 3 m down the corridor, then in a room.

✅ Pass: the green path appears, the drone follows it, and Tab 3 prints lines like

```
[cost_planner]: NEW GOAL (5.8, 0.2): D=5.9 m  risk=0.41  E=0.61 Wh  K=0.3 rad (0 turns)  J=12.1  battery=96%  85 ms
```

Show the cost terms working (good for the video and the report):

```bash
# Tab 6: watch the four terms of every plan
ros2 topic echo /cost_planner/cost
# make collision risk matter much more: the path moves to the middle of corridors
ros2 param set /cost_planner w_risk 12.0
# make complexity matter much more: fewer turns, longer straight legs
ros2 param set /cost_planner w_complexity 3.0
# back to the defaults
ros2 param set /cost_planner w_risk 4.0 && ros2 param set /cost_planner w_complexity 0.6
```

## Step 5: Test B: moving obstacles and replanning

1. Keep Tabs 1 to 3 running, with the drone hovering.
2. Tab 4: `ros2 launch drone_planning obstacles.launch.py`
   Two orange 2 m tall cylinders appear: `walker_1` paces 8 m along the north corridor
   (y = 6), `walker_2` paces 3 m along the east corridor (x = 6.2).
   Look in Gazebo or RViz (they show up in `/scan` and the costmaps). If one clips a
   wall, change its spawn point in `launch/obstacles.launch.py` (`WALKERS`) or its length
   in `config/obstacles.yaml`, then rebuild.
3. Click a goal at the far end of the north corridor, for example (-5, 6).

✅ Pass when, as a walker comes towards the drone, Tab 3 prints
`REPLAN #1: obstacle on the old path at ...`, a red sphere appears there, the green path
swings round the walker (or goes the other way round the loop), and the drone reaches the
goal without touching it. Repeat 3 times.

## Step 6: Test C: full demo (exploration with walkers)

Tabs 1 to 4 as above, then Tab 5:

```bash
ros2 launch drone_explore explore.launch.py
```

explore_lite picks frontiers, and every goal goes through the cost planner. Leave it running
until the map is complete. Land with
`ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"`.

## Step 7: Freeze M2c (Tab 6)

```bash
mkdir -p ~/slam_drone/maps
ros2 run nav2_map_server map_saver_cli -f ~/slam_drone/maps/m2c_building \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
cd ~/slam_drone
git add src/drone_planning maps
git commit -m "M2c: cost-aware planner (distance, risk, battery, complexity) + moving obstacles + replanning"
git tag m2c
```

Record with Win+Alt+R: RViz (green path, red replan sphere, costmap) next to Gazebo, with
Tab 3 visible for the `REPLAN` lines.

---

## When something breaks

| Symptom | Check |
|---|---|
| bt_navigator: `Action server compute_path_to_pose_cost not available` | `cost_planner` died or started late. Look for its first red line in Tab 3. `ros2 action list \| grep cost` should list it. |
| bt_navigator error about a port such as `error_code_id` or `server_timeout` | Send me a photo. One XML attribute needs to match your Nav2 version. |
| `plan failed: no costmap received yet` | `ros2 topic hz /global_costmap/costmap` should be about 4 Hz. If nothing comes, planner_server isn't active (check the lifecycle line). |
| `plan failed: TF ...` | `ros2 run tf2_ros tf2_echo map base_link` must work (slam_toolbox + rf2o). |
| `plan failed: goal occupied` | The goal is inside a wall or its inflation. Click further from walls. |
| Path hugs walls | Raise `w_risk` (Step 4 shows how). |
| Plans take over 400 ms (shown at the end of each log line) | Set `epsilon: 1.5` or `plan_resolution: 0.15` in `config/cost_planner.yaml`. |
| Walkers don't move | `ros2 topic echo /model/walker_1/cmd_vel` should show values. Check that `walker_bridge` started in Tab 4. |
| Walkers drift into walls | Shorten `lengths` in `config/obstacles.yaml`. |
| A walker leaves a "ghost" wall in the map | slam_toolbox clears it after a few passes. For the recording, start the walkers after the area is mapped. |
| Drone stops in front of a walker and waits | That's the RPP collision check. It goes on once the next plan routes round the walker (within 0.5 s) or the walker passes. |
