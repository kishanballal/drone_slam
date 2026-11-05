# M2b: Autonomous exploration (step-by-step)

**Goal (demo):** start the stack, press "start", and the drone maps the whole building by itself, with GPS off.

**When to start:** after M2a is working and frozen (git tag `m2a`). M2b sits on top of the M2a stack and changes nothing in it.

**What you get (all files are in `drone_explore/`, next to this guide):**

| File | What it does |
|---|---|
| `drone_explore/cmd_vel_bridge.py` | **Kishan's node.** The only thing that talks offboard to PX4. It arms, takes off to 1.5 m, then turns Nav2's `/cmd_vel` (body FLU) into NED velocity setpoints with altitude held at 1.5 m. It also accepts `/drone/position_target` for plan B, and lands on `/drone/land`. |
| `drone_explore/frontier_explorer.py` | **Plan B** (no Nav2). Finds the nearest reachable frontier on `/map`, plans a BFS path, and sends position targets. |
| `drone_explore/core.py` | Frame maths and grid search. No ROS, so it is unit tested. |
| `config/nav2_params.yaml` | Nav2 tuned for a drone: 0.4 m radius, 0.5 m/s, scan plane at about 1.76 m. |
| `config/explore_params.yaml`, `bridge_params.yaml`, `frontier_params.yaml` | Parameters. |
| `launch/m2b.launch.py` | The M2b launch file: M2a stack + bridge + Nav2. |
| `launch/explore.launch.py` | The "start button": explore_lite (plan A) or frontier_explorer (plan B). |
| `test/test_core.py` | Unit tests (frame conversions, frontier search). |

Already checked: all Python compiles, the 5 unit tests pass, and I ran plan B's algorithm in a simple 2D simulation of `building.sdf` (8 m LiDAR, 0.57 m/s). It covered 99.6% of the floor in about 104 s and came home, and it never got closer than 0.42 m to a wall. That run had no Gazebo, PX4 or Nav2 in it, so real flights will differ.

---

## Terminal tabs for M2b

| Tab | Runs |
|---|---|
| **Tab 1** | Simulation with the M2a GPS-off airframe: `PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev` (use the exact Tab 1 command from the M2a guide) |
| **Tab 2** | `MicroXRCEAgent udp4 -p 8888` |
| **Tab 3** | `ros2 launch drone_explore m2b.launch.py` (M2a stack + bridge + Nav2) |
| **Tab 4** | `ros2 launch drone_explore explore.launch.py` (the start button) |
| **Tab 5** | One-off commands and checks |

---

## Step 1: Install Nav2 (any tab, once)

```bash
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

Check: `ros2 pkg list | grep -c nav2` should print a number above 20.

## Step 2: Build explore_lite from source (any tab, once)

```bash
cd ~/slam_drone/src
git clone https://github.com/robo-friends/m-explore-ros2.git
touch m-explore-ros2/map_merge/COLCON_IGNORE      # multi-robot part, not needed
cd ~/slam_drone
rosdep install --from-paths src/m-explore-ros2 --ignore-src -r -y
colcon build --symlink-install --packages-select explore_lite
source install/setup.bash
ros2 pkg executables explore_lite                  # expect: explore_lite explore
```

Deprecation warnings about `tf2_geometry_msgs.h` are normal. If the build fails with an **error**, send me a photo of the first red error line.

## Step 3: Copy the package into the workspace (any tab)

Download the `drone_explore` folder from the project files to Windows `Downloads`, then:

```bash
cp -r /mnt/c/Users/kisha/Downloads/drone_explore ~/slam_drone/src/
ls ~/slam_drone/src/drone_explore     # package.xml setup.py setup.cfg drone_explore launch config resource test
```

## Step 4: Check M2a's waypoint switch (nothing to edit)

M2a's `m2a.launch.py` already has a `waypoints` argument (default `false`), and `m2b.launch.py` passes `waypoints:=false`. So the M2a waypoint node never runs in M2b and can't fight the bridge for control. **Don't edit m2a.launch.py.** Just check it: `ros2 launch slam_drone m2a.launch.py` should bring up everything without starting `waypoint_flight`.

**Also in the slam_toolbox params (M1 config):** set `map_update_interval: 1.0`. The default of 5 s makes exploration sluggish. This only changes how often the map is published, so M1/M2a still work.

## Step 5: Build and run the unit tests (any tab)

```bash
cd ~/slam_drone
colcon build --symlink-install --packages-select drone_explore
source install/setup.bash
python3 -m pytest -q src/drone_explore/test        # expect: 5 passed
```

Make sure `source ~/slam_drone/install/setup.bash` is in `~/.bashrc` (after the `ws_px4` line) so every tab sees the package.

---

## Step 6: Test A: bridge only (no Nav2, no exploring)

`mode:=fallback` starts M2a + bridge without Nav2, which makes it a clean bridge test.

- **Tab 1:** start the sim. Wait for "Ready for takeoff!".
- **Tab 2:** `MicroXRCEAgent udp4 -p 8888`
- **Tab 3:**
  ```bash
  ros2 launch drone_explore m2b.launch.py mode:=fallback
  ```
  You should see `state -> ARMING`, then `Requesting offboard + arm`, then `state -> TAKEOFF`, then `READY at 1.5 m`. The drone climbs to 1.5 m and hovers.
- **Tab 5:** fly it by hand through the same path Nav2 will use:
  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
  ```
  The drone should fly **forward** (the direction the drone's nose points, the +x arrow of base_link in RViz). Press Ctrl+C after about 3 s and it should stop and hover in place within 0.5 s.
  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
  ```
  The drone should **turn left** (counter-clockwise seen from above). Ctrl+C to stop.
  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"
  ```
  The drone should slide **left**. Ctrl+C.
- **Land:** `ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"`

✅ Pass when forward, left and turn-left all match, and the altitude stays at 1.5 m ± 0.15. If any direction is wrong, stop and send me a photo. Don't continue to Nav2 until this passes.

## Step 7: Test B: Nav2 flies to a goal you click

- Tabs 1 and 2 as before.
- **Tab 3:** `ros2 launch drone_explore m2b.launch.py` (Nav2 starts 8 s after launch)
  Wait for `lifecycle_manager_navigation: Managed nodes are active` and `READY at 1.5 m`.
- **RViz:** set Fixed Frame to `map` and add these displays (then **Ctrl+S**, saving as `~/slam_drone/src/drone_explore/config/m2b.rviz`):
  - Map `/map`
  - Map `/global_costmap/costmap` (Color Scheme: costmap)
  - Map `/local_costmap/costmap` (Color Scheme: costmap)
  - Path `/plan`
  - MarkerArray `/explore/frontiers`
  - Path `/frontier/path` and Marker `/frontier/goal` (for plan B)
- Click **2D Goal Pose** in the RViz toolbar, then click about 3 m down the corridor. You should see a green `/plan` line, and the drone turns to face along it and flies there at about 0.5 m/s.
- Next, click a goal in the SE room. The drone should go through the doorway at x ≈ 5.

✅ Pass when it reaches 3 goals in a row without hitting anything.

## Step 8: Test C: full autonomous exploration (the demo)

- Tabs 1–3 as in Test B. Wait for `READY` and `Managed nodes are active`.
- **Tab 4 (start button):**
  ```bash
  ros2 launch drone_explore explore.launch.py
  ```
  Frontiers appear as markers. The drone picks one, flies there, and repeats. When everything is mapped, explore_lite logs that there are no frontiers left (wording varies), and the drone hovers.
- **Tab 5:** land with `ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"`

## Step 9: Freeze M2b

```bash
# Tab 5, before landing: save the map
mkdir -p ~/slam_drone/maps
ros2 run nav2_map_server map_saver_cli -f ~/slam_drone/maps/m2b_building --ros-args -p use_sim_time:=true

cd ~/slam_drone
echo "src/m-explore-ros2/" >> .gitignore
git -C src/m-explore-ros2 rev-parse HEAD > src/drone_explore/EXPLORE_LITE_COMMIT
git add .gitignore src/drone_explore maps src/slam_drone
git commit -m "M2b: autonomous exploration (Nav2 + explore_lite + cmd_vel bridge)"
git tag m2b
```

Record the run with Win+Alt+R, showing RViz with the map growing and frontiers disappearing, next to Gazebo.

---

## Plan B: custom frontier node (no Nav2)

**When to switch:** if Test B isn't working by **Thu 12 Nov evening**. The code is already written. The only extra step is running it:

- **Tab 3:** `ros2 launch drone_explore m2b.launch.py mode:=fallback`
- **Tab 4:** `ros2 launch drone_explore explore.launch.py mode:=fallback`

It logs `New frontier goal (x, y) ...` for each target, shows the path on `/frontier/path`, and when finished says `exploration complete, returning home` and hovers at the start point. The parameters are in `config/frontier_params.yaml`. `max_target_step` in `bridge_params.yaml` sets the speed (about 0.95 × the value, in m/s).

---

## When something breaks

| Symptom | Likely cause and check |
|---|---|
| Bridge keeps printing `Waiting for PX4 data` | The XRCE agent isn't running, or the topic names differ. Run `ros2 topic list \| grep fmu` and fix the names in `bridge_params.yaml`. |
| Bridge prints `cmd_vel bridge up` and then nothing | No `/clock`, so sim-time timers never fire. Run `ros2 topic hz /clock`; the M2a stack must bridge it. |
| `Requesting offboard + arm` repeats forever | Look at Tab 1 (pxh>) for the reason. "No valid position" means EKF2 isn't using the rf2o odometry (an M2a problem). Check with `listener vehicle_local_position` in pxh. |
| `state -> MANUAL` | PX4 left offboard (you typed a commander mode, or a failsafe). Restart Tab 3. |
| Wrong direction in Test A | Frame bug: send me a photo of Tab 3 plus `ros2 topic echo /fmu/out/vehicle_local_position_v1 --field heading --once`. |
| Nav2: `Timed out waiting for transform from base_link to map` | slam_toolbox or rf2o isn't running. Run `ros2 run tf2_ros tf2_echo map base_link`. |
| Lifecycle manager: `Failed to bring up all requested nodes` | Scroll up to the **first** red error in Tab 3 and send me a photo. |
| Costmap shows the map but no live obstacles | The scan is filtered out by height. Check `max_obstacle_height: 3.0` is in the params. |
| `Failed to create plan` through a doorway | The doorways are 1.2 m and the drone is 0.8 m wide. Try `robot_radius: 0.35` and `inflation_radius: 0.45`. If it still fails, see the risk below. |
| Drone wobbles or overshoots on Nav2 paths | Lower `desired_linear_vel` to 0.35 and `rotate_to_heading_angular_vel` to 0.6. |
| explore_lite: `Waiting for costmap to become available` | `/map` is missing. Run `ros2 topic info /map -v` (needs the slam_toolbox publisher). |
| explore_lite stops straight away | The map isn't growing fast enough (set `map_update_interval: 1.0`), or `min_frontier_size` is too big. |
| Drone slowly drifts while hovering in a long corridor | rf2o loses features along long, featureless corridors. This is an M2a tuning issue, so fix it there. |

## Deadline risks (flagging now)

1. **M2a and M2b share the same week (9–15 Nov).** If M2a slips, M2b gets squeezed. Time-box Nav2 (Test B) to 12 Nov. Plan B is ready and needs no Nav2.
2. **Doorways are tight.** 1.2 m doors and a 0.8 m drone leave 0.2 m each side. Nav2 should manage, but rf2o drift can make the gap look closed. The cheapest fix is for Yathivandith to widen both doorways in `building.sdf` to 1.6 m. That changes the world, so re-run the M1/M2a demos once afterwards.
3. **explore_lite on Jazzy is built from source.** If it won't compile within an hour, go straight to plan B. It runs the same demo, and you can say in the report that the frontier node is your own.
