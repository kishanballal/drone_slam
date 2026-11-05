# M1 step 5: waypoint offboard flight (Kishan)

Goal: the drone takes off by itself, flies the corridor loop at 1.5 m and lands, with no `commander` typing.

What the node does: streams setpoints at 10 Hz, switches PX4 to OFFBOARD, arms, climbs to 1.5 m at 0.5 m/s,
flies (0,0) → (6.2,0) → (6.2,6) → (−6.2,6) → (−6.2,0) → (0,0) (ENU) at 0.5 m/s, hovers 3 s, lands and exits.
The whole flight takes about 85 s. If PX4 or you switch it out of OFFBOARD mid-flight, the node stops and leaves the drone to PX4.

Topic names (checked against PX4 v1.17 source): it publishes `/fmu/in/offboard_control_mode`,
`/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` (no suffix) and listens to
`/fmu/out/vehicle_status_v1` and `/fmu/out/vehicle_local_position_v1`. Step 3 confirms this on your machine.

---

## Part A: test it on its own (today)

### Step 1, Tab 1 (simulation)
```bash
pkill -f "gz sim"; pkill -f px4
cd ~/PX4-Autopilot
PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d
```
Wait for `Ready for takeoff!`. Do **not** type `commander takeoff`; the node does that.

### Step 2, Tab 2 (PX4 ↔ ROS 2 agent, only for this standalone test)
```bash
MicroXRCEAgent udp4 -p 8888
```
Wait until it prints a list of `create_topic` / `create_datawriter` lines.

### Step 3, Tab 3 (checks): check the topic names
```bash
ros2 topic list | grep -E "offboard_control_mode|trajectory_setpoint|vehicle_command$|vehicle_status|vehicle_local_position"
```
You should see exactly these five (send me a photo):
```
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v1
```
If a name differs, you don't have to edit the file. Pass it on the command line, for example
`--ros-args -p topic_status:=/fmu/out/vehicle_status_v2`.

### Step 4, Tab 3: get the file
Download `waypoint_flight.py` from this thread (it lands in Windows Downloads), then:
```bash
mkdir -p ~/offboard_test
cp /mnt/c/Users/kisha/Downloads/waypoint_flight.py ~/offboard_test/
ls -l ~/offboard_test
```

### Step 5, Tab 3: fly
This test only checks the flight, so you don't need RViz or the bridges. Watch the drone in Gazebo. Then:
```bash
python3 ~/offboard_test/waypoint_flight.py
```
Expected output, in order:
```
waypoint_flight ready: 6 waypoints at 1.5 m, 0.50 m/s ...
phase WAIT_PX4 -> ENGAGE
requesting OFFBOARD + ARM (...)
OFFBOARD + ARMED, taking off
reached take-off altitude ...
reached waypoint 1/6 ... up to 6/6
loop complete, landing
landed and disarmed. Mission complete.
```
The node exits on its own. Ctrl+C at any time stops it; PX4 then goes into its offboard-loss failsafe
(hover/land). In Tab 1 you can always type `commander land`.

Send me a photo of Tab 3 at the end (or of wherever it got stuck), plus Tab 1 if PX4 printed red/yellow lines.

**If it hangs at "waiting for PX4"**: the agent in Tab 2 isn't running, or a topic name differs (Step 3).
**If it repeats "requesting OFFBOARD + ARM"**: read the reason in Tab 1 (`pxh>`) and photograph it.

---

## Part B: move it into the M1 package (after the "SLAM mapping workspace" thread has built ~/slam_drone)

### Step 6, Tab 3: copy into the package and build
```bash
cp ~/offboard_test/waypoint_flight.py ~/slam_drone/src/slam_drone/slam_drone/   # overwrites the placeholder
grep waypoint_flight ~/slam_drone/src/slam_drone/setup.py
cd ~/slam_drone && colcon build --symlink-install && source install/setup.bash
```
The `grep` must show `'waypoint_flight = slam_drone.waypoint_flight:main',`. If it shows nothing, add that line
inside `console_scripts` in `setup.py` and rebuild.

### Step 7: run it as a ROS 2 executable (still standalone, agent still in Tab 2)
```bash
ros2 run slam_drone waypoint_flight
```

### Step 8: full M1 demo (follow M1_guide.md from the SLAM workspace thread from here on)
Stop the agent in Tab 2 with Ctrl+C first. `m1.launch.py` starts its own MicroXRCEAgent, the bridges (including /clock) and the TFs,
and two agents on port 8888 conflict. With Tab 1 running, in **Tab 2**:
```bash
cd ~/slam_drone && source install/setup.bash
ros2 launch slam_drone m1.launch.py fly:=true
```
The waypoint node starts 8 s after launch on sim time, and the map draws itself in RViz while the drone flies the loop. Record with Win+Alt+R.

---

## Tuning (only if needed)
```bash
python3 ~/offboard_test/waypoint_flight.py --ros-args -p cruise_speed:=0.3 -p altitude:=1.5
```
| Parameter | Default | Meaning |
|---|---|---|
| `altitude` | 1.5 | m above take-off point |
| `cruise_speed` | 0.5 | m/s along each leg (slower = cleaner map) |
| `climb_speed` | 0.5 | m/s on the take-off climb |
| `acceptance_radius` | 0.3 | m, when a waypoint counts as reached |
| `hold_time` | 3.0 | s hover before landing |
| `waypoints_enu` | the loop | flat list `[x0, y0, x1, y1, ...]` in Gazebo ENU |
| `align_to_world` | true | rotate the loop by PX4's heading offset from the Gazebo world (fixes the ~6° tilt) |
