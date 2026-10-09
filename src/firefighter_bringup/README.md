# firefighter_bringup

One launch file for the whole robot.

```bash
cd ~/firefighter_ws && ./build.sh && source install/setup.bash
ros2 launch firefighter_bringup bringup.launch.py
```

## What comes up

| Stage | Node | Produces |
|---|---|---|
| model | `firefighter_description` (`robot_state_publisher`) | `/robot_description`, **`base_link → laser`**, **`base_link → thermal_camera`** |
| sensing | `sllidar_ros2` | `/scan` |
| odometry | `rf2o_laser_odometry` | `/odom_rf2o` |
| odometry | `robot_localization` `ekf_filter_node` | `/odometry/filtered`, **`odom → base_link`** |
| robot | `firefighter_bridge` | `/wheel_odom`, `/beacon_event`, `/battery`, `/diagnostics` |
| robot | `firefighter_perception` | `/thermal/image`, `/flame_event` |
| autonomy | `slam_toolbox` | **`map → odom`** |
| autonomy | `nav2_bringup` | `navigate_to_pose` action |
| autonomy | `firefighter_mission` | mission FSM |
| operator | `rviz2` | `rviz/firefighter.rviz` |

## TF ownership (REP-105)

Exactly one node publishes each transform - this is the thing that most often
goes wrong:

```
map ──(slam_toolbox)──> odom ──(robot_localization EKF)──> base_link ──(robot_state_publisher)──> laser
                                                                    └──(robot_state_publisher)──> thermal_camera
```

* The sensor mounts come from the URDF in `firefighter_description`
  (`urdf/firefighter.urdf.xacro`, exported from `model/firefighter.blend`). To move
  a sensor, change the model and re-export - there are no mount launch arguments.

* `rf2o` has **`publish_tf: false`** (`config/rf2o.yaml`). It feeds `odom → base_link`
  into the EKF, it does not broadcast it.
* The bridge publishes `/wheel_odom` with **`publish_tf: false`** for the same reason.
* The EKF fuses `/odom_rf2o` (pose) with `/wheel_odom` (velocity) and owns the
  `odom → base_link` broadcast.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | LiDAR |
| `bridge_port` | `/dev/ttyAMA0` | UART to the mobility ESP32 |
| `config_dir` | `$FIREFIGHTER_WS/config` | upstream param files |
| `slam` | `true` | `true` = SLAM Toolbox; `false` = AMCL on `map` |
| `map` | `$FIREFIGHTER_WS/maps/map.yaml` | used when `slam:=false` |
| `nav2_enabled` | `true` | set `false` to bring up mapping only |
| `rviz` | `true` | |
| `startup_delay` | `3.0` | wait before SLAM/Nav2 so TF and `/scan` exist |
| `thermal_reader_path` | `$FIREFIGHTER_WS/scripts/thermal/mlx90641_frames` | MLX90641 reader |
| `hardware` | `true` | `false` skips the LiDAR and ESP32 bridge (simulation) |
| `model` | `firefighter_description/urdf/firefighter.urdf.xacro` | XACRO published on `/robot_description` |

`FIREFIGHTER_WS` defaults to `~/firefighter_ws`; set it if the workspace moves.
The sensor positions in the model are **estimated from photos** - measure the robot.

## Simulation (Gazebo Harmonic, WSL2)

`sim.launch.py` starts Gazebo with `firefighter_description/worlds/firehouse.sdf`
(three rooms, obstacles, a fire in the far room), spawns the robot and runs this
same bringup with `hardware:=false`. Gazebo publishes what the hardware would:
`/scan`, `/wheel_odom` and `/joint_states`, and drives the wheels from `/cmd_vel`.

One-off setup in WSL2 Ubuntu 24.04 with ROS 2 Jazzy (`ros-jazzy-desktop`,
`ros-jazzy-ros-gz`), with the workspace cloned to `~/firefighter_ws`:

```bash
cd ~/firefighter_ws
rosdep install --from-paths src --ignore-src -r -y   # slam_toolbox, nav2, robot_localization, ...
JOBS=4 ./build.sh && source install/setup.bash
```

```bash
ros2 launch firefighter_bringup sim.launch.py                      # Gazebo + SLAM + Nav2 + RViz
ros2 launch firefighter_bringup sim.launch.py nav2_enabled:=false  # mapping only
ros2 run teleop_twist_keyboard teleop_twist_keyboard               # drive it (second terminal)
```

To look at the model alone in RViz, without Gazebo:

```bash
ros2 launch firefighter_description description.launch.py
rviz2 -d $(ros2 pkg prefix firefighter_bringup)/share/firefighter_bringup/rviz/firefighter.rviz
```

The thermal camera is simulated too: a 16×12 Gazebo thermal sensor publishes
`/thermal/raw`, and `firefighter_perception`'s `sim_thermal_frames` feeds it to the
unmodified perception node in place of the MLX90641 reader. The fire in the east
room reads about 326 °C against a 15 °C background, so `/flame_event` fires when
the robot faces it.

## Mapping then localizing

```bash
# 1. build a map (drive around with teleop; Nav2 holds the map frame)
ros2 launch firefighter_bringup bringup.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/firefighter_ws/maps/map

# 2. later, localize against it instead of mapping
ros2 launch firefighter_bringup bringup.launch.py slam:=false
```

## Caveats

`config/slam_toolbox.yaml` and `config/nav2_params.yaml` are the stock Jazzy
defaults with only frames, odom source and robot radius adapted. **Nothing is
tuned** - expect to revisit speeds, accelerations, inflation and the robot radius
once the robot actually drives.
