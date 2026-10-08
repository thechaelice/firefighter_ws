# firefighter_bringup

One launch file for the whole robot.

```bash
cd ~/firefighter_ws && ./build.sh && source install/setup.bash
ros2 launch firefighter_bringup bringup.launch.py
```

## What comes up

| Stage | Node | Produces |
|---|---|---|
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
map ──(slam_toolbox)──> odom ──(robot_localization EKF)──> base_link ──(static)──> laser
                                                                    └──(static)──> thermal_camera
```

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
| `laser_{x,y,z,yaw}` | `0, 0, 0.12, 0` | LiDAR mount |
| `thermal_camera_{x,y,z,yaw}` | `0.05, 0, 0.15, 0` | thermal mount |

`FIREFIGHTER_WS` defaults to `~/firefighter_ws`; set it if the workspace moves.
The mount offsets are **placeholders** - measure the robot.

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
