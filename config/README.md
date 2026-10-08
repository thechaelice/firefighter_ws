# config/

Parameter files for **upstream packages whose source we don't own**. Keeping them
here rather than inside `src/` means a workspace rebuild can never overwrite tuned
values, and `firefighter_bringup` loads them by absolute path.

| File | Package | Status |
|---|---|---|
| `rf2o.yaml` | `rf2o_laser_odometry` | adapted — `publish_tf: false` (the EKF owns `odom → base_link`) |
| `ekf.yaml` | `robot_localization` | purpose-written — fuses `/odom_rf2o` + `/wheel_odom` |
| `slam_toolbox.yaml` | `slam_toolbox` | stock defaults, `base_frame` → `base_link`. **Not tuned** |
| `nav2_params.yaml` | `nav2_bringup` | stock defaults, frames/odom/radius adapted. **Not tuned** |

## Provenance

`slam_toolbox.yaml` and `nav2_params.yaml` are copies of the shipped Jazzy configs:

* `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`
* `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`

Only frames, the odometry source and the robot radius were changed; each file has a
header comment listing exactly what differs. **Nothing has been validated on real
hardware** — a stock Nav2 config is a starting point, not a tuned one. Speeds,
acceleration limits, inflation radii and the progress checker all need revisiting
once the robot drives. `robot_radius: 0.18` in particular is a placeholder.

`ekf.yaml` and `rf2o.yaml` were written for this robot rather than copied.

Package-owned parameters live with their package (e.g.
`src/firefighter_bridge/config/bridge.yaml`).
