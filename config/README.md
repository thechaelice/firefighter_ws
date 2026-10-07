# config/

Parameter files for **upstream packages whose source we don't own**. Keeping them
here rather than inside `src/` means a workspace rebuild can never overwrite tuned
values, and each is launched explicitly with `--params-file`.

| File | Package | Purpose | Status |
|---|---|---|---|
| `slam_toolbox.yaml` | `slam_toolbox` | online async SLAM | not created yet |
| `nav2_params.yaml` | `nav2_bringup` | planner / controller / costmaps | not created yet |
| `ekf.yaml` | `robot_localization` | fuse `/odom_rf2o` + `/wheel_odom`, own the TF | not created yet |
| `rf2o.yaml` | `rf2o_laser_odometry` | laser odometry tuning | not created yet |

Deliberately empty for now — a guessed Nav2/SLAM config is worse than none. They
get written once the robot is actually driven and the values can be tuned.

Package-owned parameters (e.g. `src/firefighter_bridge/config/bridge.yaml`) stay
in their package, and the bridge launch file can take a `params_file` override.
