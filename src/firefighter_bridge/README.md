# firefighter_bridge

UART bridge between the **mobility ESP32** and the ROS 2 graph.

```
/cmd_vel  --SET_TWIST-->  [ESP32]  --WHEEL_ODOM-->  /wheel_odom  (nav_msgs/Odometry)
/beacon_event  <--BEACON_EVENT-- [ESP32] <--ESP-NOW-- beacon
                                   --STATUS-->      /battery, /diagnostics
```

The wire format is defined once in `firefighter_protocol.h` (ESP32) and
`firefighter_bridge/protocol.py` (Pi). The golden-frame tests in
`tests/test_protocol_codec.py` keep the two in lock-step.

## Run

```bash
cd ~/firefighter_ws && ./build.sh          # or: colcon build --packages-select firefighter_interfaces firefighter_bridge
source install/setup.bash
ros2 launch firefighter_bridge bridge.launch.py port:=/dev/ttyAMA0
```

See `docs/runbook.md` for enabling the PL011 UART on GPIO14/15.

## Topics & services

| Name | Type | Dir | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | in | streamed as `SET_TWIST` at `cmd_rate_hz` |
| `/wheel_odom` | `nav_msgs/Odometry` | out | integrated from encoder ticks |
| `/beacon_event` | `firefighter_interfaces/BeaconEvent` | out | forwarded beacon alerts |
| `/battery` | `sensor_msgs/BatteryState` | out | voltage is `NaN` until a battery ADC exists |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | out | ESP32 mode / flags / link health |
| `~/estop` | `std_srvs/Trigger` | srv | latches `ESTOP` |
| `~/clear_fault` | `std_srvs/Trigger` | srv | sends `RESET_FAULT` |

## Why `/wheel_odom` and not `/odom`

`rf2o_laser_odometry` already broadcasts `odom -> base_link`. A second publisher
of the same transform makes the TF tree ambiguous, so the bridge publishes to
`/wheel_odom` and leaves `publish_tf` off. A `robot_localization` EKF can later
fuse `/odom_rf2o` + `/wheel_odom` and take over the TF.

## Safety

* The node streams `SET_TWIST` every cycle (zero when idle), which keeps the
  ESP32 watchdog fed. If the node dies, the frames stop and the ESP32 stops
  the motors itself after `PI_TIMEOUT_MS` (500 ms).
* `/cmd_vel` older than `cmd_timeout_s` is treated as zero.
