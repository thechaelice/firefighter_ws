# firefighter_mission

Mission behaviour state machine: **beacon alert → navigate → search → suppress → verify**.

The sequencing logic is pure Python in `mission_fsm.py` (no ROS), so it is unit
tested without a ROS install (`tests/test_mission_fsm.py`). `mission_node.py` is a
thin adapter that maps ROS messages to FSM events and executes the returned actions.

## States

```
IDLE ──alert──> CONFIRMING ──2nd alert──> NAVIGATING ──reached──> SEARCHING
                    │                          │                      │
              timeout -> IDLE             abort/timeout          FLAME_FOUND
                                               │                      v
                                               │                APPROACHING
                                               │                      │ IN_RANGE
                                               │                      v
                                               │   FLAME_STILL_PRESENT  SUPPRESSING
                                               │        ┌──────── VERIFYING <──┐
                                               │        │ (max 3 attempts)     │
                                               └──────> ABORTED <──┘        COMPLETE -> IDLE
```

`ESTOP` from any state drops straight to `ABORTED`; only `RESET` leaves it.

## Topics

| Name | Type | Dir | Notes |
|---|---|---|---|
| `/beacon_event` | `BeaconEvent` | in | alert only when `fire && smoke` |
| `/flame_event` | `FlameEvent` | in | from `firefighter_perception` (later) |
| `~/event` | `String` | in | **manual event injection** for bring-up |
| `/goal_pose` | `PoseStamped` | out | navigation target |
| `/cmd_vel` | `Twist` | out | search spin + stop (feeds the bridge) |
| `~/suppress` | `Bool` | out | extinguisher on/off |
| `~/state` | `String` | out | current state, on change |
| `/diagnostics` | `DiagnosticArray` | out | state, attempts, alerting MAC |
| `~/estop`, `~/reset` | `Trigger` | srv | drive the FSM directly |

## Run

```bash
cd ~/firefighter_ws && ./build.sh && source install/setup.bash
ros2 launch firefighter_mission mission.launch.py
```

## Bring-up without Nav2 or a thermal camera

Navigation is delegated: the node publishes `/goal_pose` and waits for the
`NAV_GOAL_REACHED` event. Until Nav2 and perception exist, drive the whole mission
by hand:

```bash
ros2 topic pub --once /firefighter_mission/event std_msgs/String "{data: nav_goal_reached}"
ros2 topic pub --once /firefighter_mission/event std_msgs/String "{data: flame_found}"
ros2 topic pub --once /firefighter_mission/event std_msgs/String "{data: in_range}"
ros2 topic pub --once /firefighter_mission/event std_msgs/String "{data: flame_extinguished}"
ros2 topic echo /firefighter_mission/state
```

Valid event names are exactly the `Event` enum values in `mission_fsm.py`.

When Nav2 lands, replace the `/goal_pose` publish with a `nav2_msgs`
`NavigateToPose` action client and feed arrival/abort back as the same events — the
FSM itself does not change.
