# firefighter_perception

Thermal flame perception from the **Melexis MLX90641** 16×12 IR array.

```
MLX90641 (I2C 0x33) ──> scripts/thermal/mlx90641_frames ──> thermal_node ──┬──> /thermal/image       (32FC1)
                              (vendored C reader)                          ├──> /thermal/camera_info
                                                                           └──> /flame_event
                                                                                   ▲
/scan ────────────────────────────────────────────────────────────────────────────┘
```

There is no MLX90641 driver on PyPI (see `docs/thermal-camera.md`), so frames come
from the vendored C tool via `thermal_reader.py`, which runs it in a background
thread and restarts it if it dies. This package needs **no numpy**.

## The array gives a bearing, not a range

A 16×12 array on a fixed-FOV lens tells you *where* the hot spot is, not how far
away it is. Range comes from the LiDAR: `scan_range_at_bearing()` looks up the
`/scan` return at the flame's bearing (median of the returns in a ±6° window, so one
spurious point cannot decide it). That also settles `IN_SUPPRESSION_RANGE`.

## What it publishes, and what it deliberately does not

| State | Meaning |
|---|---|
| `FLAME_FOUND` | a hot spot is present |
| `IN_SUPPRESSION_RANGE` | ...and the LiDAR says it is within `suppression_range_m` |
| `FLAME_LOST` | nothing hot in view |

It never publishes `FLAME_EXTINGUISHED`. Deciding that "gone" means "it went out"
needs to know that you had been suppressing — that is the mission's job, and
`firefighter_mission` maps `FLAME_LOST` → extinguished while suppressing/verifying.

## Topics

| Name | Type | Dir |
|---|---|---|
| `/thermal/image` | `sensor_msgs/Image` (32FC1, degrees C) | out |
| `/thermal/camera_info` | `sensor_msgs/CameraInfo` | out |
| `/flame_event` | `firefighter_interfaces/FlameEvent` | out |
| `/scan` | `sensor_msgs/LaserScan` | in |

## Run

```bash
cd ~/firefighter_ws && ./build.sh && source install/setup.bash
ros2 launch firefighter_perception perception.launch.py
```

`reader_path` defaults to `$FIREFIGHTER_WS/scripts/thermal/mlx90641_frames`
(override with `reader_path:=`). Build the reader first with
`bash scripts/thermal/build.sh`.

## Tuning

`config/perception.yaml`. The thresholds are `min(threshold_c, ambient + delta_c)`,
so a warm room where nothing reaches 60 °C still triggers on a 20 °C rise. Note the
sensor delivers roughly **2 Hz**, not the 5 Hz the acquisition setting suggests —
the MLX90641's subpage reads dominate. Images are published one per *new* frame, not
at the timer rate, so a dead reader cannot keep reporting a stale `FLAME_FOUND`.
