"""Simulated stand-in for ``scripts/thermal/mlx90641_frames``.

The perception node reads thermal frames from a child process that prints a
12x16 block of comma-separated temperatures (deg C) per frame, blank-line
separated. On the robot that process is the vendored MLX90641 C tool. In
simulation it is this script: it subscribes to the Gazebo thermal camera
(bridged to ROS as a 16-bit image) and prints the same format, so the node, the
flame detector and the mission run exactly as they do on hardware.

Called like the real tool - ``sim_thermal_frames <frames> <delay_ms>`` - but both
arguments are ignored: Gazebo sets the frame rate (``update_rate`` in
``firefighter.gazebo.xacro``) and the stream runs until the process is killed.

Only CSV goes to stdout; ROS logging goes to stderr, which the node discards.
"""
from __future__ import annotations

import sys
from array import array

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

ROWS = 12
COLS = 16

#: Kelvin per count of the Gazebo thermal image. MUST MATCH <resolution> of the
#: ThermalSensor plugin in firefighter_description/urdf/firefighter.gazebo.xacro.
KELVIN_PER_COUNT = 0.01
KELVIN_OFFSET = 273.15


def image_to_csv(msg: Image) -> str:
    """Convert a 16-bit thermal image to the reader's CSV block (deg C)."""
    if (msg.height, msg.width) != (ROWS, COLS):
        raise ValueError(f"expected a {COLS}x{ROWS} image, got {msg.width}x{msg.height}")
    if msg.encoding not in ("mono16", "16UC1"):
        raise ValueError(f"expected a 16-bit image, got encoding {msg.encoding!r}")

    counts = array("H")
    counts.frombytes(bytes(msg.data))
    if bool(msg.is_bigendian) != (sys.byteorder == "big"):
        counts.byteswap()

    words_per_row = msg.step // 2
    lines = []
    for r in range(ROWS):
        row = counts[r * words_per_row:r * words_per_row + COLS]
        lines.append(",".join(f"{c * KELVIN_PER_COUNT - KELVIN_OFFSET:.2f}" for c in row))
    return "\n".join(lines)


class SimThermalFrames(Node):
    def __init__(self) -> None:
        super().__init__("sim_thermal_frames")
        self.declare_parameter("raw_topic", "/thermal/raw")
        topic = self.get_parameter("raw_topic").value
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f"printing thermal frames from {topic}")

    def _on_image(self, msg: Image) -> None:
        try:
            block = image_to_csv(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        sys.stdout.write(block + "\n\n")
        sys.stdout.flush()


def main() -> None:
    # argv carries the real tool's <frames> <delay_ms>; neither is a ROS argument.
    rclpy.init(args=None)
    node = SimThermalFrames()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
