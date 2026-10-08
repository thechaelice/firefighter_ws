"""Thermal perception node: MLX90641 -> thermal image + /flame_event.

The sensor is a 16x12 IR array on the I2C bus (see ``docs/thermal-camera.md``).
There is no MLX90641 driver on PyPI, so frames come from the vendored C tool
``scripts/thermal/mlx90641_frames`` via :mod:`thermal_reader`; this node runs it in
a background thread and keeps only the newest frame.

The array gives a *bearing* to the hot spot but no range, so distance comes from
the LiDAR: the scan return at the flame's bearing. That also decides
IN_SUPPRESSION_RANGE.

Division of labour - this node reports what it *sees*:
    FLAME_FOUND           a hot spot is present
    IN_SUPPRESSION_RANGE  ...and the LiDAR says it is within suppression range
    FLAME_LOST            nothing hot in view
It never reports FLAME_EXTINGUISHED: deciding that "gone" means "it went out" is
the mission's job, since only the mission knows whether it had been suppressing.
"""
from __future__ import annotations

import math
import threading
from array import array
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Header

from firefighter_interfaces.msg import FlameEvent

try:
    from . import flame_detect as fd
    from .thermal_reader import ThermalReader
except ImportError:  # pragma: no cover
    import flame_detect as fd
    from thermal_reader import ThermalReader


class ThermalPerception(Node):
    """Turns MLX90641 frames into a thermal image and flame events."""

    def __init__(self) -> None:
        super().__init__("firefighter_perception")

        # ---- reader ------------------------------------------------------
        self.declare_parameter("reader_path", "")
        self.declare_parameter("delay_ms", 200)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("rows", fd.ROWS)
        self.declare_parameter("cols", fd.COLS)

        # ---- detection ---------------------------------------------------
        self.declare_parameter("fov_h_deg", 55.0)
        self.declare_parameter("threshold_c", 60.0)
        self.declare_parameter("delta_c", 20.0)
        self.declare_parameter("use_frame_mean_ambient", True)
        self.declare_parameter("ambient_c", 25.0)
        self.declare_parameter("suppression_range_m", 1.0)
        self.declare_parameter("stale_after_s", 5.0)

        # ---- topics ------------------------------------------------------
        self.declare_parameter("image_topic", "/thermal/image")
        self.declare_parameter("camera_info_topic", "/thermal/camera_info")
        self.declare_parameter("flame_topic", "/flame_event")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("frame_id", "thermal_camera")

        gp = self.get_parameter
        self._reader_path = gp("reader_path").value
        self._delay_ms = int(gp("delay_ms").value)
        self._rows = int(gp("rows").value)
        self._cols = int(gp("cols").value)
        self._fov_h_deg = float(gp("fov_h_deg").value)
        self._threshold_c = float(gp("threshold_c").value)
        self._delta_c = float(gp("delta_c").value)
        self._use_frame_mean = bool(gp("use_frame_mean_ambient").value)
        self._ambient_c = float(gp("ambient_c").value)
        self._suppression_range_m = float(gp("suppression_range_m").value)
        self._stale_after_s = float(gp("stale_after_s").value)
        self._frame_id = gp("frame_id").value
        publish_rate = max(0.1, float(gp("publish_rate_hz").value))

        self._check_interface_constants()

        # ---- state -------------------------------------------------------
        self._latest_frame = None          # newest thermal frame, or None
        self._latest_scan: Optional[LaserScan] = None
        self._frames_seen = 0
        self._published_frame_count = 0
        self._last_frame_stamp = None
        self._stale_warned = False
        self._reader_errors = 0

        # ---- IO ----------------------------------------------------------
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._image_pub = self.create_publisher(Image, gp("image_topic").value, image_qos)
        self._info_pub = self.create_publisher(
            CameraInfo, gp("camera_info_topic").value, image_qos
        )
        self._flame_pub = self.create_publisher(FlameEvent, gp("flame_topic").value, event_qos)

        self.create_subscription(
            LaserScan, gp("scan_topic").value, self._on_scan, qos_profile_sensor_data
        )

        # ---- reader thread ------------------------------------------------
        self._reader: Optional[ThermalReader] = None
        if not self._reader_path:
            self.get_logger().error(
                "reader_path is empty - no thermal frames will be read. "
                "Pass reader_path:=/path/to/scripts/thermal/mlx90641_frames"
            )
        else:
            self._reader = ThermalReader(
                self._reader_path,
                rows=self._rows,
                cols=self._cols,
                delay_ms=self._delay_ms,
            )
            self._reader_thread = threading.Thread(
                target=self._read_frames, name="mlx90641", daemon=True
            )
            self._reader_thread.start()
            self.get_logger().info(f"reading thermal frames from {self._reader_path}")

        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"perception up | {self._cols}x{self._rows} @ {self._fov_h_deg:.0f} deg fov | "
            f"flame events on {gp('flame_topic').value}"
        )

    # ------------------------------------------------------------------
    def _check_interface_constants(self) -> None:
        """Fail loudly if FlameEvent.msg was renumbered under us."""
        expected = (
            FlameEvent.FLAME_FOUND,
            FlameEvent.FLAME_LOST,
            FlameEvent.FLAME_EXTINGUISHED,
            FlameEvent.IN_SUPPRESSION_RANGE,
        )
        if expected != (fd.STATE_FOUND, fd.STATE_LOST, fd.STATE_EXTINGUISHED, fd.STATE_IN_RANGE):
            self.get_logger().error(
                "FlameEvent constants changed - update flame_detect.py to match"
            )

    # ------------------------------------------------------------------
    def _read_frames(self) -> None:
        """Background thread: keep only the newest frame."""
        if self._reader is None:
            return
        try:
            for frame in self._reader.frames():
                self._latest_frame = frame
                self._frames_seen += 1
        except Exception as exc:  # noqa: BLE001 - the thread must never die silently
            self._reader_errors += 1
            self.get_logger().error(f"thermal reader stopped: {exc}")

    def _check_reader_staleness(self) -> None:
        """Complain once when the reader produces no frames (or never did)."""
        if self._reader is None:
            return                      # already reported: reader_path is empty
        if self._last_frame_stamp is None:
            self._last_frame_stamp = self.get_clock().now()
            return
        silent = (self.get_clock().now() - self._last_frame_stamp).nanoseconds * 1e-9
        if silent > self._stale_after_s and not self._stale_warned:
            self._stale_warned = True
            reason = self._reader.last_error or "the reader is running but silent"
            self.get_logger().error(
                f"no thermal frames for {silent:.1f}s ({reason}; "
                f"{self._reader.restarts} restarts) - no flame events are being "
                "published. Is the sensor connected and the reader built "
                "(bash scripts/thermal/build.sh)?"
            )

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        frame = self._latest_frame
        if frame is None:
            self._check_reader_staleness()   # a reader that never delivers must not be silent
            return

        # Publish only on a NEW frame. Re-publishing the last one at the timer
        # rate would keep reporting a stale FLAME_FOUND after the reader dies.
        if self._frames_seen == self._published_frame_count:
            self._check_reader_staleness()
            return
        self._published_frame_count = self._frames_seen
        self._last_frame_stamp = self.get_clock().now()
        self._stale_warned = False

        try:
            detection = fd.analyse_frame(
                frame,
                ambient_c=None if self._use_frame_mean else self._ambient_c,
                threshold_c=self._threshold_c,
                delta_c=self._delta_c,
                fov_h_deg=self._fov_h_deg,
            )
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(f"bad thermal frame: {exc}")
            return

        now = self.get_clock().now()
        self._publish_image(frame, now)
        self._publish_flame(detection, now)

    # ------------------------------------------------------------------
    def _publish_image(self, frame, now) -> None:
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self._frame_id

        flat = array("f", [float(value) for row in frame for value in row])

        image = Image()
        image.header = header
        image.height = len(frame)
        image.width = len(frame[0])
        image.encoding = "32FC1"
        image.is_bigendian = 0          # Pi and ESP32 are little-endian
        image.step = image.width * 4
        image.data = flat.tobytes()
        self._image_pub.publish(image)
        self._info_pub.publish(self._camera_info(header, image.width, image.height))

    def _camera_info(self, header: Header, width: int, height: int) -> CameraInfo:
        fx = (width / 2.0) / math.tan(math.radians(self._fov_h_deg) / 2.0)
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0

        info = CameraInfo()
        info.header = header
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish_flame(self, detection: fd.FlameDetection, now) -> None:
        distance = self._lidar_distance(detection.bearing_rad)

        if not detection.present:
            state = fd.STATE_LOST
        elif distance is not None and distance <= self._suppression_range_m:
            state = fd.STATE_IN_RANGE
        else:
            state = fd.STATE_FOUND

        msg = FlameEvent()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        msg.state = state
        msg.bearing_rad = float(detection.bearing_rad)
        msg.distance_m = float("nan") if distance is None else float(distance)
        self._flame_pub.publish(msg)

    def _lidar_distance(self, bearing_rad: float) -> Optional[float]:
        scan = self._latest_scan
        if scan is None or not scan.ranges:
            return None
        try:
            return fd.scan_range_at_bearing(
                scan.angle_min,
                scan.angle_increment,
                scan.ranges,
                bearing_rad,
                max_range=float(scan.range_max) if scan.range_max > 0.0 else 12.0,
            )
        except (ValueError, TypeError, ZeroDivisionError):
            return None

    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        if self._reader is not None:
            self._reader.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThermalPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
