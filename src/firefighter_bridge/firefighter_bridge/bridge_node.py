"""ROS 2 bridge between the mobility ESP32 (UART) and the ROS graph.

Subscriptions / services
    /cmd_vel                (geometry_msgs/Twist)  -> streamed as SET_TWIST
    ~/estop                 (std_srvs/Trigger)     -> latches ESTOP on the ESP32
    ~/clear_fault           (std_srvs/Trigger)     -> sends RESET_FAULT

Publications
    /wheel_odom  (nav_msgs/Odometry)                  integrated from WHEEL_ODOM
    /beacon_event (firefighter_interfaces/BeaconEvent) forwarded beacon alerts
    /battery     (sensor_msgs/BatteryState)           from STATUS
    /diagnostics (diagnostic_msgs/DiagnosticArray)    flags / mode / link health

The wire format lives in :mod:`firefighter_bridge.protocol` and must stay in
lock-step with ``firmware/robot_mobility/firefighter_protocol.h``.

NOTE on topics: wheel odometry is published on ``/wheel_odom``, *not* ``/odom``,
and does not broadcast TF by default.  ``rf2o_laser_odometry`` already owns
``odom -> base_link``; publishing a second parent for the same child would make
the TF tree ambiguous.  Later, an EKF (``config/ekf.yaml``) can fuse
``/odom_rf2o`` and ``/wheel_odom`` and take over the TF.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from firefighter_interfaces.msg import BeaconEvent as BeaconEventMsg

try:
    from . import protocol
    from .serial_link import SerialLink
except ImportError:  # pragma: no cover
    import protocol
    from serial_link import SerialLink


class FirefighterBridge(Node):
    """Bridges the mobility ESP32 to ROS 2 over a framed serial link."""

    def __init__(self) -> None:
        super().__init__("firefighter_bridge")

        # ---- parameters ------------------------------------------------
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("odom_topic", "/wheel_odom")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheel_radius_m", 0.0325)
        self.declare_parameter("ticks_per_rev", 1440)
        self.declare_parameter("track_width_m", 0.20)
        self.declare_parameter("beacon_topic", "/beacon_event")
        self.declare_parameter("battery_topic", "/battery")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("esp_timeout_s", 2.0)

        gp = self.get_parameter
        self._port = gp("port").value
        self._baud = int(gp("baud").value)
        self._cmd_timeout_s = float(gp("cmd_timeout_s").value)
        self._publish_tf = bool(gp("publish_tf").value)
        self._odom_frame = gp("odom_frame").value
        self._base_frame = gp("base_frame").value
        self._wheel_radius = float(gp("wheel_radius_m").value)
        self._ticks_per_rev = int(gp("ticks_per_rev").value)
        self._track_width = float(gp("track_width_m").value)
        self._esp_timeout_s = float(gp("esp_timeout_s").value)
        cmd_rate = max(1.0, float(gp("cmd_rate_hz").value))
        hb_rate = max(0.1, float(gp("heartbeat_hz").value))

        self._meters_per_tick = (2.0 * math.pi * self._wheel_radius) / self._ticks_per_rev

        # ---- odometry state --------------------------------------------
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._prev_ticks = None
        self._last_odom_time = None

        # ---- command state ---------------------------------------------
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._last_cmd_time = None
        self._estop = False
        self._hb_seq = 0

        # ---- link health -----------------------------------------------
        self._last_esp_rx = None
        self._esp_silent = False

        # ---- IO ---------------------------------------------------------
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._odom_pub = self.create_publisher(
            Odometry, gp("odom_topic").value, reliable
        )
        self._beacon_pub = self.create_publisher(
            BeaconEventMsg, gp("beacon_topic").value, reliable
        )
        self._battery_pub = self.create_publisher(
            BatteryState, gp("battery_topic").value, reliable
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, gp("diagnostics_topic").value, reliable
        )
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(
            Twist, gp("cmd_vel_topic").value, self._on_cmd_vel, cmd_qos
        )

        self.create_service(Trigger, "~/estop", self._srv_estop)
        self.create_service(Trigger, "~/clear_fault", self._srv_clear_fault)

        # ---- the serial link --------------------------------------------
        self._link = SerialLink(
            port=self._port,
            baud=self._baud,
            on_frame=self._on_frame,
            on_event=self._on_link_event,
            logger=self.get_logger(),
        )
        self._link.start()

        self.create_timer(1.0 / cmd_rate, self._on_cmd_timer)
        self.create_timer(1.0 / hb_rate, self._on_heartbeat_timer)
        self.create_timer(1.0, self._on_watchdog_timer)

        self.get_logger().info(
            f"bridge up: {self._port} @ {self._baud} | "
            f"odom->{gp('odom_topic').value} | tf={'on' if self._publish_tf else 'off'}"
        )

    # ------------------------------------------------------------------
    # Subscriptions / services
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_linear = msg.linear.x
        self._cmd_angular = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    def _srv_estop(self, request, response):  # noqa: ARG002
        self._estop = True
        self._link.send(protocol.MsgId.ESTOP, protocol.pack_empty())
        response.success = True
        response.message = "ESTOP latched on mobility ESP32"
        self.get_logger().warn(response.message)
        return response

    def _srv_clear_fault(self, request, response):  # noqa: ARG002
        self._estop = False
        self._link.send(protocol.MsgId.RESET_FAULT, protocol.pack_empty())
        response.success = True
        response.message = "faults cleared on mobility ESP32"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------
    def _on_cmd_timer(self) -> None:
        """Stream SET_TWIST every cycle.

        Sending zeros when there is no fresh command means the ESP32 watchdog
        stays fed while the robot is intentionally idle; if this node dies, the
        frames stop and the ESP32 stops the motors on its own.
        """
        now = self.get_clock().now()
        fresh = (
            self._last_cmd_time is not None
            and (now - self._last_cmd_time).nanoseconds * 1e-9 <= self._cmd_timeout_s
        )
        if self._estop or not fresh:
            lin = ang = 0.0
        else:
            lin, ang = self._cmd_linear, self._cmd_angular

        self._link.send(protocol.MsgId.SET_TWIST, protocol.pack_set_twist(lin, ang))

    def _on_heartbeat_timer(self) -> None:
        self._hb_seq = (self._hb_seq + 1) & 0xFFFF
        self._link.send(protocol.MsgId.HEARTBEAT, protocol.pack_heartbeat(self._hb_seq))

    def _on_watchdog_timer(self) -> None:
        if self._last_esp_rx is None:
            return
        silent = (self.get_clock().now() - self._last_esp_rx).nanoseconds * 1e-9
        if silent > self._esp_timeout_s:
            if not self._esp_silent:
                self._esp_silent = True
                self.get_logger().warn(
                    f"no frames from the mobility ESP32 for {silent:.1f}s"
                )
        elif self._esp_silent:
            self._esp_silent = False
            self.get_logger().info("mobility ESP32 is responding again")

    # ------------------------------------------------------------------
    # Incoming frames
    # ------------------------------------------------------------------
    def _on_frame(self, frame) -> None:
        self._last_esp_rx = self.get_clock().now()
        msg = frame.msg

        if msg is protocol.MsgId.WHEEL_ODOM:
            self._handle_wheel_odom(protocol.unpack_wheel_odom(frame.payload))
        elif msg is protocol.MsgId.STATUS:
            self._handle_status(protocol.unpack_status(frame.payload))
        elif msg is protocol.MsgId.BEACON_EVENT:
            self._handle_beacon(protocol.unpack_beacon_event(frame.payload))
        elif msg is protocol.MsgId.HB_ESP:
            pass  # ESP32 heartbeat: reception already refreshed the watchdog
        elif msg is protocol.MsgId.ACK:
            pass
        else:
            self.get_logger().debug(f"unhandled frame id 0x{frame.msg_id:02X}")

    def _handle_wheel_odom(self, odom: protocol.WheelOdom) -> None:
        now = self.get_clock().now()

        if self._prev_ticks is None:
            self._prev_ticks = (odom.ticks_left, odom.ticks_right)
            self._last_odom_time = now
            return

        d_left = (odom.ticks_left - self._prev_ticks[0]) * self._meters_per_tick
        d_right = (odom.ticks_right - self._prev_ticks[1]) * self._meters_per_tick
        self._prev_ticks = (odom.ticks_left, odom.ticks_right)

        dt = max(odom.dt_us * 1e-6, 1e-6)
        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self._track_width

        # integrate at the midpoint heading (standard 2D differential drive)
        self._x += d_center * math.cos(self._theta + 0.5 * d_theta)
        self._y += d_center * math.sin(self._theta + 0.5 * d_theta)
        self._theta = self._wrap(self._theta + d_theta)

        vx = d_center / dt
        vth = d_theta / dt

        out = Odometry()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose.pose.position.x = self._x
        out.pose.pose.position.y = self._y
        out.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        out.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        out.twist.twist.linear.x = vx
        out.twist.twist.angular.z = vth
        self._odom_pub.publish(out)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = out.pose.pose.orientation.z
            tf.transform.rotation.w = out.pose.pose.orientation.w
            self._tf_broadcaster.sendTransform(tf)

    def _handle_status(self, status: protocol.Status) -> None:
        now = self.get_clock().now()

        battery = BatteryState()
        battery.header.stamp = now.to_msg()
        battery.header.frame_id = self._base_frame
        # 0 mV means "no battery ADC yet" in the firmware
        battery.voltage = float("nan") if status.vbat_mv == 0 else status.vbat_mv / 1000.0
        battery.percentage = float("nan")
        battery.present = True
        self._battery_pub.publish(battery)

        mode = "pi" if status.mode == protocol.MODE_PI else "fallback"
        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()
        entry = DiagnosticStatus()
        entry.name = "firefighter_bridge: mobility esp32"
        entry.hardware_id = "mobility_esp32"
        entry.level = DiagnosticStatus.OK if status.link_ok else DiagnosticStatus.WARN
        entry.message = f"mode={mode} flags={','.join(status.flags_set) or 'none'}"
        entry.values = [
            KeyValue(key="vbat_mv", value=str(status.vbat_mv)),
            KeyValue(key="mode", value=mode),
            KeyValue(key="flags", value=str(status.flags)),
            KeyValue(key="link_ok", value=str(status.link_ok)),
            KeyValue(key="estop", value=str(status.estop)),
        ]
        diag.status.append(entry)
        self._diag_pub.publish(diag)

    def _handle_beacon(self, event: protocol.BeaconEvent) -> None:
        msg = BeaconEventMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.message_number = event.message_number
        msg.fire_detected = event.fire_detected
        msg.smoke_detected = event.smoke_detected
        msg.rssi = event.rssi
        msg.mac = list(event.mac)
        self._beacon_pub.publish(msg)
        self.get_logger().info(
            f"beacon #{event.message_number} "
            f"fire={event.fire_detected} smoke={event.smoke_detected}"
        )

    # ------------------------------------------------------------------
    def _on_link_event(self, kind: str, detail) -> None:
        if kind == "connected":
            self.get_logger().info(f"serial link up: {detail}")
        elif kind == "disconnected":
            self.get_logger().warn(f"serial link down: {detail}")

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def shutdown(self) -> None:
        self._link.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FirefighterBridge()
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
