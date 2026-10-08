#!/usr/bin/env python3
"""Throwaway: drive forward only after fire AND smoke are confirmed.

A positive event is one BeaconEvent with BOTH fire_detected and
smoke_detected set (fire AND smoke, not either). `confirm_count` such events
must arrive back to back; any event missing one of the two resets the streak.

Once confirmed, publish a Twist on /cmd_vel while events stay fresh (within
`hold_s`); the bridge streams it as SET_TWIST to the mobility ESP32. A negative
or stale event publishes a zero Twist so the robot stops.

Speed defaults to 0.35 m/s: the firmware maps MAX_WHEEL_MPS = 0.50 to full
PWM, so 0.35 m/s ~= PWM 180, which actually turns the wheels (the old 0.15 m/s
only made them hum).

Run after sourcing the workspace:
    source install/setup.bash
    python3 scripts/fire_forward.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from firefighter_interfaces.msg import BeaconEvent


class FireForward(Node):
    def __init__(self):
        super().__init__("fire_forward")
        self.declare_parameter("speed", 0.35)          # m/s (~PWM 180, wheels turn)
        self.declare_parameter("confirm_count", 2)     # consecutive fire+smoke events to move
        self.declare_parameter("hold_s", 1.0)          # keep moving this long after last hit
        self.declare_parameter("cmd_rate_hz", 10.0)

        self.speed = float(self.get_parameter("speed").value)
        self.confirm_count = max(1, int(self.get_parameter("confirm_count").value))
        self.hold_s = float(self.get_parameter("hold_s").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self.create_subscription(BeaconEvent, "/beacon_event", self._on_beacon, qos)

        self.streak = 0          # consecutive fire+smoke events seen
        self.confirmed = False   # latched once streak reaches confirm_count
        self.last_seen = None    # time of the last positive event

        rate = max(1.0, float(self.get_parameter("cmd_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"need {self.confirm_count} consecutive fire+smoke events to move "
            f"forward at {self.speed} m/s"
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_beacon(self, msg):
        if msg.fire_detected and msg.smoke_detected:
            self.last_seen = self._now()
            self.streak += 1
            if self.confirmed:
                return
            if self.streak >= self.confirm_count:
                self.confirmed = True
                self.get_logger().warn("fire+smoke confirmed -> moving forward")
            else:
                self.get_logger().info(f"confirm {self.streak}/{self.confirm_count} (fire+smoke)")
        else:
            self.streak = 0
            if self.confirmed:
                self.get_logger().info("negative event -> stop")
            self.confirmed = False

    def _tick(self):
        fresh = self.last_seen is not None and (self._now() - self.last_seen) <= self.hold_s
        active = self.confirmed and fresh
        if self.confirmed and not fresh:
            self.confirmed = False
            self.streak = 0
            self.get_logger().info("stale -> stop")

        twist = Twist()
        twist.linear.x = self.speed if active else 0.0
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = FireForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # leave the robot stopped
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
