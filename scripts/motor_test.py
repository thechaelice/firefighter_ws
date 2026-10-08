#!/usr/bin/env python3
"""Drive the mobility ESP32's motors from the Raspberry Pi.

Bench / bring-up tool.  It publishes ``geometry_msgs/Twist`` on ``/cmd_vel``,
which ``firefighter_bridge`` streams to the ESP32 as ``SET_TWIST`` frames, and it
reads ``/wheel_odom`` back so you can see the wheels actually turning rather than
just assuming the command landed.

    your script  --/cmd_vel-->  firefighter_bridge  --UART-->  ESP32  --> motors
                 <--/wheel_odom--                  <--UART--

Two modes:

  scripted     hold one command for N seconds, then stop
  interactive  live keyboard driving (w/s/a/d)

SAFETY
  * Put the robot on blocks or lift the wheels for the first run.
  * Do NOT run ``firefighter_mission`` at the same time - it also publishes
    ``/cmd_vel`` and the two will fight over the motors.
  * Zero is published on every exit path (Ctrl+C, 'q', or any exception).
  * Stopping this script stops the robot anyway: ``/cmd_vel`` older than
    ``firefighter_bridge``'s ``cmd_timeout_s`` (0.5 s) is treated as zero, and
    the ESP32 stops the motors itself if no frame arrives for ``PI_TIMEOUT_MS``
    (500 ms).  ``--estop`` additionally latches a stop on the ESP32.

EXAMPLES
  # creep forward for 2 s, then stop
  python3 scripts/motor_test.py --linear 0.10 --duration 2

  # spin in place (positive = counter-clockwise / left)
  python3 scripts/motor_test.py --angular 0.5 --duration 3

  # drive with the keyboard
  python3 scripts/motor_test.py --interactive

  # latch an e-stop through the bridge when finished
  python3 scripts/motor_test.py --linear 0.1 --duration 2 --estop

Run after sourcing the workspace:

  cd ~/firefighter_ws && source install/setup.bash
  python3 scripts/motor_test.py --help
"""
from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


KEY_HELP = """\
keys:
  w / s   linear  +step / -step   (forward / reverse)
  a / d   angular +step / -step   (a = turn left, d = turn right)
  space   stop (zero both)
  x       same as space
  q       quit
"""


class MotorTest(Node):
    """Publishes /cmd_vel at a fixed rate and reports wheel odometry."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("motor_test")

        self._rate = max(1.0, args.rate)
        self._max_linear = abs(args.max_linear)
        self._max_angular = abs(args.max_angular)

        self.linear = 0.0
        self.angular = 0.0
        self.last_vx = 0.0
        self.last_vth = 0.0
        self.last_odom_wall: float | None = None

        # depth=1 + reliable mirrors firefighter_bridge's /cmd_vel subscriber, so
        # a burst of stale commands can never queue up behind a fresh one.
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Twist, args.cmd_vel_topic, cmd_qos)
        self._msg = Twist()

        self._odom_sub = None
        if not args.no_odom:
            self._odom_sub = self.create_subscription(
                Odometry, args.odom_topic, self._on_odom, odom_qos
            )

        self.create_timer(1.0 / self._rate, self._on_timer)

        self._estop_client = None
        if args.estop or args.clear_fault:
            self._estop_client = self.create_client(Trigger, args.estop_service)

        self.get_logger().info(
            f"motor_test up: publishing {args.cmd_vel_topic} at {self._rate:.0f} Hz "
            f"(cap linear={self._max_linear:+.2f} m/s, angular={self._max_angular:+.2f} rad/s)"
        )

    # ------------------------------------------------------------------
    # ROS plumbing
    # ------------------------------------------------------------------
    @property
    def rate(self) -> float:
        """Publish rate in Hz (used by the run loops)."""
        return self._rate

    def _on_timer(self) -> None:
        self._msg.linear.x = self.linear
        self._msg.angular.z = self.angular
        self._pub.publish(self._msg)

    def _on_odom(self, msg: Odometry) -> None:
        self.last_vx = msg.twist.twist.linear.x
        self.last_vth = msg.twist.twist.angular.z
        self.last_odom_wall = time.monotonic()

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def set_cmd(self, linear: float, angular: float) -> None:
        self.linear = max(-self._max_linear, min(self._max_linear, linear))
        self.angular = max(-self._max_angular, min(self._max_angular, angular))

    def stop(self) -> None:
        self.linear = 0.0
        self.angular = 0.0

    def publish_zero(self, cycles: int = 5) -> None:
        """Push an explicit, immediate zero several times.

        The periodic timer would do it within one cycle, but sending it at once
        makes the stop deterministic even if we are about to tear down.
        """
        zero = Twist()
        for _ in range(cycles):
            self._pub.publish(zero)
            time.sleep(1.0 / self._rate)

    def call_estop_service(self, name: str):
        if self._estop_client is None:
            return None
        if not self._estop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"service {name} unavailable - is firefighter_bridge running?")
            return None
        future = self._estop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        res = future.result()
        if res is None:
            self.get_logger().error(f"{name} call timed out")
        else:
            self.get_logger().info(f"{name}: {res.message}")
        return res

    # ------------------------------------------------------------------
    # Health checks
    # ------------------------------------------------------------------
    def wait_for_subscriber(self, timeout_s: float = 3.0) -> bool:
        """Wait for something (i.e. firefighter_bridge) to subscribe to /cmd_vel."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._pub.get_subscription_count() > 0:
                return True
        return False

    def odom_age(self) -> float | None:
        if self.last_odom_wall is None:
            return None
        return time.monotonic() - self.last_odom_wall


# ----------------------------------------------------------------------
# Keyboard
# ----------------------------------------------------------------------
class RawTerminal:
    """Put stdin into cbreak mode, restoring it on exit."""

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._saved = None

    def __enter__(self):
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
        return False

    def read_key(self, timeout: float) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else ""


def run_interactive(node: MotorTest, step: float) -> None:
    if not sys.stdin.isatty():
        print(
            "error: --interactive needs a terminal (stdin is not a TTY).\n"
            "       Use the scripted form instead, e.g. "
            "--linear 0.1 --duration 2",
            file=sys.stderr,
        )
        raise SystemExit(2)

    print(KEY_HELP)
    print(f"step = {step} (m/s, rad/s)\n")

    with RawTerminal() as term:
        last_report = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0 / node.rate)
            key = term.read_key(0.0)

            if key:
                if key in ("q", "\x03"):  # q or Ctrl-C
                    break
                elif key == "w":
                    node.set_cmd(node.linear + step, node.angular)
                elif key == "s":
                    node.set_cmd(node.linear - step, node.angular)
                elif key == "a":
                    node.set_cmd(node.linear, node.angular + step)
                elif key == "d":
                    node.set_cmd(node.linear, node.angular - step)
                elif key in (" ", "x"):
                    node.stop()
                else:
                    continue

                if key in (" ", "x"):
                    print(f"\rcmd linear={node.linear:+.2f} m/s  angular={node.angular:+.2f} rad/s  (STOP)   ")
                else:
                    print(f"\rcmd linear={node.linear:+.2f} m/s  angular={node.angular:+.2f} rad/s          ")

            # periodic telemetry
            now = time.monotonic()
            if now - last_report >= 0.5:
                last_report = now
                age = node.odom_age()
                if age is None:
                    fb = "no /wheel_odom yet"
                elif age > 1.0:
                    fb = f"/wheel_odom stale ({age:.1f}s)"
                else:
                    fb = f"wheels: vx={node.last_vx:+.3f} m/s  wz={node.last_vth:+.3f} rad/s"
                print(f"\r  {fb:<44}", end="", flush=True)

    print()


def run_scripted(node: MotorTest, duration: float) -> None:
    if node.linear == 0.0 and node.angular == 0.0:
        print("note: linear and angular are both zero - nothing will move.")

    print(
        f"cmd linear={node.linear:+.2f} m/s  angular={node.angular:+.2f} rad/s "
        f"for {duration:g}s  (Ctrl+C to stop early)"
    )

    deadline = time.monotonic() + duration
    last_report = 0.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=1.0 / node.rate)

        now = time.monotonic()
        if now - last_report >= 1.0:
            last_report = now
            age = node.odom_age()
            if age is None:
                fb = "no /wheel_odom yet"
            elif age > 1.0:
                fb = f"/wheel_odom stale ({age:.1f}s)"
            else:
                fb = f"wheels: vx={node.last_vx:+.3f} m/s  wz={node.last_vth:+.3f} rad/s"
            print(f"  t={duration - (deadline - now):5.1f}s  {fb}")


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------
def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Drive the robot's motors from the Pi via /cmd_vel.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("EXAMPLES", 1)[-1].replace("  #", "#"),
    )
    p.add_argument(
        "--linear", type=float, default=0.0, metavar="MPS",
        help="forward speed in m/s (negative = reverse). Default: 0.0",
    )
    p.add_argument(
        "--angular", type=float, default=0.0, metavar="RPS",
        help="turn rate in rad/s (negative = clockwise). Default: 0.0",
    )
    p.add_argument(
        "--duration", type=float, default=2.0, metavar="S",
        help="seconds to hold the command in scripted mode. Default: 2.0",
    )
    p.add_argument(
        "-i", "--interactive", action="store_true",
        help="drive with the keyboard (w/s/a/d/space/q) instead of a fixed command",
    )
    p.add_argument(
        "--step", type=float, default=0.05, metavar="DELTA",
        help="per-keypress change in interactive mode. Default: 0.05",
    )
    p.add_argument(
        "--rate", type=float, default=20.0, metavar="HZ",
        help="publish rate; keep >= the bridge's cmd_rate_hz (20). Default: 20.0",
    )
    p.add_argument(
        "--max-linear", type=float, default=0.30, metavar="MPS",
        help="safety clamp on |linear|. Default: 0.30",
    )
    p.add_argument(
        "--max-angular", type=float, default=1.00, metavar="RPS",
        help="safety clamp on |angular|. Default: 1.00",
    )
    p.add_argument(
        "--cmd-vel-topic", default="/cmd_vel", help="Default: /cmd_vel",
    )
    p.add_argument(
        "--odom-topic", default="/wheel_odom",
        help="bridge's wheel odometry topic, shown as feedback. Default: /wheel_odom",
    )
    p.add_argument(
        "--no-odom", action="store_true",
        help="do not subscribe to wheel odometry",
    )
    p.add_argument(
        "--estop", action="store_true",
        help="latch an ESTOP through the bridge after stopping",
    )
    p.add_argument(
        "--clear-fault", action="store_true",
        help="clear the latched ESTOP at start (before moving)",
    )
    p.add_argument(
        "--estop-service", default="/firefighter_bridge/estop",
        help="Default: /firefighter_bridge/estop",
    )
    p.add_argument(
        "--yes", "-y", action="store_true",
        help="skip the 'is the robot on blocks?' confirmation",
    )
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    if not args.interactive and not args.yes:
        print(
            "About to command the MOTORS.\n"
            "Make sure the robot is on blocks / wheels off the ground.\n"
            "Press Enter to continue, Ctrl+C to abort.",
            end=" ",
            flush=True,
        )
        try:
            input()
        except KeyboardInterrupt:
            print("\naborted")
            return 130

    rclpy.init()
    node = MotorTest(args)
    exit_code = 0
    try:
        if args.clear_fault:
            node.call_estop_service(args.estop_service.replace("/estop", "/clear_fault"))

        if node.wait_for_subscriber(timeout_s=3.0):
            node.get_logger().info("bridge detected on /cmd_vel")
        else:
            node.get_logger().warn(
                "nothing is subscribed to %s - is firefighter_bridge running?\n"
                "  ros2 launch firefighter_bridge bridge.launch.py port:=/dev/ttyAMA0",
                args.cmd_vel_topic,
            )

        node.set_cmd(args.linear, args.angular)

        if args.interactive:
            run_interactive(node, args.step)
        else:
            run_scripted(node, args.duration)

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        node.stop()
        print("stopping: publishing zero on /cmd_vel")
        try:
            node.publish_zero()
        except Exception as exc:  # noqa: BLE001 - best-effort stop
            print(f"warning: failed to publish zero: {exc}", file=sys.stderr)

        if args.estop:
            node.call_estop_service(args.estop_service)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
