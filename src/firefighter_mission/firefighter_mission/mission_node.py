"""ROS 2 node wrapping :class:`~firefighter_mission.mission_fsm.MissionFSM`.

Adapters
    /beacon_event  (BeaconEvent)  -> BEACON_ALERT   (only when fire AND smoke)
    /flame_event   (FlameEvent)   -> FLAME_* / IN_RANGE
    ~/event        (String)       -> manual event injection for bring-up
    ~/estop, ~/reset (Trigger)    -> ESTOP / RESET

Actuators
    navigate_to_pose  (nav2_msgs/NavigateToPose) navigation goal, when use_nav2
    ~/goal_pose       (PoseStamped)              the goal, published for display
    /cmd_vel          (Twist)                    search spin, and stop
    ~/suppress        (Bool)                     extinguisher on/off
    ~/state           (String)                   current FSM state, on change
    /diagnostics      (DiagnosticArray)

Navigation is delegated to Nav2 through the ``navigate_to_pose`` action.  Goal
success/abort is fed back into the FSM as NAV_GOAL_REACHED / NAV_GOAL_FAILED, so
the FSM does not know Nav2 exists.

With ``use_nav2:=false`` the goal is only published and arrival must be injected
by hand - useful without Nav2 running, and for driving transitions Nav2 cannot
produce::

    ros2 topic pub --once /firefighter_mission/event std_msgs/String \
        "{data: nav_goal_reached}"
"""
from __future__ import annotations

import math
from typing import List

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from firefighter_interfaces.msg import BeaconEvent, FlameEvent

try:
    from .mission_fsm import Action, Event, MissionConfig, MissionFSM, State
except ImportError:  # pragma: no cover
    from mission_fsm import Action, Event, MissionConfig, MissionFSM, State


class MissionNode(Node):
    """Coordinates beacon -> navigate -> search -> suppress -> verify."""

    def __init__(self) -> None:
        super().__init__("firefighter_mission")

        # ---- topology --------------------------------------------------
        self.declare_parameter("beacon_topic", "/beacon_event")
        self.declare_parameter("flame_topic", "/flame_event")
        self.declare_parameter("goal_viz_topic", "~/goal_pose")
        self.declare_parameter("use_nav2", True)
        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("suppress_topic", "~/suppress")
        self.declare_parameter("state_topic", "~/state")
        self.declare_parameter("event_topic", "~/event")
        self.declare_parameter("diagnostics_topic", "/diagnostics")

        # ---- goal selection --------------------------------------------
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("beacon_goals_file", "")

        # ---- behaviour -------------------------------------------------
        self.declare_parameter("search_spin_rate", 0.5)
        self.declare_parameter("approach_speed", 0.15)
        self.declare_parameter("approach_turn_gain", 1.5)
        self.declare_parameter("approach_max_turn_rate", 0.8)
        self.declare_parameter("approach_align_rad", 0.25)
        self.declare_parameter("flame_stale_s", 1.0)
        self.declare_parameter("flame_lost_frames", 3)
        self.declare_parameter("tick_rate_hz", 10.0)

        # ---- FSM timings ------------------------------------------------
        cfg_defaults = MissionConfig()
        self.declare_parameter("confirm_window_s", cfg_defaults.confirm_window_s)
        self.declare_parameter("nav_timeout_s", cfg_defaults.nav_timeout_s)
        self.declare_parameter("search_timeout_s", cfg_defaults.search_timeout_s)
        self.declare_parameter("approach_timeout_s", cfg_defaults.approach_timeout_s)
        self.declare_parameter("suppress_duration_s", cfg_defaults.suppress_duration_s)
        self.declare_parameter("verify_timeout_s", cfg_defaults.verify_timeout_s)
        self.declare_parameter("complete_hold_s", cfg_defaults.complete_hold_s)
        self.declare_parameter("max_suppress_attempts", cfg_defaults.max_suppress_attempts)

        gp = self.get_parameter
        self._goal_frame = gp("goal_frame").value
        self._search_spin_rate = float(gp("search_spin_rate").value)
        self._approach_speed = float(gp("approach_speed").value)
        self._approach_turn_gain = float(gp("approach_turn_gain").value)
        self._approach_max_turn = abs(float(gp("approach_max_turn_rate").value))
        self._approach_align_rad = abs(float(gp("approach_align_rad").value))
        self._flame_stale_s = float(gp("flame_stale_s").value)
        self._flame_lost_frames = max(1, int(gp("flame_lost_frames").value))
        self._beacon_goals = self._load_goals(gp("beacon_goals_file").value)
        self._last_alert_mac = ""

        # ---- flame tracking ----------------------------------------------
        self._flame_bearing = 0.0        # last reported bearing (+left), rad
        self._flame_seen_at = None       # time of the last FOUND / IN_RANGE
        self._lost_streak = 0            # consecutive FLAME_LOST frames

        self._fsm = MissionFSM(
            MissionConfig(
                confirm_window_s=float(gp("confirm_window_s").value),
                nav_timeout_s=float(gp("nav_timeout_s").value),
                search_timeout_s=float(gp("search_timeout_s").value),
                approach_timeout_s=float(gp("approach_timeout_s").value),
                suppress_duration_s=float(gp("suppress_duration_s").value),
                verify_timeout_s=float(gp("verify_timeout_s").value),
                complete_hold_s=float(gp("complete_hold_s").value),
                max_suppress_attempts=int(gp("max_suppress_attempts").value),
            )
        )

        # ---- IO ---------------------------------------------------------
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._goal_pub = self.create_publisher(
            PoseStamped, gp("goal_viz_topic").value, reliable
        )
        self._cmd_vel_pub = self.create_publisher(Twist, gp("cmd_vel_topic").value, reliable)
        self._suppress_pub = self.create_publisher(Bool, gp("suppress_topic").value, reliable)
        self._state_pub = self.create_publisher(String, gp("state_topic").value, reliable)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, gp("diagnostics_topic").value, reliable
        )

        self.create_subscription(BeaconEvent, gp("beacon_topic").value, self._on_beacon, reliable)
        self.create_subscription(FlameEvent, gp("flame_topic").value, self._on_flame, reliable)
        self.create_subscription(String, gp("event_topic").value, self._on_manual_event, reliable)

        self.create_service(Trigger, "~/estop", self._srv_estop)
        self.create_service(Trigger, "~/reset", self._srv_reset)

        # ---- navigation ----------------------------------------------------
        self._use_nav2 = bool(gp("use_nav2").value)
        self._nav_handle = None
        self._nav_seq = 0                # bumped per request/cancel; stale callbacks are ignored
        self._nav_pending = None         # goal waiting for the action server to appear
        self._nav_wait_warned = False
        self._nav_client = (
            ActionClient(self, NavigateToPose, gp("nav_action").value)
            if self._use_nav2
            else None
        )

        tick_rate = max(0.1, float(gp("tick_rate_hz").value))
        self.create_timer(1.0 / tick_rate, self._on_tick)

        self._last_state = None
        self._publish_state()

        self.get_logger().info(
            f"mission ready | goal=({gp('goal_x').value}, {gp('goal_y').value}) "
            f"in '{self._goal_frame}' | inject events on {gp('event_topic').value}"
        )

    # ------------------------------------------------------------------
    # Clock
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def _on_beacon(self, msg: BeaconEvent) -> None:
        # The beacon transmits continuously and the ESP32 forwards every packet,
        # so the fire/smoke gate lives here (mirrors the firmware's rule).
        if not (msg.fire_detected and msg.smoke_detected):
            return
        self._last_alert_mac = ":".join(f"{byte:02x}" for byte in msg.mac)
        self.get_logger().warn(
            f"alert #{msg.message_number} from {self._last_alert_mac}"
        )
        self._dispatch(Event.BEACON_ALERT, message_number=msg.message_number)

    def _on_flame(self, msg: FlameEvent) -> None:
        state = msg.state

        if state == FlameEvent.FLAME_LOST:
            # Perception emits FLAME_LOST for every frame without a hot pixel, so
            # one frame proves nothing (noise, spray in front of the sensor).
            # Act only on `flame_lost_frames` in a row.
            self._lost_streak += 1
            if self._lost_streak < self._flame_lost_frames:
                return
            self._lost_streak = 0
            # Perception reports what it sees, not what it means: losing sight of
            # the flame while suppressing or verifying is how "it went out"
            # arrives. Anywhere else it is just a lost track.
            if self._fsm.state in (State.SUPPRESSING, State.VERIFYING):
                self._dispatch(Event.FLAME_EXTINGUISHED)
            else:
                self._dispatch(Event.FLAME_LOST)
            return

        self._lost_streak = 0
        seen = state in (FlameEvent.FLAME_FOUND, FlameEvent.IN_SUPPRESSION_RANGE)
        if seen:
            self._flame_bearing = float(msg.bearing_rad)
            self._flame_seen_at = self._now()

        if state == FlameEvent.FLAME_EXTINGUISHED:
            self._dispatch(Event.FLAME_EXTINGUISHED)
        elif seen and self._fsm.state is State.VERIFYING:
            # still visible during verification (in range or not) means it never
            # actually went out
            self._dispatch(Event.FLAME_STILL_PRESENT)
        elif state == FlameEvent.IN_SUPPRESSION_RANGE:
            self._dispatch(Event.IN_RANGE)
        elif state == FlameEvent.FLAME_FOUND:
            self._dispatch(Event.FLAME_FOUND)

    def _on_manual_event(self, msg: String) -> None:
        name = msg.data.strip().lower()
        try:
            event = Event(name)
        except ValueError:
            valid = ", ".join(e.value for e in Event)
            self.get_logger().warn(f"unknown event '{msg.data}' (valid: {valid})")
            return
        self.get_logger().info(f"injected event '{event.value}'")
        self._dispatch(event)

    def _srv_estop(self, request, response):  # noqa: ARG002
        self._dispatch(Event.ESTOP)
        response.success = True
        response.message = f"mission {self._fsm.state.value}"
        return response

    def _srv_reset(self, request, response):  # noqa: ARG002
        self._dispatch(Event.RESET)
        response.success = True
        response.message = f"mission {self._fsm.state.value}"
        return response

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------
    def _on_tick(self) -> None:
        self._run(self._fsm.update(self._now()))
        # a goal requested before Nav2 was up goes out as soon as it is
        if self._nav_pending is not None:
            self._send_pending_goal()
        # continuous sweep while searching for the flame
        if self._fsm.state is State.SEARCHING:
            self._publish_cmd_vel(self._search_spin_rate)
        elif self._fsm.state is State.APPROACHING:
            self._drive_approach()

    def _drive_approach(self) -> None:
        """Close on the flame: turn to its bearing, then creep forward.

        Perception turns this into IN_RANGE once the LiDAR range at that bearing
        drops under its suppression range. With no fresh sighting the robot holds
        still rather than driving on a stale bearing.
        """
        fresh = (
            self._flame_seen_at is not None
            and self._now() - self._flame_seen_at <= self._flame_stale_s
        )
        if not fresh:
            self._publish_cmd_vel(0.0)
            return

        bearing = self._flame_bearing
        turn = max(
            -self._approach_max_turn,
            min(self._approach_max_turn, self._approach_turn_gain * bearing),
        )
        aligned = abs(bearing) <= self._approach_align_rad
        self._publish_cmd_vel(turn, self._approach_speed if aligned else 0.0)

    # ------------------------------------------------------------------
    # Outputs
    # ------------------------------------------------------------------
    def _dispatch(self, event: Event, **data) -> List[Action]:
        actions = self._fsm.handle_event(event, now=self._now(), **data)
        self._run(actions)
        return actions

    def _run(self, actions: List[Action]) -> None:
        for action in actions:
            if action is Action.STOP:
                self._publish_cmd_vel(0.0)
            elif action is Action.STOP_SEARCH:
                self._publish_cmd_vel(0.0)
            elif action is Action.REQUEST_NAVIGATE:
                self._request_navigation()
            elif action is Action.CANCEL_NAVIGATION:
                self._cancel_navigation()
            elif action is Action.START_SEARCH:
                self.get_logger().info("searching for the flame")
            elif action is Action.START_APPROACH:
                self.get_logger().info("approaching the flame")
            elif action is Action.START_SUPPRESS:
                self._publish_cmd_vel(0.0)      # the approach may still be driving
                self._suppress_pub.publish(Bool(data=True))
                self.get_logger().warn(f"suppressing (attempt {self._fsm.attempts})")
            elif action is Action.STOP_SUPPRESS:
                self._suppress_pub.publish(Bool(data=False))

        if actions:
            self._publish_state()
            self._publish_diagnostics()

    def _publish_cmd_vel(self, angular_z: float, linear_x: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    def _publish_state(self) -> None:
        state = self._fsm.state.value
        if state == self._last_state:
            return
        self._last_state = state
        self._state_pub.publish(String(data=state))
        self.get_logger().info(f"state -> {state}")

    def _publish_diagnostics(self) -> None:
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        entry = DiagnosticStatus()
        entry.name = "firefighter_mission: state"
        entry.hardware_id = "mission"
        entry.level = DiagnosticStatus.OK
        entry.message = self._fsm.state.value
        entry.values = [
            KeyValue(key="attempts", value=str(self._fsm.attempts)),
            KeyValue(key="last_alert_mac", value=self._last_alert_mac),
            KeyValue(key="active", value=str(self._fsm.is_active)),
        ]
        diag.status.append(entry)
        self._diag_pub.publish(diag)

    # ------------------------------------------------------------------
    # Goals
    # ------------------------------------------------------------------
    def _lookup_goal(self):
        """Return (x, y, yaw) for the alerting beacon, else the defaults."""
        if self._last_alert_mac and self._last_alert_mac in self._beacon_goals:
            return self._beacon_goals[self._last_alert_mac]
        return self._default_goal()

    def _default_goal(self):
        gp = self.get_parameter
        return (
            float(gp("goal_x").value),
            float(gp("goal_y").value),
            float(gp("goal_yaw").value),
        )

    def _goal_pose(self) -> PoseStamped:
        x, y, yaw = self._lookup_goal()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._goal_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    # ------------------------------------------------------------------
    # Navigation (Nav2 NavigateToPose action)
    # ------------------------------------------------------------------
    def _request_navigation(self) -> None:
        pose = self._goal_pose()
        self._goal_pub.publish(pose)          # display / manual fallback

        x, y, yaw = self._lookup_goal()
        self.get_logger().info(
            f"navigating to ({x:.2f}, {y:.2f}, {yaw:.2f}) in '{self._goal_frame}'"
        )

        if self._nav_client is None:
            self.get_logger().warn(
                "use_nav2 is false - waiting for a manual nav_goal_reached event"
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_seq += 1
        self._nav_handle = None
        self._nav_pending = goal
        self._nav_wait_warned = False
        self._send_pending_goal()

    def _send_pending_goal(self) -> None:
        """Send the waiting goal once the action server exists.

        A goal sent to a server that is not up yet is silently lost, so it is held
        here and retried from the tick. The FSM's nav timeout still bounds the wait.
        """
        if self._nav_pending is None or self._nav_client is None:
            return
        if not self._nav_client.server_is_ready():
            if not self._nav_wait_warned:
                self._nav_wait_warned = True
                self.get_logger().warn(
                    "Nav2 action server is not available yet - the goal will be "
                    "sent when it appears (is Nav2 running and active?)"
                )
            return

        goal, self._nav_pending = self._nav_pending, None
        seq = self._nav_seq
        self._nav_client.send_goal_async(goal).add_done_callback(
            lambda future, seq=seq: self._on_goal_response(future, seq)
        )

    def _on_goal_response(self, future, seq: int) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - a failed future must not kill us
            self.get_logger().error(f"navigation goal failed to send: {exc}")
            if seq == self._nav_seq:
                self._dispatch(Event.NAV_GOAL_FAILED)
            return

        if seq != self._nav_seq:
            # cancelled (or replaced) before Nav2 answered: do not let it run
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
                self.get_logger().info("navigation cancelled (goal accepted late)")
            return

        if handle is None or not handle.accepted:
            self.get_logger().warn("navigation goal rejected")
            self._dispatch(Event.NAV_GOAL_FAILED)
            return

        self._nav_handle = handle
        handle.get_result_async().add_done_callback(
            lambda future, seq=seq: self._on_nav_result(future, seq)
        )

    def _on_nav_result(self, future, seq: int) -> None:
        if seq != self._nav_seq:
            return                      # result of a goal we already gave up on
        self._nav_handle = None
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"navigation result unavailable: {exc}")
            self._dispatch(Event.NAV_GOAL_FAILED)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._dispatch(Event.NAV_GOAL_REACHED)
        else:
            self.get_logger().warn(f"navigation ended with status {status}")
            self._dispatch(Event.NAV_GOAL_FAILED)

    def _cancel_navigation(self) -> None:
        # Bumping the sequence invalidates every outstanding callback, including a
        # goal Nav2 has not accepted yet (it is cancelled when the answer arrives).
        self._nav_seq += 1
        pending, self._nav_pending = self._nav_pending, None
        handle, self._nav_handle = self._nav_handle, None
        if handle is not None:
            handle.cancel_goal_async()
        if handle is not None or pending is not None:
            self.get_logger().info("navigation cancelled")

    def _load_goals(self, path: str):
        """Load a MAC -> pose map, e.g.:  'aa:bb:cc:dd:ee:ff: {x: 3.0, y: 1.5, yaw: 0.0}'."""
        if not path:
            return {}
        try:
            with open(path) as handle:
                data = yaml.safe_load(handle) or {}
        except OSError as exc:
            self.get_logger().warn(f"cannot read beacon goals file '{path}': {exc}")
            return {}

        goals = {}
        for mac, pose in data.items():
            try:
                goals[str(mac).lower()] = (
                    float(pose["x"]),
                    float(pose["y"]),
                    float(pose.get("yaw", 0.0)),
                )
            except (TypeError, KeyError, ValueError) as exc:
                self.get_logger().warn(f"bad goal for {mac}: {exc}")
        self.get_logger().info(f"loaded {len(goals)} beacon goal(s) from {path}")
        return goals


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
