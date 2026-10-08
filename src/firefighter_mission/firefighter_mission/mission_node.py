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
        self._beacon_goals = self._load_goals(gp("beacon_goals_file").value)
        self._last_alert_mac = ""

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
        if state == FlameEvent.IN_SUPPRESSION_RANGE:
            self._dispatch(Event.IN_RANGE)
        elif state == FlameEvent.FLAME_EXTINGUISHED:
            self._dispatch(Event.FLAME_EXTINGUISHED)
        elif state == FlameEvent.FLAME_FOUND:
            # "found" during verification means it never actually went out
            if self._fsm.state is State.VERIFYING:
                self._dispatch(Event.FLAME_STILL_PRESENT)
            else:
                self._dispatch(Event.FLAME_FOUND)
        elif state == FlameEvent.FLAME_LOST:
            # Perception reports what it sees, not what it means: losing sight of
            # the flame while suppressing or verifying is how "it went out"
            # arrives. Anywhere else it is just a lost track.
            if self._fsm.state in (State.SUPPRESSING, State.VERIFYING):
                self._dispatch(Event.FLAME_EXTINGUISHED)
            else:
                self._dispatch(Event.FLAME_LOST)

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
        # continuous sweep while searching for the flame
        if self._fsm.state is State.SEARCHING:
            self._publish_cmd_vel(self._search_spin_rate)

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
                self._suppress_pub.publish(Bool(data=True))
                self.get_logger().warn(f"suppressing (attempt {self._fsm.attempts})")
            elif action is Action.STOP_SUPPRESS:
                self._suppress_pub.publish(Bool(data=False))

        if actions:
            self._publish_state()
            self._publish_diagnostics()

    def _publish_cmd_vel(self, angular_z: float) -> None:
        msg = Twist()
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
        self._nav_handle = None
        self._nav_client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - a failed future must not kill us
            self.get_logger().error(f"navigation goal failed to send: {exc}")
            self._dispatch(Event.NAV_GOAL_FAILED)
            return

        if handle is None or not handle.accepted:
            self.get_logger().warn("navigation goal rejected")
            self._dispatch(Event.NAV_GOAL_FAILED)
            return

        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
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
        handle, self._nav_handle = self._nav_handle, None
        if handle is not None:
            handle.cancel_goal_async()
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
