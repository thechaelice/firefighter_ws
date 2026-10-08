#!/usr/bin/env python3
"""Browser-based operations dashboard for the firefighting robot.

One page showing everything the robot is doing right now:

  * LiDAR        - live 2D scan (and an optional PointCloud2 in 3D)
  * Beacon       - last alert: fire/smoke, RSSI + rough range, MAC, packet count
  * Mobility ESP32 - link health, mode, flags, e-stop, battery (via /diagnostics)
  * Mission      - FSM state, suppression, current goal
  * Odometry     - wheel odometry (bridge) and laser odometry (rf2o)
  * Command      - what /cmd_vel is currently being commanded
  * Topics       - every watched topic with its rate and freshness
  * Events       - a rolling log of detections, state changes and link changes

Data is gathered by one rclpy node and served as JSON on ``/api/telemetry``;
the LiDAR canvas streams from ``/api/scan`` (or ``/api/cloud``).  The page is
plain HTML/CSS/JS with no external assets, so it works on an isolated network.

    cd ~/firefighter_ws && source install/setup.bash
    python3 scripts/robot_dashboard.py
    # then open http://<pi-ip>:8080/

The server binds to all interfaces and has **no authentication** - only run it
on a trusted network.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String

try:  # the workspace messages; degrade gracefully if they are not built
    from firefighter_interfaces.msg import BeaconEvent, FlameEvent

    HAVE_ROBOT_MSGS = True
except ImportError:  # pragma: no cover - depends on a sourced workspace
    BeaconEvent = FlameEvent = None  # type: ignore[assignment]
    HAVE_ROBOT_MSGS = False


EVENT_LOG_SIZE = 40
DIAGNOSTIC_LEVELS = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def _finite(value) -> float | None:
    """Coerce to a JSON-safe float - ``None`` for NaN/inf/None."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _age(monotonic_stamp: float | None) -> float | None:
    return None if monotonic_stamp is None else time.monotonic() - monotonic_stamp


def _format_mac(raw) -> str:
    try:
        return ":".join(f"{int(byte):02x}" for byte in raw)
    except (TypeError, ValueError):
        return ""


def _coerce_int(value) -> int | None:
    """Coerce a ROS integer field to a Python ``int``.

    Some ROS 2 builds expose ``byte`` fields (e.g. ``DiagnosticStatus.level``)
    as single-byte ``bytes`` objects such as ``b'\\x00'`` rather than ``int``;
    ``int(b'\\x00')`` raises ``ValueError``.  Accept both representations.
    """
    if value is None:
        return None
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class TopicStats:
    """Arrival rate and freshness for one subscribed topic."""

    def __init__(self, topic: str, type_name: str) -> None:
        self.topic = topic
        self.type_name = type_name
        self.total = 0
        self._times: deque[float] = deque(maxlen=25)

    def touch(self) -> None:
        self.total += 1
        self._times.append(time.monotonic())

    def snapshot(self) -> dict:
        if not self._times:
            return {
                "topic": self.topic,
                "type": self.type_name,
                "total": 0,
                "hz": None,
                "age": None,
            }
        span = self._times[-1] - self._times[0]
        hz = None
        if len(self._times) > 1 and span > 1e-6:
            hz = (len(self._times) - 1) / span
        return {
            "topic": self.topic,
            "type": self.type_name,
            "total": self.total,
            "hz": hz,
            "age": time.monotonic() - self._times[-1],
        }


class RobotDashboard(Node):
    """Collect the robot's operational state and expose it as JSON."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("robot_dashboard")

        # Re-entrant: _log_event takes it too, and is called with it held.
        self._lock = threading.RLock()
        self._event_seq = 0
        self._max_cloud_points = args.max_cloud_points

        self._scan: dict | None = None
        self._cloud: dict | None = None
        self._beacon: dict | None = None
        self._flame: dict | None = None
        self._command: dict | None = None
        self._goal: dict | None = None
        self._odom: dict[str, dict] = {}
        self._mission = {"state": None, "state_at": None, "suppress": None, "suppress_at": None}
        self._battery: dict | None = None
        self._diagnostics: dict[str, dict] = {}
        self._events: deque[dict] = deque(maxlen=EVENT_LOG_SIZE)
        self._stats: dict[str, TopicStats] = {}

        # Reliable + volatile matches what firefighter_bridge and
        # firefighter_mission publish; /scan and /cloud use sensor QoS instead.
        self._status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._subscribe(LaserScan, args.scan_topic, "sensor_msgs/LaserScan",
                        self._on_scan, sensor=True)
        if args.pointcloud_topic:
            self._subscribe(PointCloud2, args.pointcloud_topic, "sensor_msgs/PointCloud2",
                            self._on_cloud, sensor=True)
        if HAVE_ROBOT_MSGS:
            self._subscribe(BeaconEvent, args.beacon_topic,
                            "firefighter_interfaces/BeaconEvent", self._on_beacon)
            self._subscribe(FlameEvent, args.flame_topic,
                            "firefighter_interfaces/FlameEvent", self._on_flame)
        else:
            self.get_logger().warn(
                "firefighter_interfaces not importable - beacon/flame panels disabled. "
                "Run ./build.sh then 'source install/setup.bash'."
            )

        self._subscribe(Odometry, args.odom_topic, "nav_msgs/Odometry", self._on_wheel_odom)
        self._subscribe(Odometry, args.rf2o_topic, "nav_msgs/Odometry", self._on_rf2o_odom)
        self._subscribe(DiagnosticArray, args.diagnostics_topic,
                        "diagnostic_msgs/DiagnosticArray", self._on_diagnostics)
        self._subscribe(BatteryState, args.battery_topic, "sensor_msgs/BatteryState",
                        self._on_battery)
        self._subscribe(String, args.mission_state_topic, "std_msgs/String",
                        self._on_mission_state)
        self._subscribe(Bool, args.mission_suppress_topic, "std_msgs/Bool",
                        self._on_mission_suppress)
        self._subscribe(Twist, args.cmd_vel_topic, "geometry_msgs/Twist", self._on_cmd_vel)
        self._subscribe(PoseStamped, args.goal_topic, "geometry_msgs/PoseStamped",
                        self._on_goal)

        self.get_logger().info(
            f"dashboard up - {len(self._stats)} topics watched, "
            f"serving on http://<pi-ip>:{args.port}/"
        )

    # ------------------------------------------------------------------
    # Subscription plumbing
    # ------------------------------------------------------------------
    def _subscribe(self, msg_type, topic: str, type_name: str, callback, sensor: bool = False) -> None:
        stats = TopicStats(topic, type_name)
        self._stats[topic] = stats

        def wrapper(message, _callback=callback, _stats=stats):
            _stats.touch()
            try:
                _callback(message)
            except Exception as exc:  # noqa: BLE001 - never kill the subscriber
                self.get_logger().error(f"{topic} handler failed: {exc}")

        self.create_subscription(
            msg_type,
            topic,
            wrapper,
            qos_profile_sensor_data if sensor else self._status_qos,
        )

    def _log_event(self, source: str, level: str, text: str) -> None:
        with self._lock:
            self._event_seq += 1
            self._events.append(
                {"id": self._event_seq, "at": time.monotonic(), "source": source,
                 "level": level, "text": text}
            )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        ranges = [float(value) if math.isfinite(value) else None for value in msg.ranges]
        with self._lock:
            self._scan = {
                "frame_id": msg.header.frame_id,
                "at": time.monotonic(),
                "angle_min": float(msg.angle_min),
                "angle_increment": float(msg.angle_increment),
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
                "ranges": ranges,
            }

    def _on_cloud(self, msg: PointCloud2) -> None:
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            self.get_logger().warning("Ignoring PointCloud2 without x, y, and z fields")
            return

        formats = {
            PointField.FLOAT32: ("f", 4),
            PointField.FLOAT64: ("d", 8),
        }
        coordinates = []
        for name in ("x", "y", "z"):
            field = fields[name]
            if field.datatype not in formats or field.count < 1:
                self.get_logger().warning(
                    f"Ignoring PointCloud2: {name} must be FLOAT32 or FLOAT64")
                return
            code, size = formats[field.datatype]
            if field.offset < 0 or field.offset + size > msg.point_step:
                self.get_logger().warning(f"Ignoring PointCloud2: invalid {name} field offset")
                return
            coordinates.append((field.offset, code))

        total = msg.width * msg.height
        if total == 0 or msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
            return
        sample_step = max(1, math.ceil(total / self._max_cloud_points))
        endian = ">" if msg.is_bigendian else "<"
        points = []
        for index in range(0, total, sample_step):
            row, column = divmod(index, msg.width)
            base = row * msg.row_step + column * msg.point_step
            if base + msg.point_step > len(msg.data):
                break
            point = tuple(
                struct.unpack_from(endian + code, msg.data, base + offset)[0]
                for offset, code in coordinates
            )
            if all(math.isfinite(value) for value in point):
                points.append(point)

        with self._lock:
            self._cloud = {
                "frame_id": msg.header.frame_id,
                "at": time.monotonic(),
                "points": points,
            }

    def _on_beacon(self, msg) -> None:
        now = time.monotonic()
        fire = bool(msg.fire_detected)
        smoke = bool(msg.smoke_detected)
        rssi = int(msg.rssi)
        with self._lock:
            previous = self._beacon
            self._beacon = {
                "at": now,
                "message_number": int(msg.message_number),
                "fire": fire,
                "smoke": smoke,
                "rssi": rssi,
                "mac": _format_mac(msg.mac),
                "count": (previous["count"] if previous else 0) + 1,
            }
        # Beacons repeat while the alarm lasts; log the transitions, not every packet.
        if previous and (previous["fire"], previous["smoke"]) == (fire, smoke):
            return
        alerts = "+".join(name for name, flag in (("FIRE", fire), ("SMOKE", smoke)) if flag)
        level = "alert" if fire else ("warn" if smoke else "ok")
        suffix = f" rssi -{rssi} dBm" if rssi else ""
        self._log_event(
            "beacon", level,
            f"beacon #{int(msg.message_number)} {alerts or 'clear'}{suffix}",
        )

    def _on_flame(self, msg) -> None:
        state = int(msg.state)
        names = {0: "FLAME_FOUND", 1: "FLAME_LOST", 2: "FLAME_EXTINGUISHED",
                 3: "IN_SUPPRESSION_RANGE"}
        name = names.get(state, f"STATE_{state}")
        with self._lock:
            previous = self._flame
            self._flame = {
                "at": time.monotonic(),
                "state": state,
                "state_name": name,
                "bearing_rad": _finite(msg.bearing_rad),
                "distance_m": _finite(msg.distance_m),
            }
        # Perception publishes one FlameEvent per thermal frame; log state changes only.
        if previous is None or previous["state"] != state:
            self._log_event("thermal", "info", f"flame event: {name}")

    def _on_wheel_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        with self._lock:
            self._odom["wheel"] = {
                "at": time.monotonic(),
                "vx": _finite(msg.twist.twist.linear.x),
                "wz": _finite(msg.twist.twist.angular.z),
                "x": _finite(pose.position.x),
                "y": _finite(pose.position.y),
                "yaw": _finite(
                    _yaw_from_quaternion(
                        pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w,
                    )
                ),
                "frame_id": msg.header.frame_id,
            }

    def _on_rf2o_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        with self._lock:
            self._odom["rf2o"] = {
                "at": time.monotonic(),
                "vx": _finite(msg.twist.twist.linear.x),
                "wz": _finite(msg.twist.twist.angular.z),
                "x": _finite(pose.position.x),
                "y": _finite(pose.position.y),
                "yaw": _finite(
                    _yaw_from_quaternion(
                        pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w,
                    )
                ),
                "frame_id": msg.header.frame_id,
            }

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        with self._lock:
            for entry in msg.status:
                values = {item.key: item.value for item in entry.values}
                level = _coerce_int(entry.level)
                level = 0 if level is None else level
                previous = self._diagnostics.get(entry.name)
                self._diagnostics[entry.name] = {
                    "name": entry.name,
                    "at": time.monotonic(),
                    "level": level,
                    "level_name": DIAGNOSTIC_LEVELS.get(level, f"LEVEL_{level}"),
                    "message": entry.message,
                    "hardware_id": entry.hardware_id,
                    "values": values,
                }
                if previous is None or previous["level"] != level:
                    self._log_event(
                        "diagnostics",
                        "alert" if level >= 2 else ("warn" if level == 1 else "ok"),
                        f"{entry.name}: {DIAGNOSTIC_LEVELS.get(level, level)}",
                    )

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._battery = {
                "at": time.monotonic(),
                "voltage": _finite(msg.voltage),
                "percentage": _finite(msg.percentage),
                "present": bool(msg.present),
            }

    def _on_mission_state(self, msg: String) -> None:
        state = msg.data
        with self._lock:
            previous = self._mission["state"]
            self._mission["state"] = state
            self._mission["state_at"] = time.monotonic()
        if state != previous:
            self._log_event("mission", "info", f"mission state: {state}")

    def _on_mission_suppress(self, msg: Bool) -> None:
        with self._lock:
            self._mission["suppress"] = bool(msg.data)
            self._mission["suppress_at"] = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._command = {
                "at": time.monotonic(),
                "linear": _finite(msg.linear.x),
                "angular": _finite(msg.angular.z),
            }

    def _on_goal(self, msg: PoseStamped) -> None:
        with self._lock:
            self._goal = {
                "at": time.monotonic(),
                "x": _finite(msg.pose.position.x),
                "y": _finite(msg.pose.position.y),
                "frame_id": msg.header.frame_id,
            }

    # ------------------------------------------------------------------
    # Snapshots
    # ------------------------------------------------------------------
    @staticmethod
    def _with_age(sample: dict | None) -> dict | None:
        # Ages are computed here, on the robot's monotonic clock, so the page
        # never has to compare the browser's wall clock with the Pi's.
        if sample is None:
            return None
        view = dict(sample)
        view["age"] = _age(view.pop("at"))
        return view

    def scan_snapshot(self) -> dict | None:
        with self._lock:
            return self._with_age(self._scan)

    def cloud_snapshot(self) -> dict | None:
        with self._lock:
            return self._with_age(self._cloud)

    def telemetry_snapshot(self) -> dict:
        with self._lock:
            beacon = dict(self._beacon) if self._beacon else None
            flame = dict(self._flame) if self._flame else None
            command = dict(self._command) if self._command else None
            goal = dict(self._goal) if self._goal else None
            odom = {key: dict(value) for key, value in self._odom.items()}
            mission = dict(self._mission)
            battery = dict(self._battery) if self._battery else None
            diagnostics = [dict(value) for value in self._diagnostics.values()]
            events = [self._with_age(event) for event in self._events]
            topics = [stats.snapshot() for stats in self._stats.values()]

        esp32 = next(
            (entry for entry in diagnostics if entry.get("hardware_id") == "mobility_esp32"),
            None,
        )
        return {
            "now": time.time(),
            "robot_msgs": HAVE_ROBOT_MSGS,
            "beacon": self._beacon_view(beacon),
            "esp32": self._esp32_view(esp32),
            "battery": battery,
            "mission": {
                "state": mission["state"],
                "age": _age(mission["state_at"]),
                "suppress": mission["suppress"],
                "suppress_age": _age(mission["suppress_at"]),
                "goal": {"x": goal["x"], "y": goal["y"], "frame_id": goal["frame_id"],
                         "age": _age(goal["at"])} if goal else None,
            },
            "command": {
                "linear": command["linear"] if command else None,
                "angular": command["angular"] if command else None,
                "age": _age(command["at"]) if command else None,
            },
            "odometry": {
                key: {
                    "vx": value["vx"], "wz": value["wz"],
                    "x": value["x"], "y": value["y"], "yaw": value["yaw"],
                    "age": _age(value["at"]),
                }
                for key, value in odom.items()
            },
            "flame": {
                "state_name": flame["state_name"],
                "bearing_rad": flame["bearing_rad"],
                "distance_m": flame["distance_m"],
                "age": _age(flame["at"]),
            } if flame else None,
            "diagnostics": [
                {
                    "name": entry["name"],
                    "level": entry["level"],
                    "level_name": entry["level_name"],
                    "message": entry["message"],
                    "hardware_id": entry.get("hardware_id", ""),
                    "values": entry["values"],
                    "age": _age(entry["at"]),
                }
                for entry in diagnostics
            ],
            "topics": sorted(topics, key=lambda item: item["topic"]),
            "events": events,
        }

    @staticmethod
    def _beacon_view(beacon: dict | None) -> dict | None:
        if beacon is None:
            return None
        return {
            "message_number": beacon["message_number"],
            "fire": beacon["fire"],
            "smoke": beacon["smoke"],
            "rssi": beacon["rssi"],
            "mac": beacon["mac"],
            "count": beacon["count"],
            "age": _age(beacon["at"]),
        }

    @staticmethod
    def _esp32_view(entry: dict | None) -> dict | None:
        if entry is None:
            return None
        values = entry["values"]

        def as_bool(key: str):
            raw = values.get(key)
            return None if raw is None else raw == "True"

        vbat_mv = None
        try:
            vbat_mv = int(values.get("vbat_mv", "0"))
        except (TypeError, ValueError):
            pass
        return {
            "level": entry["level"],
            "level_name": entry["level_name"],
            "message": entry["message"],
            "age": _age(entry["at"]),
            "mode": values.get("mode"),
            "link_ok": as_bool("link_ok"),
            "estop": as_bool("estop"),
            "encoder_fault": as_bool("encoder_fault"),
            "flags": values.get("flags"),
            "mobility_moving": "moving" in entry["message"],
            "vbat_v": (vbat_mv / 1000.0) if vbat_mv else None,
        }


PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" type="image/svg+xml" href="/favicon.svg">
  <title>Firefighter Robot · Dashboard</title>
  <style>
    :root {
      color-scheme: dark;
      --bg:#0f1317; --panel:#161c22; --panel2:#1b232b; --line:#2b343d;
      --text:#e6edf3; --muted:#93a1ae; --dim:#6b7783;
      --ok:#3fb950; --warn:#d29922; --alert:#f85149; --info:#58a6ff;
      --fire:#ff6b35; --smoke:#d9a520;
      font: 14px/1.45 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
    }
    * { box-sizing: border-box; }
    body { margin:0; background:var(--bg); color:var(--text); }
    header { position:sticky; top:0; z-index:5; display:flex; flex-wrap:wrap; align-items:center;
             gap:14px; padding:11px 18px; background:var(--panel); border-bottom:1px solid var(--line); }
    h1 { font-size:16px; margin:0; font-weight:600; letter-spacing:.01em; }
    h1 span { color:var(--dim); font-weight:400; }
    #conn { margin-left:auto; color:var(--muted); font-variant-numeric: tabular-nums; }
    #conn.error { color:var(--alert); }
    main { padding:14px; max-width:1400px; margin:0 auto; display:grid; gap:14px; }
    #alerts { display:grid; gap:8px; }
    #alerts:empty { display:none; }
    .banner { display:flex; flex-wrap:wrap; align-items:baseline; gap:4px 12px; padding:12px 16px;
              border-radius:8px; font-weight:600; letter-spacing:.02em; }
    .banner.fire, .banner.alert { background:#3d1512; border:1px solid #7f2a22; color:#ffb4ac; }
    .banner.smoke, .banner.warn { background:#33290c; border:1px solid #6d5a17; color:#ffdf9e; }
    body.offline .card { opacity:.5; }
    .banner .detail { font-weight:400; color:var(--muted); }
    .cards { display:grid; gap:14px; grid-template-columns:repeat(auto-fit, minmax(255px, 1fr)); }
    .split { display:grid; gap:14px; grid-template-columns:minmax(0, 2fr) minmax(0, 1fr); }
    @media (max-width: 1000px) { .split { grid-template-columns:1fr; } }
    .card { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:12px 14px; min-width:0; }
    .card > h2 { margin:0 0 10px; font-size:12px; text-transform:uppercase; letter-spacing:.09em;
                 color:var(--muted); font-weight:600; display:flex; align-items:center; gap:8px; }
    .card > h2 .hint { margin-left:auto; font-size:11px; letter-spacing:0; text-transform:none; color:var(--dim); }
    .kv { display:grid; grid-template-columns:auto 1fr; gap:4px 12px; font-variant-numeric:tabular-nums; }
    .kv dt { color:var(--muted); white-space:nowrap; }
    .kv dd { margin:0; text-align:right; }
    .kv dd.num { font-family:ui-monospace, SFMono-Regular, Menlo, monospace; }
    .badges { display:flex; flex-wrap:wrap; gap:6px; margin-bottom:10px; }
    .badge { display:inline-flex; align-items:center; gap:5px; padding:2px 8px; border-radius:999px;
             font-size:12px; font-weight:600; border:1px solid transparent; background:#232c34; color:var(--muted); }
    .badge.ok    { background:#12301b; border-color:#1f5a30; color:#71d68d; }
    .badge.warn  { background:#33290c; border-color:#6d5a17; color:#e5c05d; }
    .badge.alert { background:#3d1512; border-color:#7f2a22; color:#ff8d80; }
    .badge.info  { background:#12283d; border-color:#1f4c73; color:#79b8ff; }
    .badge.fire  { background:#3d1512; border-color:#8a3025; color:#ff9b7a; }
    .badge.smoke { background:#33290c; border-color:#6d5a17; color:#e8c96b; }
    .badge.idle  { opacity:.55; }
    .dot { width:8px; height:8px; border-radius:50%; background:var(--dim); display:inline-block; }
    .dot.ok { background:var(--ok); } .dot.warn { background:var(--warn); }
    .dot.alert { background:var(--alert); }
    .bar { height:6px; border-radius:3px; background:#232c34; overflow:hidden; margin-top:5px; }
    .bar > i { display:block; height:100%; background:var(--info); }
    .bar.ok > i { background:var(--ok); } .bar.warn > i { background:var(--warn); }
    .bar.alert > i { background:var(--alert); }
    .empty { color:var(--dim); font-style:italic; }
    .toolbar { display:flex; flex-wrap:wrap; align-items:center; gap:14px; color:var(--muted); margin-bottom:10px; }
    .toolbar label { display:flex; align-items:center; gap:8px; }
    input[type=range] { width:130px; }
    canvas { display:block; width:100%; height:min(64vh, 680px); min-height:300px; background:#0b0e11;
             border:1px solid var(--line); border-radius:6px; touch-action:none; }
    nav { display:flex; gap:6px; }
    button { color:inherit; background:#222a32; border:1px solid #3a4651; border-radius:5px;
             padding:5px 11px; cursor:pointer; font-size:13px; }
    button.active { background:#176b87; border-color:#2796b8; }
    table { width:100%; border-collapse:collapse; font-size:13px; font-variant-numeric:tabular-nums; table-layout:fixed; }
    th { text-align:left; color:var(--muted); font-weight:600; font-size:11px; text-transform:uppercase;
         letter-spacing:.07em; padding:0 6px 6px; border-bottom:1px solid var(--line); white-space:nowrap; }
    td { padding:5px 6px; border-bottom:1px solid #222a31; vertical-align:top; }
    td.mono { font-family:ui-monospace, SFMono-Regular, Menlo, monospace; white-space:nowrap;
              overflow:hidden; text-overflow:ellipsis; }
    td.right, th.right { text-align:right; }
    tr.stale td { color:var(--dim); }
    #events { list-style:none; margin:0; padding:0; max-height:340px; overflow:auto; }
    #events li { display:grid; grid-template-columns:84px 1fr; gap:9px; padding:5px 0;
                 border-bottom:1px solid #222a31; align-items:baseline; }
    #events .t { color:var(--dim); white-space:nowrap; font-family:ui-monospace, SFMono-Regular, Menlo, monospace; font-size:12px; }
    #events .src { color:var(--muted); font-size:11px; text-transform:uppercase; letter-spacing:.06em; }
    #events li.alert .txt { color:#ff9b8f; } #events li.warn .txt { color:#e5c05d; }
    #events li.ok .txt { color:#8fdc9f; }
    .scroll { max-height:420px; overflow:auto; }
  </style>
</head>
<body>
  <header>
    <h1>Firefighter Robot <span>· operations dashboard</span></h1>
    <nav>
      <button id="twoD" class="active">2D scan</button>
      <button id="threeD">3D view</button>
    </nav>
    <span id="conn">Connecting…</span>
  </header>
  <main>
    <div id="alerts" role="status" aria-live="polite"></div>

    <div class="cards">
      <div class="card">
        <h2>Beacon <span class="hint" id="beaconHint"></span></h2>
        <div class="badges" id="beaconBadges"></div>
        <dl class="kv" id="beaconKv"></dl>
        <div id="beaconBar"></div>
      </div>

      <div class="card">
        <h2>Mobility ESP32 <span class="hint" id="espHint"></span></h2>
        <div class="badges" id="espBadges"></div>
        <dl class="kv" id="espKv"></dl>
      </div>

      <div class="card">
        <h2>Mission <span class="hint" id="missionHint"></span></h2>
        <div class="badges" id="missionBadges"></div>
        <dl class="kv" id="missionKv"></dl>
      </div>

      <div class="card">
        <h2>Odometry <span class="hint">wheel vs laser</span></h2>
        <dl class="kv" id="odomKv"></dl>
        <div class="badges" id="odomBadges" style="margin-top:10px"></div>
      </div>
    </div>

    <div class="split">
      <div class="card">
        <h2>LiDAR <span class="hint" id="scanHint"></span></h2>
        <div class="toolbar">
          <label>Range <input id="range" type="range" min="2" max="30" step="1" value="8"><span id="rangeValue">8 m</span></label>
          <span id="cloudInfo"></span>
        </div>
        <canvas id="view" aria-label="LiDAR scan visualization"></canvas>
      </div>

      <div class="card">
        <h2>Topics <span class="hint">rate · freshness</span></h2>
        <div class="scroll">
          <table>
            <colgroup><col style="width:20px"><col><col style="width:64px"><col style="width:64px"></colgroup>
            <thead><tr><th></th><th>Topic</th><th class="right">Rate</th><th class="right">Age</th></tr></thead>
            <tbody id="topics"></tbody>
          </table>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Event log <span class="hint">detections · state changes · link</span></h2>
      <ul id="events"></ul>
    </div>
  </main>

  <script>
  (() => {
    const $ = id => document.getElementById(id);
    const canvas = $('view'), ctx = canvas.getContext('2d');
    const rangeSlider = $('range'), rangeValue = $('rangeValue');
    let mode = '2d', scan = null, cloud = null;
    let azimuth = -0.7, elevation = 0.65, zoom = 1;
    let dragging = false, lastX = 0, lastY = 0;
    let pollBusy = false, telemetryBusy = false;
    let telemetry = null, offline = null, lastOk = null, scanNote = '';
    const FETCH_TIMEOUT_MS = 2500, SCAN_STALE_S = 2;
    const eventTimes = new Map(), lastHtml = new Map();

    // ---------- formatting helpers ----------
    const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
    function fmtAge(sec) {
      if (sec == null) return 'never';
      if (sec < 1) return (sec * 1000).toFixed(0) + ' ms';
      if (sec < 60) return sec.toFixed(1) + ' s';
      if (sec < 3600) return (sec / 60).toFixed(1) + ' min';
      return (sec / 3600).toFixed(1) + ' h';
    }
    function fmtHz(hz) { return hz == null ? '—' : hz.toFixed(1) + ' Hz'; }
    function fmtNum(v, digits = 2) { return v == null ? '—' : Number(v).toFixed(digits); }
    function ageKind(sec, liveAge = 1.5, warnAge = 5) {
      if (sec == null) return 'idle';
      if (sec <= liveAge) return 'ok';
      if (sec <= warnAge) return 'warn';
      return 'alert';
    }
    function badge(text, kind) { return `<span class="badge ${kind}">${esc(text)}</span>`; }
    function kv(rows) {
      return rows.map(([k, v, cls]) =>
        `<dt>${esc(k)}</dt><dd class="${cls || ''}">${v}</dd>`).join('');
    }
    // Skip untouched DOM so scroll position and text selection survive a poll.
    function setHtml(id, html) {
      if (lastHtml.get(id) === html) return;
      lastHtml.set(id, html); $(id).innerHTML = html;
    }
    async function getJson(url) {
      try {
        const res = await fetch(url, {cache: 'no-store', signal: AbortSignal.timeout(FETCH_TIMEOUT_MS)});
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return await res.json();
      } catch (err) {
        throw new Error(err.name === 'TimeoutError' ? 'timed out' : err.message);
      }
    }
    function flameActive(f) {
      return !!f && f.bearing_rad != null && f.age != null && f.age < SCAN_STALE_S &&
        (f.state_name === 'FLAME_FOUND' || f.state_name === 'IN_SUPPRESSION_RANGE');
    }
    function fmtBearing(rad) {
      const deg = rad * 180 / Math.PI;
      return `${Math.abs(deg).toFixed(0)}° ${deg >= 0 ? 'left' : 'right'}`;
    }
    function topicKind(age) { return age == null ? 'idle' : age <= 1.5 ? 'ok' : age <= 5 ? 'warn' : 'alert'; }

    // ---------- LiDAR canvas ----------
    function resize() {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(rect.width * dpr));
      canvas.height = Math.max(1, Math.round(rect.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    }
    function dimensions() {
      const rect = canvas.getBoundingClientRect();
      return { w: rect.width, h: rect.height, cx: rect.width / 2, cy: rect.height / 2 };
    }
    function color(t) {
      const hue = 205 - 175 * Math.max(0, Math.min(1, t));
      return `hsl(${hue} 90% 62%)`;
    }
    function draw2d() {
      const {w, h, cx, cy} = dimensions();
      const radius = Number(rangeSlider.value);
      const scale = Math.min(w, h) * 0.46 / radius;
      ctx.fillStyle = '#0b0e11'; ctx.fillRect(0, 0, w, h);
      ctx.lineWidth = 1; ctx.strokeStyle = '#26313a';
      for (let r = 1; r <= radius; r++) {
        ctx.beginPath(); ctx.arc(cx, cy, r * scale, 0, Math.PI * 2); ctx.stroke();
        if (r % 2 === 0 || radius <= 8) {
          ctx.fillStyle = '#75818c'; ctx.font = '11px system-ui';
          ctx.fillText(`${r}m`, cx + 4, cy - r * scale + 13);
        }
      }
      ctx.strokeStyle = '#44515d';
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke();
      ctx.fillStyle = '#9daab6'; ctx.font = '12px system-ui';
      ctx.fillText('+x forward', cx + 7, cy - 8);
      ctx.fillText('+y left', cx + 7, cy - Math.min(h * 0.4, 50));
      if (!scan) return;
      const values = scan.ranges;
      ctx.fillStyle = scanNote ? '#3b6e5c' : '#53d6a2';
      ctx.beginPath();
      for (let i = 0; i < values.length; i++) {
        const r = values[i];
        if (r === null || !Number.isFinite(r) || r < scan.range_min || r > radius) continue;
        const angle = scan.angle_min + i * scan.angle_increment;
        const x = cx + Math.cos(angle) * r * scale;
        const y = cy - Math.sin(angle) * r * scale;
        ctx.moveTo(x + 2.2, y); ctx.arc(x, y, 2.2, 0, Math.PI * 2);
      }
      ctx.fill();
      const flame = telemetry && telemetry.flame;
      if (flameActive(flame)) {
        // FlameEvent.bearing_rad is relative to the robot, +left - same as the scan.
        const known = flame.distance_m != null && flame.distance_m <= radius;
        const reach = known ? flame.distance_m : radius;
        const fx = cx + Math.cos(flame.bearing_rad) * reach * scale;
        const fy = cy - Math.sin(flame.bearing_rad) * reach * scale;
        ctx.strokeStyle = ctx.fillStyle = '#ff6b35'; ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(fx, fy); ctx.stroke();
        ctx.setLineDash([]);
        if (known) { ctx.beginPath(); ctx.arc(fx, fy, 6, 0, Math.PI * 2); ctx.fill(); }
        ctx.font = '12px system-ui';
        ctx.fillText(known ? `flame ${flame.distance_m.toFixed(2)} m` : 'flame', fx + 10, fy + 4);
      }
      ctx.fillStyle = '#ffbf69'; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI * 2); ctx.fill();
    }
    function drawNote() {
      if (!scanNote) return;
      const {w} = dimensions();
      ctx.font = '600 13px system-ui';
      const width = ctx.measureText(scanNote).width + 20;
      ctx.fillStyle = '#33290c'; ctx.fillRect((w - width) / 2, 10, width, 26);
      ctx.fillStyle = '#ffdf9e'; ctx.textAlign = 'center';
      ctx.fillText(scanNote, w / 2, 28); ctx.textAlign = 'start';
    }
    function project(point, distance, right, up, direction, cx, cy, focal) {
      const depth = distance - (point[0]*direction[0] + point[1]*direction[1] + point[2]*direction[2]);
      if (depth <= 0.05) return null;
      return [cx + (point[0]*right[0] + point[1]*right[1] + point[2]*right[2]) * focal / depth,
              cy - (point[0]*up[0] + point[1]*up[1] + point[2]*up[2]) * focal / depth, depth];
    }
    function draw3d() {
      const {w, h, cx, cy} = dimensions();
      ctx.fillStyle = '#0b0e11'; ctx.fillRect(0, 0, w, h);
      const points = cloud && cloud.points && cloud.points.length
        ? cloud.points
        : scan ? scan.ranges.map((r, i) => {
            if (r === null || !Number.isFinite(r)) return null;
            const a = scan.angle_min + i * scan.angle_increment;
            return [Math.cos(a) * r, Math.sin(a) * r, 0];
          }).filter(Boolean) : [];
      if (!points.length) return;
      const viewRange = Number(rangeSlider.value);
      const distance = Math.max(5, viewRange * 2.8 / zoom);
      const ca = Math.cos(azimuth), sa = Math.sin(azimuth);
      const ce = Math.cos(elevation), se = Math.sin(elevation);
      const direction = [ce*ca, ce*sa, se];
      const right = [-sa, ca, 0];
      const up = [-se*ca, -se*sa, ce];
      const focal = Math.min(w, h) * 0.9;
      const gridMax = Math.min(30, Math.max(4, viewRange));
      ctx.lineWidth = 1; ctx.strokeStyle = '#26313a';
      for (let n = -gridMax; n <= gridMax; n += 1) {
        for (const axis of [0, 1]) {
          const a = axis === 0 ? [n, -gridMax, 0] : [-gridMax, n, 0];
          const b = axis === 0 ? [n, gridMax, 0] : [gridMax, n, 0];
          const pa = project(a, distance, right, up, direction, cx, cy, focal);
          const pb = project(b, distance, right, up, direction, cx, cy, focal);
          if (!pa || !pb) continue;
          ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke();
        }
      }
      const axes = [[[0,0,0],[gridMax,0,0],'#ff625c','+X'], [[0,0,0],[0,gridMax,0],'#52d273','+Y'], [[0,0,0],[0,0,gridMax],'#5ca9ff','+Z']];
      for (const [a,b,c,label] of axes) {
        const pa = project(a, distance, right, up, direction, cx, cy, focal);
        const pb = project(b, distance, right, up, direction, cx, cy, focal);
        if (!pa || !pb) continue;
        ctx.strokeStyle = c; ctx.beginPath(); ctx.moveTo(pa[0],pa[1]); ctx.lineTo(pb[0],pb[1]); ctx.stroke();
        ctx.fillStyle = c; ctx.font = '12px system-ui'; ctx.fillText(label, pb[0], pb[1]);
      }
      const drawn = [];
      for (const p of points) {
        if (!p || p.length < 3 || !p.every(Number.isFinite)) continue;
        const q = project(p, distance, right, up, direction, cx, cy, focal);
        if (q && q[0] >= 0 && q[0] <= w && q[1] >= 0 && q[1] <= h) drawn.push(q);
      }
      drawn.sort((a,b) => b[2] - a[2]);
      for (const p of drawn) {
        ctx.fillStyle = color(Math.min(1, p[2] / (distance + viewRange)));
        ctx.fillRect(p[0], p[1], 3, 3);
      }
    }
    function draw() { if (mode === '2d') draw2d(); else draw3d(); drawNote(); }

    // ---------- panels ----------
    function renderAlerts(d) {
      const items = [];
      if (offline) {
        const since = lastOk ? `since ${lastOk.toLocaleTimeString()}` : 'yet';
        items.push(['alert', 'DASHBOARD OFFLINE',
          `${esc(offline)} · no telemetry ${since} · everything below is frozen`]);
      }
      const b = d && d.beacon, e = d && d.esp32, m = d && d.mission;
      if (b && (b.fire || b.smoke) && b.age != null && b.age < 60) {
        const text = b.fire && b.smoke ? 'FIRE + SMOKE DETECTED' : b.fire ? 'FIRE DETECTED' : 'SMOKE DETECTED';
        const where = b.mac ? ` · from ${esc(b.mac)}` : '';
        const rssi = b.rssi ? ` · rssi -${b.rssi} dBm` : '';
        items.push([b.fire ? 'fire' : 'smoke', text,
          `beacon #${b.message_number}${rssi}${where} · ${fmtAge(b.age)} ago`]);
      }
      if (e && e.estop) items.push(['alert', 'E-STOP LATCHED', 'reported by the mobility ESP32']);
      if (e && e.encoder_fault) {
        items.push(['alert', 'ENCODER FAULT', 'wheel speed control is off · wheels are running open-loop']);
      }
      if (e && e.age > 3) {
        items.push(['warn', 'ESP32 STATUS STALE', `no /diagnostics from the bridge for ${fmtAge(e.age)}`]);
      } else if (e && e.link_ok === false) {
        items.push(['warn', 'ESP32 LINK DOWN', 'the mobility ESP32 reports its link to the Pi is down']);
      }
      if (m && String(m.state).toLowerCase() === 'aborted') {
        items.push(['alert', 'MISSION ABORTED', 'only a reset leaves this state']);
      }
      setHtml('alerts', items.map(([kind, text, detail]) =>
        `<div class="banner ${kind}">${esc(text)} <span class="detail">${detail}</span></div>`).join(''));
    }

    function renderBeacon(d) {
      const b = d.beacon, badges = $('beaconBadges'), kvRows = [], bar = $('beaconBar');
      $('beaconHint').textContent = b ? `${b.count} packet${b.count === 1 ? '' : 's'}` : '';
      if (!b) {
        badges.innerHTML = badge('no packets yet', 'idle');
        $('beaconKv').innerHTML = kv([
          ['Status', '<span class="empty">waiting for a beacon packet…</span>'],
          ['Path', 'beacon → ESP-NOW → mobility ESP32 → UART → bridge'],
        ]);
        bar.innerHTML = '';
        return;
      }
      badges.innerHTML =
        (b.fire ? badge('FIRE', 'fire') : badge('no fire', 'idle')) +
        (b.smoke ? badge('SMOKE', 'smoke') : badge('no smoke', 'idle')) +
        badge(ageKind(b.age) === 'ok' ? 'live' : 'stale', ageKind(b.age));
      const rssi = b.rssi || 0;
      const quality = !rssi ? {t:'unknown', k:'idle', pct:0}
        : rssi <= 55 ? {t:'strong (<2 m)', k:'ok', pct:Math.max(5, Math.min(100, (90 - rssi) / 50 * 100))}
        : rssi <= 75 ? {t:'moderate (2–6 m)', k:'warn', pct:Math.max(5, Math.min(100, (90 - rssi) / 50 * 100))}
        : {t:'weak (>6 m)', k:'alert', pct:Math.max(5, Math.min(100, (90 - rssi) / 50 * 100))};
      $('beaconKv').innerHTML = kv([
        ['Message #', esc(b.message_number), 'num'],
        ['Last seen', fmtAge(b.age), 'num'],
        ['RSSI', rssi ? `−${rssi} dBm` : 'n/a', 'num'],
        ['Range', quality.t, 'num'],
        ['Source MAC', b.mac ? `<span class="mono">${esc(b.mac)}</span>` : '—'],
      ]);
      bar.innerHTML = `<div class="bar ${quality.k}"><i style="width:${quality.pct.toFixed(0)}%"></i></div>`;
    }

    function renderEsp(d) {
      const e = d.esp32, badges = $('espBadges');
      if (!e) {
        badges.innerHTML = badge('no status yet', 'idle');
        $('espKv').innerHTML = kv([
          ['Status', '<span class="empty">no /diagnostics from the bridge</span>'],
          ['Check', 'UART to the ESP32, and that bridge.launch.py is running'],
        ]);
        $('espHint').textContent = '';
        return;
      }
      const linkKind = e.link_ok ? 'ok' : (e.age > 3 ? 'idle' : 'alert');
      badges.innerHTML =
        badge(e.link_ok ? 'link ok' : 'link down', linkKind) +
        badge(`mode: ${e.mode ?? '?'}`, e.mode === 'pi' ? 'info' : 'warn') +
        (e.estop ? badge('E-STOP LATCHED', 'alert') : badge('e-stop clear', 'idle')) +
        (e.encoder_fault ? badge('ENCODER FAULT', 'alert') : '') +
        (e.mobility_moving ? badge('moving', 'info') : badge('stopped', 'idle'));
      $('espHint').textContent = `diag ${e.level_name} · ${fmtAge(e.age)} ago`;
      $('espKv').innerHTML = kv([
        ['Mode', esc(e.mode ?? '—'), 'num'],
        ['Flags', esc(e.flags ?? '—'), 'num'],
        ['Battery', e.vbat_v == null ? 'no ADC' : `${fmtNum(e.vbat_v, 2)} V`, 'num'],
        ['Diagnostics', esc(e.message)],
      ]);
    }

    function renderMission(d) {
      const m = d.mission, badges = $('missionBadges');
      // the FSM publishes lower-case state names ("idle", "aborted", ...)
      const state = m.state ? String(m.state).toLowerCase() : null;
      const kind = !state ? 'idle'
        : state === 'aborted' ? 'alert'
        : state === 'suppressing' ? 'fire'
        : state === 'complete' ? 'ok'
        : state === 'idle' ? 'idle'
        : 'info';
      badges.innerHTML = badge(state ? state.toUpperCase() : 'no state yet', kind) +
        (m.suppress === true ? badge('SUPPRESSING', 'alert') : badge('suppressor off', 'idle'));
      $('missionHint').textContent = state ? `${fmtAge(m.age)} ago` : 'published on change';
      const rows = [['State', esc(state ?? '—')]];
      if (m.goal) {
        rows.push(['Goal', `x ${fmtNum(m.goal.x)}  y ${fmtNum(m.goal.y)} (${esc(m.goal.frame_id || '?')})`, 'num']);
        rows.push(['Goal set', fmtAge(m.goal.age), 'num']);
      } else {
        rows.push(['Goal', '<span class="empty">none</span>']);
      }
      rows.push(['Command', `v ${fmtNum(d.command.linear, 2)} m/s · ω ${fmtNum(d.command.angular, 2)} rad/s`, 'num']);
      rows.push(['Cmd seen', fmtAge(d.command.age), 'num']);
      if (d.flame) {
        rows.push(['Flame', `${esc(d.flame.state_name)} · ${fmtAge(d.flame.age)} ago`, 'num']);
        if (flameActive(d.flame)) {
          const range = d.flame.distance_m == null ? 'range unknown' : `${fmtNum(d.flame.distance_m)} m`;
          rows.push(['Flame at', `${fmtBearing(d.flame.bearing_rad)} · ${range}`, 'num']);
        }
      }
      $('missionKv').innerHTML = kv(rows);
    }

    function renderOdom(d) {
      const wheel = d.odometry.wheel, rf2o = d.odometry.rf2o;
      const badges = $('odomBadges');
      badges.innerHTML =
        badge(wheel ? `wheel ${ageKind(wheel.age) === 'ok' ? 'live' : 'stale'}` : 'wheel —',
              wheel ? ageKind(wheel.age) : 'idle') +
        badge(rf2o ? `rf2o ${ageKind(rf2o.age) === 'ok' ? 'live' : 'stale'}` : 'rf2o —',
              rf2o ? ageKind(rf2o.age) : 'idle');
      const rows = [];
      if (wheel) {
        rows.push(['Wheel vx', `${fmtNum(wheel.vx, 3)} m/s`, 'num']);
        rows.push(['Wheel ω', `${fmtNum(wheel.wz, 3)} rad/s`, 'num']);
        rows.push(['Wheel pose', `x ${fmtNum(wheel.x)}  y ${fmtNum(wheel.y)}  θ ${fmtNum(wheel.yaw)}`, 'num']);
      } else {
        rows.push(['Wheel odom', '<span class="empty">none — is the UART link up?</span>']);
      }
      if (rf2o) {
        rows.push(['Laser vx', `${fmtNum(rf2o.vx, 3)} m/s`, 'num']);
        rows.push(['Laser ω', `${fmtNum(rf2o.wz, 3)} rad/s`, 'num']);
        rows.push(['Laser pose', `x ${fmtNum(rf2o.x)}  y ${fmtNum(rf2o.y)}  θ ${fmtNum(rf2o.yaw)}`, 'num']);
      } else {
        rows.push(['Laser odom', '<span class="empty">/odom_rf2o silent</span>']);
      }
      $('odomKv').innerHTML = kv(rows);
    }

    function renderTopics(d) {
      $('topics').innerHTML = d.topics.map(t => {
        const kind = topicKind(t.age);
        const stale = kind === 'idle' || kind === 'alert';
        return `<tr class="${stale ? 'stale' : ''}">
          <td><span class="dot ${kind}"></span></td>
          <td class="mono">${esc(t.topic)}</td>
          <td class="right mono">${fmtHz(t.hz)}</td>
          <td class="right mono">${fmtAge(t.age)}</td>
        </tr>`;
      }).join('');
    }

    function renderEvents(d) {
      // Stamp each event once, in browser time, from the age the robot reported.
      const seen = new Set();
      for (const e of d.events) {
        seen.add(e.id);
        if (!eventTimes.has(e.id)) eventTimes.set(e.id, new Date(Date.now() - e.age * 1000));
      }
      for (const id of eventTimes.keys()) if (!seen.has(id)) eventTimes.delete(id);
      setHtml('events', d.events.slice().reverse().map(e => {
        const t = eventTimes.get(e.id).toLocaleTimeString();
        return `<li class="${esc(e.level)}">
          <span class="t">${esc(t)}</span>
          <span><span class="src">${esc(e.source)}</span> <span class="txt">${esc(e.text)}</span></span>
        </li>`;
      }).join('') || '<li class="empty">no events yet</li>');
    }

    async function pollTelemetry() {
      if (telemetryBusy) return;
      telemetryBusy = true;
      try {
        const d = await getJson('/api/telemetry');
        telemetry = d; offline = null; lastOk = new Date();
        document.body.classList.remove('offline');
        $('conn').className = '';
        $('conn').textContent = `updated ${lastOk.toLocaleTimeString()}`;
        renderAlerts(d); renderBeacon(d); renderEsp(d);
        renderMission(d); renderOdom(d); renderTopics(d); renderEvents(d);
      } catch (err) {
        offline = err.message;
        document.body.classList.add('offline');
        $('conn').className = 'error';
        $('conn').textContent = `telemetry error: ${err.message}`;
        renderAlerts(telemetry);
      } finally { telemetryBusy = false; }
    }

    // ---------- LiDAR polling ----------
    async function pollScan() {
      if (pollBusy) return;
      pollBusy = true;
      try {
        const data = await getJson(mode === '2d' ? '/api/scan' : '/api/cloud');
        if (mode === '2d') scan = data.scan;
        else { cloud = data.cloud; scan = data.scan; }
        const sample = mode === '3d' && cloud ? cloud : scan;
        const age = sample ? sample.age : null;
        $('scanHint').textContent = age == null ? 'no data'
          : `${sample.frame_id || '?'} · updated ${fmtAge(age)} ago`;
        scanNote = age == null ? 'waiting for LiDAR data'
          : age > SCAN_STALE_S ? `LiDAR STALE · last data ${fmtAge(age)} ago` : '';
        $('cloudInfo').textContent = mode === '3d' && cloud && cloud.points && cloud.points.length
          ? `${cloud.points.length} cloud points`
          : mode === '3d' ? 'showing planar LaserScan' : '';
        draw();
      } catch (err) {
        $('scanHint').textContent = `error: ${err.message}`;
        scanNote = 'LiDAR view frozen · dashboard offline'; draw();
      } finally { pollBusy = false; }
    }

    // ---------- wiring ----------
    $('twoD').addEventListener('click', e => {
      mode = '2d'; e.currentTarget.classList.add('active');
      $('threeD').classList.remove('active'); pollScan();
    });
    $('threeD').addEventListener('click', e => {
      mode = '3d'; e.currentTarget.classList.add('active');
      $('twoD').classList.remove('active'); pollScan();
    });
    rangeSlider.addEventListener('input', () => { rangeValue.textContent = `${rangeSlider.value} m`; draw(); });
    canvas.addEventListener('pointerdown', e => {
      dragging = true; lastX = e.clientX; lastY = e.clientY; canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener('pointermove', e => {
      if (!dragging) return;
      azimuth += (e.clientX - lastX) * 0.008;
      elevation = Math.max(-1.35, Math.min(1.35, elevation + (e.clientY - lastY) * 0.008));
      lastX = e.clientX; lastY = e.clientY;
      if (mode === '3d') draw();
    });
    canvas.addEventListener('pointerup', () => { dragging = false; });
    canvas.addEventListener('wheel', e => {
      e.preventDefault(); zoom = Math.max(0.25, Math.min(5, zoom * Math.exp(-e.deltaY * 0.001))); draw();
    }, {passive: false});
    window.addEventListener('resize', resize);

    resize(); pollScan(); pollTelemetry();
    setInterval(pollScan, 200);
    setInterval(pollTelemetry, 400);
  })();
  </script>
</body>
</html>
"""


def make_handler(node: RobotDashboard):
    favicon = Path(__file__).with_name("robot-dashboard.svg").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/":
                body = PAGE.encode("utf-8")
                content_type = "text/html; charset=utf-8"
            elif path == "/favicon.svg":
                body = favicon
                content_type = "image/svg+xml"
            elif path == "/api/scan":
                body = json.dumps({"scan": node.scan_snapshot()}, allow_nan=False).encode("utf-8")
                content_type = "application/json"
            elif path == "/api/cloud":
                body = json.dumps(
                    {"scan": node.scan_snapshot(), "cloud": node.cloud_snapshot()},
                    allow_nan=False,
                ).encode("utf-8")
                content_type = "application/json"
            elif path == "/api/telemetry":
                body = json.dumps(node.telemetry_snapshot(), allow_nan=False).encode("utf-8")
                content_type = "application/json"
            elif path == "/healthz":
                body = b"ok\n"
                content_type = "text/plain; charset=utf-8"
            else:
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def log_request(self, code="-", size="-") -> None:
            pass  # the page polls several times a second; only errors are logged

        def log_message(self, format: str, *args) -> None:
            node.get_logger().info(format % args)

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address (default: all interfaces)")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    parser.add_argument("--scan-topic", default="/scan", help="LaserScan topic (default: /scan)")
    parser.add_argument(
        "--pointcloud-topic", default="",
        help="Optional PointCloud2 topic for true 3D data (default: disabled)",
    )
    parser.add_argument(
        "--max-cloud-points", type=int, default=5000,
        help="Maximum PointCloud2 points retained for the browser (default: 5000)",
    )
    parser.add_argument("--beacon-topic", default="/beacon_event",
                        help="BeaconEvent topic (default: /beacon_event)")
    parser.add_argument("--flame-topic", default="/flame_event",
                        help="FlameEvent topic (default: /flame_event)")
    parser.add_argument("--odom-topic", default="/wheel_odom",
                        help="Wheel odometry topic (default: /wheel_odom)")
    parser.add_argument("--rf2o-topic", default="/odom_rf2o",
                        help="Laser odometry topic (default: /odom_rf2o)")
    parser.add_argument("--battery-topic", default="/battery",
                        help="BatteryState topic (default: /battery)")
    parser.add_argument("--diagnostics-topic", default="/diagnostics",
                        help="DiagnosticArray topic (default: /diagnostics)")
    parser.add_argument("--mission-state-topic", default="/firefighter_mission/state",
                        help="Mission state topic (default: /firefighter_mission/state)")
    parser.add_argument("--mission-suppress-topic", default="/firefighter_mission/suppress",
                        help="Mission suppressor topic (default: /firefighter_mission/suppress)")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel",
                        help="Commanded velocity topic (default: /cmd_vel)")
    parser.add_argument("--goal-topic", default="/firefighter_mission/goal_pose",
                        help="Navigation goal topic "
                             "(default: /firefighter_mission/goal_pose)")
    args, ros_args = parser.parse_known_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.max_cloud_points < 1:
        parser.error("--max-cloud-points must be positive")
    args.ros_args = ros_args
    return args


def main() -> None:
    args = parse_args()
    rclpy.init(args=args.ros_args)
    node = RobotDashboard(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, name="ros-spin", daemon=True)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(node))
    server.daemon_threads = True

    node.get_logger().info(f"Open http://<pi-ip>:{args.port}/ in a browser")
    try:
        ros_thread.start()
        server.serve_forever()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down robot dashboard")
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
