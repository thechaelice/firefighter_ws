#!/usr/bin/env python3
"""Serve a browser-based 2D/3D ROS 2 LiDAR viewer."""

from __future__ import annotations

import argparse
import json
import math
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" type="image/svg+xml" href="/favicon.svg">
  <title>LiDAR Viewer</title>
  <style>
    :root { color-scheme: dark; font: 14px system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; background: #101418; color: #e6edf3; }
    header { display: flex; flex-wrap: wrap; align-items: center; gap: 14px; padding: 12px 18px; background: #171d23; border-bottom: 1px solid #303840; }
    h1 { font-size: 17px; margin: 0 auto 0 0; }
    #status { color: #9daab6; }
    #status.error { color: #ff7b72; }
    nav { display: flex; gap: 6px; }
    button { color: inherit; background: #222a32; border: 1px solid #3a4651; border-radius: 5px; padding: 7px 12px; cursor: pointer; }
    button.active { background: #176b87; border-color: #2796b8; }
    main { padding: 14px; max-width: 1200px; margin: auto; }
    .toolbar { display: flex; flex-wrap: wrap; align-items: center; gap: 16px; min-height: 34px; color: #aab6c2; }
    .toolbar label { display: flex; align-items: center; gap: 8px; }
    input[type=range] { width: 130px; }
    canvas { display: block; width: 100%; height: min(72vh, 760px); min-height: 320px; background: #0b0e11; border: 1px solid #303840; border-radius: 6px; touch-action: none; }
    #help { color: #8996a2; font-size: 12px; padding: 9px 2px; }
    [hidden] { display: none !important; }
  </style>
</head>
<body>
  <header>
    <h1>LiDAR Viewer</h1>
    <span id="status">Connecting…</span>
    <nav><button id="twoD" class="active">2D scan</button><button id="threeD">3D view</button></nav>
  </header>
  <main>
    <div class="toolbar">
      <label>Range <input id="range" type="range" min="2" max="30" step="1" value="8"><span id="rangeValue">8 m</span></label>
      <span id="frame"></span>
      <span id="cloudInfo"></span>
    </div>
    <canvas id="view" aria-label="LiDAR scan visualization"></canvas>
    <div id="help">ROS axes: x forward, y left. 3D view: drag to orbit, scroll to zoom. A LaserScan is planar; configure a PointCloud2 topic for volumetric data.</div>
  </main>
  <script>
  (() => {
    const canvas = document.getElementById('view');
    const ctx = canvas.getContext('2d');
    const status = document.getElementById('status');
    const frame = document.getElementById('frame');
    const cloudInfo = document.getElementById('cloudInfo');
    const rangeSlider = document.getElementById('range');
    const rangeValue = document.getElementById('rangeValue');
    let mode = '2d', scan = null, cloud = null, azimuth = -0.7, elevation = 0.65, zoom = 1;
    let dragging = false, lastX = 0, lastY = 0, pollBusy = false;

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
          ctx.fillStyle = '#75818c'; ctx.font = '11px system-ui'; ctx.fillText(`${r}m`, cx + 4, cy - r * scale + 13);
        }
      }
      ctx.strokeStyle = '#44515d';
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke();
      ctx.fillStyle = '#9daab6'; ctx.font = '12px system-ui';
      ctx.fillText('+x forward', cx + 7, cy - 8); ctx.fillText('+y left', cx + 7, cy - Math.min(h * 0.4, 50));
      if (!scan) return;
      const values = scan.ranges;
      ctx.fillStyle = '#53d6a2';
      for (let i = 0; i < values.length; i++) {
        const r = values[i];
        if (r === null || !Number.isFinite(r) || r < scan.range_min || r > radius) continue;
        const angle = scan.angle_min + i * scan.angle_increment;
        const x = cx + Math.cos(angle) * r * scale;
        const y = cy - Math.sin(angle) * r * scale;
        ctx.beginPath(); ctx.arc(x, y, 2.2, 0, Math.PI * 2); ctx.fill();
      }
      ctx.fillStyle = '#ffbf69'; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI * 2); ctx.fill();
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
    function draw() { if (mode === '2d') draw2d(); else draw3d(); }
    async function poll() {
      if (pollBusy) return;
      pollBusy = true;
      try {
        const response = await fetch(mode === '2d' ? '/api/scan' : '/api/cloud', {cache: 'no-store'});
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const data = await response.json();
        if (mode === '2d') scan = data.scan;
        else { cloud = data.cloud; scan = data.scan; }
        const sample = mode === '3d' && cloud && cloud.received_at ? cloud : scan;
        const now = Date.now() / 1000;
        const stamp = sample && sample.received_at;
        const age = stamp ? now - stamp : Infinity;
        status.className = age > 1.5 ? 'error' : '';
        status.textContent = age === Infinity ? 'Waiting for LiDAR data' : age > 1.5 ? `No recent data · ${age.toFixed(1)} s old` : `Live · ${age.toFixed(2)} s`;
        frame.textContent = sample && sample.frame_id ? `Frame: ${sample.frame_id}` : '';
        cloudInfo.textContent = mode === '3d' && cloud && cloud.points && cloud.points.length ? `${cloud.points.length} cloud points` :
          mode === '3d' ? 'Showing planar LaserScan' : '';
        draw();
      } catch (error) {
        status.className = 'error'; status.textContent = `Viewer error: ${error.message}`;
      } finally { pollBusy = false; }
    }
    document.getElementById('twoD').addEventListener('click', e => {
      mode = '2d'; e.currentTarget.classList.add('active'); document.getElementById('threeD').classList.remove('active'); poll();
    });
    document.getElementById('threeD').addEventListener('click', e => {
      mode = '3d'; e.currentTarget.classList.add('active'); document.getElementById('twoD').classList.remove('active'); poll();
    });
    rangeSlider.addEventListener('input', () => { rangeValue.textContent = `${rangeSlider.value} m`; draw(); });
    canvas.addEventListener('pointerdown', e => { dragging = true; lastX = e.clientX; lastY = e.clientY; canvas.setPointerCapture(e.pointerId); });
    canvas.addEventListener('pointermove', e => {
      if (!dragging) return;
      azimuth += (e.clientX - lastX) * 0.008;
      elevation = Math.max(-1.35, Math.min(1.35, elevation + (e.clientY - lastY) * 0.008));
      lastX = e.clientX; lastY = e.clientY; if (mode === '3d') draw();
    });
    canvas.addEventListener('pointerup', () => { dragging = false; });
    canvas.addEventListener('wheel', e => { e.preventDefault(); zoom = Math.max(0.25, Math.min(5, zoom * Math.exp(-e.deltaY * 0.001))); draw(); }, {passive: false});
    window.addEventListener('resize', resize);
    resize(); poll(); setInterval(poll, 200);
  })();
  </script>
</body>
</html>
"""


class LidarViewerNode(Node):
    """Keep the latest scan and an optional sampled point cloud."""

    def __init__(self, scan_topic: str, pointcloud_topic: str, max_cloud_points: int):
        super().__init__("lidar_web_viewer")
        self._lock = threading.Lock()
        self._scan: dict | None = None
        self._cloud: dict | None = None
        self._max_cloud_points = max_cloud_points
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(f"Listening for LaserScan on {scan_topic}")
        if pointcloud_topic:
            self.create_subscription(
                PointCloud2, pointcloud_topic, self._on_cloud, qos_profile_sensor_data)
            self.get_logger().info(f"Listening for PointCloud2 on {pointcloud_topic}")

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = [
            float(value) if math.isfinite(value) else None
            for value in msg.ranges
        ]
        with self._lock:
            self._scan = {
                "frame_id": msg.header.frame_id,
                "received_at": time.time(),
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
                "received_at": time.time(),
                "points": points,
            }

    def scan_snapshot(self) -> dict | None:
        with self._lock:
            return self._scan

    def cloud_snapshot(self) -> dict | None:
        with self._lock:
            return self._cloud


def make_handler(node: LidarViewerNode):
    favicon = Path(__file__).with_name("lidar-viewer.svg").read_bytes()

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

        def log_message(self, format: str, *args) -> None:
            node.get_logger().info(format % args)

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
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
    node = LidarViewerNode(args.scan_topic, args.pointcloud_topic, args.max_cloud_points)
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
        node.get_logger().info("Shutting down LiDAR web viewer")
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
