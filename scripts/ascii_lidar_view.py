#!/usr/bin/env python3
"""Live ASCII LiDAR viewer (headless).

Subscribes to /scan and renders a live polar "radar" view in the terminal.
Run after sourcing the workspace:  python3 scripts/ascii_lidar_view.py
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class AsciiLidarView(Node):
    def __init__(self):
        super().__init__('ascii_lidar_view')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('size', 41)
        self.declare_parameter('rate', 10.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.max_range = self.get_parameter('max_range').value
        self.size = int(self.get_parameter('size').value)
        self.rate = self.get_parameter('rate').value

        if self.size % 2 == 0:
            self.size += 1  # keep an odd size so there is a true centre cell

        self.last_scan = None
        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self._cb, 1)
        # Render on a timer; the subscription just keeps the latest scan.
        self.timer = self.create_timer(1.0 / self.rate, self._render_cb)

    def _cb(self, msg):
        self.last_scan = msg

    def _render_cb(self):
        self.render()

    def render(self):
        msg = self.last_scan
        if msg is None:
            sys.stdout.write('Waiting for scan on %s...\n' % self.scan_topic)
            sys.stdout.flush()
            return

        size = self.size
        centre = size // 2
        scale = (size // 2 - 1) / self.max_range
        grid = [[' '] * size for _ in range(size)]

        # cross-hair axes
        for i in range(size):
            grid[centre][i] = '.'
            grid[i][centre] = '.'
        grid[centre][centre] = '@'

        angle_min = msg.angle_min
        step = msg.angle_increment if msg.angle_increment > 0 else 0.01

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r > self.max_range:
                r = self.max_range
            ang = angle_min + i * step
            # ROS x forward, y left, CCW positive. Screen: +x right, +y up.
            col = centre + int(round(r * math.cos(ang) * scale))
            row = centre - int(round(r * math.sin(ang) * scale))
            if 0 <= col < size and 0 <= row < size:
                grid[row][col] = '#'

        border = '+' + '-' * size + '+'
        out = ['\033[H\033[2J']
        out.append(
            'LiDAR: %s | %d rays | view radius %.1f m | Ctrl+C to quit\n'
            % (msg.header.frame_id, len(msg.ranges), self.max_range))
        out.append(border + '\n')
        for row in grid:
            out.append('|' + ''.join(row) + '|\n')
        out.append(border + '\n')
        sys.stdout.write(''.join(out))
        sys.stdout.flush()


def main():
    rclpy.init()
    node = AsciiLidarView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
