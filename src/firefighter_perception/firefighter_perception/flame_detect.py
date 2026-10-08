"""Flame detection from a thermal frame - **pure Python, no ROS**.

Split out of the node so the detection maths and the LiDAR bearing lookup can be
unit tested without a ROS installation (``tests/test_flame_detect.py``).

The MLX90641 is a 16x12 array on a fixed-FOV lens, so a frame tells us *where* the
hottest pixel is (a bearing) but not *how far away* it is. Range comes from the
LiDAR: we look up the scan return at the flame's bearing. That is why
:func:`scan_range_at_bearing` lives here too.
"""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import List, Optional, Sequence

ROWS = 12
COLS = 16

# FlameEvent.state values - MUST match firefighter_interfaces/msg/FlameEvent.msg
# and stay in sync with the perception node's runtime sanity check.
STATE_FOUND = 0
STATE_LOST = 1
STATE_EXTINGUISHED = 2
STATE_IN_RANGE = 3


@dataclass(frozen=True)
class FlameDetection:
    """Result of analysing one thermal frame."""

    present: bool
    row: int
    col: int
    max_temp_c: float
    mean_temp_c: float
    ambient_c: float
    bearing_rad: float      # +left of the robot, ROS convention (CCW, z up)
    hot_pixels: int

    def __repr__(self) -> str:  # pragma: no cover - debug aid
        return (
            f"FlameDetection(present={self.present}, {self.max_temp_c:.1f}C "
            f"at ({self.row},{self.col}), bearing={math.degrees(self.bearing_rad):.1f}deg)"
        )


def analyse_frame(
    frame: Sequence[Sequence[float]],
    *,
    ambient_c: Optional[float] = None,
    threshold_c: float = 60.0,
    delta_c: float = 20.0,
    fov_h_deg: float = 55.0,
) -> FlameDetection:
    """Find the hottest pixel and decide whether it is a flame.

    A pixel counts as hot if it is either above ``threshold_c`` in absolute terms
    or ``delta_c`` above ambient - whichever is the lower bar - so the detector
    still fires in a warm room where nothing reaches 60 C.

    ``ambient_c`` defaults to the frame mean when not supplied.
    """
    rows = len(frame)
    if rows == 0:
        raise ValueError("empty thermal frame")
    cols = len(frame[0])
    if cols == 0:
        raise ValueError("empty thermal frame")

    max_temp = float("-inf")
    max_r = max_c = 0
    total = 0.0
    count = 0

    for r in range(rows):
        row = frame[r]
        for c in range(cols):
            value = float(row[c])
            total += value
            count += 1
            if value > max_temp:
                max_temp, max_r, max_c = value, r, c

    mean_temp = total / count
    ambient = mean_temp if ambient_c is None else float(ambient_c)
    hot_threshold = min(float(threshold_c), ambient + float(delta_c))

    hot_pixels = 0
    for r in range(rows):
        for c in range(cols):
            if float(frame[r][c]) >= hot_threshold:
                hot_pixels += 1

    # Column 0 is the left edge of the sensor. Positive bearing is to the left
    # (ROS convention), so the mapping is mirrored.
    col_fraction = (max_c + 0.5) / cols
    bearing = (0.5 - col_fraction) * math.radians(fov_h_deg)

    return FlameDetection(
        present=max_temp >= hot_threshold,
        row=max_r,
        col=max_c,
        max_temp_c=max_temp,
        mean_temp_c=mean_temp,
        ambient_c=ambient,
        bearing_rad=bearing,
        hot_pixels=hot_pixels,
    )


def scan_range_at_bearing(
    angle_min: float,
    angle_increment: float,
    ranges: Sequence[float],
    bearing_rad: float,
    *,
    tolerance_rad: float = math.radians(6.0),
    min_range: float = 0.05,
    max_range: float = 12.0,
) -> Optional[float]:
    """Median LiDAR range near ``bearing_rad``, or None if nothing valid is there.

    Median rather than minimum so a single spurious return does not decide the
    distance to the fire.
    """
    if not ranges or not angle_increment:
        return None
    if bearing_rad < angle_min or bearing_rad > angle_min + angle_increment * (len(ranges) - 1):
        return None

    centre = int(round((bearing_rad - angle_min) / angle_increment))
    span = int(round(tolerance_rad / abs(angle_increment))) if tolerance_rad > 0 else 0
    lo = max(0, centre - span)
    hi = min(len(ranges) - 1, centre + span)

    valid: List[float] = []
    for value in ranges[lo:hi + 1]:
        if value is None:
            continue
        value = float(value)
        if math.isfinite(value) and min_range <= value <= max_range:
            valid.append(value)

    if not valid:
        return None
    return statistics.median(valid)
