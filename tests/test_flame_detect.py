"""Thermal perception tests - pure Python, no ROS, no numpy.

    uv run --group test pytest tests/test_flame_detect.py
    python3 -m pytest tests/test_flame_detect.py
"""
from __future__ import annotations

import itertools
import math
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "src" / "firefighter_perception")
)

from firefighter_perception import flame_detect as fd  # noqa: E402
from firefighter_perception import thermal_reader as tr  # noqa: E402


def make_frame(fill: float = 25.0, hot=None, hot_value: float = 90.0, rows=fd.ROWS, cols=fd.COLS):
    frame = [[fill] * cols for _ in range(rows)]
    if hot is not None:
        r, c = hot
        frame[r][c] = hot_value
    return frame


# ------------------------------------------------------------
# analyse_frame
# ------------------------------------------------------------
def test_finds_the_hottest_pixel():
    det = fd.analyse_frame(make_frame(hot=(3, 11)))
    assert det.row == 3 and det.col == 11
    assert det.max_temp_c == pytest.approx(90.0)
    assert det.present
    assert det.hot_pixels == 1


def test_cool_frame_is_not_a_flame():
    det = fd.analyse_frame(make_frame(fill=25.0))
    assert not det.present
    assert det.hot_pixels == 0


def test_absolute_threshold_fires_in_a_warm_room():
    # ambient 50 -> delta bar would be 70, but the 60 C absolute bar is lower
    det = fd.analyse_frame(make_frame(fill=50.0, hot=(0, 0), hot_value=62.0), ambient_c=50.0)
    assert det.present
    assert det.ambient_c == pytest.approx(50.0)


def test_delta_above_ambient_fires_below_absolute_threshold():
    det = fd.analyse_frame(make_frame(fill=30.0, hot=(6, 6), hot_value=55.0))
    assert det.present  # 55 >= min(60, ~30 + 20)


def test_below_both_bars_is_not_a_flame():
    det = fd.analyse_frame(make_frame(fill=20.0, hot=(6, 6), hot_value=35.0))
    assert not det.present
    assert det.max_temp_c == pytest.approx(35.0)


def test_ambient_defaults_to_frame_mean():
    det = fd.analyse_frame(make_frame(fill=25.0))
    assert det.ambient_c == pytest.approx(25.0)
    assert det.mean_temp_c == pytest.approx(25.0)


def test_bearing_is_positive_to_the_left():
    left = fd.analyse_frame(make_frame(hot=(6, 0)))
    right = fd.analyse_frame(make_frame(hot=(6, fd.COLS - 1)))
    assert left.bearing_rad > 0.0
    assert right.bearing_rad < 0.0
    assert left.bearing_rad == pytest.approx(-right.bearing_rad)


def test_bearing_is_zero_at_the_centre():
    det = fd.analyse_frame(make_frame(hot=(6, fd.COLS // 2)))
    # the array is even-width, so the two innermost columns straddle 0
    assert abs(det.bearing_rad) <= math.radians(55.0) / fd.COLS


def test_bearing_scales_with_fov():
    narrow = fd.analyse_frame(make_frame(hot=(6, 0)), fov_h_deg=55.0)
    wide = fd.analyse_frame(make_frame(hot=(6, 0)), fov_h_deg=110.0)
    assert wide.bearing_rad == pytest.approx(2.0 * narrow.bearing_rad)


def test_hot_pixel_cluster_counted():
    frame = make_frame(fill=20.0)
    for c in range(4):
        frame[2][c] = 70.0
    det = fd.analyse_frame(frame)
    assert det.hot_pixels == 4
    assert det.present


def test_empty_frame_raises():
    with pytest.raises(ValueError):
        fd.analyse_frame([])
    with pytest.raises(ValueError):
        fd.analyse_frame([[]])


def test_state_constants_match_the_interface_contract():
    """Guards against renumbering FlameEvent.msg without updating the detector."""
    assert (fd.STATE_FOUND, fd.STATE_LOST, fd.STATE_EXTINGUISHED, fd.STATE_IN_RANGE) == (0, 1, 2, 3)


# ------------------------------------------------------------
# scan_range_at_bearing
# ------------------------------------------------------------
def _scan(angle_min=-math.pi, n=360, ranges=None):
    increment = 2.0 * math.pi / n
    return angle_min, increment, (ranges if ranges is not None else [5.0] * n)


def test_range_lookup_straight_ahead():
    angle_min, inc, ranges = _scan()
    assert fd.scan_range_at_bearing(angle_min, inc, ranges, 0.0) == pytest.approx(5.0)


def test_range_lookup_ignores_invalid_returns():
    angle_min, inc, ranges = _scan(ranges=[3.0] * 360)
    for i in range(175, 186):
        ranges[i] = float("inf")
    ranges[180] = float("nan")
    ranges[181] = 0.0  # below min_range
    ranges[182] = None
    value = fd.scan_range_at_bearing(angle_min, inc, ranges, 0.0)
    assert value is not None and math.isfinite(value) and value > 0.0


def test_range_lookup_returns_none_when_nothing_valid():
    angle_min, inc, _ = _scan()
    ranges = [float("inf")] * 360
    assert fd.scan_range_at_bearing(angle_min, inc, ranges, 0.0) is None


def test_range_lookup_outside_the_scan_returns_none():
    angle_min, inc, ranges = _scan(angle_min=-0.5, n=60)
    assert fd.scan_range_at_bearing(angle_min, inc, ranges, -1.0) is None


def test_range_lookup_uses_the_median_not_the_minimum():
    angle_min, inc, ranges = _scan(ranges=[9.0] * 360)
    # a lone near return inside the window must not drag the answer to 0.2
    centre = 180
    ranges[centre] = 0.2
    for i in range(centre - 5, centre + 6):
        if i != centre:
            ranges[i] = 4.0
    assert fd.scan_range_at_bearing(angle_min, inc, ranges, 0.0) == pytest.approx(4.0)


def test_range_lookup_handles_empty_input():
    assert fd.scan_range_at_bearing(-math.pi, 0.01, [], 0.0) is None
    assert fd.scan_range_at_bearing(-math.pi, 0.0, [1.0, 2.0], 0.0) is None


# ------------------------------------------------------------
# parse_block
# ------------------------------------------------------------
def _block(rows=12, cols=16, value="1.5"):
    return "\n".join(",".join([value] * cols) for _ in range(rows))


def test_parse_block_accepts_a_well_formed_frame():
    frame = tr.parse_block(_block())
    assert frame is not None
    assert len(frame) == 12 and len(frame[0]) == 16
    assert frame[0][0] == pytest.approx(1.5)


def test_parse_block_tolerates_surrounding_blank_lines():
    assert tr.parse_block("\n" + _block() + "\n\n") is not None


def test_parse_block_rejects_wrong_shape_or_junk():
    assert tr.parse_block(_block(rows=11)) is None
    assert tr.parse_block(_block(cols=15)) is None
    assert tr.parse_block("\n".join(["a,b,c"] * 12)) is None
    assert tr.parse_block("") is None


# ------------------------------------------------------------
# ThermalReader
# ------------------------------------------------------------
FAKE_READER = """#!/usr/bin/env python3
import sys
for f in range(2):
    for r in range(12):
        print(",".join(f"{f * 10 + r * 0.1:.2f}" for _ in range(16)))
    print()
"""


def _fake_reader(tmp_path: Path) -> Path:
    script = tmp_path / "fake_reader"
    script.write_text(FAKE_READER)
    os.chmod(script, 0o755)
    return script


def test_reader_argv_shape():
    reader = tr.ThermalReader("/bin/true", delay_ms=250, frames=7)
    assert reader.argv() == ["/bin/true", "7", "250"]


def test_reader_streams_parsed_frames_from_the_child(tmp_path):
    reader = tr.ThermalReader(str(_fake_reader(tmp_path)), delay_ms=1, restart_delay=60.0)
    stream = reader.frames()
    try:
        frames = list(itertools.islice(stream, 2))
    finally:
        reader.stop()
        stream.close()

    assert len(frames) == 2
    assert len(frames[0]) == 12 and len(frames[0][0]) == 16
    assert frames[0][0][0] == pytest.approx(0.0)
    assert frames[1][0][0] == pytest.approx(10.0)


def test_stop_is_safe_when_the_reader_never_ran():
    tr.ThermalReader("/nonexistent/reader").stop()  # must not raise
