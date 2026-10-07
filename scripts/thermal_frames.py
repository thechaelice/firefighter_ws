#!/usr/bin/env python3
"""Capture thermal frames from the Melexis MLX90641 and render them as PNGs.

Usage:
    uv run scripts/thermal_frames.py                    # 3 frames -> docs/images
    uv run scripts/thermal_frames.py -n 4 -d 1000
    uv run scripts/thermal_frames.py -n 3 -o /tmp/frames

The sensor is a 16x12 (192 px) MLX90641 on I2C 0x33. There is no MLX90641 driver
on PyPI, so frames are read by the small C reader built by
``bash scripts/thermal/build.sh``. See docs/thermal-camera.md.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[1]
READER = REPO / "scripts" / "thermal" / "mlx90641_frames"
ROWS, COLS = 12, 16
_TAG = re.compile(r"frame (\d+):\s*Vdd=([\d.]+)\s*Ta=([\d.]+)\s*subpage=(\d+)")


def capture(n_frames: int, delay_ms: int) -> tuple[list[np.ndarray], list[dict]]:
    """Run the C reader and parse its CSV stdout + diagnostics stderr."""
    if not READER.exists():
        sys.exit(f"reader not found: {READER}\nbuild it first: bash scripts/thermal/build.sh")

    proc = subprocess.run(
        [str(READER), str(n_frames), str(delay_ms)],
        capture_output=True, text=True, check=True,
    )

    meta: list[dict] = []
    for m in _TAG.finditer(proc.stderr):
        meta.append({"idx": int(m[1]), "vdd": float(m[2]),
                     "ta": float(m[3]), "subpage": int(m[4])})

    frames: list[np.ndarray] = []
    for block in proc.stdout.strip().split("\n\n"):
        if not block.strip():
            continue
        rows = [[float(v) for v in line.split(",")] for line in block.strip().splitlines()]
        arr = np.array(rows, dtype=float)
        if arr.shape == (ROWS, COLS):
            frames.append(arr)

    if not frames:
        sys.exit(f"no frames captured.\nstderr:\n{proc.stderr}")
    return frames, meta


def render(frames: list[np.ndarray], meta: list[dict], outdir: Path,
           title: str = "MLX90641") -> list[Path]:
    outdir.mkdir(parents=True, exist_ok=True)
    vmin = float(min(f.min() for f in frames))
    vmax = float(max(f.max() for f in frames))
    written: list[Path] = []

    def _ta(k: int) -> str:
        return f"{meta[k]['ta']:.1f} °C" if k < len(meta) else ""

    # --- one PNG per frame ---------------------------------------------------
    for i, f in enumerate(frames, 1):
        fig, ax = plt.subplots(figsize=(5.6, 4.6), dpi=130, constrained_layout=True)
        im = ax.imshow(f, cmap="inferno", vmin=vmin, vmax=vmax, interpolation="nearest")
        ax.set_title(f"{title} — frame {i}\nTa={_ta(i - 1)}   ({f.min():.1f}–{f.max():.1f} °C)",
                     fontsize=10)
        ax.set_xlabel("column")
        ax.set_ylabel("row")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03, label="°C")
        p = outdir / f"thermal-frame-{i:02d}.png"
        fig.savefig(p)
        plt.close(fig)
        written.append(p)

    # --- montage -------------------------------------------------------------
    n = len(frames)
    ncols = min(n, 3)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, dpi=130, squeeze=False,
                             figsize=(4.1 * ncols, 3.5 * nrows + 0.7),
                             constrained_layout=True)
    flat = axes.ravel()
    im = None
    for k, ax in enumerate(flat):
        if k < n:
            im = ax.imshow(frames[k], cmap="inferno", vmin=vmin, vmax=vmax,
                           interpolation="nearest")
            ax.set_title(f"frame {k + 1}   Ta={_ta(k)}", fontsize=9)
            ax.set_xticks([])
            ax.set_yticks([])
        else:
            ax.axis("off")
    fig.suptitle(f"{title} — {n} live frames, {COLS}×{ROWS} IR array "
                 f"({vmin:.1f}–{vmax:.1f} °C)", fontsize=12)
    if im is not None:
        fig.colorbar(im, ax=flat.tolist(), fraction=0.025, pad=0.02, label="°C")
    p = outdir / "thermal-frames.png"
    fig.savefig(p)
    plt.close(fig)
    written.append(p)

    return written


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--frames", type=int, default=3, help="number of frames (default 3)")
    ap.add_argument("-d", "--delay-ms", type=int, default=800, help="delay between frames (ms)")
    ap.add_argument("-o", "--outdir", type=Path, default=REPO / "docs" / "images",
                    help="output directory (default docs/images)")
    args = ap.parse_args()

    frames, meta = capture(args.frames, args.delay_ms)
    for k, f in enumerate(frames):
        ta = f"{meta[k]['ta']:.2f} °C" if k < len(meta) else "?"
        print(f"frame {k + 1}: {f.min():.2f} .. {f.max():.2f} °C  (Ta={ta})")

    for p in render(frames, meta, args.outdir):
        print(f"wrote {p.relative_to(REPO) if p.is_relative_to(REPO) else p}")


if __name__ == "__main__":
    main()
