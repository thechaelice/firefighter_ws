"""Streaming reader for the vendored MLX90641 CSV tool - **pure Python, no ROS**.

There is no MLX90641 driver on PyPI (see ``docs/thermal-camera.md``), so frames
come from ``scripts/thermal/mlx90641_frames``, which prints a 12x16 block of
comma-separated temperatures per frame and then exits after N frames. We run it
with a very large frame count and restart it if it dies, which turns it into a
continuous stream without touching the C++.
"""
from __future__ import annotations

import subprocess
import threading
import time
from typing import Iterator, List, Optional, Sequence

ROWS = 12
COLS = 16

#: how many frames to ask the reader for; effectively "run until killed"
STREAM_FOREVER = 1_000_000_000


def parse_block(text: str, rows: int = ROWS, cols: int = COLS) -> Optional[List[List[float]]]:
    """Parse one CSV block into a 2D list, or None if it is malformed."""
    lines = [line for line in text.strip().splitlines() if line.strip()]
    if len(lines) != rows:
        return None
    try:
        out: List[List[float]] = []
        for line in lines:
            values = [float(v) for v in line.split(",")]
            if len(values) != cols:
                return None
            out.append(values)
        return out
    except ValueError:
        return None


class ThermalReader:
    """Runs the MLX90641 CSV tool as a child process and yields frames."""

    def __init__(
        self,
        reader_path: str,
        *,
        rows: int = ROWS,
        cols: int = COLS,
        delay_ms: int = 200,
        frames: int = STREAM_FOREVER,
        restart_delay: float = 2.0,
    ) -> None:
        self.reader_path = str(reader_path)
        self.rows = rows
        self.cols = cols
        self.delay_ms = int(delay_ms)
        self.frame_count = int(frames)
        self.restart_delay = float(restart_delay)

        self._stop = threading.Event()
        self._proc: Optional[subprocess.Popen] = None
        self.restarts = 0

    def argv(self) -> List[str]:
        return [self.reader_path, str(self.frame_count), str(self.delay_ms)]

    def stop(self) -> None:
        self._stop.set()
        proc = self._proc
        if proc is not None and proc.poll() is None:
            proc.terminate()

    def frames(self) -> Iterator[List[List[float]]]:
        """Yield thermal frames until :meth:`stop` is called.

        Diagnostics from the tool go to stderr, which is discarded - leaving that
        pipe unread would eventually block the child.
        """
        while not self._stop.is_set():
            try:
                self._proc = subprocess.Popen(
                    self.argv(),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    bufsize=1,
                )
            except OSError:
                # reader missing / not executable - wait and retry
                if self._stop.wait(self.restart_delay):
                    return
                continue

            block: List[str] = []
            try:
                assert self._proc.stdout is not None
                for line in self._proc.stdout:
                    if self._stop.is_set():
                        return
                    line = line.strip()
                    if not line:
                        frame = parse_block("\n".join(block), self.rows, self.cols)
                        block = []
                        if frame is not None:
                            yield frame
                        continue
                    block.append(line)
            finally:
                proc, self._proc = self._proc, None
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()

            if self._stop.is_set():
                return

            self.restarts += 1
            # the tool exited on its own: back off briefly, then start it again
            if self._stop.wait(self.restart_delay):
                return


def capture_once(
    reader_path: str,
    frames: int = 1,
    delay_ms: int = 800,
    *,
    rows: int = ROWS,
    cols: int = COLS,
    timeout: float = 30.0,
) -> Optional[List[List[float]]]:
    """Run the reader for a fixed number of frames and return the first one."""
    try:
        proc = subprocess.run(
            [str(reader_path), str(frames), str(delay_ms)],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None

    for chunk in proc.stdout.strip().split("\n\n"):
        frame = parse_block(chunk, rows, cols)
        if frame is not None:
            return frame
    return None
