"""Threaded, non-blocking UART transport for the firefighter protocol.

The port is owned by a background thread: it reads bytes, feeds them to a
:class:`~firefighter_bridge.protocol.FrameParser` and hands each decoded frame to
a callback.  Writes are lock-protected so ROS callbacks can send from the executor
thread.  If the port disappears (ESP32 reset, cable yanked) the thread keeps
retrying, so the node never needs a restart.

The transport itself does no interpretation of frames - the node does that.
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

import serial

try:  # normal, when imported as part of the installed package
    from . import protocol
    from .protocol import Frame
except ImportError:  # pragma: no cover - direct-path import for standalone tools
    import protocol
    from protocol import Frame

FrameCallback = Callable[[Frame], None]
EventCallback = Callable[[str, Optional[str]], None]


class SerialLink:
    """Background UART reader/writer speaking the firefighter protocol."""

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        on_frame: Optional[FrameCallback] = None,
        on_event: Optional[EventCallback] = None,
        reconnect_delay: float = 1.0,
        read_timeout: float = 0.05,
        logger=None,
    ) -> None:
        self._port = port
        self._baud = baud
        self._on_frame = on_frame or (lambda _frame: None)
        self._on_event = on_event or (lambda _kind, _detail: None)
        self._reconnect_delay = reconnect_delay
        self._read_timeout = read_timeout
        self._logger = logger

        self._ser: Optional[serial.Serial] = None
        self._parser = protocol.FrameParser()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._connected = False

        self.stats = {
            "bytes_rx": 0,
            "frames_rx": 0,
            "frames_tx": 0,
            "reconnects": 0,
        }

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="serial_link", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def crc_errors(self) -> int:
        return self._parser.crc_errors

    def send(self, msg_id: int, payload: bytes = b"") -> bool:
        """Encode and write one frame. Returns False if the link is down."""
        try:
            frame = protocol.encode_frame(msg_id, payload)
        except protocol.ProtocolError as exc:
            self._log("error", f"encode failed: {exc}")
            return False

        with self._write_lock:
            ser = self._ser
            if ser is None or not self._connected:
                return False
            try:
                ser.write(frame)
                self.stats["frames_tx"] += 1
                return True
            except (serial.SerialException, OSError) as exc:
                self._log("warn", f"write failed: {exc}")
                self._close()
                return False

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            if self._open():
                self._read_loop()
                self._close()
            if not self._stop.is_set():
                self._stop.wait(self._reconnect_delay)

    def _open(self) -> bool:
        try:
            self._ser = serial.Serial(
                self._port,
                self._baud,
                timeout=self._read_timeout,
                write_timeout=0.5,
            )
        except (serial.SerialException, OSError) as exc:
            self._ser = None
            self._log("warn", f"cannot open {self._port}: {exc}")
            self._stop.wait(self._reconnect_delay)
            return False

        self._parser.reset()
        self._connected = True
        self._log("info", f"connected to {self._port} @ {self._baud} baud")
        self._on_event("connected", self._port)
        return True

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._ser.read(256)
            except (serial.SerialException, OSError) as exc:
                self._log("warn", f"read failed: {exc}")
                return
            if not data:
                continue

            self.stats["bytes_rx"] += len(data)
            for frame in self._parser.feed(data):
                self.stats["frames_rx"] += 1
                try:
                    self._on_frame(frame)
                except Exception as exc:  # noqa: BLE001 - never kill the reader
                    self._log("error", f"frame handler raised: {exc}")

    def _close(self) -> None:
        was_connected = self._connected
        self._connected = False
        with self._write_lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001 - closing must never raise
                pass
        if was_connected:
            self.stats["reconnects"] += 1
            self._log("info", "serial link closed")
            self._on_event("disconnected", self._port)

    # ------------------------------------------------------------------
    def _log(self, level: str, message: str) -> None:
        if self._logger is not None:
            getattr(self._logger, level, self._logger.info)(message)
