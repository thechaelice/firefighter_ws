"""Wire protocol shared with ``firmware/robot_mobility/firefighter_protocol.h``.

Frame layout (every multi-byte field little-endian)::

    +------+------+------+--------+---------------+--------+--------+
    | 0xA5 | 0x5A | LEN  | MSG_ID | PAYLOAD (LEN) | CRC_LO | CRC_HI |
    +------+------+------+--------+---------------+--------+--------+

``CRC`` is **CRC-16/CCITT-FALSE** (poly ``0x1021``, init ``0xFFFF``, not
reflected, no final XOR) computed over ``LEN, MSG_ID, PAYLOAD`` - *not* over the
sync bytes.  That lets the ESP32 validate a frame while it is still being read,
and lets this side recompute it the same way.

Reserved sync bytes make re-synchronisation after corruption cheap: the parser
simply discards bytes until it sees ``A5 5A`` again.

This module deliberately has **no ROS and no third-party dependencies** so it can
be unit-tested without a ROS installation (see ``tests/test_protocol_codec.py``).

Keep this file and the C header in lock-step; ``tests/test_protocol_codec.py``
holds golden frames that guard against the two drifting apart.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterator, List

SYNC0 = 0xA5
SYNC1 = 0x5A
MAX_PAYLOAD = 64
FRAME_OVERHEAD = 6  # SYNC0 + SYNC1 + LEN + MSG_ID + CRC_LO + CRC_HI
MAX_FRAME = MAX_PAYLOAD + FRAME_OVERHEAD


class ProtocolError(ValueError):
    """A frame or payload could not be encoded/decoded."""


class MsgId(IntEnum):
    """Message identifiers. Pi -> ESP32 are low, ESP32 -> Pi are high."""

    # ---- Raspberry Pi -> ESP32 -------------------------------
    SET_TWIST = 0x01    # f32 linear (m/s), f32 angular (rad/s)
    ESTOP = 0x02        # (no payload) latch stop
    RESET_FAULT = 0x03  # (no payload) clear latch
    HEARTBEAT = 0x04    # u16 seq

    # ---- ESP32 -> Raspberry Pi -------------------------------
    WHEEL_ODOM = 0x81   # i32 ticksL, i32 ticksR, u32 dt_us
    STATUS = 0x82       # u16 vbat_mV, u8 flags, u8 mode
    BEACON_EVENT = 0x83  # u32 msgNum, u8 fire, u8 smoke, u8 rssi, u8 mac[6]
    ACK = 0x84          # u8 acked_msg_id
    HB_ESP = 0x85       # u16 seq


# ------------------------------------------------------------
# STATUS flag bits (mirror StatusFlags in the C header)
# ------------------------------------------------------------
FLAG_LINK_OK = 1 << 0
FLAG_ESTOP = 1 << 1
FLAG_MOVING = 1 << 2
FLAG_FALLBACK = 1 << 3

FLAG_NAMES = {
    FLAG_LINK_OK: "link_ok",
    FLAG_ESTOP: "estop",
    FLAG_MOVING: "moving",
    FLAG_FALLBACK: "fallback",
}

MODE_PI = 0
MODE_FALLBACK = 1


def describe_flags(flags: int) -> List[str]:
    """Return the human-readable names of the set bits in ``flags``."""
    return [name for bit, name in FLAG_NAMES.items() if flags & bit]


# ------------------------------------------------------------
# CRC-16/CCITT-FALSE
# Check value: crc16_ccitt(b"123456789") == 0x29B1
# ------------------------------------------------------------
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    crc &= 0xFFFF
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ------------------------------------------------------------
# Framing
# ------------------------------------------------------------
@dataclass(frozen=True)
class Frame:
    """A decoded frame."""

    msg_id: int
    payload: bytes

    @property
    def msg(self) -> "MsgId | None":
        """The ``MsgId`` for this frame, or ``None`` if it is unknown."""
        try:
            return MsgId(self.msg_id)
        except ValueError:
            return None

    def __repr__(self) -> str:  # pragma: no cover - debug aid
        name = self.msg.name if self.msg else f"0x{self.msg_id:02X}"
        return f"Frame({name}, {self.payload.hex()})"


def encode_frame(msg_id: int, payload: bytes = b"") -> bytes:
    """Serialise one complete frame."""
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([len(payload), int(msg_id)]) + payload
    return bytes([SYNC0, SYNC1]) + body + struct.pack("<H", crc16_ccitt(body))


def decode_frame(buf: bytes) -> Frame:
    """Decode exactly one complete frame (convenience for tests/tools)."""
    if len(buf) < FRAME_OVERHEAD:
        raise ProtocolError(f"short frame: {len(buf)} bytes")
    if buf[0] != SYNC0 or buf[1] != SYNC1:
        raise ProtocolError("bad sync bytes")
    length = buf[2]
    if length > MAX_PAYLOAD:
        raise ProtocolError(f"bad length field: {length}")
    if len(buf) != length + FRAME_OVERHEAD:
        raise ProtocolError(f"length mismatch: {len(buf)} != {length + FRAME_OVERHEAD}")
    body = buf[2:4 + length]
    (crc_rx,) = struct.unpack_from("<H", buf, 4 + length)
    if crc16_ccitt(body) != crc_rx:
        raise ProtocolError("CRC mismatch")
    return Frame(buf[3], bytes(buf[4:4 + length]))


class FrameParser:
    """Incremental byte-stream -> :class:`Frame` parser.

    Feed arbitrary chunks of bytes; it yields zero or more frames per chunk and
    transparently re-syncs after garbage or a bad CRC.
    """

    _S0, _S1, _LEN, _ID, _PAY, _CRCL, _CRCH = range(7)

    def __init__(self) -> None:
        self.reset()
        self.frames_ok = 0
        self.crc_errors = 0

    def reset(self) -> None:
        self._state = self._S0
        self._len = 0
        self._msg = 0
        self._buf = bytearray()
        self._crc_rx = 0

    def feed(self, data: bytes) -> Iterator[Frame]:
        for byte in bytes(data):
            frame = self._feed_byte(byte)
            if frame is not None:
                yield frame

    def _feed_byte(self, byte: int) -> "Frame | None":
        state = self._state

        if state == self._S0:
            if byte == SYNC0:
                self._state = self._S1

        elif state == self._S1:
            if byte == SYNC1:
                self._state = self._LEN
            elif byte != SYNC0:
                self._state = self._S0
            # a repeated SYNC0 keeps us here (overlapping sync run)

        elif state == self._LEN:
            if byte > MAX_PAYLOAD:
                self._state = self._S0
            else:
                self._len = byte
                self._buf.clear()
                self._state = self._ID

        elif state == self._ID:
            self._msg = byte
            self._state = self._PAY if self._len else self._CRCL

        elif state == self._PAY:
            self._buf.append(byte)
            if len(self._buf) >= self._len:
                self._state = self._CRCL

        elif state == self._CRCL:
            self._crc_rx = byte
            self._state = self._CRCH

        elif state == self._CRCH:
            self._crc_rx |= byte << 8
            self._state = self._S0
            body = bytes([self._len, self._msg]) + bytes(self._buf)
            if crc16_ccitt(body) == self._crc_rx:
                self.frames_ok += 1
                return Frame(self._msg, bytes(self._buf))
            self.crc_errors += 1

        return None


# ------------------------------------------------------------
# Decoded payload types (ESP32 -> Pi)
# ------------------------------------------------------------
@dataclass(frozen=True)
class WheelOdom:
    ticks_left: int
    ticks_right: int
    dt_us: int


@dataclass(frozen=True)
class Status:
    vbat_mv: int
    flags: int
    mode: int

    @property
    def link_ok(self) -> bool:
        return bool(self.flags & FLAG_LINK_OK)

    @property
    def estop(self) -> bool:
        return bool(self.flags & FLAG_ESTOP)

    @property
    def moving(self) -> bool:
        return bool(self.flags & FLAG_MOVING)

    @property
    def flags_set(self) -> List[str]:
        return describe_flags(self.flags)


@dataclass(frozen=True)
class BeaconEvent:
    message_number: int
    fire_detected: bool
    smoke_detected: bool
    rssi: int
    mac: bytes


# ------------------------------------------------------------
# Payload codecs - Pi -> ESP32
# ------------------------------------------------------------
def pack_set_twist(linear: float, angular: float) -> bytes:
    return struct.pack("<ff", float(linear), float(angular))


def pack_heartbeat(seq: int) -> bytes:
    return struct.pack("<H", int(seq) & 0xFFFF)


def pack_empty() -> bytes:
    return b""


# ------------------------------------------------------------
# Payload codecs - ESP32 -> Pi
# ------------------------------------------------------------
def unpack_wheel_odom(payload: bytes) -> WheelOdom:
    if len(payload) != 12:
        raise ProtocolError(f"WHEEL_ODOM payload is {len(payload)} bytes, expected 12")
    ticks_l, ticks_r, dt_us = struct.unpack("<iiI", payload)
    return WheelOdom(ticks_l, ticks_r, dt_us)


def unpack_status(payload: bytes) -> Status:
    if len(payload) != 4:
        raise ProtocolError(f"STATUS payload is {len(payload)} bytes, expected 4")
    vbat_mv, flags, mode = struct.unpack("<HBB", payload)
    return Status(vbat_mv, flags, mode)


def unpack_beacon_event(payload: bytes) -> BeaconEvent:
    if len(payload) != 13:
        raise ProtocolError(f"BEACON_EVENT payload is {len(payload)} bytes, expected 13")
    msg_num, fire, smoke, rssi = struct.unpack_from("<IBBB", payload, 0)
    return BeaconEvent(msg_num, bool(fire), bool(smoke), rssi, bytes(payload[7:13]))


def unpack_ack(payload: bytes) -> int:
    if len(payload) != 1:
        raise ProtocolError(f"ACK payload is {len(payload)} bytes, expected 1")
    return payload[0]


def unpack_hb_esp(payload: bytes) -> int:
    if len(payload) != 2:
        raise ProtocolError(f"HB_ESP payload is {len(payload)} bytes, expected 2")
    (seq,) = struct.unpack("<H", payload)
    return seq


def decode_payload(frame: Frame):
    """Decode a frame's payload into its dataclass (or ``None`` if unknown)."""
    msg = frame.msg
    if msg is MsgId.WHEEL_ODOM:
        return unpack_wheel_odom(frame.payload)
    if msg is MsgId.STATUS:
        return unpack_status(frame.payload)
    if msg is MsgId.BEACON_EVENT:
        return unpack_beacon_event(frame.payload)
    if msg is MsgId.ACK:
        return unpack_ack(frame.payload)
    if msg is MsgId.HB_ESP:
        return unpack_hb_esp(frame.payload)
    return None
