"""Protocol codec tests - golden frames shared with the ESP32 C header.

Deliberately standalone: pure Python, no ROS, no rclpy, no pyserial.

    uv run --group test pytest tests/test_protocol_codec.py
    python3 -m pytest tests/test_protocol_codec.py

The golden byte strings were produced by the reference implementation and must
match what ``firmware/robot_mobility/firefighter_protocol.h`` puts on the wire for
the same inputs.  If you change the framing, change both sides and re-bless these.
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import pytest

# Make the un-built ament_python package importable without a colcon build.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "firefighter_bridge"))

from firefighter_bridge import protocol as p  # noqa: E402


# ------------------------------------------------------------
# Golden frames: (MsgId, payload bytes, exact bytes on the wire)
# ------------------------------------------------------------
GOLDEN = [
    (p.MsgId.SET_TWIST, p.pack_set_twist(0.1, 0.0), "a55a0801cdcccc3d000000005da1"),
    (p.MsgId.ESTOP, p.pack_empty(), "a55a00024d3d"),
    (p.MsgId.HEARTBEAT, p.pack_heartbeat(1), "a55a020401005986"),
    (
        p.MsgId.WHEEL_ODOM,
        struct.pack("<iiI", 100, -50, 20000),
        "a55a0c8164000000ceffffff204e000090fe",
    ),
    (p.MsgId.STATUS, struct.pack("<HBB", 12000, 0x05, 0), "a55a0482e02e05003a88"),
    (
        p.MsgId.BEACON_EVENT,
        struct.pack("<IBBB", 7, 1, 1, 0) + bytes(range(1, 7)),
        "a55a0d8307000000010100010203040506e139",
    ),
    (p.MsgId.ACK, bytes([0x01]), "a55a018401d13c"),
    (p.MsgId.HB_ESP, struct.pack("<H", 9), "a55a028509009a03"),
]

GOLDEN_IDS = [case[0].name for case in GOLDEN]


# ------------------------------------------------------------
# CRC
# ------------------------------------------------------------
def test_crc16_ccitt_known_check_value():
    """CRC-16/CCITT-FALSE of "123456789" is the standard 0x29B1."""
    assert p.crc16_ccitt(b"123456789") == 0x29B1


def test_crc16_ccitt_of_empty_is_init_value():
    assert p.crc16_ccitt(b"") == 0xFFFF


def test_crc16_ccitt_never_exceeds_16_bits():
    assert 0 <= p.crc16_ccitt(bytes(range(256))) <= 0xFFFF


# ------------------------------------------------------------
# Framing
# ------------------------------------------------------------
@pytest.mark.parametrize("msg_id,payload,expected_hex", GOLDEN, ids=GOLDEN_IDS)
def test_encode_matches_golden_bytes(msg_id, payload, expected_hex):
    assert p.encode_frame(msg_id, payload).hex() == expected_hex


@pytest.mark.parametrize("msg_id,payload,expected_hex", GOLDEN, ids=GOLDEN_IDS)
def test_decode_matches_golden_bytes(msg_id, payload, expected_hex):
    frame = p.decode_frame(bytes.fromhex(expected_hex))
    assert frame.msg is msg_id
    assert frame.payload == payload


def test_frame_starts_with_sync_bytes():
    frame = p.encode_frame(p.MsgId.ESTOP)
    assert frame[0] == p.SYNC0 and frame[1] == p.SYNC1


def test_frame_length_field_matches_payload():
    frame = p.encode_frame(p.MsgId.SET_TWIST, b"\x00" * 8)
    assert frame[2] == 8
    assert len(frame) == 8 + p.FRAME_OVERHEAD


def test_encode_rejects_oversized_payload():
    with pytest.raises(p.ProtocolError):
        p.encode_frame(p.MsgId.SET_TWIST, b"\x00" * (p.MAX_PAYLOAD + 1))


def test_encode_accepts_max_payload():
    frame = p.encode_frame(p.MsgId.SET_TWIST, b"\x00" * p.MAX_PAYLOAD)
    assert p.decode_frame(frame).payload == b"\x00" * p.MAX_PAYLOAD


# ------------------------------------------------------------
# Parser
# ------------------------------------------------------------
def test_parser_decodes_a_whole_batch():
    stream = b"".join(p.encode_frame(mid, payload) for mid, payload, _ in GOLDEN)
    got = list(p.FrameParser().feed(stream))
    assert [(f.msg_id, f.payload) for f in got] == [
        (int(mid), payload) for mid, payload, _ in GOLDEN
    ]


def test_parser_works_byte_at_a_time():
    parser = p.FrameParser()
    frame = p.encode_frame(p.MsgId.HEARTBEAT, p.pack_heartbeat(42))
    out = []
    for byte in frame:
        out.extend(parser.feed(bytes([byte])))
    assert len(out) == 1
    assert out[0].payload == p.pack_heartbeat(42)


def test_parser_resyncs_after_garbage():
    parser = p.FrameParser()
    good = p.encode_frame(p.MsgId.ESTOP)
    out = list(parser.feed(b"\x01\x02\xa5\x99\x03" + good))
    assert len(out) == 1
    assert out[0].msg is p.MsgId.ESTOP


def test_parser_handles_split_sync_run():
    """A run of 0xA5 bytes must not confuse the sync detector."""
    parser = p.FrameParser()
    good = p.encode_frame(p.MsgId.ESTOP)
    out = list(parser.feed(b"\xa5\xa5\xa5" + good))
    assert len(out) == 1
    assert out[0].msg is p.MsgId.ESTOP


def test_parser_rejects_bad_crc():
    parser = p.FrameParser()
    corrupted = bytearray(p.encode_frame(p.MsgId.ESTOP))
    corrupted[-1] ^= 0xFF
    assert list(parser.feed(bytes(corrupted))) == []
    assert parser.crc_errors == 1
    assert parser.frames_ok == 0


def test_parser_recovers_after_bad_crc():
    parser = p.FrameParser()
    corrupted = bytearray(p.encode_frame(p.MsgId.ESTOP))
    corrupted[-1] ^= 0xFF
    good = p.encode_frame(p.MsgId.HEARTBEAT, p.pack_heartbeat(3))
    out = list(parser.feed(bytes(corrupted) + good))
    assert len(out) == 1
    assert out[0].payload == p.pack_heartbeat(3)


def test_decode_frame_rejects_malformed_input():
    with pytest.raises(p.ProtocolError):
        p.decode_frame(b"\xa5\x5a")  # too short
    with pytest.raises(p.ProtocolError):
        p.decode_frame(b"\x00\x00\x00\x01\x00\x00")  # bad sync
    good = p.encode_frame(p.MsgId.ESTOP)
    with pytest.raises(p.ProtocolError):
        p.decode_frame(good + b"\x00")  # trailing byte -> length mismatch


def test_unknown_msg_id_is_preserved_but_not_recognised():
    frame = p.decode_frame(p.encode_frame(0x7F, b"\x01\x02"))
    assert frame.msg_id == 0x7F
    assert frame.msg is None
    assert p.decode_payload(frame) is None


# ------------------------------------------------------------
# Payload codecs (ESP32 -> Pi)
# ------------------------------------------------------------
def test_unpack_wheel_odom():
    odom = p.unpack_wheel_odom(struct.pack("<iiI", 100, -50, 20000))
    assert (odom.ticks_left, odom.ticks_right, odom.dt_us) == (100, -50, 20000)


def test_unpack_wheel_odom_handles_negative_ticks():
    odom = p.unpack_wheel_odom(struct.pack("<iiI", -1, -2, 3))
    assert (odom.ticks_left, odom.ticks_right) == (-1, -2)


def test_unpack_status_flags_and_mode():
    status = p.unpack_status(
        struct.pack("<HBB", 12000, p.FLAG_LINK_OK | p.FLAG_MOVING, p.MODE_PI)
    )
    assert status.vbat_mv == 12000
    assert status.link_ok and status.moving
    assert not status.estop
    assert status.mode == p.MODE_PI
    assert status.flags_set == ["link_ok", "moving"]


def test_unpack_status_encoder_fault_flag():
    clear = p.unpack_status(struct.pack("<HBB", 0, p.FLAG_LINK_OK, p.MODE_PI))
    assert not clear.encoder_fault

    faulted = p.unpack_status(
        struct.pack("<HBB", 0, p.FLAG_LINK_OK | p.FLAG_ENC_FAULT, p.MODE_PI)
    )
    assert faulted.encoder_fault
    assert faulted.flags_set == ["link_ok", "encoder_fault"]


def test_unpack_beacon_event():
    event = p.unpack_beacon_event(
        struct.pack("<IBBB", 7, 1, 1, 0) + bytes(range(1, 7))
    )
    assert event.message_number == 7
    assert event.fire_detected and event.smoke_detected
    assert event.mac == bytes(range(1, 7))


def test_unpack_ack_and_hb():
    assert p.unpack_ack(b"\x81") == 0x81
    assert p.unpack_hb_esp(struct.pack("<H", 65535)) == 65535


@pytest.mark.parametrize(
    "func,payload",
    [
        (p.unpack_wheel_odom, b"\x00" * 8),
        (p.unpack_status, b"\x00" * 3),
        (p.unpack_beacon_event, b"\x00" * 12),
        (p.unpack_ack, b"\x00" * 2),
        (p.unpack_hb_esp, b"\x00" * 3),
    ],
)
def test_payload_length_guard(func, payload):
    with pytest.raises(p.ProtocolError):
        func(payload)


def test_decode_payload_dispatch():
    frame = p.decode_frame(p.encode_frame(p.MsgId.WHEEL_ODOM, struct.pack("<iiI", 1, 2, 3)))
    assert p.decode_payload(frame) == p.WheelOdom(1, 2, 3)
