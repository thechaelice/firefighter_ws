// ============================================================
// Firefighter Robot - Pi <-> Mobility ESP32 serial protocol
// ------------------------------------------------------------
// Shared wire-format definition. The Raspberry Pi bridge node
// MUST implement the exact same framing (see the matching
// Python codec in the bridge package).
//
// WIRE FORMAT (little-endian for all multi-byte fields)
//
//   +------+------+------+--------+---------------+--------+--------+
//   | 0xA5 | 0x5A | LEN  | MSG_ID | PAYLOAD (LEN) | CRC_LO | CRC_HI |
//   | sync | sync | u8   | u8     | 0..64 bytes   |  u16 little    |
//   +------+------+------+--------+---------------+--------+--------+
//
//   CRC = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect)
//         computed over  LEN, MSG_ID, PAYLOAD  (NOT the sync bytes).
//
// Reserved sync bytes make re-synchronisation after corruption cheap:
// the parser simply drops bytes until it sees A5 5A again.
// ============================================================

#pragma once

#include <Arduino.h>
#include <string.h>

namespace ff {

// ------------------------------------------------------------
// Framing constants
// ------------------------------------------------------------
constexpr uint8_t SYNC0 = 0xA5;
constexpr uint8_t SYNC1 = 0x5A;
constexpr uint8_t MAX_PAYLOAD = 64;

// ------------------------------------------------------------
// Message IDs
// ------------------------------------------------------------
enum MsgId : uint8_t {
  // ---- Raspberry Pi -> ESP32 -------------------------------
  MSG_SET_TWIST    = 0x01,  // f32 linear (m/s), f32 angular (rad/s)
  MSG_ESTOP        = 0x02,  // (no payload) latch stop
  MSG_RESET_FAULT  = 0x03,  // (no payload) clear latch
  MSG_HEARTBEAT    = 0x04,  // u16 seq

  // ---- ESP32 -> Raspberry Pi -------------------------------
  MSG_WHEEL_ODOM   = 0x81,  // i32 ticksL, i32 ticksR, u32 dt_us
  MSG_STATUS       = 0x82,  // u16 vbat_mV, u8 flags, u8 mode
  MSG_BEACON_EVENT = 0x83,  // u32 msgNum, u8 fire, u8 smoke, u8 rssi, u8 mac[6]
  MSG_ACK          = 0x84,  // u8 acked_msg_id
  MSG_HB_ESP       = 0x85,  // u16 seq
};

// ------------------------------------------------------------
// STATUS flag bits
// ------------------------------------------------------------
enum StatusFlags : uint8_t {
  FLAG_LINK_OK  = 1 << 0,  // a valid Pi frame arrived within the watchdog window
  FLAG_ESTOP    = 1 << 1,  // stop latched by MSG_ESTOP
  FLAG_MOVING   = 1 << 2,  // at least one motor is energised
  FLAG_FALLBACK = 1 << 3,  // running the on-board fixed maneuver
  FLAG_ENC_FAULT = 1 << 4, // wheel speed loop disabled: an encoder disagrees with its motor
};

// ------------------------------------------------------------
// Motion-source mode reported in STATUS
// ------------------------------------------------------------
constexpr uint8_t MODE_PI       = 0;  // Pi owns motion (SET_TWIST)
constexpr uint8_t MODE_FALLBACK = 1;  // ESP32 runs the fixed maneuver

// ------------------------------------------------------------
// CRC-16/CCITT-FALSE
// ------------------------------------------------------------
inline uint16_t crc16_ccitt(const uint8_t* data, uint16_t len, uint16_t crc = 0xFFFF) {
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ------------------------------------------------------------
// Little-endian serialisation helpers
//
// NOTE: ESP32 (Xtensa) and the Pi (ARM64) are both little-endian
// IEEE-754, so put_f32/get_f32 are safe without byte swapping.
// ------------------------------------------------------------
inline void put_u8 (uint8_t* p, uint8_t  v) { p[0] = v; }
inline void put_u16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); }
inline void put_u32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)((v >> (8 * i)) & 0xFF); }
inline void put_i32(uint8_t* p, int32_t  v) { put_u32(p, (uint32_t)v); }
inline void put_f32(uint8_t* p, float    v) { memcpy(p, &v, 4); }

inline uint16_t get_u16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
inline uint32_t get_u32(const uint8_t* p) { uint32_t v = 0; for (int i = 0; i < 4; i++) v |= (uint32_t)p[i] << (8 * i); return v; }
inline float    get_f32(const uint8_t* p) { float v; memcpy(&v, p, 4); return v; }

// ------------------------------------------------------------
// Frame encoder
// Writes a complete frame into `out` (needs >= len + 6 bytes).
// Returns the total number of bytes written.
// ------------------------------------------------------------
inline uint16_t encodeFrame(uint8_t* out, uint8_t msgId, const uint8_t* payload, uint8_t len) {
  out[0] = SYNC0;
  out[1] = SYNC1;
  out[2] = len;
  out[3] = msgId;
  if (len && payload) memcpy(&out[4], payload, len);

  uint16_t crc = crc16_ccitt(&out[2], (uint16_t)(2 + len));  // LEN, MSG_ID, PAYLOAD
  out[4 + len]     = (uint8_t)(crc & 0xFF);
  out[4 + len + 1] = (uint8_t)((crc >> 8) & 0xFF);
  return (uint16_t)(len + 6);
}

// ------------------------------------------------------------
// Decoded frame
// ------------------------------------------------------------
struct Frame {
  uint8_t msgId;
  uint8_t len;
  uint8_t payload[MAX_PAYLOAD];
};

// ------------------------------------------------------------
// Incremental (byte-at-a-time) frame parser
//
// Feed one byte at a time. Returns true exactly once per
// complete, CRC-valid frame, with `frame` filled in.
// ------------------------------------------------------------
class Parser {
 public:
  bool feed(uint8_t byte, Frame& frame) {
    switch (state_) {
      case WAIT_S0:
        if (byte == SYNC0) state_ = WAIT_S1;
        break;

      case WAIT_S1:
        if (byte == SYNC1)            state_ = WAIT_LEN;
        else if (byte != SYNC0)       state_ = WAIT_S0;
        // if byte == SYNC0 stay here (overlapping sync run)
        break;

      case WAIT_LEN:
        if (byte > MAX_PAYLOAD) { state_ = WAIT_S0; break; }
        len_ = byte;
        idx_ = 0;
        state_ = WAIT_ID;
        break;

      case WAIT_ID:
        id_ = byte;
        state_ = (len_ == 0) ? WAIT_CRC_LO : WAIT_PAYLOAD;
        break;

      case WAIT_PAYLOAD:
        payload_[idx_++] = byte;
        if (idx_ >= len_) state_ = WAIT_CRC_LO;
        break;

      case WAIT_CRC_LO:
        crcRx_ = byte;
        state_ = WAIT_CRC_HI;
        break;

      case WAIT_CRC_HI: {
        crcRx_ |= (uint16_t)byte << 8;
        state_ = WAIT_S0;

        uint8_t buf[2 + MAX_PAYLOAD];
        buf[0] = len_;
        buf[1] = id_;
        if (len_) memcpy(&buf[2], payload_, len_);

        if (crc16_ccitt(buf, (uint16_t)(2 + len_)) == crcRx_) {
          frame.msgId = id_;
          frame.len   = len_;
          if (len_) memcpy(frame.payload, payload_, len_);
          return true;
        }
        break;
      }
    }
    return false;
  }

 private:
  enum State { WAIT_S0, WAIT_S1, WAIT_LEN, WAIT_ID, WAIT_PAYLOAD, WAIT_CRC_LO, WAIT_CRC_HI };

  State    state_ = WAIT_S0;
  uint8_t  len_ = 0;
  uint8_t  id_  = 0;
  uint8_t  idx_ = 0;
  uint8_t  payload_[MAX_PAYLOAD];
  uint16_t crcRx_ = 0;
};

}  // namespace ff
