// ============================================================
// FIREFIGHTING ROBOT - MOBILITY / GATEWAY ESP32
// ESP32 Arduino Core 2.0.x
//
// ROLE
//   * Low-level motor + encoder controller, with closed-loop
//     (feedforward + PI) wheel speed control on SET_TWIST
//   * UART gateway between the Raspberry Pi (ROS 2) and the
//     wireless fire beacon (ESP-NOW)
//
// TOPOLOGY
//   Beacon --ESP-NOW--> [this ESP32] --UART2--> Raspberry Pi
//
// MOTION OWNERSHIP  (see MOTION_OWNER_IS_PI below)
//   1 -> the Pi sends SET_TWIST frames and owns the motion.
//        Beacon events are only forwarded to the Pi.
//   0 -> fallback: the ESP32 runs its original fixed
//        forward / right / left maneuver when the beacon reports
//        two consecutive FIRE + SMOKE packets.
//
// SAFETY
//   * Watchdog: in Pi mode the motors stop if no valid Pi frame
//     arrives for PI_TIMEOUT_MS.
//   * MSG_ESTOP latches a stop until MSG_RESET_FAULT arrives.
//
// !! NON-BLOCKING: the control path uses millis() only.
//    There is not a single delay() call in loop() or any of the
//    motion / sequence code.
// ============================================================

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "firefighter_protocol.h"


// ============================================================
// COMPILE-TIME CONFIGURATION
// ============================================================

// ---- Who owns motion ---------------------------------------
// 1 = Raspberry Pi (normal, Option A)
// 0 = on-board fixed maneuver (legacy fallback)
#define MOTION_OWNER_IS_PI 1

// ---- Debug output (USB serial, UART0) ----------------------
// Set DEBUG_ENABLED to 0 to compile out all debug output.
#define DEBUG_ENABLED 1

class NullStream : public Print {
 public:
  void   begin(unsigned long) {}
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t*, size_t n) override { return n; }
};
static NullStream DBG_null;

#if DEBUG_ENABLED
  #define DBG Serial       // USB-serial debug stream
#else
  #define DBG DBG_null     // silent sink
#endif


// ============================================================
// UART LINK TO THE RASPBERRY PI   (UART2)
// ------------------------------------------------------------
// ESP32 GPIO16 (RX) <- Pi GPIO14 (TXD)
// ESP32 GPIO17 (TX) -> Pi GPIO15 (RXD)
// GND <-> GND   (3.3 V logic on both sides, no shifter)
//
// Do NOT use UART1 (GPIO9/10): those pins are wired to the SPI
// flash on WROOM modules.
// ============================================================

const int UART_RX_PIN = 16;
const int UART_TX_PIN = 17;
const long UART_BAUD  = 115200;

// (not named PI: Arduino.h #defines PI as 3.14159...)
HardwareSerial& PiLink = Serial2;   // the UART link to the Raspberry Pi


// ============================================================
// TIMING (all non-blocking, millis() based)
// ============================================================

const uint32_t PI_TIMEOUT_MS   = 500;   // no valid Pi frame -> stop
const uint32_t ODOM_PERIOD_MS  = 20;    // 50 Hz wheel odometry
const uint32_t STATUS_PERIOD_MS = 200;  // 5 Hz status
const uint32_t HB_PERIOD_MS    = 1000;  // 1 Hz ESP32 heartbeat

// Fallback maneuver timings (match the legacy firmware)
const uint32_t FB_FWD_MS   = 5000;
const uint32_t FB_RIGHT_MS = 3000;
const uint32_t FB_LEFT_MS  = 5000;
const uint32_t FB_PAUSE_MS = 500;


// ============================================================
// MOTOR DRIVER PINS
// ============================================================

// Motor 1 (left)
const int ENA = 14;
const int IN1 = 27;
const int IN2 = 26;

// Motor 2 (right)
const int IN3 = 25;
const int IN4 = 33;
const int ENB = 32;


// ============================================================
// ENCODER PINS
// ============================================================

// Motor 1 encoder
const int M1_ENC_A = 18;   // Green
const int M1_ENC_B = 19;   // Blue

// Motor 2 encoder
const int M2_ENC_A = 22;   // Green
const int M2_ENC_B = 23;   // Blue

// Flip these if a wheel counts backwards on your wiring
const bool M1_ENC_INVERT = false;
const bool M2_ENC_INVERT = false;


// ============================================================
// PWM SETTINGS (ESP32 Arduino Core 2.0.x and 3.x)
// ============================================================

const int PWM_CHANNEL_A  = 0;
const int PWM_CHANNEL_B  = 1;
const int PWM_FREQ       = 5000;
const int PWM_RESOLUTION = 8;     // 0..255

// Core 3.x dropped ledcSetup()/ledcAttachPin(); ledcWrite() now takes
// the PIN instead of the channel. PWM_A / PWM_B are whatever ledcWrite()
// expects on the core being built against.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_A ENA
  #define PWM_B ENB
#else
  #define PWM_A PWM_CHANNEL_A
  #define PWM_B PWM_CHANNEL_B
#endif

// Legacy open-loop speed used by the fallback maneuver (~71%)
const int MOTOR_SPEED = 180;

// Below this magnitude the motor is off (avoids buzzing at low duty)
const int MOTOR_DEADBAND = 8;


// ============================================================
// DRIVE GEOMETRY  (used to turn SET_TWIST into wheel speeds)
// Adjust to the real robot.
// ============================================================

const float TRACK_WIDTH_M  = 0.20f;   // distance between wheels (m)
const float MAX_WHEEL_MPS  = 0.50f;   // fastest wheel speed a command may ask for
const float MIN_WHEEL_MPS  = 0.01f;   // commands below this are treated as stop

// Wheel + encoder (MUST MATCH wheel_radius_m / ticks_per_rev in the
// bridge's config/bridge.yaml, or the two sides disagree about speed).
const float WHEEL_RADIUS_M        = 0.0325f;
const long  ENCODER_TICKS_PER_REV = 1440;     // 360 CPR x4 quadrature
const float METERS_PER_TICK =
    (2.0f * 3.14159265f * WHEEL_RADIUS_M) / (float)ENCODER_TICKS_PER_REV;

// Lowest PWM at which the wheels reliably turn under load. Used as the
// feedforward starting point; the speed loop corrects from there. CALIBRATE on
// the robot (scripts/motor_test.py).
const int MOTOR_MIN_MOVE_PWM = 120;


// ============================================================
// CLOSED-LOOP WHEEL SPEED CONTROL
//
// Each wheel runs feedforward + PI on its encoder speed, so a
// SET_TWIST of 0.05 m/s gives 0.05 m/s rather than whatever a fixed
// PWM happens to produce. Gains are starting values - TUNE on the
// robot: raise VEL_KI if slow commands take long to start, lower
// VEL_KP / VEL_KI if the wheels surge or oscillate.
//
// REQUIRES each encoder to count UP when its wheel drives the robot
// forward. If one counts down, flip M1_ENC_INVERT / M2_ENC_INVERT.
// A wheel whose encoder disagrees with its motor (wrong sign, or no
// counts at all) trips encoderFault: both wheels drop back to
// open-loop feedforward until MSG_RESET_FAULT, so a bad encoder can
// never wind the loop up to full speed.
//
// Set VELOCITY_CONTROL_ENABLED to 0 to run open-loop feedforward only.
// ============================================================

#define VELOCITY_CONTROL_ENABLED 1

const uint32_t VEL_PERIOD_MS    = 20;       // 50 Hz control loop
const float    VEL_KP           = 150.0f;   // PWM per (m/s) of speed error
const float    VEL_KI           = 1500.0f;  // PWM per (m/s) of error, per second
const float    VEL_FILTER_ALPHA = 0.5f;     // low-pass on measured speed (1 = none)

const int      ENC_FAULT_PWM     = 150;     // "driving hard" threshold
const float    ENC_FAULT_MIN_MPS = 0.02f;   // slower than this counts as not moving
const uint32_t ENC_FAULT_MS      = 700;     // how long the disagreement must last


// ============================================================
// ESP-NOW MESSAGE (MUST MATCH THE BEACON EXACTLY)
// ============================================================

typedef struct struct_message {
  int  messageNumber;
  bool fireDetected;
  bool smokeDetected;
} struct_message;

struct_message incomingData;

// Fallback trigger requires this many consecutive detections
const int REQUIRED_DETECTIONS = 2;

// ---- Beacon pairing ----------------------------------------
// The beacon does not hardcode this board's MAC. It broadcasts a
// PAIR_REQUEST; we answer with a broadcast PAIR_REPLY and the beacon
// reads our MAC from the reply's source address.
// (MUST MATCH beacon.ino EXACTLY. Its size differs from
// struct_message, so the two never get confused.)
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t PAIR_MAGIC   = 0x46465052;   // "FFPR"
const uint8_t  PAIR_REQUEST = 1;            // beacon   -> broadcast
const uint8_t  PAIR_REPLY   = 2;            // mobility -> broadcast

typedef struct __attribute__((packed)) pair_message {
  uint32_t magic;
  uint8_t  type;
} pair_message;

// Set in the ESP-NOW callback, answered from loop()
volatile bool pairReplyPending = false;


// ============================================================
// STATE
// ============================================================

ff::Parser parser;
ff::Frame  frame;

volatile long motor1_encoder_count = 0;
volatile long motor2_encoder_count = 0;

// Last commanded motor magnitudes (for the MOVING status flag)
int motorCmdA = 0;
int motorCmdB = 0;

// Per-wheel speed controller. Declared here, before the first function,
// for the same prototype-hoisting reason as FbState below.
struct WheelCtl {
  float    target;      // commanded speed, m/s (+ = robot forward)
  float    measured;    // filtered encoder speed, m/s
  float    integral;    // PI integral term, in PWM counts
  long     prevTicks;   // encoder count at the previous control step
  uint32_t badSinceMs;  // when encoder and motor started to disagree (0 = fine)
};

WheelCtl wheelL = {0.0f, 0.0f, 0.0f, 0, 0};
WheelCtl wheelR = {0.0f, 0.0f, 0.0f, 0, 0};

bool     velControlActive = false;   // true while SET_TWIST owns the motors
bool     encoderFault     = false;   // latched: speed loop disabled, open-loop only
uint32_t lastVelMs        = 0;

// Watchdog / faults
uint32_t lastPiFrameMs = 0;
bool     linkFault     = false;   // Pi silent
bool     estop         = false;   // latched by MSG_ESTOP

// Periodic bookkeeping
uint32_t lastOdomMs   = 0;
uint32_t lastStatusMs = 0;
uint32_t lastHbMs     = 0;
uint16_t hbSeq        = 0;

// Fallback detection
volatile int consecutiveDetectionCount = 0;

// Fallback maneuver states. Declared here, before the first function,
// because the Arduino IDE hoists auto-generated prototypes (including
// fbEnter(FbState)) to just above the first function in the sketch.
enum FbState { FB_IDLE, FB_FWD, FB_PAUSE1, FB_RIGHT, FB_PAUSE2, FB_LEFT, FB_DONE };


// ============================================================
// LOW-LEVEL MOTOR CONTROL
//
// setMotor() takes a SIGNED speed where positive = that wheel
// drives the robot FORWARD. Motor 2 is mounted as a mirror
// image, so its direction pins are inverted here - this is the
// mirroring that the old moveForward()/turnRight() helpers
// encoded by hand.
// ============================================================

void setMotor(uint8_t motor, int signedSpeed) {
  int mag = abs(signedSpeed);
  if (mag < MOTOR_DEADBAND) mag = 0;

  if (motor == 1) {
    digitalWrite(IN1, signedSpeed >= 0 ? HIGH : LOW);
    digitalWrite(IN2, signedSpeed >= 0 ? LOW  : HIGH);
    ledcWrite(PWM_A, mag);
    motorCmdA = mag;
  } else {
    // Motor 2 is physically mirrored
    digitalWrite(IN3, signedSpeed >= 0 ? LOW  : HIGH);
    digitalWrite(IN4, signedSpeed >= 0 ? HIGH : LOW);
    ledcWrite(PWM_B, mag);
    motorCmdB = mag;
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
  motorCmdA = 0;
  motorCmdB = 0;

  // Also release the speed loop, or its next step would re-energise the motors.
  velControlActive = false;
  wheelL.target = wheelR.target = 0.0f;
  wheelL.integral = wheelR.integral = 0.0f;
}

// Open-loop feedforward: wheel speed (m/s) -> signed PWM. The motors do
// not turn below MOTOR_MIN_MOVE_PWM, so a non-zero speed is mapped onto
// [MOTOR_MIN_MOVE_PWM .. 255] instead of [0 .. 255]; otherwise slow
// commands (search spin, Nav2 fine positioning) only make them hum.
int wheelSpeedToPwm(float v) {
  float mag = fabsf(v);
  if (mag < MIN_WHEEL_MPS) return 0;
  if (mag > MAX_WHEEL_MPS) mag = MAX_WHEEL_MPS;
  float pwm = MOTOR_MIN_MOVE_PWM +
              (mag / MAX_WHEEL_MPS) * (255.0f - MOTOR_MIN_MOVE_PWM);
  return v >= 0.0f ? (int)pwm : -(int)pwm;
}

// Hand the wheels a new speed target (m/s). The motors are driven by
// updateVelocityControl(), at most VEL_PERIOD_MS later.
void setWheelTargets(float vL, float vR) {
  wheelL.target = constrain(vL, -MAX_WHEEL_MPS, MAX_WHEEL_MPS);
  wheelR.target = constrain(vR, -MAX_WHEEL_MPS, MAX_WHEEL_MPS);
  velControlActive = true;
}

// One control step for one wheel: update its measured speed and return
// the signed PWM to apply.
int wheelControlStep(WheelCtl& w, long ticks, float dt, uint32_t now) {
  float raw = (float)(ticks - w.prevTicks) * METERS_PER_TICK / dt;
  w.prevTicks = ticks;
  w.measured += VEL_FILTER_ALPHA * (raw - w.measured);

  // A zero target means stop now, not "regulate to zero".
  if (fabsf(w.target) < MIN_WHEEL_MPS) {
    w.integral   = 0.0f;
    w.badSinceMs = 0;
    return 0;
  }

  int ff = wheelSpeedToPwm(w.target);
  if (!VELOCITY_CONTROL_ENABLED || encoderFault) return ff;

  float err = w.target - w.measured;
  float out = (float)ff + VEL_KP * err + w.integral;

  // Only ever drive in the commanded direction: the loop may ease off
  // to zero, but it does not reverse the motor to brake.
  float lo = (w.target > 0.0f) ? 0.0f : -255.0f;
  float hi = (w.target > 0.0f) ? 255.0f : 0.0f;
  bool atHi = out >= hi;
  bool atLo = out <= lo;
  out = constrain(out, lo, hi);

  // Anti-windup: stop integrating in the direction that is already saturated.
  if (!(atHi && err > 0.0f) && !(atLo && err < 0.0f)) {
    w.integral = constrain(w.integral + VEL_KI * err * dt, -255.0f, 255.0f);
  }

  // Encoder sanity: turning the wrong way, or driven hard and not turning.
  bool wrongWay = (w.measured * w.target < 0.0f) &&
                  (fabsf(w.measured) > ENC_FAULT_MIN_MPS);
  bool noMotion = (fabsf(out) >= (float)ENC_FAULT_PWM) &&
                  (fabsf(w.measured) < ENC_FAULT_MIN_MPS);
  if (wrongWay || noMotion) {
    if (w.badSinceMs == 0) {
      w.badSinceMs = now ? now : 1;
    } else if (now - w.badSinceMs > ENC_FAULT_MS) {
      encoderFault = true;
      DBG.printf("[vel] ENCODER FAULT (%s) -> open-loop until RESET_FAULT\n",
                 wrongWay ? "counts opposite to the motor" : "no counts / stalled");
      return ff;
    }
  } else {
    w.badSinceMs = 0;
  }

  return (int)out;
}

// Runs every loop(); steps both wheel controllers at VEL_PERIOD_MS.
void updateVelocityControl() {
  uint32_t now = millis();
  if (now - lastVelMs < VEL_PERIOD_MS) return;
  float dt = (float)(now - lastVelMs) * 0.001f;
  lastVelMs = now;

  long t1, t2;
  readEncoderCounts(t1, t2);

  // Always step, so the measured speeds stay current while the fallback
  // maneuver or a stop owns the motors.
  int pwmL = wheelControlStep(wheelL, t1, dt, now);
  int pwmR = wheelControlStep(wheelR, t2, dt, now);
  if (!velControlActive) return;

  setMotor(1, pwmL);
  setMotor(2, pwmR);
}

// Named helpers used by the fallback maneuver
void moveForward() { setMotor(1,  MOTOR_SPEED); setMotor(2,  MOTOR_SPEED); }
void turnRight()   { setMotor(1, -MOTOR_SPEED); setMotor(2,  MOTOR_SPEED); }
void turnLeft()    { setMotor(1,  MOTOR_SPEED); setMotor(2, -MOTOR_SPEED); }


// ============================================================
// ENCODERS - quadrature x4 counting
// ============================================================

void IRAM_ATTR motor1ISR() {
  bool a = digitalRead(M1_ENC_A);
  bool b = digitalRead(M1_ENC_B);
  if (a == b) motor1_encoder_count += M1_ENC_INVERT ? -1 : 1;
  else        motor1_encoder_count += M1_ENC_INVERT ?  1 : -1;
}

void IRAM_ATTR motor2ISR() {
  bool a = digitalRead(M2_ENC_A);
  bool b = digitalRead(M2_ENC_B);
  if (a == b) motor2_encoder_count += M2_ENC_INVERT ? -1 : 1;
  else        motor2_encoder_count += M2_ENC_INVERT ?  1 : -1;
}

void readEncoderCounts(long& t1, long& t2) {
  noInterrupts();
  t1 = motor1_encoder_count;
  t2 = motor2_encoder_count;
  interrupts();
}


// ============================================================
// FRAME TRANSMIT HELPERS
// ============================================================

void sendFrame(uint8_t msgId, const uint8_t* payload, uint8_t len) {
  uint8_t buf[ff::MAX_PAYLOAD + 6];
  uint16_t n = ff::encodeFrame(buf, msgId, payload, len);
  PiLink.write(buf, n);
}

void sendAck(uint8_t ackedId) {
  uint8_t p[1];
  ff::put_u8(p, ackedId);
  sendFrame(ff::MSG_ACK, p, 1);
}

void sendHeartbeat() {
  uint8_t p[2];
  ff::put_u16(p, ++hbSeq);
  sendFrame(ff::MSG_HB_ESP, p, 2);
}

void sendWheelOdom(long ticks1, long ticks2, uint32_t dtUs) {
  uint8_t p[12];
  ff::put_i32(p,     (int32_t)ticks1);
  ff::put_i32(p + 4, (int32_t)ticks2);
  ff::put_u32(p + 8, dtUs);
  sendFrame(ff::MSG_WHEEL_ODOM, p, 12);
}

void sendStatus() {
  uint8_t flags = 0;
  if (!linkFault && (millis() - lastPiFrameMs <= PI_TIMEOUT_MS)) flags |= ff::FLAG_LINK_OK;
  if (estop)                              flags |= ff::FLAG_ESTOP;
  if (motorCmdA > 0 || motorCmdB > 0)     flags |= ff::FLAG_MOVING;
  if (!MOTION_OWNER_IS_PI)                flags |= ff::FLAG_FALLBACK;
  if (encoderFault)                       flags |= ff::FLAG_ENC_FAULT;

  uint8_t p[4];
  ff::put_u16(p,     0);          // vbat_mV: no battery ADC yet
  ff::put_u8 (p + 2, flags);
  ff::put_u8 (p + 3, MOTION_OWNER_IS_PI ? ff::MODE_PI : ff::MODE_FALLBACK);
  sendFrame(ff::MSG_STATUS, p, 4);
}

void sendBeaconEvent(const struct_message& m, uint8_t rssi, const uint8_t* mac) {
  uint8_t p[13];
  ff::put_u32(p,     (uint32_t)m.messageNumber);
  ff::put_u8 (p + 4, m.fireDetected  ? 1 : 0);
  ff::put_u8 (p + 5, m.smokeDetected ? 1 : 0);
  ff::put_u8 (p + 6, rssi);       // signal strength magnitude in dBm (e.g. 65 for -65 dBm)
  memcpy(p + 7, mac, 6);
  sendFrame(ff::MSG_BEACON_EVENT, p, 13);
}


// ============================================================
// WATCHDOG
// ============================================================

void notePiFrame() {
  lastPiFrameMs = millis();
  if (linkFault) {
    linkFault = false;
    DBG.printf("[link] Pi frames resumed\n");
  }
}

void updateWatchdog() {
  if (!MOTION_OWNER_IS_PI) return;

  if (estop) {
    stopMotors();
    return;
  }

  if (millis() - lastPiFrameMs > PI_TIMEOUT_MS) {
    if (!linkFault) {
      linkFault = true;
      stopMotors();
      DBG.printf("[link] watchdog timeout -> MOTORS STOPPED\n");
    }
    return;
  }

  if (linkFault) stopMotors();   // stay stopped until a new SET_TWIST
}


// ============================================================
// COMMAND HANDLING (Pi -> ESP32)
// ============================================================

void applyTwist(float lin, float ang) {
  if (!MOTION_OWNER_IS_PI) return;
  if (estop || linkFault)  return;

  float vL = lin - ang * (TRACK_WIDTH_M / 2.0f);
  float vR = lin + ang * (TRACK_WIDTH_M / 2.0f);
  setWheelTargets(vL, vR);
}

void handleFrame(const ff::Frame& f) {
  switch (f.msgId) {
    case ff::MSG_SET_TWIST:
      if (f.len == 8) {
        applyTwist(ff::get_f32(f.payload), ff::get_f32(f.payload + 4));
        notePiFrame();
      }
      sendAck(f.msgId);
      break;

    case ff::MSG_ESTOP:
      estop = true;
      stopMotors();
      notePiFrame();
      DBG.printf("[cmd] ESTOP latched\n");
      sendAck(f.msgId);
      break;

    case ff::MSG_RESET_FAULT:
      estop        = false;
      linkFault    = false;
      encoderFault = false;
      wheelL.badSinceMs = wheelR.badSinceMs = 0;
      notePiFrame();
      DBG.printf("[cmd] faults cleared\n");
      sendAck(f.msgId);
      break;

    case ff::MSG_HEARTBEAT:
      notePiFrame();
      break;

    default:
      break;
  }
}


// ============================================================
// FALLBACK MANEUVER (only when MOTION_OWNER_IS_PI == 0)
//
// Same behaviour as the legacy firmware, but driven by a
// millis() state machine so the UART is serviced continuously.
// ============================================================

// (enum FbState is declared in the STATE section, above the first function)

FbState  fbState      = FB_IDLE;
uint32_t fbStateStart = 0;

void fbEnter(FbState s) {
  fbState      = s;
  fbStateStart = millis();

  switch (s) {
    case FB_FWD:    DBG.printf("[fb] FORWARD\n"); moveForward(); break;
    case FB_PAUSE1: DBG.printf("[fb] PAUSE\n");   stopMotors();  break;
    case FB_RIGHT:  DBG.printf("[fb] RIGHT\n");   turnRight();   break;
    case FB_PAUSE2: DBG.printf("[fb] PAUSE\n");   stopMotors();  break;
    case FB_LEFT:   DBG.printf("[fb] LEFT\n");    turnLeft();    break;
    case FB_DONE:   DBG.printf("[fb] DONE\n");    stopMotors();  break;
    case FB_IDLE:   break;
  }
}

void fallbackUpdate() {
  if (MOTION_OWNER_IS_PI) return;
  if (fbState == FB_IDLE) return;

  uint32_t elapsed = millis() - fbStateStart;

  switch (fbState) {
    case FB_FWD:    if (elapsed >= FB_FWD_MS)   fbEnter(FB_PAUSE1); break;
    case FB_PAUSE1: if (elapsed >= FB_PAUSE_MS) fbEnter(FB_RIGHT);  break;
    case FB_RIGHT:  if (elapsed >= FB_RIGHT_MS) fbEnter(FB_PAUSE2); break;
    case FB_PAUSE2: if (elapsed >= FB_PAUSE_MS) fbEnter(FB_LEFT);   break;
    case FB_LEFT:   if (elapsed >= FB_LEFT_MS)  fbEnter(FB_DONE);   break;
    case FB_DONE:
      // Require two NEW detections before the next maneuver
      consecutiveDetectionCount = 0;
      incomingData.fireDetected  = false;
      incomingData.smokeDetected = false;
      fbState = FB_IDLE;
      DBG.printf("[fb] listening for new FIRE+SMOKE\n");
      break;
    default:
      break;
  }
}


// ============================================================
// ESP-NOW & PROMISCUOUS RSSI CAPTURE
//
// In ESP-IDF 5.x / Core 3.x, esp_now_recv_info_t provides rx_ctrl.
// In ESP-IDF 4.x / Core 2.0.x, a promiscuous sniffer hook extracts
// the raw RSSI from incoming 802.11 frames.
// ============================================================

static volatile int8_t latestRssi = 0;

void promiscuousRxCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t* pkt = (const wifi_promiscuous_pkt_t*)buf;
  latestRssi = pkt->rx_ctrl.rssi;
}

// True (and queues a PAIR_REPLY) if the packet is a beacon PAIR_REQUEST
bool handlePairRequest(const uint8_t* data, int len) {
  if (len != (int)sizeof(pair_message)) return false;

  pair_message msg;
  memcpy(&msg, data, sizeof(msg));
  if (msg.magic != PAIR_MAGIC || msg.type != PAIR_REQUEST) return false;

  pairReplyPending = true;
  return true;
}

void sendPairReply() {
  pair_message msg = { PAIR_MAGIC, PAIR_REPLY };
  esp_now_send(BROADCAST_MAC, (const uint8_t*)&msg, sizeof(msg));
  DBG.printf("[espnow] pair request answered\n");
}

#if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingDataBytes, int len) {
  if (handlePairRequest(incomingDataBytes, len)) return;
  if (len != (int)sizeof(incomingData)) return;

  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  int8_t rawRssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : latestRssi;
  uint8_t rssiMag = (rawRssi < 0) ? (uint8_t)(-rawRssi) : (uint8_t)rawRssi;

  const uint8_t* mac_addr = info ? info->src_addr : nullptr;
  static const uint8_t zeroMac[6] = {0};
  if (!mac_addr) mac_addr = zeroMac;

  // 1) Gateway: forward to the Pi with RSSI
  sendBeaconEvent(incomingData, rssiMag, mac_addr);

  DBG.printf("[espnow] #%d FIRE=%d SMOKE=%d RSSI=-%d dBm\n",
      incomingData.messageNumber,
      incomingData.fireDetected  ? 1 : 0,
      incomingData.smokeDetected ? 1 : 0,
      rssiMag);

  // 2) Fallback trigger
  if (MOTION_OWNER_IS_PI) return;
  if (fbState != FB_IDLE)  return;   // ignore during a maneuver

  if (incomingData.fireDetected && incomingData.smokeDetected) {
    consecutiveDetectionCount++;
    if (consecutiveDetectionCount >= REQUIRED_DETECTIONS) {
      consecutiveDetectionCount = 0;
      fbEnter(FB_FWD);
    }
  } else {
    consecutiveDetectionCount = 0;
  }
}
#else
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* incomingDataBytes, int len) {
  if (handlePairRequest(incomingDataBytes, len)) return;
  if (len != (int)sizeof(incomingData)) return;

  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  int8_t rawRssi = latestRssi;
  uint8_t rssiMag = (rawRssi < 0) ? (uint8_t)(-rawRssi) : (uint8_t)rawRssi;

  // 1) Gateway: forward to the Pi with RSSI
  sendBeaconEvent(incomingData, rssiMag, mac_addr);

  DBG.printf("[espnow] #%d FIRE=%d SMOKE=%d RSSI=-%d dBm\n",
      incomingData.messageNumber,
      incomingData.fireDetected  ? 1 : 0,
      incomingData.smokeDetected ? 1 : 0,
      rssiMag);

  // 2) Fallback trigger
  if (MOTION_OWNER_IS_PI) return;
  if (fbState != FB_IDLE)  return;   // ignore during a maneuver

  if (incomingData.fireDetected && incomingData.smokeDetected) {
    consecutiveDetectionCount++;
    if (consecutiveDetectionCount >= REQUIRED_DETECTIONS) {
      consecutiveDetectionCount = 0;
      fbEnter(FB_FWD);
    }
  } else {
    consecutiveDetectionCount = 0;
  }
}
#endif


// ============================================================
// SETUP
// ============================================================

void setup() {
  // ---- Debug (USB, UART0) --------------------------------
  DBG.begin(115200);
  delay(300);   // one-off, before the control loop starts

  DBG.println();
  DBG.println("========================================");
  DBG.println(" FIREFIGHTING ROBOT - MOBILITY ESP32");
  DBG.println("========================================");

  // ---- Motor direction pins ------------------------------
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ---- Encoders ------------------------------------------
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), motor1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B), motor1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), motor2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_B), motor2ISR, CHANGE);

  // ---- PWM -----------------------------------------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(ENA, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_A);
  ledcAttachChannel(ENB, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_B);
#else
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);
#endif

  // Safety: motors off at startup
  stopMotors();

  // ---- UART link to the Pi -------------------------------
  PiLink.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  lastPiFrameMs = millis();

  DBG.printf("UART2 to Pi: RX=%d TX=%d @ %ld\n", UART_RX_PIN, UART_TX_PIN, UART_BAUD);

  // ---- Wi-Fi + ESP-NOW -----------------------------------
  WiFi.mode(WIFI_STA);
  delay(100);   // one-off, pre-loop

  // Enable Wi-Fi promiscuous rx sniffer to capture packet RSSI in core 2.0.x
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_MGMT
  };
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous_rx_cb(promiscuousRxCallback);
  esp_wifi_set_promiscuous(true);

  DBG.print("Receiver MAC Address: ");
  DBG.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    DBG.println("ERROR: ESP-NOW init failed!");
    stopMotors();
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Broadcast peer, used to answer beacon pair requests
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    DBG.println("ERROR: failed to add broadcast peer (beacon pairing disabled)");
  }

  DBG.println("ESP-NOW ready.");

  if (MOTION_OWNER_IS_PI) {
    DBG.println("Mode: PI-OWNED motion. Waiting for SET_TWIST / HEARTBEAT...");
  } else {
    DBG.println("Mode: FALLBACK maneuver. Waiting for FIRE+SMOKE x2...");
  }
  DBG.println("========================================");
}


// ============================================================
// MAIN LOOP   (no delay() in the control path)
// ============================================================

void loop() {
  // ---- 1. Service the Pi UART (highest priority) ----------
  while (PiLink.available()) {
    if (parser.feed((uint8_t)PiLink.read(), frame)) {
      handleFrame(frame);
    }
  }

  // ---- 2. Safety -----------------------------------------
  updateWatchdog();

  // ---- 3. Fallback maneuver ------------------------------
  fallbackUpdate();

  // ---- 3b. Wheel speed loop (after safety, so a stop wins) -
  updateVelocityControl();

  // ---- 4. Beacon pairing ---------------------------------
  if (pairReplyPending) {
    pairReplyPending = false;
    sendPairReply();
  }

  // ---- 5. Periodic telemetry -----------------------------
  uint32_t now = millis();

  if (now - lastOdomMs >= ODOM_PERIOD_MS) {
    uint32_t dtMs = now - lastOdomMs;
    lastOdomMs = now;
    long t1, t2;
    readEncoderCounts(t1, t2);
    sendWheelOdom(t1, t2, dtMs * 1000UL);
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus();
  }

  if (now - lastHbMs >= HB_PERIOD_MS) {
    lastHbMs = now;
    sendHeartbeat();
  }

  // ---- 6. Yield ------------------------------------------
  delay(1);
}
