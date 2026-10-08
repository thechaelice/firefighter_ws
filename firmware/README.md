# ESP32 Firmware Documentation

This directory contains the embedded firmware for the Firefighter Robot system, split into two ESP32 microcontrollers communicating wirelessly via **ESP-NOW**.

---

## Directory Layout

```text
firmware/
├── beacon/
│   └── beacon.ino             # Fire & smoke detector beacon sketch (Sender)
├── robot_mobility/
│   └── robot_mobility.ino     # Robot motor driver & ESP-NOW receiver sketch
└── README.md                  # Firmware guide, pinouts, and flashing instructions
```

---

## System Architecture

```mermaid
flowchart LR
    subgraph Beacon [Fire Beacon - ESP32]
        Sensor[Flame / Smoke Sensor] --> BeaconESP[ESP32 Sender]
        BeaconESP -->|ESP-NOW Broadcast / Unicast| RecvESP
    end

    subgraph Robot [Robot Mobility - ESP32]
        RecvESP[ESP32 Receiver] --> Drivers[L298N / Motor Driver]
        Drivers --> M1[Motor 1 (Left)]
        Drivers --> M2[Motor 2 (Right)]
        Encoders[Quadrature Encoders] -.-> RecvESP
    end
```

---

## 1. Wireless Protocol (ESP-NOW)

The beacon and the robot communicate using **ESP-NOW**, a low-latency, low-power peer-to-peer 2.4 GHz wireless protocol developed by Espressif.

### Packet Structure
Both sender (`beacon.ino`) and receiver (`robot_mobility.ino`) must share the exact same message struct:

```cpp
typedef struct struct_message {
    int messageNumber;
    bool fireDetected;
    bool smokeDetected;
} struct_message;
```

### Pairing & MAC Address Setup
Pairing is automatic; no MAC address is hardcoded.

1. The beacon broadcasts a `PAIR_REQUEST` every 500 ms until it has a receiver.
2. The mobility ESP32 answers with a broadcast `PAIR_REPLY`.
3. The beacon takes the receiver MAC from the source address of that reply, registers it as its ESP-NOW peer and starts sending alert packets to it (`Receiver found: XX:XX:XX:XX:XX:XX` on the beacon's Serial Monitor).
4. After 5 consecutive undelivered packets the beacon forgets the receiver and goes back to step 1, so a swapped or rebooted mobility ESP32 is picked up without reflashing.

Both sketches must be flashed with matching `pair_message` definitions. The beacon pairs with the first mobility ESP32 that answers, so only one should be powered within range.

### Signal Strength (RSSI) & Distance Estimation
* **Interim (Active)**: The mobility ESP32 captures the received packet signal strength (RSSI in dBm) via a promiscuous Wi-Fi hook (Core 2.0.x) or `esp_now_recv_info_t` (Core 3.0.x) and forwards it in `MSG_BEACON_EVENT` to ROS 2 topic `/beacon_event`.
  * Typical indoor Wi-Fi path-loss formula: $\text{RSSI} \approx \text{RSSI}_0 - 10 n \log_{10}(d)$.
  * Provides coarse proximity indication (e.g. Strong: $-40\text{ to }-55\text{ dBm} \rightarrow <2\text{m}$, Moderate: $-60\text{ to }-75\text{ dBm} \rightarrow 2\text{--}6\text{m}$, Weak: $<-80\text{ dBm} \rightarrow >6\text{m}$).
* **Target Upgrade (UWB Ranging)**: Ultra-Wideband Two-Way Ranging (DWM1000/DWM3000) for high-precision $\pm 5\text{--}10\text{ cm}$ indoor ranging. See the [UWB Migration Guide](../docs/uwb-ranging-migration.md).

---

## 2. Hardware Pinout & Wiring

### Robot Mobility Controller (`robot_mobility.ino`)

#### Motor Driver (e.g. L298N / Dual H-Bridge)
| Signal | ESP32 GPIO | Description |
|---|---|---|
| **ENA** | GPIO 14 | Motor 1 Speed (PWM Channel 0) |
| **IN1** | GPIO 27 | Motor 1 Direction A |
| **IN2** | GPIO 26 | Motor 1 Direction B |
| **IN3** | GPIO 25 | Motor 2 Direction A |
| **IN4** | GPIO 33 | Motor 2 Direction B |
| **ENB** | GPIO 32 | Motor 2 Speed (PWM Channel 1) |

#### Optical / Magnetic Encoders
| Encoder | Channel A (Green) | Channel B (Blue) | Description |
|---|---|---|---|
| **Motor 1** | GPIO 18 | GPIO 19 | Left wheel quadrature encoder |
| **Motor 2** | GPIO 22 | GPIO 23 | Right wheel quadrature encoder |

#### PWM Configuration
* **Frequency**: 5 kHz
* **Resolution**: 8-bit (values `0` – `255`)
* **Default Speed**: `180` (~71% duty cycle)

---

## 3. Behavior & Trigger Logic

> **Note:** the legacy trigger/maneuver described here now runs **only as a fallback**
> (`MOTION_OWNER_IS_PI 0`). By default the Pi owns motion — see [§5](#5-pi--esp32-uart-link-option-a--active).

### Trigger Condition
* The robot requires **2 consecutive packets** with both `fireDetected == true` and `smokeDetected == true` before triggering movement to prevent false positives.
* If a packet arrives without both conditions satisfied, the consecutive counter resets to 0.

### Maneuver Sequence
When triggered, the robot executes the following fixed sequence:
1. **Forward**: 5 seconds
2. **Stop**: 0.5 seconds
3. **Turn Right**: 3 seconds
4. **Stop**: 0.5 seconds
5. **Turn Left**: 5 seconds
6. **Final Stop**: Shuts down motors, clears old flags, and resumes listening for new alert packets.

*Note: During execution of the sequence, all incoming ESP-NOW commands are safely ignored.*

---

## 4. Flashing & Setup Instructions

### Prerequisites
* **Arduino IDE** (v1.8.x or v2.x) or **VS Code with PlatformIO / Arduino extension**
* **ESP32 Board Package**: `esp32` by Espressif Systems (Core 2.0.x recommended)

### Board Settings in Arduino IDE
* **Board**: `ESP32 Dev Module` (or your specific ESP32 board)
* **CPU Frequency**: `240MHz (WiFi/BT)`
* **Flash Frequency**: `80MHz`
* **Upload Speed**: `921600` (or `115200` if flashing fails)
* **Baud Rate**: `115200`

---

## 5. Pi ↔ ESP32 UART Link (Option A — active)

The mobility ESP32 is now a **gateway**: it keeps the ESP-NOW link to the beacon and
bridges everything to the Raspberry Pi over a framed binary UART protocol.

```
Beacon --ESP-NOW--> [Mobility ESP32] --UART2--> Raspberry Pi (ROS 2)
```

### Wiring
| Pi | ESP32 | Note |
|---|---|---|
| GPIO14 (TXD) | GPIO16 (RX) | UART2, 115200 8N1 |
| GPIO15 (RXD) | GPIO17 (TX) | do **not** use UART1 (GPIO9/10 — flash pins) |
| GND | GND | required |

Both sides are 3.3 V — no level shifter. The Pi must expose a UART on GPIO14/15
(see [`docs/runbook.md`](../docs/runbook.md): `dtoverlay=miniuart-bt` and remove the serial console).

### Protocol
Defined once in [`robot_mobility/firefighter_protocol.h`](robot_mobility/firefighter_protocol.h).
Frame: `A5 5A | LEN | MSG_ID | PAYLOAD | CRC16-LE` (CRC-16/CCITT-FALSE over LEN..payload).

| Pi → ESP32 | | ESP32 → Pi | |
|---|---|---|---|
| `SET_TWIST` `0x01` | f32 lin, ang | `WHEEL_ODOM` `0x81` | i32 ticksL/R, u32 dt_us |
| `ESTOP` `0x02` | — | `STATUS` `0x82` | u16 vbat, u8 flags, u8 mode |
| `RESET_FAULT` `0x03` | — | `BEACON_EVENT` `0x83` | u32 msg, fire, smoke, rssi, mac |
| `HEARTBEAT` `0x04` | u16 seq | `ACK` `0x84` / `HB_ESP` `0x85` | |

### Motion ownership
Set by `MOTION_OWNER_IS_PI` at the top of `robot_mobility.ino`:

* **`1` (default)** — the Pi owns motion: it streams `SET_TWIST`, and beacon events are
  only *forwarded* so the Pi's autonomy stack decides what to do.
* **`0`** — fallback: the ESP32 runs its original fixed forward/right/left maneuver on
  two consecutive beacon FIRE + SMOKE detections (still non-blocking).

### Safety
* **Watchdog** — in Pi mode, no valid frame for `PI_TIMEOUT_MS` (500 ms) → motors stop.
* **E-stop** — `ESTOP` latches a stop until `RESET_FAULT` arrives.
* The whole control path is `millis()`-based; the legacy blocking `delay()` maneuver
  is preserved as `robot_mobility.blocking.bak` for reference.

---

## 6. Option B — micro-ROS (deferred)

An alternative where the ESP32 becomes a native ROS 2 node via `micro_ros_agent`,
removing the custom protocol. Kept on the shelf, not built.
See [`docs/option-b-micro-ros.md`](../docs/option-b-micro-ros.md).
