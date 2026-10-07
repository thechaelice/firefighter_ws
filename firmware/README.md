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
1. Flash `robot_mobility.ino` to the robot ESP32 and open the Serial Monitor (115200 baud).
2. Note the printed Receiver MAC Address:
   ```text
   Receiver MAC Address: XX:XX:XX:XX:XX:XX
   ```
3. Enter this MAC address in `firmware/beacon/beacon.ino` inside the `broadcastAddress` array:
   ```cpp
   uint8_t broadcastAddress[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};
   ```

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
