# Fire/smoke alert forwarded from the ESP-NOW beacon by the mobility ESP32.
# Mirrors the MSG_BEACON_EVENT payload in firefighter_protocol.h.
std_msgs/Header header

uint32 message_number   # rolling counter from the beacon
bool fire_detected
bool smoke_detected
uint8 rssi              # 0 when the ESP32 core cannot report it
uint8[6] mac            # beacon MAC address
