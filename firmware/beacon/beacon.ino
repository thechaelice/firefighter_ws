// ============================================================
// FIREFIGHTING ROBOT - WIRELESS FIRE BEACON (SENDER)
// ESP32 Arduino Core 2.0.x
//
// ROLE
//   * Senses fire and smoke conditions (or simulates them)
//   * Periodically broadcasts/unicasts alert packets to the
//     mobility controller ESP32 via ESP-NOW
//
// !! NON-BLOCKING: the control and transmission path uses
//    millis() only. No blocking delay() calls in loop().
// ============================================================

#include <WiFi.h>
#include <esp_now.h>

// ============================================================
// RECEIVER MAC ADDRESS (Robot Mobility ESP32)
// Update this with the MAC address printed by robot_mobility.ino
// ============================================================
uint8_t receiverMAC[] = {
  0x3C, 0x8A, 0x1F, 0x7E, 0x36, 0x8C
};


// ============================================================
// MESSAGE STRUCTURE (MUST MATCH RECEIVER EXACTLY)
// ============================================================
typedef struct struct_message {
  int  messageNumber;
  bool fireDetected;
  bool smokeDetected;
} struct_message;

struct_message outgoingData;


// ============================================================
// TIMING (all non-blocking, millis() based)
// ============================================================
const uint32_t BEACON_PERIOD_MS = 2000;   // transmission period (2 seconds)
uint32_t lastSendMs = 0;


// ============================================================
// STATE
// ============================================================
int messageCounter = 0;


// ============================================================
// SEND CALLBACK
// ============================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Delivery Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("SUCCESS");
  } else {
    Serial.println("FAILED");
  }
}


// ============================================================
// SENSOR ACQUISITION & TRANSMISSION
// ============================================================
void readSensors(bool &fire, bool &smoke) {
  // Currently simulating sensor states.
  // In real hardware, read digital/analog GPIO pins here.
  fire = false;
  smoke = false;
}

void sendBeaconPacket() {
  messageCounter++;
  outgoingData.messageNumber = messageCounter;

  // Read current fire/smoke conditions
  readSensors(outgoingData.fireDetected, outgoingData.smokeDetected);

  Serial.println();
  Serial.println("--------------------------------");
  Serial.print("Sending Message #");
  Serial.println(outgoingData.messageNumber);

  Serial.print("Fire: ");
  Serial.println(outgoingData.fireDetected ? "YES" : "NO");

  Serial.print("Smoke: ");
  Serial.println(outgoingData.smokeDetected ? "YES" : "NO");

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(
    receiverMAC,
    (uint8_t *)&outgoingData,
    sizeof(outgoingData)
  );

  if (result == ESP_OK) {
    Serial.println("Packet submitted to ESP-NOW.");
  } else {
    Serial.print("ESP-NOW send error: ");
    Serial.println(result);
  }
  Serial.println("--------------------------------");
}


// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);   // one-off, pre-loop setup delay

  Serial.println();
  Serial.println("================================");
  Serial.println("   ESP32 ESP-NOW BEACON SENDER");
  Serial.println("================================");

  // WiFi station mode
  WiFi.mode(WIFI_STA);
  delay(100);   // one-off, pre-loop setup delay

  Serial.print("Sender MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW initialization failed!");
    return;
  }
  Serial.println("ESP-NOW initialized successfully.");

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register receiver as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add receiver!");
    return;
  }

  Serial.println("Receiver added successfully.");
  Serial.println();
  Serial.println("Beginning transmission...");
  Serial.println("================================");
}


// ============================================================
// MAIN LOOP (no delay() in transmission path)
// ============================================================
void loop() {
  uint32_t now = millis();

  // ---- Periodic Beacon Transmission ------------------------
  if (now - lastSendMs >= BEACON_PERIOD_MS) {
    lastSendMs = now;
    sendBeaconPacket();
  }

  // ---- Yield for FreeRTOS ----------------------------------
  delay(1);
}
