// ============================================================
// FIREFIGHTING ROBOT - WIRELESS FIRE BEACON (SENDER)
// ESP32 Arduino Core 2.0.x
//
// ROLE
//   * Senses fire (HL-01 IR flame sensor) and smoke (MQ-2)
//   * Periodically broadcasts/unicasts alert packets to the
//     mobility controller ESP32 via ESP-NOW
//
// !! NON-BLOCKING: the control and transmission path uses
//    millis() only. No blocking delay() calls in loop().
// ============================================================

#include <WiFi.h>
#include <esp_now.h>

// ============================================================
// RECEIVER DISCOVERY (Robot Mobility ESP32)
//
// The receiver MAC is not hardcoded. The beacon broadcasts a
// PAIR_REQUEST until the mobility ESP32 answers with a PAIR_REPLY,
// then takes the receiver MAC from the source address of that reply.
// If the receiver stops acknowledging packets the beacon forgets it
// and starts discovering again.
// ============================================================
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Pairing message (MUST MATCH robot_mobility.ino EXACTLY).
// Its size differs from struct_message, so the two never get confused.
const uint32_t PAIR_MAGIC   = 0x46465052;   // "FFPR"
const uint8_t  PAIR_REQUEST = 1;            // beacon   -> broadcast
const uint8_t  PAIR_REPLY   = 2;            // mobility -> broadcast

typedef struct __attribute__((packed)) pair_message {
  uint32_t magic;
  uint8_t  type;
} pair_message;

const uint32_t DISCOVERY_PERIOD_MS = 500;   // PAIR_REQUEST interval while searching
const int      MAX_SEND_FAILURES   = 5;     // consecutive failed packets -> rediscover

uint8_t receiverMAC[6] = {0};
bool    receiverKnown  = false;
uint32_t lastDiscoveryMs = 0;

// Written by the ESP-NOW callbacks (Wi-Fi task), consumed in loop()
volatile bool pairReplyPending = false;
uint8_t       pairReplyMAC[6]  = {0};
volatile int  sendFailCount    = 0;


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
// SENSOR / LED PINS
// ============================================================
const int FLAME_PIN = 18;   // HL-01 IR flame sensor, D0
const int SMOKE_PIN = 19;   // MQ-2 smoke sensor, D0
const int LED_PIN   = 2;    // alarm LED

// Both modules use an LM393 comparator whose D0 output goes LOW
// when the threshold (set by the on-board trimmer) is exceeded.
// Flip these if a module reads the other way round.
const int FLAME_ACTIVE_LEVEL = LOW;
const int SMOKE_ACTIVE_LEVEL = LOW;


// ============================================================
// STATE
// ============================================================
int messageCounter = 0;

// Detections latched since the last packet, so an event shorter
// than BEACON_PERIOD_MS is still reported.
bool fireLatched  = false;
bool smokeLatched = false;


// ============================================================
// SEND CALLBACK
// ============================================================
// ESP-IDF 5.5 (Arduino core 3.3.x) changed the first argument of the
// send callback from the peer MAC to a wifi_tx_info_t.
#if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0))
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  if (!receiverKnown) return;   // discovery broadcasts are never acknowledged

  Serial.print("Delivery Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("SUCCESS");
    sendFailCount = 0;
  } else {
    Serial.println("FAILED");
    sendFailCount = sendFailCount + 1;
  }
}


// ============================================================
// RECEIVE CALLBACK (pairing replies only)
// ============================================================
void handlePairReply(const uint8_t *mac, const uint8_t *data, int len) {
  if (receiverKnown || pairReplyPending) return;
  if (!mac || len != (int)sizeof(pair_message)) return;

  pair_message msg;
  memcpy(&msg, data, sizeof(msg));
  if (msg.magic != PAIR_MAGIC || msg.type != PAIR_REPLY) return;

  memcpy(pairReplyMAC, mac, 6);
  pairReplyPending = true;
}

#if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handlePairReply(info ? info->src_addr : nullptr, data, len);
}
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  handlePairReply(mac_addr, data, len);
}
#endif


// ============================================================
// RECEIVER DISCOVERY
// ============================================================
void sendPairRequest() {
  pair_message msg = { PAIR_MAGIC, PAIR_REQUEST };
  esp_now_send(BROADCAST_MAC, (const uint8_t *)&msg, sizeof(msg));
  Serial.println("Searching for mobility ESP32...");
}

void forgetReceiver() {
  esp_now_del_peer(receiverMAC);
  receiverKnown = false;
  sendFailCount = 0;
  Serial.println("Receiver lost. Restarting discovery.");
}

// Runs in loop(): finish a pending pairing, drop a dead receiver,
// and keep broadcasting PAIR_REQUEST while no receiver is known.
void updateDiscovery(uint32_t now) {
  if (receiverKnown && sendFailCount >= MAX_SEND_FAILURES) {
    forgetReceiver();
  }

  if (pairReplyPending) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, pairReplyMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      memcpy(receiverMAC, pairReplyMAC, 6);
      sendFailCount = 0;
      receiverKnown = true;
      Serial.printf("Receiver found: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    receiverMAC[0], receiverMAC[1], receiverMAC[2],
                    receiverMAC[3], receiverMAC[4], receiverMAC[5]);
    } else {
      Serial.println("ERROR: Failed to add receiver!");
    }
    pairReplyPending = false;
  }

  if (!receiverKnown && now - lastDiscoveryMs >= DISCOVERY_PERIOD_MS) {
    lastDiscoveryMs = now;
    sendPairRequest();
  }
}


// ============================================================
// SENSOR ACQUISITION & TRANSMISSION
// ============================================================
void readSensors(bool &fire, bool &smoke) {
  fire  = digitalRead(FLAME_PIN) == FLAME_ACTIVE_LEVEL;
  smoke = digitalRead(SMOKE_PIN) == SMOKE_ACTIVE_LEVEL;
}

// Poll the sensors, latch any detection for the next packet and
// drive the alarm LED from the live readings.
void updateSensors() {
  bool fire, smoke;
  readSensors(fire, smoke);

  if (fire)  fireLatched  = true;
  if (smoke) smokeLatched = true;

  digitalWrite(LED_PIN, (fire || smoke) ? HIGH : LOW);
}

void sendBeaconPacket() {
  messageCounter++;
  outgoingData.messageNumber = messageCounter;

  // Report anything detected since the previous packet
  outgoingData.fireDetected  = fireLatched;
  outgoingData.smokeDetected = smokeLatched;
  fireLatched  = false;
  smokeLatched = false;

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

  // Sensors + alarm LED. Pull-ups keep a disconnected sensor
  // reading "no detection" instead of floating.
  pinMode(FLAME_PIN, INPUT_PULLUP);
  pinMode(SMOKE_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Broadcast peer, used to find the receiver
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add broadcast peer!");
    return;
  }

  Serial.println();
  Serial.println("Beginning receiver discovery...");
  Serial.println("================================");
}


// ============================================================
// MAIN LOOP (no delay() in transmission path)
// ============================================================
void loop() {
  uint32_t now = millis();

  // ---- Sensors + alarm LED ---------------------------------
  updateSensors();

  // ---- Receiver discovery ---------------------------------
  updateDiscovery(now);

  // ---- Periodic Beacon Transmission ------------------------
  // Detections stay latched until a receiver is known.
  if (receiverKnown && now - lastSendMs >= BEACON_PERIOD_MS) {
    lastSendMs = now;
    sendBeaconPacket();
  }

  // ---- Yield for FreeRTOS ----------------------------------
  delay(1);
}
