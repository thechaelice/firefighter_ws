// ============================================================
// FIREFIGHTING ROBOT - RECEIVER ESP32
// ESP32 Arduino Core 2.0.0
//
// SEQUENCE:
// FIRE = YES + SMOKE = YES twice consecutively
//
//          ↓
//
// FORWARD 5 seconds
// STOP    0.5 seconds
// RIGHT   3 seconds
// STOP    0.5 seconds
// LEFT    5 seconds
// STOP
//
//          ↓
//
// Resume listening to sender
//
// During movement, incoming ESP-NOW sensor commands are ignored.
// ============================================================

#include <WiFi.h>
#include <esp_now.h>


// ============================================================
// MOTOR DRIVER PINS
// ============================================================

// Motor 1
const int ENA = 14;
const int IN1 = 27;
const int IN2 = 26;

// Motor 2
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


// Encoder counters
// Not used for movement control yet.
volatile long motor1_encoder_count = 0;
volatile long motor2_encoder_count = 0;


// ============================================================
// PWM SETTINGS
// ESP32 Arduino Core 2.0.0
// ============================================================

const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;

const int PWM_FREQ = 5000;

// 8-bit PWM:
// 0   = OFF
// 255 = maximum
const int PWM_RESOLUTION = 8;


// ============================================================
// MOTOR SPEED
// ============================================================

// Range = 0 - 255
// Starting at about 71%
const int MOTOR_SPEED = 180;


// ============================================================
// ESP-NOW MESSAGE STRUCTURE
//
// MUST MATCH THE SENDER EXACTLY
// ============================================================

typedef struct struct_message {

  int messageNumber;

  bool fireDetected;

  bool smokeDetected;

} struct_message;


struct_message incomingData;


// ============================================================
// DETECTION VARIABLES
// ============================================================

// Number of consecutive FIRE + SMOKE detections
volatile int consecutiveDetectionCount = 0;

// Require two consecutive packets
const int REQUIRED_DETECTIONS = 2;


// ============================================================
// MOVEMENT FLAGS
// ============================================================

// Tells loop() to start movement
volatile bool startMovement = false;

// TRUE while robot is moving
volatile bool movementInProgress = false;

// Prevents multiple triggers before movement starts
volatile bool sequenceTriggered = false;


// ============================================================
// MOTOR SPEED FUNCTION
// ============================================================

void setMotorSpeed(int speedA, int speedB) {

  ledcWrite(
    PWM_CHANNEL_A,
    speedA
  );

  ledcWrite(
    PWM_CHANNEL_B,
    speedB
  );
}


// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors() {

  // Motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Motor 2
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Disable motor outputs
  setMotorSpeed(0, 0);

  Serial.println(">>> MOTORS STOPPED");
}


// ============================================================
// MOVE FORWARD
//
// IMPORTANT:
// Based on your actual physical motor test.
//
// The motors are mounted as mirror images.
// Therefore they need opposite ELECTRICAL directions
// to propel the robot in the same PHYSICAL direction.
// ============================================================

void moveForward() {

  Serial.println(">>> MOVING FORWARD");

  // Motor 1
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor 2 - opposite electrical direction
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Both motors same PWM speed
  setMotorSpeed(
    MOTOR_SPEED,
    MOTOR_SPEED
  );
}


// ============================================================
// TURN RIGHT
//
// Both motors receive the same electrical direction.
// Because the motors are physically mirrored, this should
// make the wheels oppose each other and rotate the chassis.
// ============================================================

void turnRight() {

  Serial.println(">>> TURNING RIGHT");

  // Motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Motor 2
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  setMotorSpeed(
    MOTOR_SPEED,
    MOTOR_SPEED
  );
}


// ============================================================
// TURN LEFT
//
// Opposite of RIGHT
// ============================================================

void turnLeft() {

  Serial.println(">>> TURNING LEFT");

  // Motor 1
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor 2
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  setMotorSpeed(
    MOTOR_SPEED,
    MOTOR_SPEED
  );
}


// ============================================================
// COMPLETE MOVEMENT SEQUENCE
// ============================================================

void executeMovementSequence() {

  // ==========================================================
  // ENTER MOVEMENT MODE
  // ==========================================================

  movementInProgress = true;

  // Old detections are no longer needed
  consecutiveDetectionCount = 0;


  Serial.println();
  Serial.println("========================================");
  Serial.println(" FIRE + SMOKE CONFIRMED TWICE");
  Serial.println("========================================");

  Serial.println(
    "Incoming sensor commands are now ignored."
  );

  Serial.println(
    "Starting movement sequence..."
  );

  Serial.println();


  // ==========================================================
  // STEP 1
  // FORWARD FOR 5 SECONDS
  // ==========================================================

  Serial.println("----------------------------------------");
  Serial.println("STEP 1: FORWARD");
  Serial.println("TIME: 5 SECONDS");
  Serial.println("----------------------------------------");

  moveForward();


  // Run forward for 5 seconds
  delay(5000);


  // Stop
  stopMotors();


  // Pause before next movement
  delay(500);


  // ==========================================================
  // STEP 2
  // TURN RIGHT FOR 3 SECONDS
  // ==========================================================

  Serial.println();

  Serial.println("----------------------------------------");
  Serial.println("STEP 2: RIGHT");
  Serial.println("TIME: 3 SECONDS");
  Serial.println("----------------------------------------");

  turnRight();


  // Turn right for 3 seconds
  delay(3000);


  // Stop
  stopMotors();


  // Pause
  delay(500);


  // ==========================================================
  // STEP 3
  // TURN LEFT FOR 5 SECONDS
  // ==========================================================

  Serial.println();

  Serial.println("----------------------------------------");
  Serial.println("STEP 3: LEFT");
  Serial.println("TIME: 5 SECONDS");
  Serial.println("----------------------------------------");

  turnLeft();


  // Turn left for 5 seconds
  delay(5000);


  // ==========================================================
  // FINAL STOP
  // ==========================================================

  stopMotors();


  Serial.println();

  Serial.println("========================================");
  Serial.println(" MOVEMENT SEQUENCE COMPLETE");
  Serial.println("========================================");


  // ==========================================================
  // CLEAR OLD SENSOR DATA
  // ==========================================================

  incomingData.fireDetected = false;

  incomingData.smokeDetected = false;

  consecutiveDetectionCount = 0;

  startMovement = false;

  sequenceTriggered = false;


  Serial.println();

  Serial.println(
    "Old FIRE/SMOKE detection cleared."
  );

  Serial.println(
    "Detection counter reset to 0."
  );


  // ==========================================================
  // ALLOW SENSOR PACKETS AGAIN
  // ==========================================================

  movementInProgress = false;


  Serial.println();

  Serial.println(
    ">>> LISTENING TO SENDER AGAIN <<<"
  );

  Serial.println(
    "Two NEW FIRE+SMOKE detections required."
  );

  Serial.println();
}


// ============================================================
// ESP-NOW RECEIVE CALLBACK
//
// Compatible with ESP32 Arduino Core 2.0.0
// ============================================================

void OnDataRecv(
  const uint8_t *mac_addr,
  const uint8_t *incomingDataBytes,
  int len
) {

  // ==========================================================
  // IGNORE ALL SENSOR COMMANDS DURING MOVEMENT
  // ==========================================================

  if (movementInProgress == true) {

    return;
  }


  // ==========================================================
  // CHECK PACKET SIZE
  // ==========================================================

  if (len != sizeof(incomingData)) {

    Serial.print(
      "ERROR: Wrong packet size: "
    );

    Serial.println(len);

    return;
  }


  // ==========================================================
  // COPY RECEIVED DATA
  // ==========================================================

  memcpy(
    &incomingData,
    incomingDataBytes,
    sizeof(incomingData)
  );


  // ==========================================================
  // DISPLAY RECEIVED DATA
  // ==========================================================

  Serial.println();

  Serial.println(
    "----------------------------------------"
  );


  Serial.print("MESSAGE #");

  Serial.println(
    incomingData.messageNumber
  );


  // ==========================================================
  // FIRE STATUS
  // ==========================================================

  Serial.print("FIRE: ");

  if (incomingData.fireDetected == true) {

    Serial.println("YES");

  }

  else {

    Serial.println("NO");
  }


  // ==========================================================
  // SMOKE STATUS
  // ==========================================================

  Serial.print("SMOKE: ");

  if (incomingData.smokeDetected == true) {

    Serial.println("YES");

  }

  else {

    Serial.println("NO");
  }


  // ==========================================================
  // CHECK WHETHER BOTH FIRE AND SMOKE ARE PRESENT
  // ==========================================================

  if (
    incomingData.fireDetected == true &&
    incomingData.smokeDetected == true
  ) {

    // Increase consecutive count
    consecutiveDetectionCount++;


    Serial.print(
      "Consecutive FIRE+SMOKE detections: "
    );

    Serial.print(
      consecutiveDetectionCount
    );

    Serial.print("/");

    Serial.println(
      REQUIRED_DETECTIONS
    );


    // ========================================================
    // TWO CONSECUTIVE DETECTIONS
    // ========================================================

    if (
      consecutiveDetectionCount >= REQUIRED_DETECTIONS &&
      sequenceTriggered == false
    ) {

      Serial.println();

      Serial.println(
        "**************************************"
      );

      Serial.println(
        " FIRE + SMOKE CONFIRMED TWICE"
      );

      Serial.println(
        " MOVEMENT REQUESTED"
      );

      Serial.println(
        "**************************************"
      );


      // Prevent duplicate trigger
      sequenceTriggered = true;


      // Request movement
      startMovement = true;
    }
  }


  // ==========================================================
  // CONDITION NOT SATISFIED
  // ==========================================================

  else {

    // Consecutive detection has been broken
    consecutiveDetectionCount = 0;


    Serial.println(
      "FIRE + SMOKE condition not satisfied."
    );

    Serial.println(
      "Detection counter reset to 0."
    );
  }


  Serial.println(
    "----------------------------------------"
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  // ==========================================================
  // SERIAL MONITOR
  // ==========================================================

  Serial.begin(115200);

  delay(1000);


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    " FIREFIGHTING ROBOT - RECEIVER ESP32"
  );

  Serial.println(
    "========================================"
  );


  // ==========================================================
  // MOTOR DIRECTION OUTPUTS
  // ==========================================================

  pinMode(IN1, OUTPUT);

  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);

  pinMode(IN4, OUTPUT);


  // ==========================================================
  // ENCODER INPUTS
  // ==========================================================

  pinMode(M1_ENC_A, INPUT);

  pinMode(M1_ENC_B, INPUT);

  pinMode(M2_ENC_A, INPUT);

  pinMode(M2_ENC_B, INPUT);


  // ==========================================================
  // CONFIGURE MOTOR PWM
  // ==========================================================

  Serial.println(
    "Configuring motor PWM..."
  );


  // Motor 1 PWM
  ledcSetup(
    PWM_CHANNEL_A,
    PWM_FREQ,
    PWM_RESOLUTION
  );


  // Motor 2 PWM
  ledcSetup(
    PWM_CHANNEL_B,
    PWM_FREQ,
    PWM_RESOLUTION
  );


  // Attach Motor 1 enable
  ledcAttachPin(
    ENA,
    PWM_CHANNEL_A
  );


  // Attach Motor 2 enable
  ledcAttachPin(
    ENB,
    PWM_CHANNEL_B
  );


  // ==========================================================
  // SAFETY STOP AT STARTUP
  // ==========================================================

  stopMotors();


  // ==========================================================
  // WIFI
  // ==========================================================

  WiFi.mode(WIFI_STA);

  delay(100);


  Serial.print(
    "Receiver MAC Address: "
  );

  Serial.println(
    WiFi.macAddress()
  );


  // ==========================================================
  // INITIALIZE ESP-NOW
  // ==========================================================

  if (esp_now_init() != ESP_OK) {

    Serial.println();

    Serial.println(
      "ERROR: ESP-NOW initialization failed!"
    );


    // Keep motors stopped
    stopMotors();

    return;
  }


  // ==========================================================
  // REGISTER RECEIVE CALLBACK
  // ==========================================================

  esp_now_register_recv_cb(
    OnDataRecv
  );


  Serial.println();

  Serial.println(
    "ESP-NOW initialized successfully."
  );


  Serial.println();

  Serial.println(
    "Waiting for FIRE + SMOKE..."
  );

  Serial.println(
    "Two consecutive detections required."
  );

  Serial.println();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // ==========================================================
  // CHECK FOR MOVEMENT REQUEST
  // ==========================================================

  if (
    startMovement == true &&
    movementInProgress == false
  ) {

    // Enter movement mode immediately.
    //
    // Any new sensor packets arriving after this
    // point will be ignored until movement finishes.

    movementInProgress = true;


    // Clear movement request
    startMovement = false;


    // ========================================================
    // EXECUTE THE ENTIRE MOVEMENT BEFORE ACCEPTING
    // NEW SENSOR COMMANDS
    // ========================================================

    executeMovementSequence();
  }


  delay(10);
}