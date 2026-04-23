#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <math.h>

// ============================================================
// STAIR Robot — One-Leg-at-a-Time Walking Test
// MERGED WITH SHELL + IMU FLIP/RECOVERY LOGIC
//
// Web UI:
// - ESP32 creates Wi-Fi hotspot
// - webpage has START and STOP
// - START waits 2s, opens shell, then begins walking
// - STOP is graceful and only stops at a full 4-leg boundary
// - IMU pitch > threshold latches a flip request
// - flip executes at next even move boundary
// - after flip, robot holds active brake until pitch is stable
//   within ±5 deg for 5 seconds
// - then shell reopens, all 4 legs rotate one cycle each,
//   and normal walking resumes
//
// Walking pattern:
//   Move 1: RL  (double cycle)
//   Move 2: FL  (double cycle)
//   Move 3: RR  (double cycle)
//   Move 4: FR  (double cycle)
//   Then repeat...
// ============================================================

// ---------------- Web UI ----------------
const char* AP_SSID = "STAIR-Robot";
const char* AP_PASS = "12345678";
WebServer server(80);

bool runEnabled      = false;
bool stopRequested   = false;
bool cycleInProgress = false;

unsigned long completedCycles    = 0;   // completed LEG MOVES
unsigned long currentCycleNumber = 0;   // current LEG MOVE number

// ---------------- Shell servo ----------------
Servo shellServo;

const int SERVO_PIN      = 8;
const int SERVO_STOP_US  = 1500;
const int SERVO_OPEN_US  = 600;
const int SERVO_CLOSE_US = 2400;

const unsigned long SHELL_PRESTART_DELAY_MS    = 2000;
const unsigned long SHELL_OPEN_TIME_MS         = 1500;
const unsigned long SHELL_CLOSE_TIME_MS_NORMAL = 1340;
const unsigned long SHELL_CLOSE_TIME_MS_LATE   = 1050;
const unsigned long LATE_STOP_WINDOW_MS        = 5000;

bool shellIsOpen = false;
unsigned long startButtonPressedMs = 0;
bool useLateShellClose = false;

// ---------------- IMU ----------------
MPU6050 mpu6050(Wire);

const int SDA_PIN = 36;
const int SCL_PIN = 35;

float angle_offset = 0.0f;
float pitchDeg = 0.0f;

const float PITCH_FLIP_THRESHOLD_DEG = 15.0f;
const float PITCH_RECOVERY_BAND_DEG  = 5.0f;
const unsigned long RECOVERY_STABLE_TIME_MS = 5000;

bool flipRequested      = false;
bool flipInProgress     = false;
bool waitingForRecovery = false;
bool rephaseInProgress  = false;

unsigned long recoveryStableStartMs = 0;

// ---------------- Rear motor pins ---------------
#define MOTOR_A_INA   2
#define MOTOR_A_INB   1
#define MOTOR_A_PWM   3

#define MOTOR_B_INA   4
#define MOTOR_B_INB   5
#define MOTOR_B_PWM   6

// ---------------- Rear encoder pins ----------------
#define ENC_A_PIN     9   // rear left
#define ENC_B_PIN     7   // rear right

// ---------------- Front motor pins ----------------
#define FL_IN1        42
#define FL_IN2        41

#define FR_IN3        40
#define FR_IN4        39

// ---------------- Front encoder pins ----------------
#define ENC_FL_PIN    12
#define ENC_FR_PIN    13

// ---------------- Counts ----------------
const long REAR_LEFT_COUNTS_PER_CYCLE   = 700;
const long REAR_RIGHT_COUNTS_PER_CYCLE  = 700;
const long FRONT_LEFT_COUNTS_PER_CYCLE  = 600;
const long FRONT_RIGHT_COUNTS_PER_CYCLE = 605;

const int LEG_MOVE_MULTIPLIER = 2;  // normal walking = 2x cycle per active leg

// 0 = RL, 1 = FL, 2 = RR, 3 = FR
const uint8_t LEG_SEQUENCE[4] = {0, 1, 2, 3};

// ---------------- Speed tuning ----------------
// uniform fixed speeds, no ramp-down
#define REAR_SPEED   200
#define FRONT_SPEED  76
#define TIMEOUT_MS   8000

// ---------------- Flip tuning ----------------
const long REAR_FLIP_PULSE_COUNTS = 120;   // tune as needed
const uint8_t REAR_FLIP_PULSE_PWM = 255;
const unsigned long POST_FLIP_FRONT_DELAY_MS = 200;

// ---------------- Encoder counts ----------------
volatile long encCountA  = 0;   // rear left
volatile long encCountB  = 0;   // rear right
volatile long encCountFL = 0;   // front left
volatile long encCountFR = 0;   // front right

// ============================================================
// Forward declarations
// ============================================================
void holdAllStopped();
void setupWebServer();
String buildStatusJson();

void openShellBeforeWalking();
void reopenShellAfterRecovery();
void closeShellAfterStopping();

void updateIMU();
void handlePostFlipRecovery();
void performFlipSequence();

const char* legName(uint8_t legIndex);

bool walkOneLegMoveWithMultiplier(uint8_t legIndex, int moveMultiplier, unsigned long &elapsedMs);
bool walkOneLegMove(uint8_t legIndex, unsigned long &elapsedMs);

bool rotateRearBothOneCycle(unsigned long &elapsedMs);
bool rotateFrontBothOneCycle(unsigned long &elapsedMs);
bool rearFlipKickPulse(unsigned long &elapsedMs);
bool rephaseAllLegsOneCycle();

// ============================================================
// Web page
// ============================================================
const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>STAIR Robot</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 30px;
      background: #f4f4f4;
    }
    h1 {
      margin-bottom: 20px;
    }
    .card {
      max-width: 460px;
      margin: 0 auto;
      background: white;
      border-radius: 14px;
      padding: 20px;
      box-shadow: 0 3px 12px rgba(0,0,0,0.12);
    }
    .btnRow {
      display: flex;
      gap: 14px;
      justify-content: center;
      margin-top: 10px;
      flex-wrap: wrap;
    }
    button {
      font-size: 24px;
      padding: 16px 28px;
      border: none;
      border-radius: 12px;
      color: white;
      cursor: pointer;
      min-width: 140px;
    }
    .startBtn {
      background: #2a9d8f;
    }
    .stopBtn {
      background: #d62828;
    }
    .status {
      margin-top: 20px;
      font-size: 20px;
      line-height: 1.7;
    }
    .small {
      font-size: 15px;
      color: #555;
      margin-top: 10px;
    }
  </style>
</head>
<body>
  <div class="card">
    <h1>STAIR Robot</h1>

    <div class="btnRow">
      <button class="startBtn" onclick="sendStart()">START</button>
      <button class="stopBtn" onclick="sendStop()">STOP</button>
    </div>

    <div class="status">
      <div><b>State:</b> <span id="state">Loading...</span></div>
      <div><b>Current move:</b> <span id="currentCycle">-</span></div>
      <div><b>Completed moves:</b> <span id="completedCycles">-</span></div>
      <div><b>Stop requested:</b> <span id="stopRequested">-</span></div>
      <div><b>Pitch:</b> <span id="pitchDeg">-</span> deg</div>
    </div>

    <div class="small">
      START waits 2 seconds, opens shell, then begins one-leg-at-a-time walking.
      STOP finishes on the next full 4-leg boundary, then closes shell.
      IMU-triggered flip executes on the next even move boundary.
    </div>
  </div>

  <script>
    async function sendStart() {
      await fetch('/start', { method: 'POST' });
      updateStatus();
    }

    async function sendStop() {
      await fetch('/stop', { method: 'POST' });
      updateStatus();
    }

    async function updateStatus() {
      try {
        const r = await fetch('/status');
        const s = await r.json();
        document.getElementById('state').textContent = s.state;
        document.getElementById('currentCycle').textContent = s.currentCycle;
        document.getElementById('completedCycles').textContent = s.completedCycles;
        document.getElementById('stopRequested').textContent = s.stopRequested ? 'YES' : 'NO';
        document.getElementById('pitchDeg').textContent = s.pitchDeg;
      } catch (e) {
        document.getElementById('state').textContent = 'Disconnected';
      }
    }

    updateStatus();
    setInterval(updateStatus, 300);
  </script>
</body>
</html>
)rawliteral";

// ============================================================
// ISRs
// ============================================================
void IRAM_ATTR encoderA_ISR() {
  encCountA++;
}

void IRAM_ATTR encoderB_ISR() {
  encCountB++;
}

void IRAM_ATTR encoderFL_ISR() {
  encCountFL++;
}

void IRAM_ATTR encoderFR_ISR() {
  encCountFR++;
}

// ============================================================
// Helper labels
// ============================================================
const char* legName(uint8_t legIndex) {
  switch (legIndex) {
    case 0: return "RL";
    case 1: return "FL";
    case 2: return "RR";
    case 3: return "FR";
    default: return "?";
  }
}

// ============================================================
// IMU helpers
// ============================================================
void updateIMU() {
  mpu6050.update();
  pitchDeg = mpu6050.getAngleX() - angle_offset;
}

// ============================================================
// Shell helpers
// ============================================================
void openShellBeforeWalking() {
  if (shellIsOpen) {
    Serial.println("Shell already open.");
    return;
  }

  Serial.println("Waiting 2000 ms before opening shell...");
  delay(SHELL_PRESTART_DELAY_MS);

  Serial.println("Opening shell...");
  shellServo.writeMicroseconds(SERVO_OPEN_US);
  delay(SHELL_OPEN_TIME_MS);

  shellServo.writeMicroseconds(SERVO_STOP_US);
  shellIsOpen = true;

  Serial.println("Shell open complete.");
}

void reopenShellAfterRecovery() {
  if (shellIsOpen) {
    Serial.println("Shell already open.");
    return;
  }

  Serial.println("Reopening shell after recovery...");
  shellServo.writeMicroseconds(SERVO_OPEN_US);
  delay(SHELL_OPEN_TIME_MS);
  shellServo.writeMicroseconds(SERVO_STOP_US);
  shellIsOpen = true;

  Serial.println("Shell reopen complete.");
}

void closeShellAfterStopping() {
  if (!shellIsOpen) {
    Serial.println("Shell already closed.");
    return;
  }

  unsigned long closeTime = useLateShellClose ? SHELL_CLOSE_TIME_MS_LATE
                                              : SHELL_CLOSE_TIME_MS_NORMAL;

  Serial.print("Closing shell... using ");
  Serial.print(closeTime);
  Serial.println(" ms");

  shellServo.writeMicroseconds(SERVO_CLOSE_US);
  delay(closeTime);

  shellServo.writeMicroseconds(SERVO_STOP_US);
  shellIsOpen = false;
  useLateShellClose = false;

  Serial.println("Shell close complete.");
}

// ============================================================
// Rear motor control
// ============================================================
void rearLeftForward(uint8_t speed) {
  digitalWrite(MOTOR_A_INA, HIGH);
  digitalWrite(MOTOR_A_INB, LOW);
  ledcWrite(MOTOR_A_PWM, speed);
}

void rearRightForward(uint8_t speed) {
  digitalWrite(MOTOR_B_INA, HIGH);
  digitalWrite(MOTOR_B_INB, LOW);
  ledcWrite(MOTOR_B_PWM, speed);
}

void rearLeftReverse(uint8_t speed) {
  digitalWrite(MOTOR_A_INA, LOW);
  digitalWrite(MOTOR_A_INB, HIGH);
  ledcWrite(MOTOR_A_PWM, speed);
}

void rearRightReverse(uint8_t speed) {
  digitalWrite(MOTOR_B_INA, LOW);
  digitalWrite(MOTOR_B_INB, HIGH);
  ledcWrite(MOTOR_B_PWM, speed);
}

// Rear active brake hold
void rearLeftStop() {
  digitalWrite(MOTOR_A_INA, HIGH);
  digitalWrite(MOTOR_A_INB, HIGH);
  ledcWrite(MOTOR_A_PWM, 255);
}

void rearRightStop() {
  digitalWrite(MOTOR_B_INA, HIGH);
  digitalWrite(MOTOR_B_INB, HIGH);
  ledcWrite(MOTOR_B_PWM, 255);
}

// ============================================================
// Front motor control
// ============================================================
void frontLeftForward(uint8_t speed) {
  ledcWrite(FL_IN1, speed);
  digitalWrite(FL_IN2, LOW);
}

void frontRightForward(uint8_t speed) {
  ledcWrite(FR_IN3, speed);
  digitalWrite(FR_IN4, LOW);
}

void frontLeftBrake() {
  ledcWrite(FL_IN1, 255);
  digitalWrite(FL_IN2, HIGH);
}

void frontRightBrake() {
  ledcWrite(FR_IN3, 255);
  digitalWrite(FR_IN4, HIGH);
}

void frontLeftRelease() {
  ledcWrite(FL_IN1, 0);
  digitalWrite(FL_IN2, LOW);
}

void frontRightRelease() {
  ledcWrite(FR_IN3, 0);
  digitalWrite(FR_IN4, LOW);
}

// ============================================================
// Global helpers
// ============================================================
void stopRearMotors() {
  rearLeftStop();
  rearRightStop();
}

void brakeFrontMotors() {
  frontLeftBrake();
  frontRightBrake();
}

void releaseFrontMotors() {
  frontLeftRelease();
  frontRightRelease();
}

void holdAllStopped() {
  stopRearMotors();
  brakeFrontMotors();
}

// ============================================================
// Web server / status
// ============================================================
String buildStatusJson() {
  String state;

  if (flipInProgress) {
    state = "FLIP";
  } else if (rephaseInProgress) {
    state = "REPHASE";
  } else if (waitingForRecovery) {
    state = "RECOVERY HOLD";
  } else if (flipRequested) {
    state = "FLIP REQUESTED";
  } else if (!runEnabled && completedCycles == 0 && !stopRequested) {
    state = "IDLE";
  } else if (!runEnabled && completedCycles > 0) {
    state = "STOPPED";
  } else if (stopRequested) {
    state = "STOP REQUESTED";
  } else {
    state = "RUNNING";
  }

  unsigned long shownCycle = cycleInProgress ? currentCycleNumber : (completedCycles + 1);

  String json = "{";
  json += "\"state\":\"" + state + "\",";
  json += "\"currentCycle\":" + String(shownCycle) + ",";
  json += "\"completedCycles\":" + String(completedCycles) + ",";
  json += "\"stopRequested\":" + String(stopRequested ? "true" : "false") + ",";
  json += "\"pitchDeg\":" + String(pitchDeg, 2);
  json += "}";

  return json;
}

void setupWebServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println();
  Serial.print("Wi-Fi SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Wi-Fi PASS: ");
  Serial.println(AP_PASS);
  Serial.print("Open browser at: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", WEB_PAGE);
  });

  server.on("/status", HTTP_GET, []() {
    server.send(200, "application/json", buildStatusJson());
  });

  server.on("/start", HTTP_POST, []() {
    if (!runEnabled && !cycleInProgress && !waitingForRecovery && !flipInProgress && !rephaseInProgress) {
      completedCycles = 0;
      currentCycleNumber = 0;
      stopRequested = false;
      flipRequested = false;
      flipInProgress = false;
      waitingForRecovery = false;
      rephaseInProgress = false;
      recoveryStableStartMs = 0;
      useLateShellClose = false;
      startButtonPressedMs = millis();

      holdAllStopped();

      Serial.println("Web START requested");
      openShellBeforeWalking();

      runEnabled = true;
      Serial.println("Robot starting one-leg-at-a-time walking from move 1");
    }
    server.send(200, "text/plain", "STARTED");
  });

  server.on("/stop", HTTP_POST, []() {
    stopRequested = true;

    unsigned long dt = millis() - startButtonPressedMs;
    useLateShellClose = (dt > LATE_STOP_WINDOW_MS);

    Serial.println("Web STOP requested");
    Serial.print("Time since START press = ");
    Serial.print(dt);
    Serial.println(" ms");

    if (useLateShellClose) {
      Serial.println("STOP was pressed AFTER 5 seconds: shell close will use 1050 ms.");
    } else {
      Serial.println("STOP was pressed within 5 seconds: shell close will use 1340 ms.");
    }

    Serial.println("Robot will stop at the next full 4-leg boundary, then close shell");
    server.send(200, "text/plain", "STOP_REQUESTED");
  });

  server.begin();
  Serial.println("Web server started");
}

// ============================================================
// Generic one-leg move
//
// legIndex:
//   0 -> RL
//   1 -> FL
//   2 -> RR
//   3 -> FR
// ============================================================
bool walkOneLegMoveWithMultiplier(uint8_t legIndex, int moveMultiplier, unsigned long &elapsedMs) {
  encCountA  = 0;
  encCountB  = 0;
  encCountFL = 0;
  encCountFR = 0;

  bool rlActive = (legIndex == 0);
  bool flActive = (legIndex == 1);
  bool rrActive = (legIndex == 2);
  bool frActive = (legIndex == 3);

  bool rlDone = !rlActive;
  bool flDone = !flActive;
  bool rrDone = !rrActive;
  bool frDone = !frActive;

  const long rlTarget = REAR_LEFT_COUNTS_PER_CYCLE   * moveMultiplier;
  const long rrTarget = REAR_RIGHT_COUNTS_PER_CYCLE  * moveMultiplier;
  const long flTarget = FRONT_LEFT_COUNTS_PER_CYCLE  * moveMultiplier;
  const long frTarget = FRONT_RIGHT_COUNTS_PER_CYCLE * moveMultiplier;

  const unsigned long moveTimeout = TIMEOUT_MS * moveMultiplier;
  unsigned long startTime = millis();

  // Hold inactive legs
  if (!rlActive) rearLeftStop();
  if (!rrActive) rearRightStop();
  if (!flActive) frontLeftBrake();
  if (!frActive) frontRightBrake();

  // Start only the active leg at fixed speed
  if (rlActive) rearLeftForward(REAR_SPEED);
  if (rrActive) rearRightForward(REAR_SPEED);
  if (flActive) frontLeftForward(FRONT_SPEED);
  if (frActive) frontRightForward(FRONT_SPEED);

  while (!(rlDone && rrDone && flDone && frDone)) {
    server.handleClient();
    yield();
    updateIMU();

    if (millis() - startTime > moveTimeout) {
      holdAllStopped();

      Serial.println("TIMEOUT - active leg did not finish move");
      Serial.print("Active leg = ");
      Serial.println(legName(legIndex));
      Serial.print("RL counts = "); Serial.println(encCountA);
      Serial.print("RR counts = "); Serial.println(encCountB);
      Serial.print("FL counts = "); Serial.println(encCountFL);
      Serial.print("FR counts = "); Serial.println(encCountFR);
      return false;
    }

    // Keep inactive legs held
    if (!rlActive) rearLeftStop();
    if (!rrActive) rearRightStop();
    if (!flActive) frontLeftBrake();
    if (!frActive) frontRightBrake();

    // Rear Left
    if (!rlDone) {
      if (encCountA >= rlTarget) {
        rearLeftStop();
        rlDone = true;
      } else {
        ledcWrite(MOTOR_A_PWM, REAR_SPEED);
      }
    }

    // Rear Right
    if (!rrDone) {
      if (encCountB >= rrTarget) {
        rearRightStop();
        rrDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_SPEED);
      }
    }

    // Front Left
    if (!flDone) {
      if (encCountFL >= flTarget) {
        frontLeftBrake();
        flDone = true;
      } else {
        ledcWrite(FL_IN1, FRONT_SPEED);
      }
    }

    // Front Right
    if (!frDone) {
      if (encCountFR >= frTarget) {
        frontRightBrake();
        frDone = true;
      } else {
        ledcWrite(FR_IN3, FRONT_SPEED);
      }
    }
  }

  elapsedMs = millis() - startTime;
  return true;
}

bool walkOneLegMove(uint8_t legIndex, unsigned long &elapsedMs) {
  return walkOneLegMoveWithMultiplier(legIndex, LEG_MOVE_MULTIPLIER, elapsedMs);
}

// ============================================================
// Flip / recovery helpers
// ============================================================
bool rotateRearBothOneCycle(unsigned long &elapsedMs) {
  encCountA = 0;
  encCountB = 0;

  bool rlDone = false;
  bool rrDone = false;

  unsigned long startTime = millis();

  rearLeftForward(REAR_SPEED);
  rearRightForward(REAR_SPEED);

  while (!(rlDone && rrDone)) {
    server.handleClient();
    yield();
    updateIMU();

    if (millis() - startTime > TIMEOUT_MS) {
      holdAllStopped();
      return false;
    }

    if (!rlDone) {
      if (encCountA >= REAR_LEFT_COUNTS_PER_CYCLE) {
        rearLeftStop();
        rlDone = true;
      } else {
        ledcWrite(MOTOR_A_PWM, REAR_SPEED);
      }
    }

    if (!rrDone) {
      if (encCountB >= REAR_RIGHT_COUNTS_PER_CYCLE) {
        rearRightStop();
        rrDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_SPEED);
      }
    }
  }

  holdAllStopped();
  elapsedMs = millis() - startTime;
  return true;
}

bool rotateFrontBothOneCycle(unsigned long &elapsedMs) {
  encCountFL = 0;
  encCountFR = 0;

  bool flDone = false;
  bool frDone = false;

  unsigned long startTime = millis();

  frontLeftForward(FRONT_SPEED);
  frontRightForward(FRONT_SPEED);

  while (!(flDone && frDone)) {
    server.handleClient();
    yield();
    updateIMU();

    if (millis() - startTime > TIMEOUT_MS) {
      holdAllStopped();
      return false;
    }

    if (!flDone) {
      if (encCountFL >= FRONT_LEFT_COUNTS_PER_CYCLE) {
        frontLeftBrake();
        flDone = true;
      } else {
        ledcWrite(FL_IN1, FRONT_SPEED);
      }
    }

    if (!frDone) {
      if (encCountFR >= FRONT_RIGHT_COUNTS_PER_CYCLE) {
        frontRightBrake();
        frDone = true;
      } else {
        ledcWrite(FR_IN3, FRONT_SPEED);
      }
    }
  }

  holdAllStopped();
  elapsedMs = millis() - startTime;
  return true;
}

bool rearFlipKickPulse(unsigned long &elapsedMs) {
  unsigned long startTime = millis();

  // short forward pulse
  encCountA = 0;
  encCountB = 0;
  bool rlFwdDone = false;
  bool rrFwdDone = false;

  rearLeftForward(REAR_FLIP_PULSE_PWM);
  rearRightForward(REAR_FLIP_PULSE_PWM);

  while (!(rlFwdDone && rrFwdDone)) {
    server.handleClient();
    yield();
    updateIMU();

    if (millis() - startTime > TIMEOUT_MS) {
      holdAllStopped();
      return false;
    }

    if (!rlFwdDone) {
      if (encCountA >= REAR_FLIP_PULSE_COUNTS) {
        rearLeftStop();
        rlFwdDone = true;
      } else {
        ledcWrite(MOTOR_A_PWM, REAR_FLIP_PULSE_PWM);
      }
    }

    if (!rrFwdDone) {
      if (encCountB >= REAR_FLIP_PULSE_COUNTS) {
        rearRightStop();
        rrFwdDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_FLIP_PULSE_PWM);
      }
    }
  }

  delay(30);

  // same-count backward pulse
  encCountA = 0;
  encCountB = 0;
  bool rlRevDone = false;
  bool rrRevDone = false;

  rearLeftReverse(REAR_FLIP_PULSE_PWM);
  rearRightReverse(REAR_FLIP_PULSE_PWM);

  while (!(rlRevDone && rrRevDone)) {
    server.handleClient();
    yield();
    updateIMU();

    if (millis() - startTime > TIMEOUT_MS) {
      holdAllStopped();
      return false;
    }

    if (!rlRevDone) {
      if (encCountA >= REAR_FLIP_PULSE_COUNTS) {
        rearLeftStop();
        rlRevDone = true;
      } else {
        ledcWrite(MOTOR_A_PWM, REAR_FLIP_PULSE_PWM);
      }
    }

    if (!rrRevDone) {
      if (encCountB >= REAR_FLIP_PULSE_COUNTS) {
        rearRightStop();
        rrRevDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_FLIP_PULSE_PWM);
      }
    }
  }

  holdAllStopped();
  elapsedMs = millis() - startTime;
  return true;
}

bool rephaseAllLegsOneCycle() {
  rephaseInProgress = true;

  unsigned long t = 0;
  bool ok = true;

  Serial.println("Rephase: RL one cycle");
  ok = ok && walkOneLegMoveWithMultiplier(0, 1, t);

  if (ok) {
    Serial.println("Rephase: FL one cycle");
    ok = ok && walkOneLegMoveWithMultiplier(1, 1, t);
  }

  if (ok) {
    Serial.println("Rephase: RR one cycle");
    ok = ok && walkOneLegMoveWithMultiplier(2, 1, t);
  }

  if (ok) {
    Serial.println("Rephase: FR one cycle");
    ok = ok && walkOneLegMoveWithMultiplier(3, 1, t);
  }

  holdAllStopped();
  rephaseInProgress = false;
  return ok;
}

void performFlipSequence() {
  flipInProgress = true;
  runEnabled = false;

  Serial.println("=== FLIP SEQUENCE START ===");

  holdAllStopped();
  closeShellAfterStopping();

  unsigned long tRearCycle = 0;
  unsigned long tKick = 0;
  unsigned long tFrontCycle = 0;

  bool okRearCycle = rotateRearBothOneCycle(tRearCycle);
  bool okKick = false;
  bool okFront = false;

  if (okRearCycle) {
    okKick = rearFlipKickPulse(tKick);
  }

  delay(POST_FLIP_FRONT_DELAY_MS);

  if (okRearCycle && okKick) {
    okFront = rotateFrontBothOneCycle(tFrontCycle);
  }

  holdAllStopped();

  Serial.print("Rear one-cycle: ");
  Serial.println(okRearCycle ? "OK" : "FAIL");

  Serial.print("Rear flip pulse: ");
  Serial.println(okKick ? "OK" : "FAIL");

  Serial.print("Front one-cycle: ");
  Serial.println(okFront ? "OK" : "FAIL");

  Serial.println("=== FLIP SEQUENCE END ===");

  flipRequested = false;
  flipInProgress = false;
  waitingForRecovery = true;
  recoveryStableStartMs = 0;

  Serial.println("Holding active brake and waiting for recovery.");
}

void handlePostFlipRecovery() {
  if (!waitingForRecovery) return;

  holdAllStopped();

  if (fabs(pitchDeg) <= PITCH_RECOVERY_BAND_DEG) {
    if (recoveryStableStartMs == 0) {
      recoveryStableStartMs = millis();
      Serial.println("Recovery band entered. Timing stability...");
    } else if (millis() - recoveryStableStartMs >= RECOVERY_STABLE_TIME_MS) {
      Serial.println("Recovery condition met: pitch stable near 0 deg for 5 seconds.");

      reopenShellAfterRecovery();

      bool ok = rephaseAllLegsOneCycle();

      if (ok) {
        completedCycles = 0;
        currentCycleNumber = 0;
        waitingForRecovery = false;
        recoveryStableStartMs = 0;
        runEnabled = true;
        stopRequested = false;
        flipRequested = false;

        Serial.println("Recovery rephase complete. Resuming normal walking.");
      } else {
        waitingForRecovery = false;
        recoveryStableStartMs = 0;
        runEnabled = false;
        Serial.println("Recovery rephase failed. Holding active brake.");
      }
    }
  } else {
    if (recoveryStableStartMs != 0) {
      Serial.println("Recovery band lost. Resetting stability timer.");
    }
    recoveryStableStartMs = 0;
  }
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("STAIR - one-leg-at-a-time walking + shell servo + IMU flip");
  Serial.println("Move 1: RL (double cycle)");
  Serial.println("Move 2: FL (double cycle)");
  Serial.println("Move 3: RR (double cycle)");
  Serial.println("Move 4: FR (double cycle)");
  Serial.println("START waits 2 seconds, opens shell, then walks.");
  Serial.println("STOP finishes on a full 4-leg boundary, then closes shell.");
  Serial.println("IMU pitch > 15 deg triggers flip on next even move boundary.");

  Serial.print("Rear left counts/cycle   = ");
  Serial.println(REAR_LEFT_COUNTS_PER_CYCLE);

  Serial.print("Rear right counts/cycle  = ");
  Serial.println(REAR_RIGHT_COUNTS_PER_CYCLE);

  Serial.print("Front left counts/cycle  = ");
  Serial.println(FRONT_LEFT_COUNTS_PER_CYCLE);

  Serial.print("Front right counts/cycle = ");
  Serial.println(FRONT_RIGHT_COUNTS_PER_CYCLE);

  Serial.print("Leg move multiplier      = ");
  Serial.println(LEG_MOVE_MULTIPLIER);

  // Motors
  pinMode(MOTOR_A_INA, OUTPUT);
  pinMode(MOTOR_A_INB, OUTPUT);
  pinMode(MOTOR_B_INA, OUTPUT);
  pinMode(MOTOR_B_INB, OUTPUT);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT);
  pinMode(FR_IN4, OUTPUT);

  ledcAttach(MOTOR_A_PWM, 20000, 8);
  ledcAttach(MOTOR_B_PWM, 20000, 8);
  ledcAttach(FL_IN1, 20000, 8);
  ledcAttach(FR_IN3, 20000, 8);

  // Encoders
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  pinMode(ENC_FL_PIN, INPUT);
  pinMode(ENC_FR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN),  encoderA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN),  encoderB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), encoderFL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), encoderFR_ISR, CHANGE);

  // Servo
  shellServo.setPeriodHertz(50);
  shellServo.attach(SERVO_PIN, 500, 2500);
  shellServo.writeMicroseconds(SERVO_STOP_US);

  // IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("I2C + MPU6050 init");
  mpu6050.begin();

  Serial.println("Calibrating gyro... keep sensor still");
  mpu6050.calcGyroOffsets(true);

  Serial.println("Settling IMU...");
  for (int i = 0; i < 100; i++) {
    mpu6050.update();
    delay(10);
  }

  angle_offset = mpu6050.getAngleX();
  Serial.print("Pitch zero offset = ");
  Serial.println(angle_offset, 4);

  holdAllStopped();
  setupWebServer();

  Serial.print("Servo attached: ");
  Serial.println(shellServo.attached());

  Serial.println("Ready. Waiting for START button...");
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  server.handleClient();
  updateIMU();

  // Post-flip recovery hold / resume logic
  if (waitingForRecovery) {
    handlePostFlipRecovery();
    delay(5);
    return;
  }

  // Idle hold
  if (!runEnabled) {
    holdAllStopped();
    delay(5);
    return;
  }

  // While walking, latch flip request if pitch exceeds threshold
  if (!flipRequested && !flipInProgress && !rephaseInProgress) {
    if (pitchDeg > PITCH_FLIP_THRESHOLD_DEG) {
      flipRequested = true;
      Serial.print("IMU flip request latched. Pitch = ");
      Serial.println(pitchDeg, 2);
    }
  }

  // Manual STOP button behavior stays the same:
  // stop only after a full 4-leg sequence boundary
  if (stopRequested && !cycleInProgress && (completedCycles % 4 == 0) && completedCycles > 0) {
    runEnabled = false;
    holdAllStopped();
    closeShellAfterStopping();
    Serial.println("Stop request satisfied at full 4-leg boundary");
    delay(5);
    return;
  }

  // IMU flip executes on next even move boundary
  if (flipRequested && !stopRequested && !cycleInProgress && (completedCycles % 2 == 0) && completedCycles > 0) {
    holdAllStopped();
    performFlipSequence();
    delay(5);
    return;
  }

  currentCycleNumber = completedCycles + 1;
  cycleInProgress = true;

  uint8_t seqIndex = (currentCycleNumber - 1) % 4;
  uint8_t legIndex = LEG_SEQUENCE[seqIndex];

  unsigned long tCycle = 0;

  Serial.println();
  Serial.print("=== LEG MOVE ");
  Serial.print(currentCycleNumber);
  Serial.print(" START: ");
  Serial.println(legName(legIndex));

  bool ok = walkOneLegMove(legIndex, tCycle);
  cycleInProgress = false;

  if (!ok) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Leg move failed. Holding brake.");
    return;
  }

  completedCycles = currentCycleNumber;

  Serial.print("Leg move ");
  Serial.print(currentCycleNumber);
  Serial.print(" complete. Active leg = ");
  Serial.println(legName(legIndex));

  Serial.print("Time = ");
  Serial.print(tCycle);
  Serial.println(" ms");

  Serial.print("RL counts = "); Serial.println(encCountA);
  Serial.print("RR counts = "); Serial.println(encCountB);
  Serial.print("FL counts = "); Serial.println(encCountFL);
  Serial.print("FR counts = "); Serial.println(encCountFR);

  // Manual STOP button behavior stays the same
  if (stopRequested && (completedCycles % 4 == 0)) {
    runEnabled = false;
    holdAllStopped();
    closeShellAfterStopping();
    Serial.println("Graceful STOP complete at full 4-leg boundary");
    return;
  }

  // IMU flip executes on next even move boundary
  if (flipRequested && !stopRequested && (completedCycles % 2 == 0)) {
    holdAllStopped();
    performFlipSequence();
    return;
  }
}