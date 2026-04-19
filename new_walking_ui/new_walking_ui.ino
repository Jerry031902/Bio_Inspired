#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// STAIR Robot — Diagonal Pair Walking Test (A A B B)
// PASSIVE-STOP VERSION
//
// Web UI:
// - ESP32 creates Wi-Fi hotspot
// - webpage has START and STOP
// - START begins motion immediately
// - STOP is graceful and only stops at an even cycle boundary
//
// Pattern:
//   Cycle 1: RL + FR
//   Cycle 2: RL + FR
//   Cycle 3: RR + FL
//   Cycle 4: RR + FL
//   Then repeat...
// ============================================================

// ---------------- Web UI ----------------
const char* AP_SSID = "STAIR-Robot";
const char* AP_PASS = "12345678";   // at least 8 chars
WebServer server(80);

bool runEnabled      = false;  // true when robot is allowed to move
bool stopRequested   = false;  // graceful stop requested
bool cycleInProgress = false;

unsigned long completedCycles    = 0;  // completed cycle count
unsigned long currentCycleNumber = 0;  // 1-based cycle number while running

// ---------------- Rear motor pins ----------------
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
const long FRONT_LEFT_COUNTS_PER_CYCLE  = 537;
const long FRONT_RIGHT_COUNTS_PER_CYCLE = 550;

// ---------------- Speed tuning ----------------
#define REAR_SPEED_MAX             255
#define REAR_SPEED_MIN             60
#define REAR_RAMP_DOWN_THRESHOLD   220

#define FRONT_SPEED_MAX            255
#define FRONT_SPEED_MIN            60
#define FRONT_RAMP_DOWN_THRESHOLD  160

#define TIMEOUT_MS                 8000

volatile long encCountA  = 0;   // rear left
volatile long encCountB  = 0;   // rear right
volatile long encCountFL = 0;   // front left
volatile long encCountFR = 0;   // front right

// ============================================================
// Forward declarations
// ============================================================
void holdAllStopped();
bool walkOneDiagonalCycle(bool pairA, unsigned long &elapsedMs);
void setupWebServer();
String buildStatusJson();

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
      max-width: 440px;
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
      <div><b>Current cycle:</b> <span id="currentCycle">-</span></div>
      <div><b>Completed cycles:</b> <span id="completedCycles">-</span></div>
      <div><b>Stop requested:</b> <span id="stopRequested">-</span></div>
    </div>

    <div class="small">
      START begins immediately.
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
      } catch (e) {
        document.getElementById('state').textContent = 'Disconnected';
      }
    }

    updateStatus();
    setInterval(updateStatus, 500);
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
// Helpers
// ============================================================
uint8_t rampSpeedFromRemaining(long remaining, long threshold, uint8_t minSpeed, uint8_t maxSpeed) {
  if (remaining >= threshold) return maxSpeed;
  if (remaining <= 0) return 0;
  return (uint8_t)map(remaining, 0, threshold, minSpeed, maxSpeed);
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

// Rear passive stop
void rearLeftStop() {
  digitalWrite(MOTOR_A_INA, LOW);
  digitalWrite(MOTOR_A_INB, LOW);
  ledcWrite(MOTOR_A_PWM, 0);
}

void rearRightStop() {
  digitalWrite(MOTOR_B_INA, LOW);
  digitalWrite(MOTOR_B_INB, LOW);
  ledcWrite(MOTOR_B_PWM, 0);
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

// Front passive stop
void frontLeftBrake() {
  ledcWrite(FL_IN1, 0);
  digitalWrite(FL_IN2, LOW);
}

void frontRightBrake() {
  ledcWrite(FR_IN3, 0);
  digitalWrite(FR_IN4, LOW);
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
// Web server
// ============================================================
String buildStatusJson() {
  String state;

  if (!runEnabled && completedCycles == 0 && !stopRequested) {
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
  json += "\"stopRequested\":" + String(stopRequested ? "true" : "false");
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
    if (!runEnabled && !cycleInProgress) {
      completedCycles = 0;
      currentCycleNumber = 0;
      stopRequested = false;
      runEnabled = true;
      holdAllStopped();
      Serial.println("Web START requested");
      Serial.println("Robot starting immediately from cycle 1");
    }
    server.send(200, "text/plain", "STARTED");
  });

  server.on("/stop", HTTP_POST, []() {
    stopRequested = true;
    Serial.println("Web STOP requested");
    Serial.println("Robot will stop at the next even cycle boundary");
    server.send(200, "text/plain", "STOP_REQUESTED");
  });

  server.begin();
  Serial.println("Web server started");
}

// ============================================================
// One diagonal-pair cycle
//
// pairA = true  -> RL + FR
// pairA = false -> RR + FL
// ============================================================
bool walkOneDiagonalCycle(bool pairA, unsigned long &elapsedMs) {
  encCountA  = 0;
  encCountB  = 0;
  encCountFL = 0;
  encCountFR = 0;

  bool rlActive = pairA;
  bool frActive = pairA;
  bool rrActive = !pairA;
  bool flActive = !pairA;

  bool rlDone = !rlActive;
  bool rrDone = !rrActive;
  bool flDone = !flActive;
  bool frDone = !frActive;

  unsigned long startTime = millis();

  // Hold inactive pair with passive stop
  if (!rlActive) rearLeftStop();
  if (!rrActive) rearRightStop();
  if (!flActive) frontLeftBrake();
  if (!frActive) frontRightBrake();

  // Start active pair
  if (rlActive) rearLeftForward(REAR_SPEED_MAX);
  if (rrActive) rearRightForward(REAR_SPEED_MAX);
  if (flActive) frontLeftForward(FRONT_SPEED_MAX);
  if (frActive) frontRightForward(FRONT_SPEED_MAX);

  while (!(rlDone && rrDone && flDone && frDone)) {
    server.handleClient();
    yield();

    if (millis() - startTime > TIMEOUT_MS) {
      holdAllStopped();

      Serial.println("TIMEOUT - one or more motors did not finish cycle");
      Serial.print("RL counts = "); Serial.println(encCountA);
      Serial.print("RR counts = "); Serial.println(encCountB);
      Serial.print("FL counts = "); Serial.println(encCountFL);
      Serial.print("FR counts = "); Serial.println(encCountFR);
      return false;
    }

    // Keep passive stop reasserted every loop
    if (!rlActive) rearLeftStop();
    if (!rrActive) rearRightStop();
    if (!flActive) frontLeftBrake();
    if (!frActive) frontRightBrake();

    if (rlDone) rearLeftStop();
    if (rrDone) rearRightStop();
    if (flDone) frontLeftBrake();
    if (frDone) frontRightBrake();

    // Rear Left
    if (!rlDone) {
      long remaining = REAR_LEFT_COUNTS_PER_CYCLE - encCountA;
      if (remaining <= 0) {
        rearLeftStop();
        rlDone = true;
      } else {
        uint8_t s = rampSpeedFromRemaining(
          remaining,
          REAR_RAMP_DOWN_THRESHOLD,
          REAR_SPEED_MIN,
          REAR_SPEED_MAX
        );
        ledcWrite(MOTOR_A_PWM, s);
      }
    }

    // Rear Right
    if (!rrDone) {
      long remaining = REAR_RIGHT_COUNTS_PER_CYCLE - encCountB;
      if (remaining <= 0) {
        rearRightStop();
        rrDone = true;
      } else {
        uint8_t s = rampSpeedFromRemaining(
          remaining,
          REAR_RAMP_DOWN_THRESHOLD,
          REAR_SPEED_MIN,
          REAR_SPEED_MAX
        );
        ledcWrite(MOTOR_B_PWM, s);
      }
    }

    // Front Left
    if (!flDone) {
      long remaining = FRONT_LEFT_COUNTS_PER_CYCLE - encCountFL;
      if (remaining <= 0) {
        frontLeftBrake();
        flDone = true;
      } else {
        uint8_t s = rampSpeedFromRemaining(
          remaining,
          FRONT_RAMP_DOWN_THRESHOLD,
          FRONT_SPEED_MIN,
          FRONT_SPEED_MAX
        );
        ledcWrite(FL_IN1, s);
      }
    }

    // Front Right
    if (!frDone) {
      long remaining = FRONT_RIGHT_COUNTS_PER_CYCLE - encCountFR;
      if (remaining <= 0) {
        frontRightBrake();
        frDone = true;
      } else {
        uint8_t s = rampSpeedFromRemaining(
          remaining,
          FRONT_RAMP_DOWN_THRESHOLD,
          FRONT_SPEED_MIN,
          FRONT_SPEED_MAX
        );
        ledcWrite(FR_IN3, s);
      }
    }
  }

  elapsedMs = millis() - startTime;
  return true;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Rear motor pins
  pinMode(MOTOR_A_INA, OUTPUT);
  pinMode(MOTOR_A_INB, OUTPUT);
  pinMode(MOTOR_B_INA, OUTPUT);
  pinMode(MOTOR_B_INB, OUTPUT);

  // Front motor pins
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT);
  pinMode(FR_IN4, OUTPUT);

  // PWM attach
  ledcAttach(MOTOR_A_PWM, 20000, 8);
  ledcAttach(MOTOR_B_PWM, 20000, 8);
  ledcAttach(FL_IN1, 20000, 8);
  ledcAttach(FR_IN3, 20000, 8);

  // Force passive stop ASAP
  holdAllStopped();

  // Encoder pins
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  pinMode(ENC_FL_PIN, INPUT);
  pinMode(ENC_FR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN),  encoderA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN),  encoderB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), encoderFL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), encoderFR_ISR, CHANGE);

  Serial.println("STAIR - diagonal pair walking (A A B B) - passive stop version");
  Serial.println("Cycle 1: RL + FR");
  Serial.println("Cycle 2: RL + FR");
  Serial.println("Cycle 3: RR + FL");
  Serial.println("Cycle 4: RR + FL");
  Serial.println("Robot stays idle until START is pressed");
  Serial.println("STOP is graceful and happens only at even cycle boundary");

  Serial.print("Rear left counts/cycle   = ");
  Serial.println(REAR_LEFT_COUNTS_PER_CYCLE);

  Serial.print("Rear right counts/cycle  = ");
  Serial.println(REAR_RIGHT_COUNTS_PER_CYCLE);

  Serial.print("Front left counts/cycle  = ");
  Serial.println(FRONT_LEFT_COUNTS_PER_CYCLE);

  Serial.print("Front right counts/cycle = ");
  Serial.println(FRONT_RIGHT_COUNTS_PER_CYCLE);

  setupWebServer();

  Serial.println("Ready. Waiting for START button...");
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  server.handleClient();

  if (!runEnabled) {
    holdAllStopped();
    delay(5);
    return;
  }

  // If STOP was requested while already at an even boundary, stop now.
  if (stopRequested && !cycleInProgress && (completedCycles % 2 == 0)) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Stop request satisfied at even cycle boundary");
    delay(5);
    return;
  }

  currentCycleNumber = completedCycles + 1;
  cycleInProgress = true;

  // AABB pattern:
  // cycles 1,2 => pair A
  // cycles 3,4 => pair B
  // repeat
  bool pairA = (((currentCycleNumber - 1) / 2) % 2 == 0);

  unsigned long tCycle = 0;

  Serial.println();
  Serial.print("=== WALK CYCLE ");
  Serial.print(currentCycleNumber);
  Serial.print(" START: ");
  Serial.println(pairA ? "RL + FR" : "RR + FL");

  bool ok = walkOneDiagonalCycle(pairA, tCycle);
  cycleInProgress = false;

  if (!ok) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Cycle failed. Holding passive stop.");
    return;
  }

  completedCycles = currentCycleNumber;

  Serial.print("Cycle ");
  Serial.print(currentCycleNumber);
  Serial.println(" complete.");
  Serial.print("Time = ");
  Serial.print(tCycle);
  Serial.println(" ms");

  Serial.print("RL counts = "); Serial.println(encCountA);
  Serial.print("RR counts = "); Serial.println(encCountB);
  Serial.print("FL counts = "); Serial.println(encCountFL);
  Serial.print("FR counts = "); Serial.println(encCountFR);

  // Graceful stop only after even-numbered cycle
  if (stopRequested && (completedCycles % 2 == 0)) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Graceful STOP complete at even cycle boundary");
  }
}