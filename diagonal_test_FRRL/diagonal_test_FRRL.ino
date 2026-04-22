#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// STAIR Robot — ESP32 2-motor active-brake test
// Selected diagonal pair: RL + FR
//
// - ESP32 creates Wi-Fi hotspot
// - webpage has START and STOP
// - START begins motion immediately
// - STOP is graceful and stops at cycle boundary
//
// Purpose:
// - test RL + FR only
// - test ACTIVE BRAKE behavior on ESP32
// ============================================================

// ---------------- Web UI ----------------
const char* AP_SSID = "STAIR-Robot";
const char* AP_PASS = "12345678";

WebServer server(80);

bool runEnabled      = false;
bool stopRequested   = false;
bool cycleInProgress = false;

unsigned long completedCycles    = 0;
unsigned long currentCycleNumber = 0;

// ---------------- Pin map (ESP32 original layout) ----------------
// Rear-left motor
#define RL_INA       2
#define RL_INB       1
#define RL_PWM       3
#define ENC_RL_PIN   9

// Front-right motor
#define FR_IN3       40
#define FR_IN4       39
#define ENC_FR_PIN   13

// ---------------- Counts ----------------
const long REAR_LEFT_COUNTS_PER_CYCLE   = 700;
const long FRONT_RIGHT_COUNTS_PER_CYCLE = 550;

// ---------------- Speed tuning ----------------
#define REAR_SPEED_MAX             255
#define REAR_SPEED_MIN             60
#define REAR_RAMP_DOWN_THRESHOLD   220

#define FRONT_SPEED_MAX            255
#define FRONT_SPEED_MIN            60
#define FRONT_RAMP_DOWN_THRESHOLD  160

#define TIMEOUT_MS                 8000

volatile long encCountRL = 0;
volatile long encCountFR = 0;

// ============================================================
// Forward declarations
// ============================================================
void holdAllStopped();
bool walkOneCycle(unsigned long &elapsedMs);
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
    h1 { margin-bottom: 20px; }
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
    .startBtn { background: #2a9d8f; }
    .stopBtn  { background: #d62828; }
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
      <div><b>Pair:</b> RL + FR</div>
      <div><b>State:</b> <span id="state">Loading...</span></div>
      <div><b>Current cycle:</b> <span id="currentCycle">-</span></div>
      <div><b>Completed cycles:</b> <span id="completedCycles">-</span></div>
      <div><b>Stop requested:</b> <span id="stopRequested">-</span></div>
      <div><b>RL counts:</b> <span id="rlCounts">-</span></div>
      <div><b>FR counts:</b> <span id="frCounts">-</span></div>
    </div>

    <div class="small">
      ESP32 active-brake test build: 1 diagonal pair only (RL + FR).
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
        document.getElementById('rlCounts').textContent = s.rlCounts;
        document.getElementById('frCounts').textContent = s.frCounts;
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
void IRAM_ATTR encoderRL_ISR() {
  encCountRL++;
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
// Rear-left motor control (VNH5019 style)
// ============================================================
void rearLeftForward(uint8_t speed) {
  digitalWrite(RL_INA, HIGH);
  digitalWrite(RL_INB, LOW);
  ledcWrite(RL_PWM, speed);
}

void rearLeftStop() {
  // ACTIVE BRAKE
  digitalWrite(RL_INA, HIGH);
  digitalWrite(RL_INB, HIGH);
  ledcWrite(RL_PWM, 255);
}

// ============================================================
// Front-right motor control (front-style 2-pin control)
// ============================================================
void frontRightForward(uint8_t speed) {
  ledcWrite(FR_IN3, speed);
  digitalWrite(FR_IN4, LOW);
}

void frontRightBrake() {
  // ACTIVE BRAKE
  ledcWrite(FR_IN3, 255);
  digitalWrite(FR_IN4, HIGH);
}

void frontRightRelease() {
  ledcWrite(FR_IN3, 0);
  digitalWrite(FR_IN4, LOW);
}

// ============================================================
// Global helpers
// ============================================================
void holdAllStopped() {
  rearLeftStop();
  frontRightBrake();
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
  json += "\"stopRequested\":" + String(stopRequested ? "true" : "false") + ",";
  json += "\"rlCounts\":" + String(encCountRL) + ",";
  json += "\"frCounts\":" + String(encCountFR);
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
      encCountRL = 0;
      encCountFR = 0;
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
    Serial.println("Robot will stop at the next cycle boundary");
    server.send(200, "text/plain", "STOP_REQUESTED");
  });

  server.begin();
  Serial.println("Web server started");
}

// ============================================================
// One cycle: RL + FR together
// ============================================================
bool walkOneCycle(unsigned long &elapsedMs) {
  encCountRL = 0;
  encCountFR = 0;

  bool rlDone = false;
  bool frDone = false;

  unsigned long startTime = millis();

  rearLeftForward(REAR_SPEED_MAX);
  frontRightForward(FRONT_SPEED_MAX);

  while (!(rlDone && frDone)) {
    server.handleClient();
    yield();

    if (millis() - startTime > TIMEOUT_MS) {
      holdAllStopped();

      Serial.println("TIMEOUT - one or more motors did not finish cycle");
      Serial.print("RL counts = "); Serial.println(encCountRL);
      Serial.print("FR counts = "); Serial.println(encCountFR);
      return false;
    }

    if (rlDone) rearLeftStop();
    if (frDone) frontRightBrake();

    // Rear Left
    if (!rlDone) {
      long remaining = REAR_LEFT_COUNTS_PER_CYCLE - encCountRL;
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
        ledcWrite(RL_PWM, s);
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
  // --- Motor control pins first ---
  pinMode(RL_INA, OUTPUT);
  pinMode(RL_INB, OUTPUT);
  pinMode(FR_IN4, OUTPUT);

  // --- PWM attach first ---
  ledcAttach(RL_PWM, 20000, 8);
  ledcAttach(FR_IN3, 20000, 8);

  // --- Encoder pins ---
  pinMode(ENC_RL_PIN, INPUT);
  pinMode(ENC_FR_PIN, INPUT);

  // --- Force active brake ASAP ---
  holdAllStopped();

  // --- Serial after outputs are already defined ---
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("STAIR - ESP32 2-motor diagonal ACTIVE-BRAKE test");
  Serial.println("Pair: RL + FR");
  Serial.println("Repeated cycle: RL + FR, RL + FR, RL + FR...");
  Serial.println("STOP is graceful at cycle boundary");

  Serial.print("Rear left counts/cycle   = ");
  Serial.println(REAR_LEFT_COUNTS_PER_CYCLE);

  Serial.print("Front right counts/cycle = ");
  Serial.println(FRONT_RIGHT_COUNTS_PER_CYCLE);

  attachInterrupt(digitalPinToInterrupt(ENC_RL_PIN), encoderRL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), encoderFR_ISR, CHANGE);

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

  if (stopRequested && !cycleInProgress) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Stop request satisfied at cycle boundary");
    delay(5);
    return;
  }

  currentCycleNumber = completedCycles + 1;
  cycleInProgress = true;

  Serial.println();
  Serial.print("=== WALK CYCLE ");
  Serial.print(currentCycleNumber);
  Serial.println(" START: RL + FR");

  unsigned long tCycle = 0;
  bool ok = walkOneCycle(tCycle);

  cycleInProgress = false;

  if (!ok) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Cycle failed. Holding active brake.");
    return;
  }

  completedCycles = currentCycleNumber;

  Serial.print("Cycle ");
  Serial.print(currentCycleNumber);
  Serial.println(" complete.");
  Serial.print("Time = ");
  Serial.print(tCycle);
  Serial.println(" ms");

  Serial.print("RL counts = "); Serial.println(encCountRL);
  Serial.print("FR counts = "); Serial.println(encCountFR);

  if (stopRequested) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Graceful STOP complete at cycle boundary");
  }
}