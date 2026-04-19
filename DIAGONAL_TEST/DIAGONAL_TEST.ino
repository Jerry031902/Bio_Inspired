#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// STAIR Robot — ESP32 2-motor active-brake test
// Selected diagonal pair: RR + FL
//
// - ESP32 creates Wi-Fi hotspot
// - webpage has START and STOP
// - START begins motion immediately
// - STOP is graceful and only stops at an EVEN cycle boundary
//
// Purpose:
// - test RR + FL only
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
// Rear-right motor
#define RR_INA       4
#define RR_INB       5
#define RR_PWM       6
#define ENC_RR_PIN   7

// Front-left motor
#define FL_IN1       42
#define FL_IN2       41
#define ENC_FL_PIN   12

// ---------------- Counts ----------------
const long REAR_RIGHT_COUNTS_PER_CYCLE = 700;
const long FRONT_LEFT_COUNTS_PER_CYCLE = 537;

// ---------------- Speed tuning ----------------
#define REAR_SPEED_MAX             255
#define REAR_SPEED_MIN             60
#define REAR_RAMP_DOWN_THRESHOLD   220

#define FRONT_SPEED_MAX            255
#define FRONT_SPEED_MIN            60
#define FRONT_RAMP_DOWN_THRESHOLD  160

#define TIMEOUT_MS                 8000

volatile long encCountRR = 0;
volatile long encCountFL = 0;

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
      <div><b>Pair:</b> RR + FL</div>
      <div><b>State:</b> <span id="state">Loading...</span></div>
      <div><b>Current cycle:</b> <span id="currentCycle">-</span></div>
      <div><b>Completed cycles:</b> <span id="completedCycles">-</span></div>
      <div><b>Stop requested:</b> <span id="stopRequested">-</span></div>
      <div><b>RR counts:</b> <span id="rrCounts">-</span></div>
      <div><b>FL counts:</b> <span id="flCounts">-</span></div>
    </div>

    <div class="small">
      ESP32 active-brake test build: 1 diagonal pair only (RR + FL). STOP happens only at an even cycle boundary.
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
        document.getElementById('rrCounts').textContent = s.rrCounts;
        document.getElementById('flCounts').textContent = s.flCounts;
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
void IRAM_ATTR encoderRR_ISR() {
  encCountRR++;
}

void IRAM_ATTR encoderFL_ISR() {
  encCountFL++;
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
// Rear-right motor control (VNH5019 style)
// ============================================================
void rearRightForward(uint8_t speed) {
  digitalWrite(RR_INA, HIGH);
  digitalWrite(RR_INB, LOW);
  ledcWrite(RR_PWM, speed);
}

void rearRightStop() {
  // ACTIVE BRAKE
  digitalWrite(RR_INA, HIGH);
  digitalWrite(RR_INB, HIGH);
  ledcWrite(RR_PWM, 255);
}

// ============================================================
// Front-left motor control (front-style 2-pin control)
// ============================================================
void frontLeftForward(uint8_t speed) {
  ledcWrite(FL_IN1, speed);
  digitalWrite(FL_IN2, LOW);
}

void frontLeftBrake() {
  // ACTIVE BRAKE
  ledcWrite(FL_IN1, 255);
  digitalWrite(FL_IN2, HIGH);
}

void frontLeftRelease() {
  ledcWrite(FL_IN1, 0);
  digitalWrite(FL_IN2, LOW);
}

// ============================================================
// Global helpers
// ============================================================
void holdAllStopped() {
  rearRightStop();
  frontLeftBrake();
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
  json += "\"rrCounts\":" + String(encCountRR) + ",";
  json += "\"flCounts\":" + String(encCountFL);
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
      encCountRR = 0;
      encCountFL = 0;
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
// One cycle: RR + FL together
// ============================================================
bool walkOneCycle(unsigned long &elapsedMs) {
  encCountRR = 0;
  encCountFL = 0;

  bool rrDone = false;
  bool flDone = false;

  unsigned long startTime = millis();

  rearRightForward(REAR_SPEED_MAX);
  frontLeftForward(FRONT_SPEED_MAX);

  while (!(rrDone && flDone)) {
    server.handleClient();
    yield();

    if (millis() - startTime > TIMEOUT_MS) {
      holdAllStopped();

      Serial.println("TIMEOUT - one or more motors did not finish cycle");
      Serial.print("RR counts = "); Serial.println(encCountRR);
      Serial.print("FL counts = "); Serial.println(encCountFL);
      return false;
    }

    if (rrDone) rearRightStop();
    if (flDone) frontLeftBrake();

    // Rear Right
    if (!rrDone) {
      long remaining = REAR_RIGHT_COUNTS_PER_CYCLE - encCountRR;
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
        ledcWrite(RR_PWM, s);
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
  }

  elapsedMs = millis() - startTime;
  return true;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  // --- Motor control pins first ---
  pinMode(RR_INA, OUTPUT);
  pinMode(RR_INB, OUTPUT);
  pinMode(FL_IN2, OUTPUT);

  // --- PWM attach first ---
  ledcAttach(RR_PWM, 20000, 8);
  ledcAttach(FL_IN1, 20000, 8);

  // --- Encoder pins ---
  pinMode(ENC_RR_PIN, INPUT);
  pinMode(ENC_FL_PIN, INPUT);

  // --- Force active brake ASAP ---
  holdAllStopped();

  // --- Serial after outputs are already defined ---
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("STAIR - ESP32 2-motor diagonal ACTIVE-BRAKE test");
  Serial.println("Pair: RR + FL");
  Serial.println("Repeated cycle: RR + FL, RR + FL, RR + FL...");
  Serial.println("STOP is graceful at EVEN cycle boundary");

  Serial.print("Rear right counts/cycle = ");
  Serial.println(REAR_RIGHT_COUNTS_PER_CYCLE);

  Serial.print("Front left counts/cycle = ");
  Serial.println(FRONT_LEFT_COUNTS_PER_CYCLE);

  attachInterrupt(digitalPinToInterrupt(ENC_RR_PIN), encoderRR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), encoderFL_ISR, CHANGE);

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

  Serial.println();
  Serial.print("=== WALK CYCLE ");
  Serial.print(currentCycleNumber);
  Serial.println(" START: RR + FL");

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

  Serial.print("RR counts = "); Serial.println(encCountRR);
  Serial.print("FL counts = "); Serial.println(encCountFL);

  // Graceful stop only after even-numbered cycle
  if (stopRequested && (completedCycles % 2 == 0)) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Graceful STOP complete at even cycle boundary");
  }
}