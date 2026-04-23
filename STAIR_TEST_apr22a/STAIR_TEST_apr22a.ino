#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <WebSocketsServer.h>
#include <Adafruit_INA260.h>
#include <Wire.h>

// ============================================================
// STAIR Robot — One-Leg-at-a-Time Walking + Shell Servo
//              + Power/IMU Data Logging
// ============================================================

// ---------------- Web UI ----------------
const char* AP_SSID = "STAIR-Robot";
const char* AP_PASS = "12345678";
WebServer server(80);
WebSocketsServer webSocket(81);

// ---------------- INA260 Power Sensors ----------------
#define INA_FRONT_ADDR  0x40
#define INA_BACK_ADDR   0x41
#define INA_SERVO_ADDR  0x44

Adafruit_INA260 ina_front;
Adafruit_INA260 ina_back;
Adafruit_INA260 ina_servo;

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 100;

// ---------------- MPU-6500 IMU ----------------
#define MPU6500_ADDR  0x68
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

void readMPU6500() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;

  uint8_t received = Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)14);
  if (received < 14) return;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  accelX = rawAx / 16384.0;
  accelY = rawAy / 16384.0;
  accelZ = rawAz / 16384.0;

  gyroX = rawGx / 131.0;
  gyroY = rawGy / 131.0;
  gyroZ = rawGz / 131.0;
}

// ---------------- Motion state ----------------
bool runEnabled      = false;
bool stopRequested   = false;
bool cycleInProgress = false;

unsigned long completedCycles    = 0;
unsigned long currentCycleNumber = 0;

// ---------------- Shell servo ----------------
Servo shellServo;

const int SERVO_PIN      = 8;
const int SERVO_STOP_US  = 1500;
const int SERVO_OPEN_US  = 600;
const int SERVO_CLOSE_US = 2400;

const unsigned long SHELL_PRESTART_DELAY_MS = 2000;
const unsigned long SHELL_OPEN_TIME_MS      = 1400;
const unsigned long SHELL_CLOSE_TIME_MS     = 1215;

bool shellIsOpen = false;

// ---------------- Rear motor pins ----------------
#define MOTOR_A_INA   2
#define MOTOR_A_INB   1
#define MOTOR_A_PWM   3

#define MOTOR_B_INA   4
#define MOTOR_B_INB   5
#define MOTOR_B_PWM   6

// ---------------- Rear encoder pins ----------------
#define ENC_A_PIN     9
#define ENC_B_PIN     7

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
const long FRONT_LEFT_COUNTS_PER_CYCLE  = 605;
const long FRONT_RIGHT_COUNTS_PER_CYCLE = 600;

const int LEG_MOVE_MULTIPLIER = 2;

const uint8_t LEG_SEQUENCE[4] = {0, 1, 2, 3};

// ---------------- Speed tuning ----------------
#define REAR_SPEED             200
#define FRONT_SPEED            66
#define TIMEOUT_MS             8000

volatile long encCountA  = 0;
volatile long encCountB  = 0;
volatile long encCountFL = 0;
volatile long encCountFR = 0;

// ============================================================
// Forward declarations
// ============================================================
void holdAllStopped();
bool walkOneLegMove(uint8_t legIndex, unsigned long &elapsedMs);
void setupWebServer();
String buildStatusJson();
void openShellBeforeWalking();
void closeShellAfterStopping();
const char* legName(uint8_t legIndex);
void sampleAndSend();

// ============================================================
// INA260 + IMU Sampling
// ============================================================
void sampleAndSend() {
  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  float p_front = ina_front.readPower();
  float p_back  = ina_back.readPower();
  float p_servo = ina_servo.readPower();

  if (p_front > 100000) p_front = 0;
  if (p_back  > 100000) p_back  = 0;
  if (p_servo > 100000) p_servo = 0;

  readMPU6500();

  String msg = String(now) + ","
             + String(p_front, 1) + ","
             + String(p_back, 1) + ","
             + String(p_servo, 1) + ","
             + String(encCountA) + ","
             + String(encCountB) + ","
             + String(encCountFL) + ","
             + String(encCountFR) + ","
             + String(accelX, 3) + ","
             + String(accelY, 3) + ","
             + String(accelZ, 3) + ","
             + String(gyroX, 2) + ","
             + String(gyroY, 2) + ","
             + String(gyroZ, 2);

  webSocket.broadcastTXT(msg);
}

// ============================================================
// WebSocket event handler
// ============================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("WebSocket client #%u connected\n", num);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("WebSocket client #%u disconnected\n", num);
  }
}

// ============================================================
// ISRs
// ============================================================
void IRAM_ATTR encoderA_ISR()  { encCountA++;  }
void IRAM_ATTR encoderB_ISR()  { encCountB++;  }
void IRAM_ATTR encoderFL_ISR() { encCountFL++; }
void IRAM_ATTR encoderFR_ISR() { encCountFR++; }

// ============================================================
// Helpers
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

void closeShellAfterStopping() {
  if (!shellIsOpen) {
    Serial.println("Shell already closed.");
    return;
  }
  Serial.println("Closing shell...");
  shellServo.writeMicroseconds(SERVO_CLOSE_US);
  delay(SHELL_CLOSE_TIME_MS);
  shellServo.writeMicroseconds(SERVO_STOP_US);
  shellIsOpen = false;
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
void stopRearMotors()     { rearLeftStop();    rearRightStop();    }
void brakeFrontMotors()   { frontLeftBrake();  frontRightBrake();  }
void releaseFrontMotors() { frontLeftRelease(); frontRightRelease(); }
void holdAllStopped()     { stopRearMotors();  brakeFrontMotors(); }

// ============================================================
// Web server + status
// ============================================================
String buildStatusJson() {
  String state;
  if (!runEnabled && completedCycles == 0 && !stopRequested) state = "IDLE";
  else if (!runEnabled && completedCycles > 0) state = "STOPPED";
  else if (stopRequested) state = "STOP REQUESTED";
  else state = "RUNNING";

  unsigned long shownCycle = cycleInProgress ? currentCycleNumber : (completedCycles + 1);

  String json = "{";
  json += "\"state\":\"" + state + "\",";
  json += "\"currentCycle\":" + String(shownCycle) + ",";
  json += "\"completedCycles\":" + String(completedCycles) + ",";
  json += "\"stopRequested\":" + String(stopRequested ? "true" : "false");
  json += "}";
  return json;
}

// ============================================================
// Web page — motor control + data logger
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
      max-width: 440px;
      margin: 0 auto 16px;
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
      font-size: 20px;
      padding: 14px 22px;
      border: none;
      border-radius: 12px;
      color: white;
      cursor: pointer;
      min-width: 120px;
    }
    .startBtn { background: #2a9d8f; }
    .stopBtn  { background: #d62828; }
    .recBtn   { background: #e76f51; }
    .dlBtn    { background: #264653; }
    .status, .dataStatus {
      margin-top: 16px;
      font-size: 18px;
      line-height: 1.7;
    }
    .small {
      font-size: 14px;
      color: #555;
      margin-top: 8px;
    }
    .live {
      font-family: monospace;
      font-size: 14px;
      background: #f9f9f9;
      padding: 10px;
      border-radius: 8px;
      margin-top: 10px;
      text-align: left;
      line-height: 1.6;
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
    </div>
    <div class="small">
      START waits 2s, opens shell, then walks one leg at a time.<br>
      STOP finishes on the next full 4-leg boundary, then closes shell.
    </div>
  </div>

  <div class="card">
    <h2>Data Logger</h2>
    <div class="btnRow">
      <button class="recBtn" id="recBtn" onclick="toggleRecord()">RECORD</button>
      <button class="dlBtn" onclick="downloadCSV()">DOWNLOAD CSV</button>
    </div>
    <div class="dataStatus">
      <div><b>Recording:</b> <span id="recStatus">NO</span></div>
      <div><b>Rows:</b> <span id="rowCount">0</span></div>
    </div>
    <div class="live" id="liveData">
      Front: -- mW | Back: -- mW | Servo: -- mW<br>
      Accel: -- / -- / -- g<br>
      Gyro: -- / -- / -- dps
    </div>
    <div class="small">
      Press RECORD then START the robot.<br>
      Press DOWNLOAD CSV when done.
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
      } catch(e) {
        document.getElementById('state').textContent = 'Disconnected';
      }
    }
    updateStatus();
    setInterval(updateStatus, 500);

    var recording = false;
    var dataRows = [];
    var ws;
    var startTs = 0;

    function connectWS() {
      var host = window.location.hostname;
      ws = new WebSocket('ws://' + host + ':81');
      ws.onmessage = function(evt) {
        var parts = evt.data.split(',');
        var pf = parseFloat(parts[1]);
        var pb = parseFloat(parts[2]);
        var ps = parseFloat(parts[3]);
        var ax = parseFloat(parts[8]);
        var ay = parseFloat(parts[9]);
        var az = parseFloat(parts[10]);
        var gx = parseFloat(parts[11]);
        var gy = parseFloat(parts[12]);
        var gz = parseFloat(parts[13]);
        document.getElementById('liveData').innerHTML =
          'Front: ' + pf.toFixed(0) + ' mW | Back: ' + pb.toFixed(0) + ' mW | Servo: ' + ps.toFixed(0) + ' mW<br>' +
          'Accel: ' + ax.toFixed(2) + ' / ' + ay.toFixed(2) + ' / ' + az.toFixed(2) + ' g<br>' +
          'Gyro: ' + gx.toFixed(1) + ' / ' + gy.toFixed(1) + ' / ' + gz.toFixed(1) + ' dps';

        if (recording) {
          if (dataRows.length === 0) startTs = parseInt(parts[0]);
          var relTime = parseInt(parts[0]) - startTs;
          dataRows.push(relTime + ',' + parts.slice(1).join(','));
          document.getElementById('rowCount').textContent = dataRows.length;
        }
      };
      ws.onclose = function() {
        setTimeout(connectWS, 1000);
      };
    }
    connectWS();

    function toggleRecord() {
      recording = !recording;
      if (recording) {
        dataRows = [];
        startTs = 0;
        document.getElementById('rowCount').textContent = '0';
      }
      document.getElementById('recStatus').textContent = recording ? 'YES' : 'NO';
      document.getElementById('recBtn').textContent = recording ? 'STOP REC' : 'RECORD';
      document.getElementById('recBtn').style.background = recording ? '#d62828' : '#e76f51';
    }

    function downloadCSV() {
      if (dataRows.length === 0) { alert('No data recorded'); return; }
      var header = 'time_ms,power_front_mW,power_back_mW,power_servo_mW,enc_RL,enc_RR,enc_FL,enc_FR,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps';
      var csv = header + '\n' + dataRows.join('\n');
      var blob = new Blob([csv], { type: 'text/csv' });
      var a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = 'stair_power_log.csv';
      a.click();
    }
  </script>
</body>
</html>
)rawliteral";

// ============================================================
// Web server setup
// ============================================================
void setupWebServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println();
  Serial.print("Wi-Fi SSID: "); Serial.println(AP_SSID);
  Serial.print("Wi-Fi PASS: "); Serial.println(AP_PASS);
  Serial.print("Open browser at: http://"); Serial.println(WiFi.softAPIP());

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
    Serial.println("Web STOP requested");
    server.send(200, "text/plain", "STOP_REQUESTED");
  });

  server.begin();
  Serial.println("Web server started on port 80");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");
}

// ============================================================
// One active-leg move
// ============================================================
bool walkOneLegMove(uint8_t legIndex, unsigned long &elapsedMs) {
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

  const long rlTarget = REAR_LEFT_COUNTS_PER_CYCLE   * LEG_MOVE_MULTIPLIER;
  const long rrTarget = REAR_RIGHT_COUNTS_PER_CYCLE  * LEG_MOVE_MULTIPLIER;
  const long flTarget = FRONT_LEFT_COUNTS_PER_CYCLE  * LEG_MOVE_MULTIPLIER;
  const long frTarget = FRONT_RIGHT_COUNTS_PER_CYCLE * LEG_MOVE_MULTIPLIER;

  const unsigned long moveTimeout = TIMEOUT_MS * LEG_MOVE_MULTIPLIER;

  unsigned long startTime = millis();

  if (!rlActive) rearLeftStop();
  if (!rrActive) rearRightStop();
  if (!flActive) frontLeftBrake();
  if (!frActive) frontRightBrake();

  if (rlActive) rearLeftForward(REAR_SPEED);
  if (rrActive) rearRightForward(REAR_SPEED);
  if (flActive) frontLeftForward(FRONT_SPEED);
  if (frActive) frontRightForward(FRONT_SPEED);

  while (!(rlDone && rrDone && flDone && frDone)) {
    server.handleClient();
    webSocket.loop();
    sampleAndSend();
    yield();

    if (millis() - startTime > moveTimeout) {
      holdAllStopped();
      Serial.println("TIMEOUT");
      Serial.print("Active leg = "); Serial.println(legName(legIndex));
      Serial.print("RL counts = "); Serial.println(encCountA);
      Serial.print("RR counts = "); Serial.println(encCountB);
      Serial.print("FL counts = "); Serial.println(encCountFL);
      Serial.print("FR counts = "); Serial.println(encCountFR);
      return false;
    }

    if (!rlActive) rearLeftStop();
    if (!rrActive) rearRightStop();
    if (!flActive) frontLeftBrake();
    if (!frActive) frontRightBrake();

    if (!rlDone) {
      if (encCountA >= rlTarget) { rearLeftStop(); rlDone = true; }
      else ledcWrite(MOTOR_A_PWM, REAR_SPEED);
    }
    if (!rrDone) {
      if (encCountB >= rrTarget) { rearRightStop(); rrDone = true; }
      else ledcWrite(MOTOR_B_PWM, REAR_SPEED);
    }
    if (!flDone) {
      if (encCountFL >= flTarget) { frontLeftBrake(); flDone = true; }
      else ledcWrite(FL_IN1, FRONT_SPEED);
    }
    if (!frDone) {
      if (encCountFR >= frTarget) { frontRightBrake(); frDone = true; }
      else ledcWrite(FR_IN3, FRONT_SPEED);
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
  delay(1000);

  Serial.println("STAIR - one-leg-at-a-time + shell servo + data logging");

  // I2C + sensors init
  Wire.begin(36, 35);

  if (!ina_front.begin(INA_FRONT_ADDR, &Wire))
    Serial.println("INA260 front (0x40) NOT found!");
  else
    Serial.println("INA260 front (0x40) OK");

  if (!ina_back.begin(INA_BACK_ADDR, &Wire))
    Serial.println("INA260 back (0x41) NOT found!");
  else
    Serial.println("INA260 back (0x41) OK");

  if (!ina_servo.begin(INA_SERVO_ADDR, &Wire))
    Serial.println("INA260 servo (0x44) NOT found!");
  else
    Serial.println("INA260 servo (0x44) OK");

  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.println("MPU-6500 initialized");

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

  // Encoder pins — PULLDOWN to prevent noise on floating pins
  pinMode(ENC_A_PIN, INPUT_PULLDOWN);
  pinMode(ENC_B_PIN, INPUT_PULLDOWN);
  pinMode(ENC_FL_PIN, INPUT_PULLDOWN);
  pinMode(ENC_FR_PIN, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN),  encoderA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN),  encoderB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), encoderFL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), encoderFR_ISR, CHANGE);

  // Shell servo
  shellServo.setPeriodHertz(50);
  shellServo.attach(SERVO_PIN, 500, 2500);
  shellServo.writeMicroseconds(SERVO_STOP_US);

  holdAllStopped();
  setupWebServer();

  Serial.print("Servo attached: "); Serial.println(shellServo.attached());
  Serial.println("Ready. Waiting for START...");
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  server.handleClient();
  webSocket.loop();
  sampleAndSend();

  if (!runEnabled) {
    holdAllStopped();
    delay(5);
    return;
  }

  if (stopRequested && !cycleInProgress && (completedCycles % 4 == 0) && completedCycles > 0) {
    runEnabled = false;
    holdAllStopped();
    closeShellAfterStopping();
    Serial.println("Stop at full 4-leg boundary");
    delay(5);
    return;
  }

  currentCycleNumber = completedCycles + 1;
  cycleInProgress = true;

  uint8_t seqIndex = (currentCycleNumber - 1) % 4;
  uint8_t legIndex = LEG_SEQUENCE[seqIndex];

  unsigned long tCycle = 0;

  Serial.println();
  Serial.print("=== LEG MOVE "); Serial.print(currentCycleNumber);
  Serial.print(": "); Serial.println(legName(legIndex));

  bool ok = walkOneLegMove(legIndex, tCycle);
  cycleInProgress = false;

  if (!ok) {
    runEnabled = false;
    holdAllStopped();
    Serial.println("Leg move failed. Holding brake.");
    return;
  }

  completedCycles = currentCycleNumber;

  Serial.print("Move "); Serial.print(currentCycleNumber);
  Serial.print(" done ("); Serial.print(legName(legIndex));
  Serial.print(") in "); Serial.print(tCycle); Serial.println(" ms");

  if (stopRequested && (completedCycles % 4 == 0)) {
    runEnabled = false;
    holdAllStopped();
    closeShellAfterStopping();
    Serial.println("Graceful STOP complete");
  }
}