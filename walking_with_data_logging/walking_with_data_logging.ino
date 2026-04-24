#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_INA260.h>
#include <math.h>

// ============================================================
// STAIR Robot — Walking + IMU Flip/Recovery + Power/IMU Logger
// MERGED: walking_with_4_leg_speed + data_collecting_interface
//
// Walking / flip behavior (unchanged):
// - ESP32 creates Wi-Fi hotspot "STAIR-Robot"
// - Web UI has START and STOP
// - START waits 2s, opens shell, then begins walking
// - STOP finishes on the next full 4-leg boundary, then closes shell
// - IMU pitch > threshold latches a flip request
// - flip executes at next even move boundary
// - after flip, robot holds active brake until pitch is stable
//   within +/-5 deg for 5 seconds, then rephases and resumes walking
//
// Walking pattern:
//   Move 1: RL  (double cycle)
//   Move 2: FL  (double cycle)
//   Move 3: RR  (double cycle)
//   Move 4: FR  (double cycle)
//   Then repeat...
//
// Data logging additions:
// - 3x INA260 power sensors (front 0x40, back 0x41, servo 0x44)
// - Raw MPU accel + gyro via direct I2C reads (same chip as tockn lib, 0x68)
// - WebSocket server on port 81 streams CSV at 100ms
// - Web page has RECORD / DOWNLOAD CSV buttons + live data display
//
// IMU strategy (per user requirement):
// - Pitch angle (for flip detection) via MPU6050_tockn library
// - Raw accel / gyro (for CSV logging) via direct I2C reads
// Both use the same I2C bus and chip at 0x68. Blocking I2C, no ISR
// collisions, so they serialize cleanly.
// ============================================================

// ---------------- Web UI ----------------
const char* AP_SSID = "STAIR-Robot";
const char* AP_PASS = "12345678";
WebServer server(80);
WebSocketsServer webSocket(81);

bool runEnabled      = false;
bool stopRequested   = false;
bool cycleInProgress = false;

unsigned long completedCycles    = 0;   // completed LEG MOVES
unsigned long currentCycleNumber = 0;   // current LEG MOVE number

// ---------------- Shell servo ----------------
Servo shellServo;

const int SERVO_PIN      = 8;
const int SERVO_STOP_US  = 1500;
const int SERVO_OPEN_US  = 2400;
const int SERVO_CLOSE_US = 600;

const unsigned long SHELL_PRESTART_DELAY_MS    = 2000;
const unsigned long SHELL_OPEN_TIME_MS         = 1500;
const unsigned long SHELL_CLOSE_TIME_MS_NORMAL = 1340;
const unsigned long SHELL_CLOSE_TIME_MS_LATE   = 1120;
const unsigned long LATE_STOP_WINDOW_MS        = 5000;

bool shellIsOpen = false;
unsigned long startButtonPressedMs = 0;
bool useLateShellClose = false;

// ---------------- IMU: tockn library for pitch ----------------
MPU6050 mpu6050(Wire);

const int SDA_PIN = 36;
const int SCL_PIN = 35;

float angle_offset = 0.0f;
float pitchDeg = 0.0f;

unsigned long flipPositiveStartMs = 0;
const unsigned long FLIP_TRIGGER_HOLD_MS = 150;
const float PITCH_FLIP_THRESHOLD_DEG = 12.0f;
const float PITCH_RECOVERY_BAND_DEG  = 5.0f;
const unsigned long RECOVERY_STABLE_TIME_MS = 5000;

bool flipRequested      = false;
bool flipInProgress     = false;
bool waitingForRecovery = false;
bool rephaseInProgress  = false;

unsigned long recoveryStableStartMs = 0;

// ---------------- IMU: raw accel/gyro for CSV logging ----------------
#define MPU6500_ADDR    0x68
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX  = 0, gyroY  = 0, gyroZ  = 0;

// ---------------- INA260 power sensors ----------------
#define INA_FRONT_ADDR  0x40
#define INA_BACK_ADDR   0x41
#define INA_SERVO_ADDR  0x44

Adafruit_INA260 ina_front;
Adafruit_INA260 ina_back;
Adafruit_INA260 ina_servo;

bool ina_front_ok = false;
bool ina_back_ok  = false;
bool ina_servo_ok = false;

// ---------------- Sample timing ----------------
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 100;

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
const long REAR_LEFT_COUNTS_PER_CYCLE   = 688;
const long REAR_RIGHT_COUNTS_PER_CYCLE  = 719;
const long FRONT_LEFT_COUNTS_PER_CYCLE  = 614;
const long FRONT_RIGHT_COUNTS_PER_CYCLE = 629;

const int LEG_MOVE_MULTIPLIER = 2;  // normal walking = 2x cycle per active leg

// 0 = RL, 1 = FL, 2 = RR, 3 = FR
const uint8_t LEG_SEQUENCE[4] = {0, 2, 1, 3};

// ---------------- Speed tuning ----------------
const uint8_t REAR_LEFT_SPEED   = 200;
const uint8_t REAR_RIGHT_SPEED  = 210;

const uint8_t FRONT_LEFT_SPEED  = 70;
const uint8_t FRONT_RIGHT_SPEED = 70;

#define TIMEOUT_MS   8000

// ---------------- Flip tuning ----------------
const long REAR_FLIP_FORWARD_COUNTS   = 600;   // A
const long REAR_FLIP_BACKWARD_COUNTS  = 1000;  // B, must be > A

const uint8_t REAR_FLIP_FORWARD_PWM_LEFT   = REAR_LEFT_SPEED;
const uint8_t REAR_FLIP_FORWARD_PWM_RIGHT  = REAR_RIGHT_SPEED;
const uint8_t REAR_FLIP_BACKWARD_PWM       = 255;
const uint8_t REAR_FLIP_RECOVER_PWM_LEFT   = REAR_LEFT_SPEED;
const uint8_t REAR_FLIP_RECOVER_PWM_RIGHT  = REAR_RIGHT_SPEED;

const unsigned long POST_FLIP_FRONT_DELAY_MS = 400;

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
void setupINA260s();
String buildStatusJson();

void openShellBeforeWalking();
void reopenShellAfterRecovery();
void closeShellAfterStopping();

void updateIMU();
void readMPU6500();
void sampleAndSend();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void handlePostFlipRecovery();
void performFlipSequence();

const char* legName(uint8_t legIndex);

bool walkOneLegMoveWithMultiplier(uint8_t legIndex, int moveMultiplier, unsigned long &elapsedMs);
bool walkOneLegMove(uint8_t legIndex, unsigned long &elapsedMs);

bool rotateRearBothOneCycle(unsigned long &elapsedMs);
bool rotateFrontBothOneCycle(unsigned long &elapsedMs);
bool rearFlipKickPulse(unsigned long &elapsedMs);
bool rephaseAllLegsOneCycle();

// Convenience: keep network + logging alive inside blocking while() loops
inline void serviceNetworkAndLog() {
  server.handleClient();
  webSocket.loop();
  sampleAndSend();
  yield();
}

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
    h2 {
      margin-top: 0;
    }
    .card {
      max-width: 460px;
      margin: 0 auto 20px auto;
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
    .recBtn   { background: #e76f51; }
    .dlBtn    { background: #264653; }
    .status {
      margin-top: 20px;
      font-size: 20px;
      line-height: 1.7;
    }
    .dataStatus {
      margin-top: 16px;
      font-size: 18px;
      line-height: 1.7;
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
        document.getElementById('pitchDeg').textContent = s.pitchDeg;
      } catch (e) {
        document.getElementById('state').textContent = 'Disconnected';
      }
    }

    updateStatus();
    setInterval(updateStatus, 300);

    // -------- Data logger / WebSocket --------
    var recording = false;
    var dataRows  = [];
    var ws;
    var startTs   = 0;

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
      ws.onclose = function() { setTimeout(connectWS, 1000); };
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
// ISRs
// ============================================================
void IRAM_ATTR encoderA_ISR()  { encCountA++;  }
void IRAM_ATTR encoderB_ISR()  { encCountB++;  }
void IRAM_ATTR encoderFL_ISR() { encCountFL++; }
void IRAM_ATTR encoderFR_ISR() { encCountFR++; }

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
// Pitch for flip detection (uses tockn library's complementary filter)
void updateIMU() {
  mpu6050.update();
  pitchDeg = mpu6050.getAngleX() - angle_offset;
}

// Raw accel + gyro for CSV logging (direct I2C read of registers 0x3B..0x48)
void readMPU6500() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;

  uint8_t received = Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)14);
  if (received < 14) return;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  accelX = rawAx / 16384.0;  // +/-2g default
  accelY = rawAy / 16384.0;
  accelZ = rawAz / 16384.0;

  gyroX = rawGx / 131.0;     // +/-250 dps default
  gyroY = rawGy / 131.0;
  gyroZ = rawGz / 131.0;
}

// Throttled sampler that streams CSV over WebSocket every SAMPLE_INTERVAL_MS
void sampleAndSend() {
  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  float p_front = ina_front_ok ? ina_front.readPower() : 0.0f;
  float p_back  = ina_back_ok  ? ina_back.readPower()  : 0.0f;
  float p_servo = ina_servo_ok ? ina_servo.readPower() : 0.0f;

  // clamp crazy reads (sensor resets, etc.)
  if (p_front > 100000) p_front = 0;
  if (p_back  > 100000) p_back  = 0;
  if (p_servo > 100000) p_servo = 0;

  readMPU6500();

  // CSV: time, power*3, encoder*4, accel*3, gyro*3
  String msg = String(now) + ","
             + String(p_front, 1) + ","
             + String(p_back,  1) + ","
             + String(p_servo, 1) + ","
             + String(encCountA)  + ","
             + String(encCountB)  + ","
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

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("WebSocket client #%u connected\n", num);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("WebSocket client #%u disconnected\n", num);
  }
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
// INA260 sensor init
// ============================================================
void setupINA260s() {
  // NOTE: Wire.begin() is NOT called here. It was already called in setup()
  // with the correct pins (SDA=36, SCL=35). Calling it again risks resetting
  // the pins on some ESP32 core versions.

  ina_front_ok = ina_front.begin(INA_FRONT_ADDR, &Wire);
  Serial.print("INA260 front (0x40): ");
  Serial.println(ina_front_ok ? "OK" : "NOT FOUND");

  ina_back_ok = ina_back.begin(INA_BACK_ADDR, &Wire);
  Serial.print("INA260 back  (0x41): ");
  Serial.println(ina_back_ok ? "OK" : "NOT FOUND");

  ina_servo_ok = ina_servo.begin(INA_SERVO_ADDR, &Wire);
  Serial.print("INA260 servo (0x44): ");
  Serial.println(ina_servo_ok ? "OK" : "NOT FOUND");
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
  if (rlActive) rearLeftForward(REAR_LEFT_SPEED);
  if (rrActive) rearRightForward(REAR_RIGHT_SPEED);
  if (flActive) frontLeftForward(FRONT_LEFT_SPEED);
  if (frActive) frontRightForward(FRONT_RIGHT_SPEED);

  while (!(rlDone && rrDone && flDone && frDone)) {
    serviceNetworkAndLog();
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
        ledcWrite(MOTOR_A_PWM, REAR_LEFT_SPEED);
      }
    }

    // Rear Right
    if (!rrDone) {
      if (encCountB >= rrTarget) {
        rearRightStop();
        rrDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_RIGHT_SPEED);
      }
    }

    // Front Left
    if (!flDone) {
      if (encCountFL >= flTarget) {
        frontLeftBrake();
        flDone = true;
      } else {
        ledcWrite(FL_IN1, FRONT_LEFT_SPEED);
      }
    }

    // Front Right
    if (!frDone) {
      if (encCountFR >= frTarget) {
        frontRightBrake();
        frDone = true;
      } else {
        ledcWrite(FR_IN3, FRONT_RIGHT_SPEED);
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

  rearLeftForward(REAR_LEFT_SPEED);
  rearRightForward(REAR_RIGHT_SPEED);

  while (!(rlDone && rrDone)) {
    serviceNetworkAndLog();
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
        ledcWrite(MOTOR_A_PWM, REAR_LEFT_SPEED);
      }
    }

    if (!rrDone) {
      if (encCountB >= REAR_RIGHT_COUNTS_PER_CYCLE) {
        rearRightStop();
        rrDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_RIGHT_SPEED);
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

  frontLeftForward(FRONT_LEFT_SPEED);
  frontRightForward(FRONT_RIGHT_SPEED);

  while (!(flDone && frDone)) {
    serviceNetworkAndLog();
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
        ledcWrite(FL_IN1, FRONT_LEFT_SPEED);
      }
    }

    if (!frDone) {
      if (encCountFR >= FRONT_RIGHT_COUNTS_PER_CYCLE) {
        frontRightBrake();
        frDone = true;
      } else {
        ledcWrite(FR_IN3, FRONT_RIGHT_SPEED);
      }
    }
  }

  holdAllStopped();
  elapsedMs = millis() - startTime;
  return true;
}

bool rearFlipKickPulse(unsigned long &elapsedMs) {
  const long recoveryCounts = REAR_FLIP_BACKWARD_COUNTS - REAR_FLIP_FORWARD_COUNTS;
  if (recoveryCounts <= 0) {
    Serial.println("Invalid rear flip pulse config: backward counts must be larger than forward counts.");
    return false;
  }

  unsigned long totalStartTime = millis();

  // Phase 1: short forward A
  encCountA = 0;
  encCountB = 0;
  bool rlFwdDone = false;
  bool rrFwdDone = false;

  unsigned long phaseStart = millis();
  rearLeftForward(REAR_FLIP_FORWARD_PWM_LEFT);
  rearRightForward(REAR_FLIP_FORWARD_PWM_RIGHT);

  while (!(rlFwdDone && rrFwdDone)) {
    serviceNetworkAndLog();
    updateIMU();

    if (millis() - phaseStart > TIMEOUT_MS) {
      holdAllStopped();
      return false;
    }

    if (!rlFwdDone) {
      if (encCountA >= REAR_FLIP_FORWARD_COUNTS) {
        rearLeftStop();
        rlFwdDone = true;
      } else {
        ledcWrite(MOTOR_A_PWM, REAR_FLIP_FORWARD_PWM_LEFT);
      }
    }

    if (!rrFwdDone) {
      if (encCountB >= REAR_FLIP_FORWARD_COUNTS) {
        rearRightStop();
        rrFwdDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_FLIP_FORWARD_PWM_RIGHT);
      }
    }
  }

  delay(20);

  // Phase 2: larger backward B fast
  encCountA = 0;
  encCountB = 0;
  bool rlRevDone = false;
  bool rrRevDone = false;

  phaseStart = millis();
  rearLeftReverse(REAR_FLIP_BACKWARD_PWM);
  rearRightReverse(REAR_FLIP_BACKWARD_PWM);

  while (!(rlRevDone && rrRevDone)) {
    serviceNetworkAndLog();
    updateIMU();

    if (millis() - phaseStart > TIMEOUT_MS) {
      holdAllStopped();
      return false;
    }

    if (!rlRevDone) {
      if (encCountA >= REAR_FLIP_BACKWARD_COUNTS) {
        rearLeftStop();
        rlRevDone = true;
      } else {
        ledcWrite(MOTOR_A_PWM, REAR_FLIP_BACKWARD_PWM);
      }
    }

    if (!rrRevDone) {
      if (encCountB >= REAR_FLIP_BACKWARD_COUNTS) {
        rearRightStop();
        rrRevDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_FLIP_BACKWARD_PWM);
      }
    }
  }

  delay(20);

  // Phase 3: forward (B - A) to return to original rear-leg position
  encCountA = 0;
  encCountB = 0;
  bool rlRecoverDone = false;
  bool rrRecoverDone = false;

  phaseStart = millis();
  rearLeftForward(REAR_FLIP_RECOVER_PWM_LEFT);
  rearRightForward(REAR_FLIP_RECOVER_PWM_RIGHT);

  while (!(rlRecoverDone && rrRecoverDone)) {
    serviceNetworkAndLog();
    updateIMU();

    if (millis() - phaseStart > TIMEOUT_MS) {
      holdAllStopped();
      return false;
    }

    if (!rlRecoverDone) {
      if (encCountA >= recoveryCounts) {
        rearLeftStop();
        rlRecoverDone = true;
      } else {
        ledcWrite(MOTOR_A_PWM, REAR_FLIP_RECOVER_PWM_LEFT);
      }
    }

    if (!rrRecoverDone) {
      if (encCountB >= recoveryCounts) {
        rearRightStop();
        rrRecoverDone = true;
      } else {
        ledcWrite(MOTOR_B_PWM, REAR_FLIP_RECOVER_PWM_RIGHT);
      }
    }
  }

  holdAllStopped();
  elapsedMs = millis() - totalStartTime;
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

  Serial.print("Rear kick pulse: ");
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

  Serial.println("STAIR - walking + shell servo + IMU flip + data logger");
  Serial.println("Move 1: RL (double cycle)");
  Serial.println("Move 2: FL (double cycle)");
  Serial.println("Move 3: RR (double cycle)");
  Serial.println("Move 4: FR (double cycle)");

  Serial.print("Rear left counts/cycle   = "); Serial.println(REAR_LEFT_COUNTS_PER_CYCLE);
  Serial.print("Rear right counts/cycle  = "); Serial.println(REAR_RIGHT_COUNTS_PER_CYCLE);
  Serial.print("Front left counts/cycle  = "); Serial.println(FRONT_LEFT_COUNTS_PER_CYCLE);
  Serial.print("Front right counts/cycle = "); Serial.println(FRONT_RIGHT_COUNTS_PER_CYCLE);
  Serial.print("Leg move multiplier      = "); Serial.println(LEG_MOVE_MULTIPLIER);
  Serial.print("Rear left speed          = "); Serial.println(REAR_LEFT_SPEED);
  Serial.print("Rear right speed         = "); Serial.println(REAR_RIGHT_SPEED);
  Serial.print("Front left speed         = "); Serial.println(FRONT_LEFT_SPEED);
  Serial.print("Front right speed        = "); Serial.println(FRONT_RIGHT_SPEED);

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

  // I2C bus - one shared bus for MPU + 3x INA260. Only Wire.begin once.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // INA260 power sensors (non-blocking if missing; logger will just send 0)
  Serial.println("Setting up INA260 power sensors...");
  setupINA260s();

  // MPU6050 library init (for pitch). This also wakes the chip (writes 0
  // to PWR_MGMT_1 reg 0x6B), so the raw readMPU6500() side does not need
  // to do its own wake-up. That was the setup overlap from the data
  // collection module.
  Serial.println("I2C + MPU6050 init");
  mpu6050.begin();

  Serial.println("Calibrating gyro... keep sensor still");
  mpu6050.calcGyroOffsets(true);   // ~3 sec, lets power rail settle

  Serial.println("Settling IMU...");
  for (int i = 0; i < 100; i++) {  // another ~1 sec
    mpu6050.update();
    delay(10);
  }

  angle_offset = mpu6050.getAngleX();
  Serial.print("Pitch zero offset = ");
  Serial.println(angle_offset, 4);

  holdAllStopped();

  // Wi-Fi AP + HTTP server last, AFTER the ~4 sec of IMU settling.
  // That delay is what lets the hotspot come up cleanly on this hardware.
  setupWebServer();

  // WebSocket server for live data streaming (port 81). Needs WiFi up.
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");

  Serial.print("Servo attached: ");
  Serial.println(shellServo.attached());

  Serial.println("Ready. Waiting for START button...");
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  server.handleClient();
  webSocket.loop();
  sampleAndSend();

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

  // While walking, latch flip request only if POSITIVE pitch stays
  // above threshold for a short time. Negative pitch never triggers.
  if (!flipRequested && !flipInProgress && !rephaseInProgress) {
    if (pitchDeg >= PITCH_FLIP_THRESHOLD_DEG) {
      if (flipPositiveStartMs == 0) {
        flipPositiveStartMs = millis();
      } else if (millis() - flipPositiveStartMs >= FLIP_TRIGGER_HOLD_MS) {
        flipRequested = true;
        flipPositiveStartMs = 0;
        Serial.print("IMU flip request latched. Pitch = ");
        Serial.println(pitchDeg, 2);
      }
    } else {
      flipPositiveStartMs = 0;
    }
  }

  // Manual STOP button behavior: stop only after a full 4-leg sequence boundary
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

  // Manual STOP button behavior
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
