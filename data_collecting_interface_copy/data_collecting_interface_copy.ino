// ============================================================
// DATA COLLECTION MODULE — INA260 Power + MPU-6500 IMU
// Merge this into your final robot code
// ============================================================
//
// What this adds:
// - 3x INA260 power reading via I2C
// - MPU-6500 accel + gyro reading via I2C
// - WebSocket server on port 81 streaming CSV every 100ms
// - Browser-side Record / Download CSV buttons
//
// Dependencies (install via Arduino Library Manager):
// - Adafruit INA260 Library (by Adafruit)
// - WebSockets (by Markus Sattler / Links2004)
//
// I2C pins: SDA=36, SCL=35
// All sensors powered by 3.3V from ESP32 in parallel
// ============================================================


// ===================== INCLUDES =====================
// Add these to your existing includes:

#include <WebSocketsServer.h>
#include <Adafruit_INA260.h>
#include <Wire.h>


// ===================== GLOBALS =====================
// Add these to your global variable section:

WebSocketsServer webSocket(81);

#define INA_FRONT_ADDR  0x40
#define INA_BACK_ADDR   0x41
#define INA_SERVO_ADDR  0x44
#define MPU6500_ADDR    0x68

Adafruit_INA260 ina_front;
Adafruit_INA260 ina_back;
Adafruit_INA260 ina_servo;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 100;


// ===================== FUNCTIONS =====================

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

  accelX = rawAx / 16384.0;  // ±2g default
  accelY = rawAy / 16384.0;
  accelZ = rawAz / 16384.0;

  gyroX = rawGx / 131.0;     // ±250 dps default
  gyroY = rawGy / 131.0;
  gyroZ = rawGz / 131.0;
}

// Call this in loop() and inside any blocking while() loops
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

  // CSV: time, power×3, encoder×4, accel×3, gyro×3
  // Replace encCountA/B/FL/FR with your actual encoder variable names
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

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("WebSocket client #%u connected\n", num);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("WebSocket client #%u disconnected\n", num);
  }
}


// ===================== SETUP =====================
// Add this inside your setup() function:

void setupDataCollection() {
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

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");
}


// ===================== LOOP =====================
// Add these two lines inside your loop() and any blocking while() loops:
//
//   webSocket.loop();
//   sampleAndSend();


// ===================== HTML =====================
// Add this card inside your web page HTML, after your existing control card:
//
// <div class="card">
//   <h2>Data Logger</h2>
//   <div class="btnRow">
//     <button class="recBtn" id="recBtn" onclick="toggleRecord()">RECORD</button>
//     <button class="dlBtn" onclick="downloadCSV()">DOWNLOAD CSV</button>
//   </div>
//   <div class="dataStatus">
//     <div><b>Recording:</b> <span id="recStatus">NO</span></div>
//     <div><b>Rows:</b> <span id="rowCount">0</span></div>
//   </div>
//   <div class="live" id="liveData">
//     Front: -- mW | Back: -- mW | Servo: -- mW<br>
//     Accel: -- / -- / -- g<br>
//     Gyro: -- / -- / -- dps
//   </div>
//   <div class="small">
//     Press RECORD then START the robot.<br>
//     Press DOWNLOAD CSV when done.
//   </div>
// </div>
//
// ===================== CSS =====================
// Add these styles:
//
// .recBtn   { background: #e76f51; }
// .dlBtn    { background: #264653; }
// .dataStatus { margin-top: 16px; font-size: 18px; line-height: 1.7; }
// .live {
//   font-family: monospace; font-size: 14px;
//   background: #f9f9f9; padding: 10px;
//   border-radius: 8px; margin-top: 10px;
//   text-align: left; line-height: 1.6;
// }
//
// ===================== JAVASCRIPT =====================
// Add this script block inside your HTML:
//
// var recording = false;
// var dataRows = [];
// var ws;
// var startTs = 0;
//
// function connectWS() {
//   var host = window.location.hostname;
//   ws = new WebSocket('ws://' + host + ':81');
//   ws.onmessage = function(evt) {
//     var parts = evt.data.split(',');
//     var pf = parseFloat(parts[1]);
//     var pb = parseFloat(parts[2]);
//     var ps = parseFloat(parts[3]);
//     var ax = parseFloat(parts[8]);
//     var ay = parseFloat(parts[9]);
//     var az = parseFloat(parts[10]);
//     var gx = parseFloat(parts[11]);
//     var gy = parseFloat(parts[12]);
//     var gz = parseFloat(parts[13]);
//     document.getElementById('liveData').innerHTML =
//       'Front: ' + pf.toFixed(0) + ' mW | Back: ' + pb.toFixed(0) + ' mW | Servo: ' + ps.toFixed(0) + ' mW<br>' +
//       'Accel: ' + ax.toFixed(2) + ' / ' + ay.toFixed(2) + ' / ' + az.toFixed(2) + ' g<br>' +
//       'Gyro: ' + gx.toFixed(1) + ' / ' + gy.toFixed(1) + ' / ' + gz.toFixed(1) + ' dps';
//     if (recording) {
//       if (dataRows.length === 0) startTs = parseInt(parts[0]);
//       var relTime = parseInt(parts[0]) - startTs;
//       dataRows.push(relTime + ',' + parts.slice(1).join(','));
//       document.getElementById('rowCount').textContent = dataRows.length;
//     }
//   };
//   ws.onclose = function() { setTimeout(connectWS, 1000); };
// }
// connectWS();
//
// function toggleRecord() {
//   recording = !recording;
//   if (recording) {
//     dataRows = [];
//     startTs = 0;
//     document.getElementById('rowCount').textContent = '0';
//   }
//   document.getElementById('recStatus').textContent = recording ? 'YES' : 'NO';
//   document.getElementById('recBtn').textContent = recording ? 'STOP REC' : 'RECORD';
//   document.getElementById('recBtn').style.background = recording ? '#d62828' : '#e76f51';
// }
//
// function downloadCSV() {
//   if (dataRows.length === 0) { alert('No data recorded'); return; }
//   var header = 'time_ms,power_front_mW,power_back_mW,power_servo_mW,enc_RL,enc_RR,enc_FL,enc_FR,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps';
//   var csv = header + '\n' + dataRows.join('\n');
//   var blob = new Blob([csv], { type: 'text/csv' });
//   var a = document.createElement('a');
//   a.href = URL.createObjectURL(blob);
//   a.download = 'stair_power_log.csv';
//   a.click();
// }
