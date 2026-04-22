#include <Arduino.h>

// ============================================================
// STAIR auto one-cycle timing test
// Auto-runs once after reset:
//
// 1) Rear Right one cycle
// 2) wait 1 second
// 3) Front Right one cycle
// 4) print summary
// ============================================================

// ---------------- Rear right motor pins ----------------
#define MOTOR_B_INA   4
#define MOTOR_B_INB   5
#define MOTOR_B_PWM   6

// ---------------- Rear right encoder pin ----------------
#define ENC_B_PIN     7

// ---------------- Front right motor pins ----------------
// Kept same as your recent walking code:
//   PWM on GPIO40, GPIO39 LOW
#define FR_IN3        40
#define FR_IN4        39

// ---------------- Front right encoder pin ----------------
#define ENC_FR_PIN    13

// ---------------- Counts from your walking code ----------------
const long REAR_RIGHT_COUNTS_PER_CYCLE  = 680;
const long FRONT_RIGHT_COUNTS_PER_CYCLE = 551;

// ---------------- Test PWM ----------------
const int REAR_RIGHT_TEST_PWM  = 150;
const int FRONT_RIGHT_TEST_PWM = 53;

const unsigned long TIMEOUT_MS = 8000;

// ---------------- Encoder counts ----------------
volatile long encCountB  = 0;
volatile long encCountFR = 0;

bool testDone = false;

// ============================================================
// ISR
// ============================================================
void IRAM_ATTR encoderB_ISR() {
  encCountB++;
}

void IRAM_ATTR encoderFR_ISR() {
  encCountFR++;
}

// ============================================================
// Motor control
// ============================================================
void rearRightForward(uint8_t speed) {
  digitalWrite(MOTOR_B_INA, HIGH);
  digitalWrite(MOTOR_B_INB, LOW);
  ledcWrite(MOTOR_B_PWM, speed);
}

void rearRightStop() {
  digitalWrite(MOTOR_B_INA, HIGH);
  digitalWrite(MOTOR_B_INB, HIGH);
  ledcWrite(MOTOR_B_PWM, 255);   // active brake
}

void frontRightForward(uint8_t speed) {
  ledcWrite(FR_IN3, speed);
  digitalWrite(FR_IN4, LOW);
}

void frontRightBrake() {
  ledcWrite(FR_IN3, 255);
  digitalWrite(FR_IN4, HIGH);
}

void holdAllStopped() {
  rearRightStop();
  frontRightBrake();
}

// ============================================================
// Tests
// ============================================================
bool runRearRightOneCycle(unsigned long &elapsedMs) {
  encCountB = 0;
  delay(100);

  Serial.println();
  Serial.println("---- REAR RIGHT TEST START ----");
  Serial.print("Target counts = ");
  Serial.println(REAR_RIGHT_COUNTS_PER_CYCLE);
  Serial.print("PWM = ");
  Serial.println(REAR_RIGHT_TEST_PWM);

  unsigned long startTime = millis();
  unsigned long lastPrint = 0;

  rearRightForward(REAR_RIGHT_TEST_PWM);

  while (encCountB < REAR_RIGHT_COUNTS_PER_CYCLE) {
    if (millis() - startTime > TIMEOUT_MS) {
      rearRightStop();
      elapsedMs = millis() - startTime;

      Serial.println("REAR RIGHT TIMEOUT");
      Serial.print("Counts reached = ");
      Serial.println(encCountB);
      Serial.print("Elapsed time = ");
      Serial.print(elapsedMs);
      Serial.println(" ms");
      return false;
    }

    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.print("RR counts = ");
      Serial.println(encCountB);
    }
  }

  rearRightStop();
  elapsedMs = millis() - startTime;
  delay(300);

  Serial.println("REAR RIGHT COMPLETE");
  Serial.print("Final counts = ");
  Serial.println(encCountB);
  Serial.print("Elapsed time = ");
  Serial.print(elapsedMs);
  Serial.println(" ms");

  return true;
}

bool runFrontRightOneCycle(unsigned long &elapsedMs) {
  encCountFR = 0;
  delay(100);

  Serial.println();
  Serial.println("---- FRONT RIGHT TEST START ----");
  Serial.print("Target counts = ");
  Serial.println(FRONT_RIGHT_COUNTS_PER_CYCLE);
  Serial.print("PWM = ");
  Serial.println(FRONT_RIGHT_TEST_PWM);

  unsigned long startTime = millis();
  unsigned long lastPrint = 0;

  frontRightForward(FRONT_RIGHT_TEST_PWM);

  while (encCountFR < FRONT_RIGHT_COUNTS_PER_CYCLE) {
    if (millis() - startTime > TIMEOUT_MS) {
      frontRightBrake();
      elapsedMs = millis() - startTime;

      Serial.println("FRONT RIGHT TIMEOUT");
      Serial.print("Counts reached = ");
      Serial.println(encCountFR);
      Serial.print("Elapsed time = ");
      Serial.print(elapsedMs);
      Serial.println(" ms");
      return false;
    }

    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.print("FR counts = ");
      Serial.println(encCountFR);
    }
  }

  frontRightBrake();
  elapsedMs = millis() - startTime;
  delay(300);

  Serial.println("FRONT RIGHT COMPLETE");
  Serial.print("Final counts = ");
  Serial.println(encCountFR);
  Serial.print("Elapsed time = ");
  Serial.print(elapsedMs);
  Serial.println(" ms");

  return true;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===== STAIR AUTO TIMING TEST =====");
  Serial.println("Will auto-run after 3 seconds...");
  Serial.println("Test order: rear right -> front right");
  Serial.println("==================================");

  pinMode(MOTOR_B_INA, OUTPUT);
  pinMode(MOTOR_B_INB, OUTPUT);

  pinMode(FR_IN3, OUTPUT);
  pinMode(FR_IN4, OUTPUT);

  ledcAttach(MOTOR_B_PWM, 20000, 8);
  ledcAttach(FR_IN3, 20000, 8);

  pinMode(ENC_B_PIN, INPUT);
  pinMode(ENC_FR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), encoderFR_ISR, CHANGE);

  holdAllStopped();
}

// ============================================================
// Loop
// ============================================================
void loop() {
  if (testDone) {
    delay(1000);
    return;
  }

  delay(3000);

  unsigned long tRear = 0;
  unsigned long tFront = 0;

  bool okRear = runRearRightOneCycle(tRear);

  delay(1000);

  bool okFront = runFrontRightOneCycle(tFront);

  Serial.println();
  Serial.println("========== SUMMARY ==========");

  Serial.print("Rear Right: ");
  if (okRear) {
    Serial.print(tRear);
    Serial.println(" ms");
  } else {
    Serial.println("FAILED / TIMEOUT");
  }

  Serial.print("Front Right: ");
  if (okFront) {
    Serial.print(tFront);
    Serial.println(" ms");
  } else {
    Serial.println("FAILED / TIMEOUT");
  }

  if (okRear && okFront) {
    long diff = (long)tRear - (long)tFront;
    Serial.print("Time difference = ");
    Serial.print(diff);
    Serial.println(" ms");

    if (diff > 0) {
      Serial.println("Rear right is slower.");
    } else if (diff < 0) {
      Serial.println("Front right is slower.");
    } else {
      Serial.println("They are matched.");
    }
  }

  Serial.println("=============================");
  holdAllStopped();
  testDone = true;
}