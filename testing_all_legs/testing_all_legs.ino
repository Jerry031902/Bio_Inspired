#include <Arduino.h>

// ============================================================
// STAIR all-motors 3-second test
//
// Auto-runs once after reset:
// 1) wait 2 seconds
// 2) drive all 4 motors forward for 3 seconds
// 3) active brake all motors
// 4) stop
//
// No encoders, no Wi-Fi, no servo
// ============================================================

// ---------------- Rear motor pins ----------------
// Rear Left
#define MOTOR_A_INA   2
#define MOTOR_A_INB   1
#define MOTOR_A_PWM   3

// Rear Right
#define MOTOR_B_INA   4
#define MOTOR_B_INB   5
#define MOTOR_B_PWM   6

// ---------------- Front motor pins ----------------
// Front Left
#define FL_IN1        42
#define FL_IN2        41

// Front Right
#define FR_IN3        40
#define FR_IN4        39

// ---------------- Test timing ----------------
const unsigned long START_DELAY_MS = 2000;
const unsigned long RUN_TIME_MS    = 3000;
const unsigned long BRAKE_HOLD_MS  = 500;

// ---------------- Test PWM ----------------
// Kept close to your current one-leg file values
const uint8_t REAR_TEST_PWM  = 200;
const uint8_t FRONT_TEST_PWM = 66;

bool testDone = false;

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

void rearLeftBrake() {
  digitalWrite(MOTOR_A_INA, HIGH);
  digitalWrite(MOTOR_A_INB, HIGH);
  ledcWrite(MOTOR_A_PWM, 255);
}

void rearRightBrake() {
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

// ============================================================
// Group helpers
// ============================================================
void driveAllForward() {
  rearLeftForward(REAR_TEST_PWM);
  rearRightForward(REAR_TEST_PWM);
  frontLeftForward(FRONT_TEST_PWM);
  frontRightForward(FRONT_TEST_PWM);
}

void brakeAll() {
  rearLeftBrake();
  rearRightBrake();
  frontLeftBrake();
  frontRightBrake();
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("===== STAIR ALL-MOTORS 3-SECOND TEST =====");
  Serial.println("Will auto-run once after reset.");
  Serial.println("Pattern: all 4 motors forward for 3 seconds");
  Serial.println("Then active brake.");
  Serial.print("Rear PWM  = ");
  Serial.println(REAR_TEST_PWM);
  Serial.print("Front PWM = ");
  Serial.println(FRONT_TEST_PWM);
  Serial.println("==========================================");

  // Output pins
  pinMode(MOTOR_A_INA, OUTPUT);
  pinMode(MOTOR_A_INB, OUTPUT);
  pinMode(MOTOR_B_INA, OUTPUT);
  pinMode(MOTOR_B_INB, OUTPUT);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT);
  pinMode(FR_IN4, OUTPUT);

  // PWM attach
  ledcAttach(MOTOR_A_PWM, 20000, 8);
  ledcAttach(MOTOR_B_PWM, 20000, 8);
  ledcAttach(FL_IN1, 20000, 8);
  ledcAttach(FR_IN3, 20000, 8);

  // Safe state first
  brakeAll();

  Serial.println("Waiting 2 seconds before motion...");
}

// ============================================================
// Loop
// ============================================================
void loop() {
  if (testDone) {
    delay(1000);
    return;
  }

  delay(START_DELAY_MS);

  Serial.println("Starting all 4 motors now...");
  driveAllForward();

  delay(RUN_TIME_MS);

  Serial.println("Applying active brake...");
  brakeAll();

  delay(BRAKE_HOLD_MS);

  Serial.println("Test complete.");
  testDone = true;
}