#include <ESP32Servo.h>

// ============================================================
// SPEED CALIBRATION TEST
// Compare:
//   - 1 rear Pololu DC leg (encoder-based 1 rev)
//   - 1 front continuous servo leg (Hall-based 1 rev)
// Goal:
//   Match average time per revolution
// ============================================================

// ---------------- Rear Left Pololu motor ----------------
#define MOTOR_INA   1
#define MOTOR_INB   2
#define MOTOR_PWM   3
#define ENC_PIN     7    // one encoder channel through divider

// ---------------- Front Left servo ----------------
#define HALL_PIN    13
#define SERVO_PIN   11

// ---------------- Calibration values to tune ----------------
#define DC_PWM_VALUE        150    // tune this
#define SERVO_FWD_US        1300   // tune this
#define SERVO_STOP_US       1500

// ---------------- Test settings ----------------
#define COUNTS_PER_REV      4800   // one encoder channel, CHANGE
#define DC_TIMEOUT_MS       5000
#define SERVO_TIMEOUT_MS    5000
#define HOME_TIMEOUT_MS     5000
#define SETTLE_MS           100
#define BETWEEN_TESTS_MS    1000
#define NUM_TRIALS          3

// ---------------- Globals ----------------
volatile long encCount = 0;
volatile bool hallTriggered = false;

Servo frontServo;

// ============================================================
// ISR
// ============================================================
void IRAM_ATTR encoderISR() {
  encCount++;
}

void IRAM_ATTR hallISR() {
  hallTriggered = true;
}

// ============================================================
// Rear motor helpers
// ============================================================
void motorForward(uint8_t speedVal) {
  digitalWrite(MOTOR_INA, HIGH);
  digitalWrite(MOTOR_INB, LOW);
  ledcWrite(MOTOR_PWM, speedVal);
}

void motorBrake() {
  // Your active-brake version that worked
  digitalWrite(MOTOR_INA, LOW);
  digitalWrite(MOTOR_INB, LOW);
  ledcWrite(MOTOR_PWM, 255);
}

// ============================================================
// Servo helpers
// ============================================================
void servoStop() {
  frontServo.writeMicroseconds(SERVO_STOP_US);
}

void servoForward() {
  frontServo.writeMicroseconds(SERVO_FWD_US);
}

// ============================================================
// Wait for Hall edge
// ============================================================
bool waitForHallEdge(unsigned long timeoutMs) {
  hallTriggered = false;
  unsigned long start = millis();

  while (!hallTriggered) {
    if (millis() - start > timeoutMs) {
      return false;
    }
  }
  return true;
}

// ============================================================
// Rear DC one revolution
// Returns true if success, false if timeout
// revTimeMs gets filled with measured time
// ============================================================
bool runOneRevDC(unsigned long &revTimeMs) {
  encCount = 0;

  unsigned long startTime = millis();
  unsigned long timeout = startTime + DC_TIMEOUT_MS;

  motorForward(DC_PWM_VALUE);

  while (encCount < COUNTS_PER_REV) {
    if (millis() > timeout) {
      motorBrake();
      delay(SETTLE_MS);
      Serial.println("DC TIMEOUT");
      Serial.print("DC count reached: ");
      Serial.println(encCount);
      return false;
    }
  }

  motorBrake();
  delay(SETTLE_MS);

  revTimeMs = millis() - startTime;

  Serial.print("DC final count after settle: ");
  Serial.println(encCount);

  return true;
}

// ============================================================
// Front servo one revolution
// Logic:
// - start moving forward
// - if magnet is already detected, wait for it to leave
// - then wait for next FALLING edge = one full revolution
// ============================================================
bool runOneRevServo(unsigned long &revTimeMs) {
  Serial.print("Initial hall state = ");
  Serial.println(digitalRead(HALL_PIN));

  servoForward();

  // If magnet starts at sensor, wait until it leaves first
  if (digitalRead(HALL_PIN) == LOW) {
    Serial.println("Servo magnet starts at sensor, waiting to leave...");

    unsigned long leaveStart = millis();
    while (digitalRead(HALL_PIN) == LOW) {
      if (millis() - leaveStart > HOME_TIMEOUT_MS) {
        servoStop();
        Serial.println("SERVO TIMEOUT waiting for magnet to leave");
        return false;
      }
    }

    Serial.println("Servo magnet left sensor");
    delay(20);
  }

  unsigned long startTime = millis();
  bool gotRev = waitForHallEdge(SERVO_TIMEOUT_MS);

  servoStop();
  delay(SETTLE_MS);

  if (!gotRev) {
    Serial.println("SERVO TIMEOUT waiting for full revolution");
    return false;
  }

  revTimeMs = millis() - startTime;
  return true;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("========================================");
  Serial.println("Rear DC + Front Servo Speed Calibration");
  Serial.println("========================================");
  Serial.print("DC PWM = ");
  Serial.println(DC_PWM_VALUE);
  Serial.print("Servo forward us = ");
  Serial.println(SERVO_FWD_US);
  Serial.println();

  // ----- Rear motor setup -----
  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);

  ledcAttach(MOTOR_PWM, 20000, 8);

  pinMode(ENC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encoderISR, CHANGE);

  motorBrake();

  // ----- Front servo setup -----
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  frontServo.setPeriodHertz(50);
  frontServo.attach(SERVO_PIN, 500, 2500);
  servoStop();

  delay(1000);

  Serial.println("Type any key to start calibration...");
  while (!Serial.available()) {
    // wait
  }
  Serial.read();

  delay(1000);

  // =========================================================
  // DC trials
  // =========================================================
  unsigned long dcTimes[NUM_TRIALS];
  unsigned long servoTimes[NUM_TRIALS];
  int dcSuccessCount = 0;
  int servoSuccessCount = 0;
  unsigned long dcSum = 0;
  unsigned long servoSum = 0;

  Serial.println();
  Serial.println("----- DC MOTOR TESTS -----");

  for (int i = 0; i < NUM_TRIALS; i++) {
    Serial.print("DC Trial ");
    Serial.print(i + 1);
    Serial.println("...");

    unsigned long t = 0;
    bool ok = runOneRevDC(t);

    if (ok) {
      dcTimes[dcSuccessCount] = t;
      dcSum += t;
      dcSuccessCount++;

      Serial.print("DC time (ms): ");
      Serial.println(t);
    }

    delay(BETWEEN_TESTS_MS);
    Serial.println();
  }

  // =========================================================
  // Servo trials
  // =========================================================
  Serial.println("----- SERVO TESTS -----");

  for (int i = 0; i < NUM_TRIALS; i++) {
    Serial.print("Servo Trial ");
    Serial.print(i + 1);
    Serial.println("...");

    unsigned long t = 0;
    bool ok = runOneRevServo(t);

    if (ok) {
      servoTimes[servoSuccessCount] = t;
      servoSum += t;
      servoSuccessCount++;

      Serial.print("Servo time (ms): ");
      Serial.println(t);
    }

    delay(BETWEEN_TESTS_MS);
    Serial.println();
  }

  // =========================================================
  // Summary
  // =========================================================
  Serial.println("========================================");
  Serial.println("SUMMARY");
  Serial.println("========================================");

  if (dcSuccessCount > 0) {
    Serial.print("DC average time (ms): ");
    Serial.println((float)dcSum / dcSuccessCount);
  } else {
    Serial.println("DC average time: no valid trials");
  }

  if (servoSuccessCount > 0) {
    Serial.print("Servo average time (ms): ");
    Serial.println((float)servoSum / servoSuccessCount);
  } else {
    Serial.println("Servo average time: no valid trials");
  }

  if (dcSuccessCount > 0 && servoSuccessCount > 0) {
    float dcAvg = (float)dcSum / dcSuccessCount;
    float servoAvg = (float)servoSum / servoSuccessCount;

    Serial.print("Difference (servo - dc) ms: ");
    Serial.println(servoAvg - dcAvg);

    Serial.println();
    Serial.println("Tuning hint:");
    if (servoAvg > dcAvg) {
      Serial.println("- Servo is slower than DC");
      Serial.println("- Make servo command farther from 1500");
      Serial.println("  (example: 1300 -> 1250)");
    } else if (servoAvg < dcAvg) {
      Serial.println("- Servo is faster than DC");
      Serial.println("- Move servo command closer to 1500");
      Serial.println("  (example: 1300 -> 1350)");
    } else {
      Serial.println("- Speeds are very close");
    }
  }

  Serial.println();
  Serial.println("Calibration run complete.");
}

void loop() {
  // nothing
}