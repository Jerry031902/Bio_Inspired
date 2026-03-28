#include <ESP32Servo.h>

// ---------------- Pins ----------------
const int HALL_PIN  = 13;   // Hall sensor digital output
const int SERVO_PIN = 11;   // Continuous servo signal

// ---------------- Servo tuning ----------------
const int SERVO_STOP_US = 1500;
const int SERVO_FWD_US  = 1300;
const int SERVO_REV_US  = 1700;   // not used in this test

// ---------------- Timing ----------------
const unsigned long HOME_TIMEOUT_MS = 5000;
const unsigned long REV_TIMEOUT_MS  = 5000;
const unsigned long SETTLE_MS       = 80;

Servo frontServo;

volatile bool hallTriggered = false;

// Common active-low Hall setup:
// magnet arrives -> output goes LOW
void IRAM_ATTR hallISR() {
  hallTriggered = true;
}

void servoStop() {
  frontServo.writeMicroseconds(SERVO_STOP_US);
}

void servoForward() {
  frontServo.writeMicroseconds(SERVO_FWD_US);
}

void servoReverse() {
  frontServo.writeMicroseconds(SERVO_REV_US);
}

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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Front servo + Hall sensor one-rev test");

  pinMode(HALL_PIN, INPUT_PULLUP);

  frontServo.setPeriodHertz(50);
  frontServo.attach(SERVO_PIN, 500, 2500);
  servoStop();

  delay(1000);

  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  Serial.println("Starting in 2 seconds...");
  delay(2000);

  Serial.print("Initial hall state = ");
  Serial.println(digitalRead(HALL_PIN));

  // Start motion once
  servoForward();

  // If magnet is already at sensor, wait for it to leave first
  if (digitalRead(HALL_PIN) == LOW) {
    Serial.println("Magnet starts at sensor. Waiting for it to leave...");

    unsigned long leaveStart = millis();
    while (digitalRead(HALL_PIN) == LOW) {
      if (millis() - leaveStart > HOME_TIMEOUT_MS) {
        servoStop();
        Serial.println("Timeout waiting for magnet to leave sensor.");
        return;
      }
    }

    Serial.println("Magnet left sensor.");
    delay(20);
  }

  // Now wait for next detection = one full revolution
  Serial.println("Waiting for next Hall trigger = one full rotation...");
  unsigned long revStart = millis();

  bool gotRev = waitForHallEdge(REV_TIMEOUT_MS);

  servoStop();
  delay(SETTLE_MS);

  if (!gotRev) {
    Serial.println("Timeout: no Hall trigger detected.");
    return;
  }

  unsigned long revTime = millis() - revStart;

  Serial.println("One revolution detected.");
  Serial.print("Revolution time (ms): ");
  Serial.println(revTime);

  Serial.println("Test complete.");
}

void loop() {
  // nothing
}