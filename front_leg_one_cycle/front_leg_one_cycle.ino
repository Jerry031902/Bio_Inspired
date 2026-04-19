#include <ESP32Servo.h>

// ---------------- Pins ----------------
const int LEFT_HALL_PIN   = 13;
const int RIGHT_HALL_PIN  = 10;

const int LEFT_SERVO_PIN  = 11;
const int RIGHT_SERVO_PIN = 8;

// ---------------- Servo tuning ----------------
const int SERVO_STOP_US = 1500;

// Left servo forward
const int LEFT_SERVO_FWD_US = 2500;

// Right servo forward
const int RIGHT_SERVO_FWD_US = 500;

// ---------------- Timing ----------------
const unsigned long HOME_TIMEOUT_MS = 5000;
const unsigned long REV_TIMEOUT_MS  = 5000;
const unsigned long SETTLE_MS       = 80;

// ---------------- Servo objects ----------------
Servo leftServo;
Servo rightServo;

// ---------------- Hall flags ----------------
volatile bool leftHallTriggered = false;
volatile bool rightHallTriggered = false;

// ---------------- ISRs ----------------
void IRAM_ATTR leftHallISR() {
  leftHallTriggered = true;
}

void IRAM_ATTR rightHallISR() {
  rightHallTriggered = true;
}

// ---------------- Servo helpers ----------------
void servoStop(Servo &s) {
  s.writeMicroseconds(SERVO_STOP_US);
}

void leftServoForward() {
  leftServo.writeMicroseconds(LEFT_SERVO_FWD_US);
}

void rightServoForward() {
  rightServo.writeMicroseconds(RIGHT_SERVO_FWD_US);
}

// ---------------- Wait for Hall edge ----------------
bool waitForHallEdge(volatile bool &hallFlag, unsigned long timeoutMs) {
  hallFlag = false;
  unsigned long start = millis();

  while (!hallFlag) {
    if (millis() - start > timeoutMs) {
      return false;
    }
  }
  return true;
}

// ---------------- One full revolution ----------------
bool rotateOneRevolutionLeft(unsigned long &revTimeMs) {
  Serial.println("Left servo: starting one-rev test");
  Serial.print("Initial hall state = ");
  Serial.println(digitalRead(LEFT_HALL_PIN));

  leftServoForward();

  if (digitalRead(LEFT_HALL_PIN) == LOW) {
    Serial.println("Left servo: magnet starts at sensor, waiting for it to leave...");

    unsigned long leaveStart = millis();
    while (digitalRead(LEFT_HALL_PIN) == LOW) {
      if (millis() - leaveStart > HOME_TIMEOUT_MS) {
        servoStop(leftServo);
        Serial.println("Left servo: timeout waiting for magnet to leave sensor");
        return false;
      }
    }

    Serial.println("Left servo: magnet left sensor");
    delay(20);
  }

  Serial.println("Left servo: waiting for next Hall trigger = one full rotation");

  unsigned long revStart = millis();
  bool gotRev = waitForHallEdge(leftHallTriggered, REV_TIMEOUT_MS);

  servoStop(leftServo);
  delay(SETTLE_MS);

  if (!gotRev) {
    Serial.println("Left servo: timeout waiting for full rotation");
    return false;
  }

  revTimeMs = millis() - revStart;

  Serial.println("Left servo: one revolution detected");
  Serial.print("Left servo: revolution time (ms) = ");
  Serial.println(revTimeMs);

  return true;
}

bool rotateOneRevolutionRight(unsigned long &revTimeMs) {
  Serial.println("Right servo: starting one-rev test");
  Serial.print("Initial hall state = ");
  Serial.println(digitalRead(RIGHT_HALL_PIN));

  rightServoForward();

  if (digitalRead(RIGHT_HALL_PIN) == LOW) {
    Serial.println("Right servo: magnet starts at sensor, waiting for it to leave...");

    unsigned long leaveStart = millis();
    while (digitalRead(RIGHT_HALL_PIN) == LOW) {
      if (millis() - leaveStart > HOME_TIMEOUT_MS) {
        servoStop(rightServo);
        Serial.println("Right servo: timeout waiting for magnet to leave sensor");
        return false;
      }
    }

    Serial.println("Right servo: magnet left sensor");
    delay(20);
  }

  Serial.println("Right servo: waiting for next Hall trigger = one full rotation");

  unsigned long revStart = millis();
  bool gotRev = waitForHallEdge(rightHallTriggered, REV_TIMEOUT_MS);

  servoStop(rightServo);
  delay(SETTLE_MS);

  if (!gotRev) {
    Serial.println("Right servo: timeout waiting for full rotation");
    return false;
  }

  revTimeMs = millis() - revStart;

  Serial.println("Right servo: one revolution detected");
  Serial.print("Right servo: revolution time (ms) = ");
  Serial.println(revTimeMs);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Front legs one-cycle test (L -> R)");

  pinMode(LEFT_HALL_PIN, INPUT_PULLUP);
  pinMode(RIGHT_HALL_PIN, INPUT_PULLUP);

  leftServo.setPeriodHertz(50);
  rightServo.setPeriodHertz(50);

  leftServo.attach(LEFT_SERVO_PIN, 500, 2500);
  rightServo.attach(RIGHT_SERVO_PIN, 500, 2500);

  servoStop(leftServo);
  servoStop(rightServo);

  delay(1000);

  attachInterrupt(digitalPinToInterrupt(LEFT_HALL_PIN), leftHallISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_HALL_PIN), rightHallISR, FALLING);

  Serial.println("Starting in 2 seconds...");
  delay(2000);

  unsigned long leftTime = 0;
  unsigned long rightTime = 0;
  bool success = false;

  Serial.println();
  Serial.println("Cycle 1");

  success = rotateOneRevolutionLeft(leftTime);
  if (!success) {
    Serial.println("Stopping test due to left servo failure/timeout.");
    servoStop(leftServo);
    servoStop(rightServo);
    return;
  }

  success = rotateOneRevolutionRight(rightTime);
  if (!success) {
    Serial.println("Stopping test due to right servo failure/timeout.");
    servoStop(leftServo);
    servoStop(rightServo);
    return;
  }

  servoStop(leftServo);
  servoStop(rightServo);

  Serial.println();
  Serial.println("=== One-cycle alternating test complete ===");
  Serial.print("Left time  = ");
  Serial.println(leftTime);
  Serial.print("Right time = ");
  Serial.println(rightTime);
}

void loop() {
  // nothing
}