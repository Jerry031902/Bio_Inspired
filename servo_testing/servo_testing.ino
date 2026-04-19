#include <ESP32Servo.h>

// ---------------- Pins ----------------
const int SERVO_PIN = 11;

// ---------------- Servo tuning ----------------
// For continuous rotation servo:
// 1500 = stop
// smaller than 1500 = one direction
// larger than 1500 = other direction
const int SERVO_STOP_US = 1500;
const int SERVO_FWD_US  = 1200;
const int SERVO_REV_US  = 1800;

Servo testServo;

void servoStop() {
  testServo.writeMicroseconds(SERVO_STOP_US);
}

void servoForward() {
  testServo.writeMicroseconds(SERVO_FWD_US);
}

void servoReverse() {
  testServo.writeMicroseconds(SERVO_REV_US);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Continuous servo only test");

  testServo.setPeriodHertz(50);
  testServo.attach(SERVO_PIN, 500, 2500);

  Serial.println("Stop");
  servoStop();
  delay(2000);

  Serial.println("Forward for 3 seconds");
  servoForward();
  delay(3000);

  Serial.println("Stop");
  servoStop();
  delay(2000);

  Serial.println("Reverse for 3 seconds");
  servoReverse();
  delay(3000);

  Serial.println("Stop");
  servoStop();

  Serial.println("Test complete");
}

void loop() {
  // nothing
}