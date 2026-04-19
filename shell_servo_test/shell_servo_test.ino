#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 8;

// Tune these if needed
const int SERVO_STOP_US = 1500;
const int SERVO_FWD_US  = 600;
const int SERVO_REV_US  = 2400;

void setup() {
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2500);

  // Stop first
  myServo.writeMicroseconds(SERVO_STOP_US);
  delay(2000);

  // Rotate forward
  myServo.writeMicroseconds(SERVO_FWD_US);
  delay(1250);
  myServo.writeMicroseconds(SERVO_STOP_US);
  // Wiggle back and forth for 10 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 30000) 
  {
    // small reverse pulse
    myServo.writeMicroseconds(SERVO_REV_US);
    delay(60);
    myServo.writeMicroseconds(SERVO_STOP_US);
    delay(60);
    // small forward pulse
    myServo.writeMicroseconds(SERVO_FWD_US);
    delay(60);
    myServo.writeMicroseconds(SERVO_STOP_US);
    delay(60);
  }

  // Final stop
  myServo.writeMicroseconds(SERVO_STOP_US);
}

void loop() {
}