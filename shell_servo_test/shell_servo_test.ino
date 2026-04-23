#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 8;
const int SERVO_STOP_US = 1500;
const int SERVO_FWD_US  = 600;
const int SERVO_REV_US  = 2400;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Servo test start ===");

  myServo.setPeriodHertz(50);
  bool ok = myServo.attach(SERVO_PIN, 500, 2500);
  Serial.print("attach() returned: ");
  Serial.println(ok);
  Serial.print("attached() status: ");
  Serial.println(myServo.attached());

  Serial.println("Writing STOP (1500us)");
  myServo.writeMicroseconds(SERVO_STOP_US);
  delay(2000);

  Serial.println("Writing FORWARD (600us)");
  myServo.writeMicroseconds(SERVO_FWD_US);
  delay(1500);

  Serial.println("Writing STOP (1500us)");
  myServo.writeMicroseconds(SERVO_STOP_US);
  delay(5000);
  myServo.writeMicroseconds(SERVO_REV_US);
  delay(1340);
  myServo.writeMicroseconds(SERVO_STOP_US);
  Serial.println("=== Servo test end ===");
}

void loop() {}