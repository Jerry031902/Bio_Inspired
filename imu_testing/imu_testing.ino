#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

const int SDA_PIN = 36;
const int SCL_PIN = 35;

float angle_offset = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("I2C + MPU6050 test");
  Serial.println("Expected MPU address: 0x68");

  mpu6050.begin();

  Serial.println("Calibrating gyro... keep sensor still");
  mpu6050.calcGyroOffsets(true);

  Serial.println("Settling...");
  for (int i = 0; i < 100; i++) {
    mpu6050.update();
    delay(10);
  }

  angle_offset = mpu6050.getAngleX();
  Serial.print("AngleX zero offset = ");
  Serial.println(angle_offset, 4);
}

void loop() {
  mpu6050.update();

  float angleX = mpu6050.getAngleX();
  float angleY = mpu6050.getAngleY();
  float angleZ = mpu6050.getAngleZ();

  float gyroX = mpu6050.getGyroX();
  float gyroY = mpu6050.getGyroY();
  float gyroZ = mpu6050.getGyroZ();

  float pitch = angleX - angle_offset;

  Serial.print("Pitch=");
  Serial.print(pitch, 2);
  Serial.print(" deg");

  Serial.print(" | AngleX=");
  Serial.print(angleX, 2);
  Serial.print("  AngleY=");
  Serial.print(angleY, 2);
  Serial.print("  AngleZ=");
  Serial.print(angleZ, 2);

  Serial.print(" | GyroX=");
  Serial.print(gyroX, 2);
  Serial.print("  GyroY=");
  Serial.print(gyroY, 2);
  Serial.print("  GyroZ=");
  Serial.println(gyroZ, 2);

  delay(100);
}