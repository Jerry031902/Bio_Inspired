#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(36, 35);  // same SDA=36, SCL=35 as your main code
  Serial.println("I2C Scanner");
}

void loop() {
  byte count = 0;
  Serial.println("Scanning...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  Serial.print("Done. ");
  Serial.print(count);
  Serial.println(" device(s) found.");
  delay(5000);
}