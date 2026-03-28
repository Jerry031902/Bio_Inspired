#define ENC_A_PIN 7
#define ENC_B_PIN 9

void setup() {
    Serial.begin(115200);
    pinMode(ENC_A_PIN, INPUT);
    pinMode(ENC_B_PIN, INPUT);
    Serial.println("Spin motor shafts by hand and watch values change.");
}

void loop() {
    Serial.print("A raw: ");
    Serial.print(digitalRead(ENC_A_PIN));
    Serial.print(" | B raw: ");
    Serial.println(digitalRead(ENC_B_PIN));
    delay(100);
}