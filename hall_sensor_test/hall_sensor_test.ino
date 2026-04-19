#define HALL_PIN 13

volatile bool triggered = false;
volatile unsigned long triggerCount = 0;

void IRAM_ATTR hallISR() {
    triggered = true;
    triggerCount++;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Hall Sensor Test — spin magnet past sensor");
    
    pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
}

void loop() {
    Serial.print("Pin state: ");
    Serial.print(digitalRead(HALL_PIN));
    Serial.print(" | Triggers: ");
    Serial.println(triggerCount);
    
    if (triggered) {
        Serial.println(">>> MAGNET DETECTED <<<");
        triggered = false;
    }
    
    delay(200);
}