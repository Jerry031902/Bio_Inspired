#define INA_PIN  1
#define INB_PIN  2
#define PWM_PIN  3

void setup() {
  Serial.begin(115200);
  pinMode(INA_PIN, OUTPUT);
  pinMode(INB_PIN, OUTPUT);
  ledcAttach(PWM_PIN, 20000, 8);

  Serial.println("Motor test starting...");
}

void loop() {
  // Forward half speed
  Serial.println("Forward");
  digitalWrite(INA_PIN, HIGH);
  digitalWrite(INB_PIN, LOW);
  ledcWrite(PWM_PIN, 128);
  delay(3000);

  // Stop
  Serial.println("Stop");
  digitalWrite(INA_PIN, LOW);
  digitalWrite(INB_PIN, LOW);
  ledcWrite(PWM_PIN, 0);
  delay(2000);

  // Reverse half speed
  Serial.println("Reverse");
  digitalWrite(INA_PIN, LOW);
  digitalWrite(INB_PIN, HIGH);
  ledcWrite(PWM_PIN, 128);
  delay(3000);

  // Stop
  Serial.println("Stop");
  digitalWrite(INA_PIN, LOW);
  digitalWrite(INB_PIN, LOW);
  ledcWrite(PWM_PIN, 0);
  delay(2000);
}