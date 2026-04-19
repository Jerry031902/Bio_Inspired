// ============================================================
// Rear Legs Only Test
// Wait for any serial key, then rotate both rear legs by 1 cycle
// Separate encoder targets for rear left and rear right
// ============================================================

// ---------------- Rear Left / Rear Right DC motors ----------------
// Rear Left
#define MOTOR_A_INA   2
#define MOTOR_A_INB   1
#define MOTOR_A_PWM   3

// Rear Right
#define MOTOR_B_INA   4
#define MOTOR_B_INB   5
#define MOTOR_B_PWM   6

// Rear encoders
#define ENC_A_PIN     9
#define ENC_B_PIN     7

// ---------------- Tuning ----------------
#define REAR_LEFT_COUNTS_PER_CYCLE   700
#define REAR_RIGHT_COUNTS_PER_CYCLE  700

#define REAR_SPEED_MAX               255
#define REAR_SPEED_MIN               60
#define REAR_RAMP_DOWN_THRESHOLD     2000

#define TIMEOUT_MS                   8000
#define STOP_SETTLE_MS               150

volatile long encCountA = 0;   // rear left
volatile long encCountB = 0;   // rear right

// ============================================================
// ISRs
// ============================================================
void IRAM_ATTR encoderA_ISR() {
  encCountA++;
}

void IRAM_ATTR encoderB_ISR() {
  encCountB++;
}

// ============================================================
// Helpers
// ============================================================
uint8_t rampSpeedFromRemaining(long remaining, long threshold, uint8_t minSpeed, uint8_t maxSpeed) {
  if (remaining >= threshold) return maxSpeed;
  if (remaining <= 0) return 0;
  return (uint8_t)map(remaining, 0, threshold, minSpeed, maxSpeed);
}

void waitForAnySerialKey() {
  Serial.println();
  Serial.println("====================================");
  Serial.println("Press any key in Serial Monitor");
  Serial.println("then press Send / Enter to start...");
  Serial.println("====================================");

  while (Serial.available() == 0) {
    delay(10);
  }

  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Start command received.");
}

// ============================================================
// Rear motor helpers
// ============================================================
void rearLeftForward(uint8_t speed) {
  digitalWrite(MOTOR_A_INA, HIGH);
  digitalWrite(MOTOR_A_INB, LOW);
  ledcWrite(MOTOR_A_PWM, speed);
}

void rearRightForward(uint8_t speed) {
  digitalWrite(MOTOR_B_INA, HIGH);
  digitalWrite(MOTOR_B_INB, LOW);
  ledcWrite(MOTOR_B_PWM, speed);
}

void rearLeftStop() {
  digitalWrite(MOTOR_A_INA, HIGH);
  digitalWrite(MOTOR_A_INB, HIGH);
  ledcWrite(MOTOR_A_PWM, 255);   // keep your previous stopping style
}

void rearRightStop() {
  digitalWrite(MOTOR_B_INA, HIGH);
  digitalWrite(MOTOR_B_INB, HIGH);
  ledcWrite(MOTOR_B_PWM, 255);   // keep your previous stopping style
}

void stopRearMotors() {
  rearLeftStop();
  rearRightStop();
}

// ============================================================
// Rotate both rear legs one cycle together
// ============================================================
bool rotateRearBothOneCycle(unsigned long &elapsedMs) {
  encCountA = 0;
  encCountB = 0;

  bool rlDone = false;
  bool rrDone = false;

  unsigned long startTime = millis();

  rearLeftForward(REAR_SPEED_MAX);
  rearRightForward(REAR_SPEED_MAX);

  while (!(rlDone && rrDone)) {
    if (millis() - startTime > TIMEOUT_MS) {
      stopRearMotors();

      Serial.println("TIMEOUT - rear cycle failed");
      Serial.print("Rear Left counts = ");
      Serial.println(encCountA);
      Serial.print("Rear Right counts = ");
      Serial.println(encCountB);
      return false;
    }

    // Rear Left
    if (!rlDone) {
      long remaining = REAR_LEFT_COUNTS_PER_CYCLE - encCountA;
      if (remaining <= 0) {
        rearLeftStop();
        rlDone = true;
      } else {
        uint8_t s = rampSpeedFromRemaining(
          remaining,
          REAR_RAMP_DOWN_THRESHOLD,
          REAR_SPEED_MIN,
          REAR_SPEED_MAX
        );
        ledcWrite(MOTOR_A_PWM, s);
      }
    }

    // Rear Right
    if (!rrDone) {
      long remaining = REAR_RIGHT_COUNTS_PER_CYCLE - encCountB;
      if (remaining <= 0) {
        rearRightStop();
        rrDone = true;
      } else {
        uint8_t s = rampSpeedFromRemaining(
          remaining,
          REAR_RAMP_DOWN_THRESHOLD,
          REAR_SPEED_MIN,
          REAR_SPEED_MAX
        );
        ledcWrite(MOTOR_B_PWM, s);
      }
    }
  }

  stopRearMotors();
  delay(STOP_SETTLE_MS);

  elapsedMs = millis() - startTime;
  return true;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  // Rear motor driver pins first
  pinMode(MOTOR_A_INA, OUTPUT);
  pinMode(MOTOR_A_INB, OUTPUT);
  pinMode(MOTOR_B_INA, OUTPUT);
  pinMode(MOTOR_B_INB, OUTPUT);

  // PWM attach first
  ledcAttach(MOTOR_A_PWM, 20000, 8);
  ledcAttach(MOTOR_B_PWM, 20000, 8);

  // Force active brake ASAP
  stopRearMotors();

  // Serial after outputs are already defined
  Serial.begin(115200);
  delay(100);

  Serial.println("Rear legs only test");
  Serial.println("Rotate both rear legs by 1 cycle");

  Serial.print("Rear Left target counts  = ");
  Serial.println(REAR_LEFT_COUNTS_PER_CYCLE);
  Serial.print("Rear Right target counts = ");
  Serial.println(REAR_RIGHT_COUNTS_PER_CYCLE);

  // Encoder pins
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderB_ISR, CHANGE);

  // Wait for manual start
  waitForAnySerialKey();

  unsigned long tCycle = 0;
  bool ok = rotateRearBothOneCycle(tCycle);

  if (!ok) {
    Serial.println("Rear test failed.");
    return;
  }

  Serial.println("Rear test complete.");
  Serial.print("Time = ");
  Serial.print(tCycle);
  Serial.println(" ms");

  Serial.print("Rear Left counts = ");
  Serial.println(encCountA);

  Serial.print("Rear Right counts = ");
  Serial.println(encCountB);
}

void loop() {
  // nothing
}