// ============================================================
// Front Legs Only Test (separate encoder counts)
// Wait for any serial key, then rotate both front legs by 1 cycle
//
// Left motor forward:
//   PWM on GPIO42, GPIO41 LOW
//
// Right motor forward:
//   PWM on GPIO39, GPIO40 LOW
//
// Active brake:
//   both inputs HIGH for a short time, then release
// ============================================================

// ---------------- Front motor driver pins ----------------
#define FL_IN1   42
#define FL_IN2   41

#define FR_IN3   40
#define FR_IN4   39

// ---------------- Front encoder pins ----------------
#define ENC_FL_PIN   12
#define ENC_FR_PIN   13

// ---------------- Tuning ----------------
#define FRONT_LEFT_COUNTS_PER_CYCLE   530
#define FRONT_RIGHT_COUNTS_PER_CYCLE  542

#define FRONT_SPEED_MAX               255
#define FRONT_SPEED_MIN               60
#define FRONT_RAMP_DOWN_THRESHOLD     1700

#define TIMEOUT_MS                    8000
#define BRAKE_HOLD_MS                 200

volatile long encCountFL = 0;
volatile long encCountFR = 0;

// ============================================================
// ISRs
// ============================================================
void IRAM_ATTR encoderFL_ISR() {
  encCountFL++;
}

void IRAM_ATTR encoderFR_ISR() {
  encCountFR++;
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
  Serial.println("then press Send / Enter to start.");
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
// Front motor control
// ============================================================

// Left motor forward:
//   IN1 = PWM
//   IN2 = LOW
void frontLeftForward(uint8_t speed) {
  ledcWrite(FL_IN1, speed);
  digitalWrite(FL_IN2, LOW);
}

// Right motor forward:
//   IN3 = PWM
//   IN4 = LOW
void frontRightForward(uint8_t speed) {
  ledcWrite(FR_IN3, speed);
  digitalWrite(FR_IN4, LOW);
}

// Active brake: both inputs HIGH
void frontLeftBrake() {
  ledcWrite(FL_IN1, 255);
  digitalWrite(FL_IN2, HIGH);
}

void frontRightBrake() {
  ledcWrite(FR_IN3, 255);
  digitalWrite(FR_IN4, HIGH);
}

// Release outputs after braking
void frontLeftRelease() {
  ledcWrite(FL_IN1, 0);
  digitalWrite(FL_IN2, LOW);
}

void frontRightRelease() {
  ledcWrite(FR_IN3, 0);
  digitalWrite(FR_IN4, LOW);
}

void brakeFrontMotors() {
  frontLeftBrake();
  frontRightBrake();
}

void releaseFrontMotors() {
  frontLeftRelease();
  frontRightRelease();
}

// ============================================================
// Rotate both front legs one cycle together
// ============================================================
bool rotateFrontBothOneCycle(unsigned long &elapsedMs) {
  encCountFL = 0;
  encCountFR = 0;

  bool flDone = false;
  bool frDone = false;

  unsigned long startTime = millis();

  frontLeftForward(FRONT_SPEED_MAX);
  frontRightForward(FRONT_SPEED_MAX);

  while (!(flDone && frDone)) {
    if (millis() - startTime > TIMEOUT_MS) {
      brakeFrontMotors();
      delay(BRAKE_HOLD_MS);
      releaseFrontMotors();

      Serial.println("TIMEOUT - front cycle failed");
      Serial.print("Front Left counts = ");
      Serial.println(encCountFL);
      Serial.print("Front Right counts = ");
      Serial.println(encCountFR);
      return false;
    }

    // Front Left
    if (!flDone) {
      long remaining = FRONT_LEFT_COUNTS_PER_CYCLE - encCountFL;
      if (remaining <= 0) {
        frontLeftBrake();
        flDone = true;
      } else {
        uint8_t s = rampSpeedFromRemaining(
          remaining,
          FRONT_RAMP_DOWN_THRESHOLD,
          FRONT_SPEED_MIN,
          FRONT_SPEED_MAX
        );
        ledcWrite(FL_IN1, s);
      }
    }

    // Front Right
    if (!frDone) {
      long remaining = FRONT_RIGHT_COUNTS_PER_CYCLE - encCountFR;
      if (remaining <= 0) {
        frontRightBrake();
        frDone = true;
      } else {
        uint8_t s = rampSpeedFromRemaining(
          remaining,
          FRONT_RAMP_DOWN_THRESHOLD,
          FRONT_SPEED_MIN,
          FRONT_SPEED_MAX
        );
        ledcWrite(FR_IN3, s);
      }
    }
  }

  delay(BRAKE_HOLD_MS);
  releaseFrontMotors();

  elapsedMs = millis() - startTime;
  return true;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Front legs only test (separate encoder counts)");
  Serial.println("Rotate both front legs by 1 cycle");

  Serial.print("Front Left target counts  = ");
  Serial.println(FRONT_LEFT_COUNTS_PER_CYCLE);
  Serial.print("Front Right target counts = ");
  Serial.println(FRONT_RIGHT_COUNTS_PER_CYCLE);

  // Motor driver pins
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT);
  pinMode(FR_IN4, OUTPUT);

  // Attach PWM to the pins actually used for forward PWM
  ledcAttach(FL_IN1, 20000, 8);
  ledcAttach(FR_IN3, 20000, 8);

  // Encoders
  pinMode(ENC_FL_PIN, INPUT);
  pinMode(ENC_FR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), encoderFL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), encoderFR_ISR, CHANGE);

  releaseFrontMotors();

  waitForAnySerialKey();

  unsigned long tCycle = 0;
  bool ok = rotateFrontBothOneCycle(tCycle);

  if (!ok) {
    Serial.println("Front test failed.");
    return;
  }

  Serial.println("Front test complete.");
  Serial.print("Time = ");
  Serial.print(tCycle);
  Serial.println(" ms");

  Serial.print("Front Left counts = ");
  Serial.println(encCountFL);

  Serial.print("Front Right counts = ");
  Serial.println(encCountFR);
}

void loop() {
  // nothing
}