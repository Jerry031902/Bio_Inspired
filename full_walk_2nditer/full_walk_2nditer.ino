// ============================================================
// STAIR Robot — Diagonal Pair Walking Test (A A B B)
// Keep everything else the same as your pasted code:
// - same pins
// - same speed tuning
// - same stop/brake behavior
// - no delay between cycles
//
// Pattern:
//   Cycle 1: RL + FR
//   Cycle 2: RL + FR
//   Cycle 3: RR + FL
//   Cycle 4: RR + FL
//   Then repeat...
// ============================================================

// ---------------- Rear motor pins ----------------
#define MOTOR_A_INA   2
#define MOTOR_A_INB   1
#define MOTOR_A_PWM   3

#define MOTOR_B_INA   4
#define MOTOR_B_INB   5
#define MOTOR_B_PWM   6

// ---------------- Rear encoder pins ----------------
#define ENC_A_PIN     9   // rear left
#define ENC_B_PIN     7   // rear right

// ---------------- Front motor pins ----------------
#define FL_IN1        42
#define FL_IN2        41

#define FR_IN3        40
#define FR_IN4        39

// ---------------- Front encoder pins ----------------
#define ENC_FL_PIN    12
#define ENC_FR_PIN    13

// ---------------- Change this as needed ----------------
const int NUM_WALK_CYCLES = 30;

// ---------------- Counts ----------------
const long REAR_LEFT_COUNTS_PER_CYCLE   = 700;
const long REAR_RIGHT_COUNTS_PER_CYCLE  = 700;
const long FRONT_LEFT_COUNTS_PER_CYCLE  = 537;
const long FRONT_RIGHT_COUNTS_PER_CYCLE = 550;

// ---------------- Speed tuning ----------------
#define REAR_SPEED_MAX             255
#define REAR_SPEED_MIN             60
#define REAR_RAMP_DOWN_THRESHOLD   220

#define FRONT_SPEED_MAX            230
#define FRONT_SPEED_MIN            60
#define FRONT_RAMP_DOWN_THRESHOLD  160

#define TIMEOUT_MS                 8000

volatile long encCountA  = 0;   // rear left
volatile long encCountB  = 0;   // rear right
volatile long encCountFL = 0;   // front left
volatile long encCountFR = 0;   // front right

// ============================================================
// ISRs
// ============================================================
void IRAM_ATTR encoderA_ISR() {
  encCountA++;
}

void IRAM_ATTR encoderB_ISR() {
  encCountB++;
}

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

// ============================================================
// Rear motor control
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

// Keep your old rear stop behavior
void rearLeftStop() {
  digitalWrite(MOTOR_A_INA, HIGH);
  digitalWrite(MOTOR_A_INB, HIGH);
  ledcWrite(MOTOR_A_PWM, 255);
}

void rearRightStop() {
  digitalWrite(MOTOR_B_INA, HIGH);
  digitalWrite(MOTOR_B_INB, HIGH);
  ledcWrite(MOTOR_B_PWM, 255);
}

// ============================================================
// Front motor control
// ============================================================

// Left front forward: PWM on 42, 41 LOW
void frontLeftForward(uint8_t speed) {
  ledcWrite(FL_IN1, speed);
  digitalWrite(FL_IN2, LOW);
}

// Right front forward: PWM on 40, 39 LOW
void frontRightForward(uint8_t speed) {
  ledcWrite(FR_IN3, speed);
  digitalWrite(FR_IN4, LOW);
}

// Front active brake: both HIGH
void frontLeftBrake() {
  ledcWrite(FL_IN1, 255);
  digitalWrite(FL_IN2, HIGH);
}

void frontRightBrake() {
  ledcWrite(FR_IN3, 255);
  digitalWrite(FR_IN4, HIGH);
}

void frontLeftRelease() {
  ledcWrite(FL_IN1, 0);
  digitalWrite(FL_IN2, LOW);
}

void frontRightRelease() {
  ledcWrite(FR_IN3, 0);
  digitalWrite(FR_IN4, LOW);
}

// ============================================================
// Global helpers
// ============================================================
void stopRearMotors() {
  rearLeftStop();
  rearRightStop();
}

void brakeFrontMotors() {
  frontLeftBrake();
  frontRightBrake();
}

void releaseFrontMotors() {
  frontLeftRelease();
  frontRightRelease();
}

void holdAllStopped() {
  stopRearMotors();
  brakeFrontMotors();
}

// ============================================================
// One diagonal-pair cycle
//
// pairA = true  -> RL + FR
// pairA = false -> RR + FL
// ============================================================
bool walkOneDiagonalCycle(bool pairA, unsigned long &elapsedMs) {
  encCountA  = 0;
  encCountB  = 0;
  encCountFL = 0;
  encCountFR = 0;

  bool rlActive = pairA;
  bool frActive = pairA;
  bool rrActive = !pairA;
  bool flActive = !pairA;

  bool rlDone = !rlActive;
  bool rrDone = !rrActive;
  bool flDone = !flActive;
  bool frDone = !frActive;

  unsigned long startTime = millis();

  // Hold inactive pair stopped/braked
  if (!rlActive) rearLeftStop();
  if (!rrActive) rearRightStop();
  if (!flActive) frontLeftBrake();
  if (!frActive) frontRightBrake();

  // Start active pair
  if (rlActive) rearLeftForward(REAR_SPEED_MAX);
  if (rrActive) rearRightForward(REAR_SPEED_MAX);
  if (flActive) frontLeftForward(FRONT_SPEED_MAX);
  if (frActive) frontRightForward(FRONT_SPEED_MAX);

  while (!(rlDone && rrDone && flDone && frDone)) {
    if (millis() - startTime > TIMEOUT_MS) {
      holdAllStopped();

      Serial.println("TIMEOUT - one or more motors did not finish cycle");
      Serial.print("RL counts = "); Serial.println(encCountA);
      Serial.print("RR counts = "); Serial.println(encCountB);
      Serial.print("FL counts = "); Serial.println(encCountFL);
      Serial.print("FR counts = "); Serial.println(encCountFR);
      return false;
    }

    // ------------------------------------------------
    // KEEP BRAKE REASSERTED EVERY LOOP
    // inactive legs
    if (!rlActive) rearLeftStop();
    if (!rrActive) rearRightStop();
    if (!flActive) frontLeftBrake();
    if (!frActive) frontRightBrake();

    // finished legs
    if (rlDone) rearLeftStop();
    if (rrDone) rearRightStop();
    if (flDone) frontLeftBrake();
    if (frDone) frontRightBrake();
    // ------------------------------------------------

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

  elapsedMs = millis() - startTime;
  return true;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("STAIR - diagonal pair walking (A A B B)");
  Serial.println("Cycle 1: RL + FR");
  Serial.println("Cycle 2: RL + FR");
  Serial.println("Cycle 3: RR + FL");
  Serial.println("Cycle 4: RR + FL");
  Serial.println("Pattern repeats with no delay between cycles");

  Serial.print("NUM_WALK_CYCLES = ");
  Serial.println(NUM_WALK_CYCLES);

  Serial.print("Rear left counts/cycle   = ");
  Serial.println(REAR_LEFT_COUNTS_PER_CYCLE);

  Serial.print("Rear right counts/cycle  = ");
  Serial.println(REAR_RIGHT_COUNTS_PER_CYCLE);

  Serial.print("Front left counts/cycle  = ");
  Serial.println(FRONT_LEFT_COUNTS_PER_CYCLE);

  Serial.print("Front right counts/cycle = ");
  Serial.println(FRONT_RIGHT_COUNTS_PER_CYCLE);

  // Rear motor pins
  pinMode(MOTOR_A_INA, OUTPUT);
  pinMode(MOTOR_A_INB, OUTPUT);
  pinMode(MOTOR_B_INA, OUTPUT);
  pinMode(MOTOR_B_INB, OUTPUT);

  // Front motor pins
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN3, OUTPUT);
  pinMode(FR_IN4, OUTPUT);

  // PWM attach
  ledcAttach(MOTOR_A_PWM, 20000, 8);
  ledcAttach(MOTOR_B_PWM, 20000, 8);
  ledcAttach(FL_IN1, 20000, 8);
  ledcAttach(FR_IN3, 20000, 8);

  // Encoder pins
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  pinMode(ENC_FL_PIN, INPUT);
  pinMode(ENC_FR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN),  encoderA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN),  encoderB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), encoderFL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), encoderFR_ISR, CHANGE);

  // Safe initial state
  stopRearMotors();
  releaseFrontMotors();

  Serial.println("Auto starting in 5 seconds...");
  delay(5000);

  for (int cycle = 1; cycle <= NUM_WALK_CYCLES; cycle++) {
    unsigned long tCycle = 0;

    // AABB pattern:
    // cycles 1,2 => pair A
    // cycles 3,4 => pair B
    // repeat
    bool pairA = (((cycle - 1) / 2) % 2 == 0);

    Serial.println();
    Serial.print("=== WALK CYCLE ");
    Serial.print(cycle);
    Serial.print(" START: ");
    Serial.println(pairA ? "RL + FR" : "RR + FL");

    bool ok = walkOneDiagonalCycle(pairA, tCycle);

    if (!ok) {
      Serial.println("Cycle failed. Stopping with brakes held.");
      holdAllStopped();
      return;
    }

    Serial.print("Cycle ");
    Serial.print(cycle);
    Serial.println(" complete.");
    Serial.print("Time = ");
    Serial.print(tCycle);
    Serial.println(" ms");

    Serial.print("RL counts = "); Serial.println(encCountA);
    Serial.print("RR counts = "); Serial.println(encCountB);
    Serial.print("FL counts = "); Serial.println(encCountFL);
    Serial.print("FR counts = "); Serial.println(encCountFR);
  }

  holdAllStopped();

  Serial.println();
  Serial.println("All requested walk cycles complete. Holding brake.");
}

void loop() {
  // nothing
}