// ============================================================
// STAIR Robot — Dual DC Motor Encoder
// Rear Left (Motor A) and Rear Right (Motor B)
// Tests: encoder counting, single-revolution stop, sequential gait
// ============================================================

// ----- Motor A (Rear Left) — VNH5019 control pins -----
#define MOTOR_A_INA   1
#define MOTOR_A_INB   2
#define MOTOR_A_PWM   3

// ----- Motor B (Rear Right) — VNH5019 control pins -----
#define MOTOR_B_INA   4
#define MOTOR_B_INB   5
#define MOTOR_B_PWM   6

// ----- Encoder pins (through voltage dividers) -----
// Only using Channel A for revolution counting
// Add Channel B later if need direction detection
#define ENC_A_PIN     7   // Rear Left encoder, Yellow wire
#define ENC_B_PIN     9   // Rear Right encoder, Yellow wire

// ----- Constants -----
#define COUNTS_PER_REV  4650  // Tuned count target for one full leg cycle
#define MOTOR_SPEED     220   // PWM duty cycle (0-255), tune as needed
#define RAMP_DOWN_THRESHOLD 2000  // Start slowing down this many counts before target
#define TIMEOUT_MS      5000  // Safety timeout in milliseconds

// ----- Encoder counters (volatile because modified in ISR) -----
volatile long encCountA = 0;
volatile long encCountB = 0;

// ----- ISRs -----
void IRAM_ATTR encoderA_ISR()
{
    encCountA++;
}

void IRAM_ATTR encoderB_ISR()
{
    encCountB++;
}

// ============================================================
// Motor control helper functions
// ============================================================

void motorForwardLeft(uint8_t speed)
{
    // Rear Left forward is reversed relative to Rear Right
    digitalWrite(MOTOR_A_INA, LOW);
    digitalWrite(MOTOR_A_INB, HIGH);
    ledcWrite(MOTOR_A_PWM, speed);
}

void motorForwardRight(uint8_t speed)
{
    // Rear Right forward direction
    digitalWrite(MOTOR_B_INA, HIGH);
    digitalWrite(MOTOR_B_INB, LOW);
    ledcWrite(MOTOR_B_PWM, speed);
}

void motorBrake(int inaPin, int inbPin, int pwmPin)
{
    // Active brake: both LOW + PWM 255
    digitalWrite(inaPin, LOW);
    digitalWrite(inbPin, LOW);
    ledcWrite(pwmPin, 255);
}

// ============================================================
// Rotate exactly one revolution with ramp-down
// Returns true if completed, false if timed out
// ============================================================

bool rotateOneRevolutionLeft(volatile long &encCount)
{
    encCount = 0;
    unsigned long timeout = millis() + TIMEOUT_MS;

    // Start at full speed
    motorForwardLeft(MOTOR_SPEED);

    // Wait for revolution, ramp down near the end
    while (encCount < COUNTS_PER_REV)
    {
        // Safety timeout
        if (millis() > timeout) {
            motorBrake(MOTOR_A_INA, MOTOR_A_INB, MOTOR_A_PWM);
            Serial.println("TIMEOUT - left motor stalled or encoder not reading!");
            Serial.print("  Counts reached: ");
            Serial.println(encCount);
            return false;
        }

        long remaining = COUNTS_PER_REV - encCount;

        // Ramp down speed as we approach target to reduce overshoot
        if (remaining < RAMP_DOWN_THRESHOLD) {
            // Map remaining counts to speed: closer to target = slower
            // Minimum speed of 60 to avoid stalling
            uint8_t rampSpeed = map(remaining, 0, RAMP_DOWN_THRESHOLD, 60, MOTOR_SPEED);
            ledcWrite(MOTOR_A_PWM, rampSpeed);
        }
    }

    // Brake immediately
    motorBrake(MOTOR_A_INA, MOTOR_A_INB, MOTOR_A_PWM);
    return true;
}

bool rotateOneRevolutionRight(volatile long &encCount)
{
    encCount = 0;
    unsigned long timeout = millis() + TIMEOUT_MS;

    // Start at full speed
    motorForwardRight(MOTOR_SPEED);

    // Wait for revolution, ramp down near the end
    while (encCount < COUNTS_PER_REV)
    {
        // Safety timeout
        if (millis() > timeout) {
            motorBrake(MOTOR_B_INA, MOTOR_B_INB, MOTOR_B_PWM);
            Serial.println("TIMEOUT - right motor stalled or encoder not reading!");
            Serial.print("  Counts reached: ");
            Serial.println(encCount);
            return false;
        }

        long remaining = COUNTS_PER_REV - encCount;

        // Ramp down speed as we approach target to reduce overshoot
        if (remaining < RAMP_DOWN_THRESHOLD) {
            // Map remaining counts to speed: closer to target = slower
            // Minimum speed of 60 to avoid stalling
            uint8_t rampSpeed = map(remaining, 0, RAMP_DOWN_THRESHOLD, 60, MOTOR_SPEED);
            ledcWrite(MOTOR_B_PWM, rampSpeed);
        }
    }

    // Brake immediately
    motorBrake(MOTOR_B_INA, MOTOR_B_INB, MOTOR_B_PWM);
    return true;
}

// ============================================================
// Setup
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("STAIR DC Motor Encoder Test");

    // ----- Motor pin setup -----
    pinMode(MOTOR_A_INA, OUTPUT);
    pinMode(MOTOR_A_INB, OUTPUT);
    pinMode(MOTOR_B_INA, OUTPUT);
    pinMode(MOTOR_B_INB, OUTPUT);

    // LEDC PWM setup (ESP32-S3 Arduino 3.x API)
    ledcAttach(MOTOR_A_PWM, 20000, 8);
    ledcAttach(MOTOR_B_PWM, 20000, 8);

    // ----- Encoder pin setup -----
    pinMode(ENC_A_PIN, INPUT);
    pinMode(ENC_B_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderB_ISR, CHANGE);

    // Ensure motors are stopped
    motorBrake(MOTOR_A_INA, MOTOR_A_INB, MOTOR_A_PWM);
    motorBrake(MOTOR_B_INA, MOTOR_B_INB, MOTOR_B_PWM);

    Serial.println("Power on motor supply, then type any key to start...");
    while (!Serial.available()) {
        // Wait for user input
    }
    Serial.read();
    delay(3000);

    // =========================================================
    // TEST 1: Single motor, single revolution
    // =========================================================
    Serial.println("\n--- TEST 1: Rear Left, one revolution ---");

    encCountA = 0;
    unsigned long startTime = millis();
    bool success = rotateOneRevolutionLeft(encCountA);
    unsigned long elapsed = millis() - startTime;

    if (success) {
        Serial.print("Rear Left done. Counts: ");
        Serial.print(encCountA);
        Serial.print(" | Time: ");
        Serial.print(elapsed);
        Serial.println(" ms");
        Serial.print("Overshoot: ");
        Serial.print(encCountA - COUNTS_PER_REV);
        Serial.println(" counts");
    }

    delay(2000);

    // =========================================================
    // TEST 2: Other motor, single revolution
    // =========================================================
    Serial.println("\n--- TEST 2: Rear Right, one revolution ---");

    encCountB = 0;
    startTime = millis();
    success = rotateOneRevolutionRight(encCountB);
    elapsed = millis() - startTime;

    if (success) {
        Serial.print("Rear Right done. Counts: ");
        Serial.print(encCountB);
        Serial.print(" | Time: ");
        Serial.print(elapsed);
        Serial.println(" ms");
        Serial.print("Overshoot: ");
        Serial.print(encCountB - COUNTS_PER_REV);
        Serial.println(" counts");
    }

    delay(2000);

    // =========================================================
    // TEST 3: Sequential gait — RL then RR (simulating walk)
    // =========================================================
    Serial.println("\n--- TEST 3: Sequential gait (RL -> RR -> RL -> RR) ---");
    Serial.println("This simulates the rear legs in walking order.");
    Serial.println("Starting in 2 seconds...\n");
    delay(2000);

    for (int cycle = 0; cycle < 4; cycle++) {
        if (cycle % 2 == 0) {
            // Rear Left
            Serial.print("Cycle ");
            Serial.print(cycle + 1);
            Serial.print(": Rear Left... ");

            encCountA = 0;
            startTime = millis();
            success = rotateOneRevolutionLeft(encCountA);
            elapsed = millis() - startTime;

            if (success) {
                Serial.print("Done (");
                Serial.print(encCountA);
                Serial.print(" counts, ");
                Serial.print(elapsed);
                Serial.println(" ms)");
            }
        } else {
            // Rear Right
            Serial.print("Cycle ");
            Serial.print(cycle + 1);
            Serial.print(": Rear Right... ");

            encCountB = 0;
            startTime = millis();
            success = rotateOneRevolutionRight(encCountB);
            elapsed = millis() - startTime;

            if (success) {
                Serial.print("Done (");
                Serial.print(encCountB);
                Serial.print(" counts, ");
                Serial.print(elapsed);
                Serial.println(" ms)");
            }
        }

        delay(500);  // Small pause between legs for visual clarity
    }

    Serial.println("\n=== All tests complete ===");
    Serial.println("\nNotes:");
    Serial.println("- If overshoot is large, reduce MOTOR_SPEED or increase RAMP_DOWN_THRESHOLD");
    Serial.println("- If motor stalls during ramp-down, increase minimum speed (60) in rotateOneRevolutionLeft/Right()");
    Serial.println("- Record the 'Time' values — you'll need these for servo calibration later");
}

void loop()
{
    // Nothing here — tests run once in setup()
}
