// ============================================================
// STAIR Robot — Dual DC Motor Encoder
// Rear Left (Motor A) and Rear Right (Motor B)
// Repeated walking cycle test: RL -> RR
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
#define ENC_A_PIN     7   // Rear Left encoder
#define ENC_B_PIN     9   // Rear Right encoder

// ----- Constants -----
#define COUNTS_PER_REV      4640
#define MOTOR_SPEED         235
#define RAMP_DOWN_THRESHOLD 2000
#define TIMEOUT_MS          5000
#define NUM_CYCLES          2   // change this to however many RL->RR cycles you want

// ----- Encoder counters -----
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
    // Rear Left forward
    digitalWrite(MOTOR_A_INA, HIGH);
    digitalWrite(MOTOR_A_INB, LOW);
    ledcWrite(MOTOR_A_PWM, speed);
}

void motorForwardRight(uint8_t speed)
{
    // Rear Right forward
    digitalWrite(MOTOR_B_INA, HIGH);
    digitalWrite(MOTOR_B_INB, LOW);
    ledcWrite(MOTOR_B_PWM, speed);
}

void motorBrake(int inaPin, int inbPin, int pwmPin)
{
    // Active brake
    digitalWrite(inaPin, LOW);
    digitalWrite(inbPin, LOW);
    ledcWrite(pwmPin, 255);
}

// ============================================================
// Rotate exactly one revolution
// ============================================================

bool rotateOneRevolutionLeft(volatile long &encCount)
{
    encCount = 0;
    unsigned long timeout = millis() + TIMEOUT_MS;

    motorForwardLeft(MOTOR_SPEED);

    while (encCount < COUNTS_PER_REV)
    {
        if (millis() > timeout) {
            motorBrake(MOTOR_A_INA, MOTOR_A_INB, MOTOR_A_PWM);
            Serial.println("TIMEOUT - left motor stalled or encoder not reading!");
            Serial.print("  Counts reached: ");
            Serial.println(encCount);
            return false;
        }

        long remaining = COUNTS_PER_REV - encCount;

        if (RAMP_DOWN_THRESHOLD > 0 && remaining < RAMP_DOWN_THRESHOLD) {
            uint8_t rampSpeed = map(remaining, 0, RAMP_DOWN_THRESHOLD, 60, MOTOR_SPEED);
            ledcWrite(MOTOR_A_PWM, rampSpeed);
        }
    }

    motorBrake(MOTOR_A_INA, MOTOR_A_INB, MOTOR_A_PWM);
    return true;
}

bool rotateOneRevolutionRight(volatile long &encCount)
{
    encCount = 0;
    unsigned long timeout = millis() + TIMEOUT_MS;

    motorForwardRight(MOTOR_SPEED);

    while (encCount < COUNTS_PER_REV)
    {
        if (millis() > timeout) {
            motorBrake(MOTOR_B_INA, MOTOR_B_INB, MOTOR_B_PWM);
            Serial.println("TIMEOUT - right motor stalled or encoder not reading!");
            Serial.print("  Counts reached: ");
            Serial.println(encCount);
            return false;
        }

        long remaining = COUNTS_PER_REV - encCount;

        if (RAMP_DOWN_THRESHOLD > 0 && remaining < RAMP_DOWN_THRESHOLD) {
            uint8_t rampSpeed = map(remaining, 0, RAMP_DOWN_THRESHOLD, 60, MOTOR_SPEED);
            ledcWrite(MOTOR_B_PWM, rampSpeed);
        }
    }

    motorBrake(MOTOR_B_INA, MOTOR_B_INB, MOTOR_B_PWM);
    return true;
}

// ============================================================
// Setup
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("STAIR DC Motor Walking Test");

    pinMode(MOTOR_A_INA, OUTPUT);
    pinMode(MOTOR_A_INB, OUTPUT);
    pinMode(MOTOR_B_INA, OUTPUT);
    pinMode(MOTOR_B_INB, OUTPUT);

    ledcAttach(MOTOR_A_PWM, 20000, 8);
    ledcAttach(MOTOR_B_PWM, 20000, 8);

    pinMode(ENC_A_PIN, INPUT);
    pinMode(ENC_B_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderB_ISR, CHANGE);

    motorBrake(MOTOR_A_INA, MOTOR_A_INB, MOTOR_A_PWM);
    motorBrake(MOTOR_B_INA, MOTOR_B_INB, MOTOR_B_PWM);

    Serial.println("Wait 2 secs...");
    delay(2000);

    unsigned long startTime;
    unsigned long elapsed;
    bool success;

    for (int cycle = 1; cycle <= NUM_CYCLES; cycle++) {
        Serial.println();
        Serial.print("--- WALK CYCLE ");
        Serial.print(cycle);
        Serial.println(" ---");

        Serial.print("Rear Left... ");
        startTime = millis();
        success = rotateOneRevolutionLeft(encCountA);
        elapsed = millis() - startTime;

        if (!success) {
            Serial.println("Rear Left failed. Stopping test.");
            return;
        }

        Serial.print("Done (");
        Serial.print(encCountA);
        Serial.print(" counts, ");
        Serial.print(elapsed);
        Serial.println(" ms)");
        delay(2000);
        Serial.print("Rear Right... ");
        startTime = millis();
        success = rotateOneRevolutionRight(encCountB);
        elapsed = millis() - startTime;

        if (!success) {
            Serial.println("Rear Right failed. Stopping test.");
            return;
        }

        Serial.print("Done (");
        Serial.print(encCountB);
        Serial.print(" counts, ");
        Serial.print(elapsed);
        Serial.println(" ms)");
        delay(2000);
    }

    Serial.println();
    Serial.println("=== All walking cycles complete ===");
}

void loop()
{
    // Nothing here — test runs once in setup()
}