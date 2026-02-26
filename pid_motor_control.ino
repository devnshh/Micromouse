/*
 * ================================================================
 *  MOTOR TEST SKETCH — Standalone for calibrating & testing motors
 * ================================================================
 *  Upload this INSTEAD of floodfill.ino when you want to:
 *    1. Verify motors spin the correct direction
 *    2. Measure TICKS_PER_CELL (drive straight, read Serial)
 *    3. Measure TICKS_PER_90_DEG (spin in place, read Serial)
 *    4. Tune PID constants (Kp, Ki, Kd)
 *    5. Find MOTOR_MIN_PWM (smallest PWM that moves the wheel)
 *
 *  !! Do NOT upload this together with floodfill.ino — they each
 *     have their own setup()/loop(). Use one at a time. !!
 * ================================================================
 */

#include <Arduino.h>

// --- TB6612FNG Motor Driver Pins (match your wiring) ---
const int PWMA = 25;
const int AIN1 = 26;
const int AIN2 = 27;
const int PWMB = 14;
const int BIN1 = 12;
const int BIN2 = 13;
const int STBY = 4;   // *** ADJUST to your wiring ***

// --- Encoder Pins (Quadrature A+B) ---
const int ENC_L_A = 32;
const int ENC_L_B = 33;
const int ENC_R_A = 16;
const int ENC_R_B = 17;

// --- LEDC PWM Config ---
const int PWM_FREQ       = 20000;
const int PWM_RESOLUTION = 8;
// (ESP32 Core 3.x: ledcAttach takes pin directly, no channel needed)

// --- Encoder Counters ---
volatile long countLeft  = 0;
volatile long countRight = 0;

// --- PID Constants (TUNE THESE!) ---
float Kp = 2.0;
float Ki = 0.02;
float Kd = 0.8;

// --- Calibration targets (fill in after measuring) ---
const long TICKS_PER_CELL   = 360;  // Example — measure yours
const long TICKS_PER_90_DEG = 190;  // Example — measure yours
const int  MOTOR_MIN_PWM    = 45;   // Minimum PWM that moves wheels

// --- Safety timeout (ms) ---
const unsigned long MOVE_TIMEOUT_MS = 3000;

// ================================================================
//  ISRs — Quadrature direction detection
// ================================================================
void IRAM_ATTR isrLeft()  { countLeft  += digitalRead(ENC_L_B) ? -1 : 1; }
void IRAM_ATTR isrRight() { countRight += digitalRead(ENC_R_B) ? -1 : 1; }

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Motor Test Sketch ===");

    // Motor driver
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH); // Enable TB6612FNG

    // LEDC PWM (Core 3.x API)
    ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);

    // Encoders
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);

    Serial.println("Type a command in Serial Monitor:");
    Serial.println("  f  = drive forward one cell");
    Serial.println("  l  = turn left 90");
    Serial.println("  r  = turn right 90");
    Serial.println("  d  = print encoder counts (for measuring)");
    Serial.println("  0  = reset encoder counts");
    Serial.println("  m  = find minimum PWM (ramp up slowly)");
}

// ================================================================
//  MOTOR HELPERS
// ================================================================
void setMotorLeft(int speed) {
    if (speed > 0) {
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    } else if (speed < 0) {
        digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
        speed = -speed;
    } else {
        digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);
    }
    if (speed > 0 && speed < MOTOR_MIN_PWM) speed = MOTOR_MIN_PWM;
    speed = constrain(speed, 0, 255);
    ledcWrite(PWMA, speed);
}

void setMotorRight(int speed) {
    if (speed > 0) {
        digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    } else if (speed < 0) {
        digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
        speed = -speed;
    } else {
        digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);
    }
    if (speed > 0 && speed < MOTOR_MIN_PWM) speed = MOTOR_MIN_PWM;
    speed = constrain(speed, 0, 255);
    ledcWrite(PWMB, speed);
}

void stopMotors() {
    setMotorLeft(0);
    setMotorRight(0);
}

// ================================================================
//  PID MOVEMENT
// ================================================================
void moveWithPID(long targetL, long targetR) {
    countLeft = 0;
    countRight = 0;

    long errL = 0, errR = 0;
    long lastErrL = 0, lastErrR = 0;
    long intL = 0, intR = 0;
    unsigned long t0 = millis();

    while (abs(targetL - countLeft) > 3 || abs(targetR - countRight) > 3) {
        if (millis() - t0 > MOVE_TIMEOUT_MS) {
            Serial.println("! TIMEOUT");
            break;
        }

        errL = targetL - countLeft;
        errR = targetR - countRight;

        intL = constrain(intL + errL, -1000L, 1000L);
        intR = constrain(intR + errR, -1000L, 1000L);

        long dL = errL - lastErrL;
        long dR = errR - lastErrR;

        int spdL = (int)(Kp * errL + Ki * intL + Kd * dL);
        int spdR = (int)(Kp * errR + Ki * intR + Kd * dR);

        setMotorLeft(spdL);
        setMotorRight(spdR);

        lastErrL = errL;
        lastErrR = errR;
        delay(10);
    }
    stopMotors();

    Serial.printf("Done. L_ticks=%ld  R_ticks=%ld\n", countLeft, countRight);
}

void moveForwardOneCell() { moveWithPID(TICKS_PER_CELL, TICKS_PER_CELL); }
void turnLeft90()         { moveWithPID(-TICKS_PER_90_DEG, TICKS_PER_90_DEG); }
void turnRight90()        { moveWithPID(TICKS_PER_90_DEG, -TICKS_PER_90_DEG); }

// ================================================================
//  FIND MINIMUM PWM (Motor Dead-Zone Finder)
// ================================================================
void findMinPWM() {
    Serial.println("Ramping left motor PWM from 0 to 100...");
    for (int pwm = 0; pwm <= 100; pwm += 5) {
        countLeft = 0;
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
        ledcWrite(PWMA, pwm);
        delay(300);
        ledcWrite(PWMA, 0);
        Serial.printf("  PWM=%3d  ticks=%ld\n", pwm, countLeft);
        delay(200);
    }
    stopMotors();
    Serial.println("Done. Use the first PWM that gave ticks > 0 as MOTOR_MIN_PWM.");
}

// ================================================================
//  LOOP — Serial command interface
// ================================================================
void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'f':
                Serial.println(">> Forward one cell");
                moveForwardOneCell();
                break;
            case 'l':
                Serial.println(">> Turn left 90");
                turnLeft90();
                break;
            case 'r':
                Serial.println(">> Turn right 90");
                turnRight90();
                break;
            case 'd':
                Serial.printf("Encoders  L=%ld  R=%ld\n", countLeft, countRight);
                break;
            case '0':
                countLeft = 0; countRight = 0;
                Serial.println("Encoders reset.");
                break;
            case 'm':
                findMinPWM();
                break;
        }
    }
}