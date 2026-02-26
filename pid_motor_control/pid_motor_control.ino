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

#if !defined(ESP8266)
#error "This sketch is configured for ESP8266. Select an ESP8266 board."
#endif

// --- L298N Motor Driver Pins (match your wiring) ---
// Keep ENA/ENB jumpers on L298N enabled; PWM is applied on IN pins.
// D8/D3/D4 are boot-strap pins on ESP8266; keep external pull states compatible.
const int IN1 = D5;
const int IN2 = D6;
const int IN3 = D7;
const int IN4 = D8;

// --- Encoder Pins (Quadrature A+B) ---
const int ENC_L_A = D1;
const int ENC_L_B = D2;
const int ENC_R_A = D3;
const int ENC_R_B = D4;

// --- PWM Config ---
const int PWM_FREQ = 20000;
const int PWM_MAX  = 255;

// --- Encoder Counters ---
volatile long countLeft  = 0;
volatile long countRight = 0;

// --- PID Constants (TUNE THESE!) ---
float Kp = 2.0;
float Ki = 0.0;
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
#define ISR_ATTR IRAM_ATTR

void ISR_ATTR isrLeft()  { countLeft  += digitalRead(ENC_L_B) ? -1 : 1; }
void ISR_ATTR isrRight() { countRight += digitalRead(ENC_R_B) ? -1 : 1; }

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Motor Test Sketch ===");

    // Motor driver
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    analogWriteRange(PWM_MAX);
    analogWriteFreq(PWM_FREQ);
    analogWrite(IN1, 0); analogWrite(IN2, 0);
    analogWrite(IN3, 0); analogWrite(IN4, 0);

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
    if (speed > 0 && speed < MOTOR_MIN_PWM) speed = MOTOR_MIN_PWM;
    if (speed < 0 && speed > -MOTOR_MIN_PWM) speed = -MOTOR_MIN_PWM;
    speed = constrain(speed, -PWM_MAX, PWM_MAX);

    if (speed > 0) {
        analogWrite(IN1, speed);
        digitalWrite(IN2, LOW);
    } else if (speed < 0) {
        digitalWrite(IN1, LOW);
        analogWrite(IN2, -speed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
}

void setMotorRight(int speed) {
    if (speed > 0 && speed < MOTOR_MIN_PWM) speed = MOTOR_MIN_PWM;
    if (speed < 0 && speed > -MOTOR_MIN_PWM) speed = -MOTOR_MIN_PWM;
    speed = constrain(speed, -PWM_MAX, PWM_MAX);

    if (speed > 0) {
        analogWrite(IN3, speed);
        digitalWrite(IN4, LOW);
    } else if (speed < 0) {
        digitalWrite(IN3, LOW);
        analogWrite(IN4, -speed);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }
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
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        analogWrite(IN1, pwm);
        delay(300);
        analogWrite(IN1, 0);
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
