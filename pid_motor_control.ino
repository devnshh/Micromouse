#include <Arduino.h>

// --- Pin Definitions (Adjust to your ESP32 wiring) ---
// L298N Motor Left
const int ENA = 25; // PWM pin
const int IN1 = 26;
const int IN2 = 27;
// L298N Motor Right
const int ENB = 14; // PWM pin
const int IN3 = 12;
const int IN4 = 13;

// Encoder Pins (Must support interrupts)
const int ENC_L_A = 32;
const int ENC_R_A = 34;

// --- Global Variables ---
volatile long countLeft = 0;
volatile long countRight = 0;

// PID Constants (You WILL need to tune these!)
float Kp = 2.5;
float Ki = 0.05;
float Kd = 1.0;

// Calibration variables
// You need to measure how many encoder ticks equal one maze cell and a 90-degree turn
const long TICKS_PER_CELL = 60; // Example value 
const long TICKS_PER_90_DEG = 60; // Example value

// --- Interrupt Service Routines for Encoders ---
void IRAM_ATTR isrLeft() { countLeft++; }
void IRAM_ATTR isrRight() { countRight++; }

void setup() {
    Serial.begin(115200);

    // Motor Pins
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    // Encoder Pins
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);
}

// --- Basic Motor Control Functions ---
void setMotorLeft(int speed) {
    if (speed > 0) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        speed = -speed;
    }
    speed = constrain(speed, 0, 255);
    analogWrite(ENA, speed); // ESP32 supports analogWrite in newer cores
}

void setMotorRight(int speed) {
    if (speed > 0) {
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
        speed = -speed;
    }
    speed = constrain(speed, 0, 255);
    analogWrite(ENB, speed);
}

// --- The PID Movement Function ---
// dir: 1 for straight, -1 for reverse. For turns, we use differential logic.
void moveWithPID(long targetTicksLeft, long targetTicksRight) {
    countLeft = 0;
    countRight = 0;
    
    long errorLeft = 0, errorRight = 0;
    long lastErrorLeft = 0, lastErrorRight = 0;
    long integralLeft = 0, integralRight = 0;
    long derivativeLeft = 0, derivativeRight = 0;
    
    // We loop until both wheels are close enough to their targets
    while (abs(targetTicksLeft - countLeft) > 5 || abs(targetTicksRight - countRight) > 5) {
        
        // 1. Calculate Error
        errorLeft = targetTicksLeft - countLeft;
        errorRight = targetTicksRight - countRight;

        // 2. Calculate Integral (accumulated error)
        integralLeft += errorLeft;
        integralRight += errorRight;

        // Anti-windup for integral (prevents it from growing infinitely)
        integralLeft = constrain(integralLeft, -1000, 1000);
        integralRight = constrain(integralRight, -1000, 1000);

        // 3. Calculate Derivative (rate of change)
        derivativeLeft = errorLeft - lastErrorLeft;
        derivativeRight = errorRight - lastErrorRight;

        // 4. Compute Output
        int speedLeft = (Kp * errorLeft) + (Ki * integralLeft) + (Kd * derivativeLeft);
        int speedRight = (Kp * errorRight) + (Ki * integralRight) + (Kd * derivativeRight);

        // Apply speeds
        setMotorLeft(speedLeft);
        setMotorRight(speedRight);

        // Save last errors
        lastErrorLeft = errorLeft;
        lastErrorRight = errorRight;

        delay(10); // Small delay to let the motors react and encoders read
    }

    // Stop motors once target is reached
    setMotorLeft(0);
    setMotorRight(0);
}

// --- Wrapper Functions for the Maze ---
void moveForwardOneCell() {
    // Both wheels drive forward
    moveWithPID(TICKS_PER_CELL, TICKS_PER_CELL);
}

void turnLeft90() {
    // Left wheel goes backward, right wheel goes forward
    moveWithPID(-TICKS_PER_90_DEG, TICKS_PER_90_DEG);
}

void turnRight90() {
    // Left wheel goes forward, right wheel goes backward
    moveWithPID(TICKS_PER_90_DEG, -TICKS_PER_90_DEG);
}

void loop() {
    delay(2000);
    moveForwardOneCell();
    delay(500);
    turnRight90();
    delay(2000);
}