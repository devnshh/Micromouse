/*
 * ===============================================
 *  GET ENCODER PPR (Pulses Per Revolution)
 * ===============================================
 *  Setup: 1x N20 motor + L298N + ESP32 via USB
 *         Testing ONE motor at a time.
 *
 *  Wiring (adjust pins below if different):
 *    L298N ENA  → ESP32 GPIO 25
 *    L298N IN1  → ESP32 GPIO 26
 *    L298N IN2  → ESP32 GPIO 27
 *    Encoder A  → ESP32 GPIO 32
 *    Encoder B  → ESP32 GPIO 33
 *    Encoder VCC → ESP32 3.3V
 *    Encoder GND → ESP32 GND
 *    Motor +/-  → L298N OUT1/OUT2
 *
 *  How to use:
 *    1. Upload this sketch, open Serial Monitor (115200)
 *    2. Ticks print live every 500ms
 *    3. Send 'r' to reset counter
 *    4. Rotate the wheel exactly 1 full revolution BY HAND
 *    5. Read the tick count → that's your PPR
 *    6. Or send 'm' to run motor for 3 sec, count revolutions
 *       visually, then PPR = ticks / revolutions
 *    7. Repeat for the second motor (swap wires on L298N
 *       and encoder pins on ESP32, or just connect the
 *       other motor to the same terminals)
 * ===============================================
 */

#include <Arduino.h>

// =============================================
//  PIN DEFINITIONS — match to YOUR wiring
// =============================================
// L298N control pins (one channel only — Motor A side)
const int ENA = 25;   // PWM enable
const int IN1 = 26;   // Direction
const int IN2 = 27;   // Direction

// Encoder pins (match your working wiring)
const int ENC_A = 21;  // Encoder channel A (interrupt)
const int ENC_B = 18;  // Encoder channel B
// =============================================

// LEDC PWM config
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;

// Encoder counter
volatile long ticks = 0;

// Simple counting ISR — just count pulses for PPR measurement
void IRAM_ATTR encoderISR() {
    ticks++;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // Motor pins
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);

    // LEDC PWM (ESP32 Core 3.x API)
    ledcAttach(ENA, PWM_FREQ, PWM_RES);
    ledcWrite(ENA, 0);

    // Encoder pins
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

    Serial.println("========================================");
    Serial.println("  ENCODER PPR — Single Motor Test");
    Serial.println("========================================");
    Serial.println("  Wiring: L298N Motor A channel");
    Serial.printf( "  ENA=%d  IN1=%d  IN2=%d\n", ENA, IN1, IN2);
    Serial.printf( "  ENC_A=%d  ENC_B=%d\n", ENC_A, ENC_B);
    Serial.println("========================================");
    Serial.println("Commands:");
    Serial.println("  r = Reset counter to 0");
    Serial.println("  p = Print tick count");
    Serial.println("  m = Run motor 3 sec (count revs visually)");
    Serial.println("  f = Run motor FORWARD 3 sec");
    Serial.println("  b = Run motor BACKWARD 3 sec");
    Serial.println("========================================");
    Serial.println("Or just rotate the wheel 1 turn by hand");
    Serial.println("and read the tick count = PPR");
    Serial.println("========================================\n");
}

void stopMotor() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
}

void runMotor(bool forward, int pwm, int durationMs) {
    ticks = 0;
    if (forward) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    }
    ledcWrite(ENA, pwm);
    delay(durationMs);
    stopMotor();
}

unsigned long lastPrint = 0;

void loop() {
    // Live tick display every 500ms
    if (millis() - lastPrint > 500) {
        Serial.printf("  Ticks: %ld\n", ticks);
        lastPrint = millis();
    }

    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'r':
                ticks = 0;
                Serial.println("\n>> Counter RESET to 0\n");
                break;

            case 'p':
                Serial.printf("\n>> Ticks = %ld\n\n", ticks);
                break;

            case 'm':
            case 'f':
                Serial.println("\n>> Running motor FORWARD for 3 seconds...");
                Serial.println("   Count full revolutions visually!");
                runMotor(true, 80, 3000);
                Serial.printf(">> Done. Ticks = %ld\n", ticks);
                Serial.println("   PPR = ticks / revolutions_you_counted\n");
                break;

            case 'b':
                Serial.println("\n>> Running motor BACKWARD for 3 seconds...");
                Serial.println("   Count full revolutions visually!");
                runMotor(false, 80, 3000);
                Serial.printf(">> Done. Ticks = %ld (should be negative)\n", ticks);
                Serial.println("   PPR = abs(ticks) / revolutions_you_counted\n");
                break;
        }
    }
}
