/*
 * ===============================================
 *  GET ENCODER PPR (Pulses Per Revolution)
 * ===============================================
 *  Board:  NodeMCU ESP8266 (USB powered)
 *  Driver: TB6612FNG
 *  Motor:  1x N20 encoder motor (6-wire)
 *
 *  Wiring:
 *    TB6612FNG PWMA → D5 (GPIO14)
 *    TB6612FNG AIN1 → D6 (GPIO12)
 *    TB6612FNG AIN2 → D7 (GPIO13)
 *    TB6612FNG VM   → NodeMCU VIN (5V from USB)
 *    TB6612FNG VCC  → NodeMCU 3.3V
 *    TB6612FNG GND  → GND
 *    TB6612FNG STBY → 3.3V (tied HIGH, enables driver)
 *    TB6612FNG A01  → Motor wire 1 (Motor +)
 *    TB6612FNG A02  → Motor wire 6 (Motor -)
 *    Encoder VCC (wire 2) → 3.3V
 *    Encoder GND (wire 5) → GND
 *    Encoder A   (wire 3) → D1 (GPIO5)
 *    Encoder B   (wire 4) → D2 (GPIO4)
 *
 *  How to use:
 *    1. Upload this sketch, open Serial Monitor (115200)
 *    2. Ticks print live every 500ms
 *    3. Send 'r' to reset counter
 *    4. Rotate the wheel exactly 1 full revolution BY HAND
 *    5. Read the tick count → that's your PPR
 *    6. Or send 'f' to run motor 3 sec, count revolutions
 *       visually, then PPR = ticks / revolutions
 * ===============================================
 */

#include <Arduino.h>

// =============================================
//  PIN DEFINITIONS
// =============================================
// TB6612FNG Motor A channel
const int PWMA = 14;  // D5
const int AIN1 = 12;  // D6
const int AIN2 = 13;  // D7

// Encoder (D1/D2 support interrupts on ESP8266)
const int ENC_A = 5;   // D1
const int ENC_B = 4;   // D2
// =============================================

// Encoder counter
volatile long ticks = 0;

// Simple counting ISR — just count pulses for PPR measurement
// ESP8266 uses ICACHE_RAM_ATTR instead of IRAM_ATTR
void ICACHE_RAM_ATTR encoderISR() {
    ticks++;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // Motor driver pins (TB6612FNG)
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    // ESP8266 analogWrite range is 0-1023 by default; set to 0-255
    analogWriteRange(255);
    analogWrite(PWMA, 0);

    // Encoder pins
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

    Serial.println("========================================");
    Serial.println("  ENCODER PPR — NodeMCU + TB6612FNG");
    Serial.println("========================================");
    Serial.println("  Motor A: PWMA=D5 AIN1=D6 AIN2=D7");
    Serial.println("  Encoder: A=D1  B=D2");
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
    // Coast stop (AIN1=L, AIN2=L on TB6612FNG)
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
}

void runMotor(bool forward, int pwm, int durationMs) {
    ticks = 0;
    if (forward) {
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    }
    analogWrite(PWMA, pwm);
    delay(durationMs);
    stopMotor();
}

unsigned long lastPrint = 0;

void loop() {
    // Live tick display every 500ms
    if (millis() - lastPrint > 500) {
        Serial.print("  Ticks: "); Serial.println(ticks);
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
                Serial.print("\n>> Ticks = "); Serial.print(ticks); Serial.println("\n");
                break;

            case 'm':
            case 'f':
                Serial.println("\n>> Running motor FORWARD for 3 seconds...");
                Serial.println("   Count full revolutions visually!");
                runMotor(true, 100, 3000);
                Serial.print(">> Done. Ticks = "); Serial.println(ticks);
                Serial.println("   PPR = ticks / revolutions_you_counted\n");
                break;

            case 'b':
                Serial.println("\n>> Running motor BACKWARD for 3 seconds...");
                Serial.println("   Count full revolutions visually!");
                runMotor(false, 100, 3000);
                Serial.print(">> Done. Ticks = "); Serial.println(ticks);
                Serial.println("   PPR = ticks / revolutions_you_counted\n");
                break;
        }
    }
}
