/*
 * ===============================================
 *  GET ENCODER PPR (Pulses Per Revolution)
 * ===============================================
 *  Board:  ESP32-WROOM-DA (battery powered)
 *  Driver: TB6612FNG
 *  Motor:  1x N20 encoder motor (left motor)
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
//  PIN DEFINITIONS (from test_90_degrees)
// =============================================
// TB6612FNG Motor A (Left)
const int PWMA = 21;
const int AIN1 = 22;
const int AIN2 = 23;

// Motor driver standby
const int STBY = 5;

// Left encoder
const int ENC_A = 4;  // Channel A
const int ENC_B = 27; // Channel B
// =============================================

// Encoder counter
volatile long ticks = 0;

// ESP32 uses IRAM_ATTR for ISRs
void IRAM_ATTR encoderISR() { ticks++; }

void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor driver pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, HIGH); // Enable motor driver
  analogWrite(PWMA, 0);

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.println("========================================");
  Serial.println("  ENCODER PPR — ESP32 + TB6612FNG");
  Serial.println("========================================");
  Serial.println("  Motor A: PWMA=21 AIN1=22 AIN2=23");
  Serial.println("  Encoder: A=4  B=27");
  Serial.println("  STBY=5");
  Serial.println("========================================");
  Serial.println("Commands:");
  Serial.println("  r = Reset counter to 0");
  Serial.println("  p = Print tick count");
  Serial.println("  f = Run motor FORWARD 3 sec");
  Serial.println("  b = Run motor BACKWARD 3 sec");
  Serial.println("========================================");
  Serial.println("Or just rotate the wheel 1 turn by hand");
  Serial.println("and read the tick count = PPR");
  Serial.println("========================================\n");
}

void stopMotor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

void runMotor(bool forward, int pwm, int durationMs) {
  ticks = 0;
  if (forward) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, pwm);
  delay(durationMs);
  stopMotor();
}

unsigned long lastPrint = 0;

void loop() {
  // Live tick display every 500ms
  if (millis() - lastPrint > 500) {
    Serial.print("  Ticks: ");
    Serial.println(ticks);
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
      Serial.print("\n>> Ticks = ");
      Serial.print(ticks);
      Serial.println("\n");
      break;

    case 'f':
      Serial.println("\n>> Running motor FORWARD for 3 seconds...");
      Serial.println("   Count full revolutions visually!");
      runMotor(true, 100, 3000);
      Serial.print(">> Done. Ticks = ");
      Serial.println(ticks);
      Serial.println("   PPR = ticks / revolutions_you_counted\n");
      break;

    case 'b':
      Serial.println("\n>> Running motor BACKWARD for 3 seconds...");
      Serial.println("   Count full revolutions visually!");
      runMotor(false, 100, 3000);
      Serial.print(">> Done. Ticks = ");
      Serial.println(ticks);
      Serial.println("   PPR = ticks / revolutions_you_counted\n");
      break;
    }
  }
}
