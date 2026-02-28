/*
 * ================================================================
 *  TEST 90° TURN — ESP32 + 2 Motors + Encoders
 * ================================================================
 *  1. Both motors run forward for 2 seconds
 *  2. Bot performs an in-place 90° turn using encoder ticks
 *  3. Repeats
 *
 *  200 ticks = 1 full wheel revolution
 *  Tune TICKS_PER_90_DEG until the bot turns exactly 90°.
 * ================================================================
 */

// ================= MOTOR A (LEFT) PINS =================
const int PWMA = 21;
const int AIN1 = 22;
const int AIN2 = 23;

// ================= MOTOR B (RIGHT) PINS =================
const int PWMB = 19;
const int BIN1 = 33;
const int BIN2 = 32;

// ================= MOTOR DRIVER STANDBY =================
const int STBY = 5;

// ================= ENCODER PINS =================
const int ENC_L_A = 4;  // <<< MISSING — set your left encoder channel A pin
const int ENC_L_B = 27; // <<< MISSING — set your left encoder channel B pin
const int ENC_R_A = 14; // <<< MISSING — set your right encoder channel A pin
const int ENC_R_B = 26; // <<< MISSING — set your right encoder channel B pin

// ================= CALIBRATION =================
const int TICKS_PER_90_DEG = 410; // <<< TUNE THIS until bot turns exactly 90°

const int DRIVE_PWM = 50; // Base PWM for driving forward (0–255)
const int TURN_PWM = 50;  // PWM for turning (lower = more control)

// ================= PID GAINS (for straight-line correction) =================
float Kp =
    2.0; // <<< Proportional — start here, increase if not correcting enough
float Ki = 0.05; // <<< Integral — fixes steady-state drift, keep small
float Kd = 1.0;  // <<< Derivative — dampens oscillation

// ================= ENCODER STATE =================
volatile long countLeft = 0;
volatile long countRight = 0;

void IRAM_ATTR isrLeftA() { countLeft += digitalRead(ENC_L_B) ? -1 : 1; }

void IRAM_ATTR isrRightA() { countRight += digitalRead(ENC_R_B) ? -1 : 1; }

// ================= MOTOR CONTROL =================

void stopMotors() {
  // Active brake briefly
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255);
  analogWrite(PWMB, 255);
  delay(50);

  // Then coast
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// Drive forward for a given duration with PID correction to keep both wheels in
// sync
void driveForwardPID(int basePwm, int durationMs) {
  // Reset encoder counts
  noInterrupts();
  countLeft = 0;
  countRight = 0;
  interrupts();

  // Set both motors to forward direction
  // Motor A forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // Motor B forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  float integral = 0;
  float lastError = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < (unsigned long)durationMs) {
    // Read encoder counts safely
    long L, R;
    noInterrupts();
    L = countLeft;
    R = countRight;
    interrupts();

    // Use absolute values — encoders may count negative depending on wiring
    long absL = abs(L);
    long absR = abs(R);

    // Error = difference in distance traveled (positive means left is ahead)
    float error = (float)(absL - absR);

    // PID terms
    integral = constrain(integral + error, -500.0f, 500.0f);
    float derivative = error - lastError;

    float correction = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Apply correction: slow down the faster motor, speed up the slower one
    int pwmLeft = constrain(basePwm - (int)(correction / 2), 0, 255);
    int pwmRight = constrain(basePwm + (int)(correction / 2), 0, 255);

    analogWrite(PWMA, pwmLeft);
    analogWrite(PWMB, pwmRight);

    delay(10); // PID update rate ~100Hz
  }

  stopMotors();

  // Print final encoder counts for debugging
  long L, R;
  noInterrupts();
  L = countLeft;
  R = countRight;
  interrupts();
  Serial.printf("Forward done — L:%ld R:%ld ticks (diff:%ld)\n", L, R, L - R);
}

// In-place 90° turn using encoders
// turnRight = true  → left wheel forward, right wheel backward
// turnRight = false → left wheel backward, right wheel forward
void turn90(bool turnRight) {
  // Reset encoder counts
  noInterrupts();
  countLeft = 0;
  countRight = 0;
  interrupts();

  if (turnRight) {
    Serial.print("Turning RIGHT 90°... ");
    // Left wheel forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    // Right wheel backward
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    Serial.print("Turning LEFT 90°... ");
    // Left wheel backward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    // Right wheel forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  analogWrite(PWMA, TURN_PWM);
  analogWrite(PWMB, TURN_PWM);

  unsigned long t0 = millis();
  while (true) {
    long L, R;
    noInterrupts();
    L = countLeft;
    R = countRight;
    interrupts();

    // Both wheels should have traveled TICKS_PER_90_DEG
    if (abs(L) >= TICKS_PER_90_DEG && abs(R) >= TICKS_PER_90_DEG) {
      Serial.printf("Done! L:%ld R:%ld ticks\n", L, R);
      break;
    }

    // Timeout safety
    if (millis() - t0 > 3000) {
      Serial.printf("TIMEOUT! L:%ld R:%ld ticks\n", L, R);
      break;
    }

    delay(1);
  }

  stopMotors();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== 90° Turn Test (ESP32) ===");

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  // Encoder pins
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, RISING);

  Serial.println("Ready — starting in 2 seconds...");
  delay(2000);
}

// ================= MAIN LOOP =================
void loop() {
  // Step 1: Drive forward for 3 seconds with PID correction
  Serial.println("Driving forward for 3 seconds (PID)...");
  driveForwardPID(DRIVE_PWM, 3000);
  delay(500);

  // Step 2: Turn 90° right
  turn90(true);
  delay(500);
}
