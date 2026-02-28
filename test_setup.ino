/*
 * ================================================================
 *  TEST SETUP — Motors Only (Run Straight)
 * ================================================================
 *  Slower motor runs at max PWM (255).
 *  Tune FASTER_MOTOR_PWM until both wheels match speed.
 * ================================================================
 */

// ================= MOTOR PINS =================
const int PWMA = 21;
const int AIN1 = 22;
const int AIN2 = 23;

const int PWMB = 19;
const int BIN1 = 33;
const int BIN2 = 32;

const int STBY = 5;

// ================= MOTOR SPEED =================
const int SLOWER_MOTOR_PWM = 255; // Slower motor — always max
const int FASTER_MOTOR_PWM = 200; // <<< TUNE THIS to match the slower motor

void setup() {
  Serial.begin(115200);

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

  // Set direction — forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  // Set speed — Motor A (left) = slower, Motor B (right) = faster
  analogWrite(PWMA, SLOWER_MOTOR_PWM);
  analogWrite(PWMB, FASTER_MOTOR_PWM);

  Serial.println("Motors running — tune FASTER_MOTOR_PWM to match speeds");
}

void loop() {
  // Nothing needed — motors keep running
}