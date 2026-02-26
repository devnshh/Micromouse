// ===== CONFIGURATION =====
// Set the number of ticks your encoder counts for exactly 1 full revolution of
// the wheel.
const int TICKS_PER_REV = 200;
const int pulsesFor90 = TICKS_PER_REV / 4;

// ===== MOTOR PINS =====
const int PWMA = 14;
const int AIN1 = 12;
const int AIN2 = 13;

// ===== ENCODER PINS =====
const int ENC_A = 5;
const int ENC_B = 4;

volatile long encoderCount = 0;

// Use ICACHE_RAM_ATTR for ESP8266 (NodeMCU) interrupts to prevent crashes
void ICACHE_RAM_ATTR encoderISR() {
  if (digitalRead(ENC_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

// Set the analog write range to 0-255 instead of ESP8266's default 0-1023
// Without this, analogWrite(180) only gives 17% power resulting in stalling and
// timeouts!
#if defined(ESP8266)
  analogWriteRange(255);
#endif

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);
}

void stopMotor() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);
  delay(50); // Short active brake

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

void moveMotor(bool forward) {
  // Clear the encoder count safely
  noInterrupts();
  encoderCount = 0;
  interrupts();

  if (forward) {
    Serial.print("Moving Forward... ");
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    Serial.print("Moving Backward... ");
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  // Start the motor
  analogWrite(PWMA, 180);

  unsigned long t0 = millis();
  while (true) {
    // Read the volatile count safely
    long currentCount;
    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    // Check if we reached the target
    if (currentCount >= pulsesFor90 || currentCount <= -pulsesFor90) {
      Serial.print("Target Reached! Ticks: ");
      Serial.println(currentCount);
      break;
    }

    // Timeout if blocked for over 3 seconds
    if (millis() - t0 > 3000) {
      Serial.print("TIMEOUT! Ticks: ");
      Serial.println(currentCount);
      break;
    }

    // VERY IMPORTANT: feed the watchdog on ESP8266 to prevent crashes
    yield();
  }

  stopMotor();
}

void loop() {
  moveMotor(true); // Move Forward 90 degrees
  delay(500);      // Pause 500ms

  moveMotor(false); // Move Backward 90 degrees
  delay(500);       // Pause 500ms
}
