/*
 * ===============================================
 *  TEST 90-DEGREE TURNS — Single Motor Setup
 * ===============================================
 *  Setup: 1x N20 motor + L298N + ESP8266 via USB
 *         Testing ONE motor at a time.
 *
 *  NOTE: For a real 90° turn you need BOTH motors
 *  (one forward, one backward). With ONE motor, this
 *  sketch lets you:
 *    - Measure how many ticks = a known rotation
 *    - Spin the single motor a set number of ticks
 *    - Use this to calculate TICKS_PER_90_DEG for
 *      when both motors are connected
 *
 *  Wiring (same as get_ppr.ino):
 *    L298N ENA  -> 5V (jumper ON)
 *    L298N IN1  -> ESP8266 D5
 *    L298N IN2  -> ESP8266 D6
 *    Encoder A  -> ESP8266 D1
 *    Encoder B  -> ESP8266 D2
 *    Encoder VCC → ESP8266 3.3V
 *    Encoder GND → ESP8266 GND
 *
 *  How to use:
 *    1. First get your PPR from get_ppr.ino
 *    2. Upload this sketch, open Serial Monitor (115200)
 *    3. Type a tick count (e.g. "200") and press Enter
 *    4. Send 'f' to spin forward that many ticks
 *    5. Send 'b' to spin backward that many ticks
 *    6. Check how far the wheel/robot actually rotated
 *    7. Adjust the tick count and repeat
 *
 *  For calculating TICKS_PER_90_DEG:
 *    - With the wheel on the ground and the other side
 *      pivoting, find how many ticks makes the robot
 *      rotate 90°. That's your value.
 * ===============================================
 */

#include <Arduino.h>

#if !defined(ESP8266)
#error "This sketch is configured for ESP8266. Select an ESP8266 board."
#endif

// =============================================
//  PIN DEFINITIONS — match to YOUR wiring
// =============================================
// Keep ENA jumper enabled on L298N; PWM is applied on IN pins.
const int IN1 = D5;
const int IN2 = D6;

const int ENC_A = D1;
const int ENC_B = D2;
// =============================================

const int PWM_FREQ = 5000;
const int PWM_MAX  = 255;

volatile long ticks = 0;
#define ISR_ATTR IRAM_ATTR

void ISR_ATTR encoderISR() {
    ticks += digitalRead(ENC_B) ? -1 : 1;
}

// Adjustable at runtime via Serial
long targetTicks = 150;
int  motorSpeed  = 100;   // PWM 0-255

const unsigned long TIMEOUT_MS = 5000;

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);

    analogWriteRange(PWM_MAX);
    analogWriteFreq(PWM_FREQ);
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

    Serial.println("========================================");
    Serial.println("  TURN TESTER — Single Motor");
    Serial.println("========================================");
    Serial.printf( "  Pins: IN1=%d IN2=%d\n", IN1, IN2);
    Serial.printf( "  Encoder: A=%d B=%d\n", ENC_A, ENC_B);
    Serial.printf( "  Target ticks: %ld\n", targetTicks);
    Serial.printf( "  Motor speed:  %d PWM\n", motorSpeed);
    Serial.println("========================================");
    Serial.println("Commands:");
    Serial.println("  <number>  = Set target ticks (e.g. 200)");
    Serial.println("  f         = Spin FORWARD target ticks");
    Serial.println("  b         = Spin BACKWARD target ticks");
    Serial.println("  s<number> = Set speed (e.g. s120)");
    Serial.println("  d         = Print current encoder ticks");
    Serial.println("  r         = Reset encoder counter");
    Serial.println("  p         = Print current settings");
    Serial.println("========================================\n");
}

void stopMotor() {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
}

void spinMotor(bool forward, long target) {
    ticks = 0;
    unsigned long t0 = millis();

    if (forward) {
        analogWrite(IN2, 0);
        analogWrite(IN1, motorSpeed);
    } else {
        analogWrite(IN1, 0);
        analogWrite(IN2, motorSpeed);
    }

    // Wait until we reach target ticks
    long absTicks = 0;
    while (absTicks < target) {
        absTicks = abs(ticks);

        // Slow down when close (last 20%)
        if (target - absTicks < target / 5) {
            int slowPwm = max(40, motorSpeed / 2);
            if (forward) analogWrite(IN1, slowPwm);
            else         analogWrite(IN2, slowPwm);
        }

        if (millis() - t0 > TIMEOUT_MS) {
            Serial.println("  ! TIMEOUT");
            break;
        }
        delay(5);
    }
    stopMotor();

    Serial.printf("  Done. Ticks = %ld (target = %ld)\n", ticks, forward ? target : -target);
}

// Serial input buffer
char inputBuf[32];
int  inputIdx = 0;

void processInput(const char* input) {
    while (*input == ' ' || *input == '\n' || *input == '\r') input++;
    if (*input == '\0') return;

    if (input[0] == 'f' && (input[1] == '\0' || input[1] == '\n' || input[1] == '\r')) {
        Serial.printf("\n>> Spinning FORWARD %ld ticks at PWM %d...\n", targetTicks, motorSpeed);
        spinMotor(true, targetTicks);
        Serial.println();

    } else if (input[0] == 'b' && (input[1] == '\0' || input[1] == '\n' || input[1] == '\r')) {
        Serial.printf("\n>> Spinning BACKWARD %ld ticks at PWM %d...\n", targetTicks, motorSpeed);
        spinMotor(false, targetTicks);
        Serial.println();

    } else if (input[0] == 's' && input[1] >= '0' && input[1] <= '9') {
        motorSpeed = constrain(atoi(input + 1), 30, 255);
        Serial.printf("\n>> Motor speed set to %d PWM\n\n", motorSpeed);

    } else if (input[0] == 'd') {
        Serial.printf("\n>> Current ticks = %ld\n\n", ticks);

    } else if (input[0] == 'r') {
        ticks = 0;
        Serial.println("\n>> Encoder RESET to 0\n");

    } else if (input[0] == 'p') {
        Serial.printf("\n  Target ticks: %ld\n", targetTicks);
        Serial.printf("  Motor speed:  %d PWM\n\n", motorSpeed);

    } else if (input[0] >= '0' && input[0] <= '9') {
        long val = atol(input);
        if (val > 0 && val < 100000) {
            targetTicks = val;
            Serial.printf("\n>> Target ticks set to: %ld\n\n", targetTicks);
        } else {
            Serial.println("\n>> Invalid (use 1-99999)\n");
        }

    } else {
        Serial.println("\n>> Unknown. Use: f, b, s<num>, d, r, p, or a number.\n");
    }
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputIdx > 0) {
                inputBuf[inputIdx] = '\0';
                processInput(inputBuf);
                inputIdx = 0;
            }
        } else if (inputIdx < 30) {
            inputBuf[inputIdx++] = c;
        }
    }
}
