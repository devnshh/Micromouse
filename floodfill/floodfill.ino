/*
 * ================================================================
 *  MICROMOUSE — Integrated Floodfill + Motor Control + Navigation
 * ================================================================
 *  Hardware:
 *    - ESP8266 (NodeMCU/Wemos style boards)
 *    - L298N motor driver (ENA/ENB jumpers installed)
 *    - 2x N20 motors with encoders (A-channel used in this sketch)
 *    - 3x HC-SR04 ultrasonic sensors (one-pin mode with ~2.2k tie)
 *    - 2x 3.7V LiPo batteries (series = 7.4V) + buck converter
 *
 *  See CHANGES.md for calibration checklist and what to adjust.
 * ================================================================
 */

#include <Arduino.h>

#if !defined(ESP8266)
#error "This sketch is configured for ESP8266. Select an ESP8266 board."
#endif

// ================================================================
//  PIN DEFINITIONS — Adjust these to match YOUR wiring
// ================================================================

// --- L298N Motor Driver ---
// Keep ENA/ENB jumpers on L298N enabled; PWM is applied on IN pins.
// D8 is a boot-strap pin on ESP8266; keep L298N input pull levels sane at boot.
const int IN1 = D5;    // Left motor direction/PWM
const int IN2 = D6;    // Left motor direction/PWM
const int IN3 = D7;    // Right motor direction/PWM
const int IN4 = D8;    // Right motor direction/PWM

// --- Encoder Pins ---
// ESP8266 pin budget is tight; this sketch uses channel A only.
const int ENC_L_A = D1;
const int ENC_R_A = D2;

// --- Ultrasonic Sensor Pins (one-pin mode per sensor) ---
// Tie each sensor Trig and Echo with ~2.2k resistor and connect to one GPIO.
// Keep ECHO voltage 3.3V-safe for ESP8266 (divider/level shifting as needed).
const int SONAR_FRONT = D0;
const int SONAR_LEFT  = D3;
const int SONAR_RIGHT = D4;

// --- Battery Voltage Monitoring (via voltage divider) ---
const int VBAT_PIN = A0;
const float VBAT_DIVIDER_RATIO = 2.0; // *** ADJUST to your divider ***
const float LOW_BATTERY_VOLTAGE = 6.0; // 2S LiPo cutoff (~3.0V/cell)

// --- ESP8266 PWM Configuration ---
const int PWM_FREQ       = 20000; // 20 kHz — silent operation
const int PWM_MAX  = 255;

// ================================================================
//  CALIBRATION VALUES — *** MUST MEASURE & ADJUST AFTER TESTING ***
// ================================================================

// Encoder ticks for one maze cell (18 cm) — drive straight, read count
long TICKS_PER_CELL    = 360;
// Encoder ticks for an in-place 90° turn — spin in place, read count
long TICKS_PER_90_DEG  = 190;

// Ultrasonic distance thresholds (cm)
const float WALL_THRESHOLD_CM  = 8.0;  // Below this → wall present
const float SIDE_CENTER_DIST   = 5.5;  // Ideal side-wall distance when centered

// Minimum PWM where your N20 motors actually start moving
const int MOTOR_MIN_PWM = 45;

// PID gains for wheel-distance control (tune Kp first, then Kd, then Ki)
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.8;

// PID gains for side-wall steering correction during straight moves
float steerKp = 3.0;
float steerKd = 1.0;

// Speeds
const int BASE_SPEED      = 120; // Exploration (0-255)
const int SPEED_RUN_SPEED = 200; // Fast run   (0-255)

// Timeout for any single PID move (ms) — safety stop
const unsigned long MOVE_TIMEOUT_MS = 3000;

// ================================================================
//  MAZE CONSTANTS & DATA
// ================================================================
#define MAZE_SIZE 16

#define WALL_NORTH 1
#define WALL_EAST  2
#define WALL_SOUTH 4
#define WALL_WEST  8

uint8_t walls[MAZE_SIZE][MAZE_SIZE];
int     distances[MAZE_SIZE][MAZE_SIZE];
bool    visited[MAZE_SIZE][MAZE_SIZE];

// Direction look-up tables: N=0, E=1, S=2, W=3
const int     DX[]        = { 0,  1,  0, -1};
const int     DY[]        = { 1,  0, -1,  0};
const uint8_t WALL_BITS[] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};

// ================================================================
//  ROBOT STATE
// ================================================================
int posX    = 0;  // Current cell X
int posY    = 0;  // Current cell Y
int heading = 0;  // 0=N  1=E  2=S  3=W

// Speed-run pre-computed path
int speedRunPath[256];
int speedRunLength = 0;

// ================================================================
//  ENCODER VARIABLES
// ================================================================
volatile long countLeft  = 0;
volatile long countRight = 0;
volatile int8_t dirLeft  = 1;
volatile int8_t dirRight = 1;

// ================================================================
//  FLOODFILL QUEUE (static circular buffer)
// ================================================================
struct Cell { int x, y; };
const int Q_CAPACITY = MAZE_SIZE * MAZE_SIZE;
Cell qBuf[Q_CAPACITY];
bool qQueued[MAZE_SIZE][MAZE_SIZE];
int  qHead = 0, qTail = 0, qCount = 0;
bool qOverflowed = false;

void qClear() {
    qHead = 0;
    qTail = 0;
    qCount = 0;
    qOverflowed = false;
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            qQueued[x][y] = false;
}

void qPush(int x, int y) {
    if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return;
    if (qQueued[x][y]) return;
    if (qCount >= Q_CAPACITY) {
        qOverflowed = true;
        return;
    }
    qBuf[qTail] = {x, y};
    qTail = (qTail + 1) % Q_CAPACITY;
    qQueued[x][y] = true;
    qCount++;
}
Cell qPop() {
    if (qCount == 0) return {0, 0};
    Cell c = qBuf[qHead];
    qHead = (qHead + 1) % Q_CAPACITY;
    qCount--;
    qQueued[c.x][c.y] = false;
    return c;
}
bool qEmpty() { return qCount == 0; }

// ================================================================
//  INTERRUPT SERVICE ROUTINES — Quadrature Encoders
// ================================================================
#define ISR_ATTR IRAM_ATTR

void ISR_ATTR isrLeft()  { countLeft  += dirLeft; }
void ISR_ATTR isrRight() { countRight += dirRight; }

// ================================================================
//  MOTOR CONTROL (L298N + analogWrite PWM)
// ================================================================
void setMotorLeft(int speed) {
    if (speed > 0 && speed < MOTOR_MIN_PWM) speed = MOTOR_MIN_PWM;
    if (speed < 0 && speed > -MOTOR_MIN_PWM) speed = -MOTOR_MIN_PWM;
    speed = constrain(speed, -PWM_MAX, PWM_MAX);

    if (speed > 0) {
        dirLeft = 1;
        analogWrite(IN2, 0);
        analogWrite(IN1, speed);
    } else if (speed < 0) {
        dirLeft = -1;
        analogWrite(IN1, 0);
        analogWrite(IN2, -speed);
    } else {
        dirLeft = 0;
        analogWrite(IN1, 0);
        analogWrite(IN2, 0);
    }
}

void setMotorRight(int speed) {
    if (speed > 0 && speed < MOTOR_MIN_PWM) speed = MOTOR_MIN_PWM;
    if (speed < 0 && speed > -MOTOR_MIN_PWM) speed = -MOTOR_MIN_PWM;
    speed = constrain(speed, -PWM_MAX, PWM_MAX);

    if (speed > 0) {
        dirRight = 1;
        analogWrite(IN4, 0);
        analogWrite(IN3, speed);
    } else if (speed < 0) {
        dirRight = -1;
        analogWrite(IN3, 0);
        analogWrite(IN4, -speed);
    } else {
        dirRight = 0;
        analogWrite(IN3, 0);
        analogWrite(IN4, 0);
    }
}

void stopMotors() {
    setMotorLeft(0);
    setMotorRight(0);
}

// ================================================================
//  ULTRASONIC SENSORS
// ================================================================
float readUltrasonicOnePin(int ioPin) {
    pinMode(ioPin, OUTPUT);
    digitalWrite(ioPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ioPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(ioPin, LOW);
    pinMode(ioPin, INPUT);
    long duration = pulseIn(ioPin, HIGH, 15000); // 15 ms timeout
    if (duration == 0) return 999.0;               // No echo → no wall
    return duration * 0.0343 / 2.0;                // → centimeters
}

// Median of 3 readings — eliminates noise spikes
float readUltrasonicFiltered(int ioPin) {
    float r[3];
    for (int i = 0; i < 3; i++) {
        r[i] = readUltrasonicOnePin(ioPin);
        delayMicroseconds(200);
    }
    // Simple bubble sort for 3 elements → return median
    if (r[0] > r[1]) { float t = r[0]; r[0] = r[1]; r[1] = t; }
    if (r[1] > r[2]) { float t = r[1]; r[1] = r[2]; r[2] = t; }
    if (r[0] > r[1]) { float t = r[0]; r[0] = r[1]; r[1] = t; }
    return r[1];
}

float readFront() { return readUltrasonicFiltered(SONAR_FRONT); }
float readLeft()  { return readUltrasonicFiltered(SONAR_LEFT);  }
float readRight() { return readUltrasonicFiltered(SONAR_RIGHT); }

// ================================================================
//  BATTERY MONITORING
// ================================================================
float readBatteryVoltage() {
    int raw = analogRead(VBAT_PIN);
    return (raw / 1023.0) * 3.3 * VBAT_DIVIDER_RATIO;
}

bool isBatteryLow() {
    return readBatteryVoltage() < LOW_BATTERY_VOLTAGE;
}

// ================================================================
//  MAZE INITIALIZATION
// ================================================================
void initMaze() {
    qClear();
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            walls[x][y]    = 0;
            visited[x][y]  = false;
            // Manhattan distance to nearest center cell (7,7)-(8,8)
            int dx = max(0, max(7 - x, x - 8));
            int dy = max(0, max(7 - y, y - 8));
            distances[x][y] = dx + dy;
        }
    }
    // Outer border walls are always present
    for (int i = 0; i < MAZE_SIZE; i++) {
        walls[i][0]             |= WALL_SOUTH;
        walls[i][MAZE_SIZE - 1] |= WALL_NORTH;
        walls[0][i]             |= WALL_WEST;
        walls[MAZE_SIZE - 1][i] |= WALL_EAST;
    }
    // Starting cell (0,0) has known south + west walls
    walls[0][0] |= (WALL_SOUTH | WALL_WEST);
}

// ================================================================
//  FLOODFILL ALGORITHM
// ================================================================
int getMinNeighborDist(int x, int y) {
    int md = 999;
    if (!(walls[x][y] & WALL_NORTH) && y < MAZE_SIZE - 1) md = min(md, distances[x][y + 1]);
    if (!(walls[x][y] & WALL_EAST)  && x < MAZE_SIZE - 1) md = min(md, distances[x + 1][y]);
    if (!(walls[x][y] & WALL_SOUTH) && y > 0)             md = min(md, distances[x][y - 1]);
    if (!(walls[x][y] & WALL_WEST)  && x > 0)             md = min(md, distances[x - 1][y]);
    return md;
}

// Propagate distance updates after a wall change
void updateDistances() {
    while (!qEmpty()) {
        Cell c = qPop();
        if ((c.x == 7 || c.x == 8) && (c.y == 7 || c.y == 8)) continue;
        int mn = getMinNeighborDist(c.x, c.y);
        if (distances[c.x][c.y] != mn + 1) {
            distances[c.x][c.y] = mn + 1;
            if (!(walls[c.x][c.y] & WALL_NORTH) && c.y < MAZE_SIZE - 1) qPush(c.x, c.y + 1);
            if (!(walls[c.x][c.y] & WALL_EAST)  && c.x < MAZE_SIZE - 1) qPush(c.x + 1, c.y);
            if (!(walls[c.x][c.y] & WALL_SOUTH) && c.y > 0)             qPush(c.x, c.y - 1);
            if (!(walls[c.x][c.y] & WALL_WEST)  && c.x > 0)             qPush(c.x - 1, c.y);
        }
    }
    if (qOverflowed) {
        Serial.println("! Flood queue overflow; run a full flood to recover");
        qOverflowed = false;
    }
}

// Record a newly-detected wall and trigger floodfill cascade
void addWall(int x, int y, uint8_t wallType) {
    if (walls[x][y] & wallType) return; // already known
    walls[x][y] |= wallType;
    if (wallType == WALL_NORTH && y < MAZE_SIZE - 1) walls[x][y + 1] |= WALL_SOUTH;
    if (wallType == WALL_EAST  && x < MAZE_SIZE - 1) walls[x + 1][y] |= WALL_WEST;
    if (wallType == WALL_SOUTH && y > 0)             walls[x][y - 1] |= WALL_NORTH;
    if (wallType == WALL_WEST  && x > 0)             walls[x - 1][y] |= WALL_EAST;
    qPush(x, y);
    updateDistances();
}

// Full BFS re-flood from a rectangular target region
void floodFrom(int x1, int y1, int x2, int y2) {
    qClear();
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            distances[x][y] = 999;
    for (int x = x1; x <= x2; x++)
        for (int y = y1; y <= y2; y++) {
            distances[x][y] = 0;
            qPush(x, y);
        }
    while (!qEmpty()) {
        Cell c = qPop();
        int nd = distances[c.x][c.y] + 1;
        if (!(walls[c.x][c.y] & WALL_NORTH) && c.y < MAZE_SIZE - 1 && distances[c.x][c.y + 1] > nd) {
            distances[c.x][c.y + 1] = nd; qPush(c.x, c.y + 1);
        }
        if (!(walls[c.x][c.y] & WALL_EAST) && c.x < MAZE_SIZE - 1 && distances[c.x + 1][c.y] > nd) {
            distances[c.x + 1][c.y] = nd; qPush(c.x + 1, c.y);
        }
        if (!(walls[c.x][c.y] & WALL_SOUTH) && c.y > 0 && distances[c.x][c.y - 1] > nd) {
            distances[c.x][c.y - 1] = nd; qPush(c.x, c.y - 1);
        }
        if (!(walls[c.x][c.y] & WALL_WEST) && c.x > 0 && distances[c.x - 1][c.y] > nd) {
            distances[c.x - 1][c.y] = nd; qPush(c.x - 1, c.y);
        }
    }
    if (qOverflowed) {
        Serial.println("! Flood queue overflow during BFS");
        qOverflowed = false;
    }
}

void floodToCenter() { floodFrom(7, 7, 8, 8); }
void floodToStart()  { floodFrom(0, 0, 0, 0); }

// ================================================================
//  SENSOR → WALL MAPPING
// ================================================================
void scanAndUpdateWalls(int x, int y, int dir) {
    float front = readFront();
    float left  = readLeft();
    float right = readRight();

    Serial.printf("  Sensors  F:%.1f  L:%.1f  R:%.1f cm\n", front, left, right);

    if (front < WALL_THRESHOLD_CM) addWall(x, y, WALL_BITS[dir]);
    if (left  < WALL_THRESHOLD_CM) addWall(x, y, WALL_BITS[(dir + 3) % 4]);
    if (right < WALL_THRESHOLD_CM) addWall(x, y, WALL_BITS[(dir + 1) % 4]);
}

// ================================================================
//  PID MOVEMENT
// ================================================================

// Generic PID move — different tick targets per wheel (used for turns)
void moveWithPID(long targetL, long targetR) {
    countLeft = 0;
    countRight = 0;

    long errL = 0, errR = 0;
    long lastErrL = 0, lastErrR = 0;
    long intL = 0, intR = 0;

    unsigned long t0 = millis();

    while (abs(targetL - countLeft) > 3 || abs(targetR - countRight) > 3) {
        if (millis() - t0 > MOVE_TIMEOUT_MS) {
            Serial.println("! PID TIMEOUT");
            stopMotors();
            return;
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
}

// Move forward one cell with side-wall steering correction
void moveForwardWithSteering(int baseSpeed) {
    countLeft  = 0;
    countRight = 0;
    float lastSteerErr = 0;
    unsigned long t0 = millis();

    while (abs(TICKS_PER_CELL - countLeft) > 3 || abs(TICKS_PER_CELL - countRight) > 3) {
        if (millis() - t0 > MOVE_TIMEOUT_MS) {
            Serial.println("! Steering TIMEOUT");
            stopMotors();
            return;
        }

        // --- Steering PID from side sensors ---
        float ld = readLeft();
        float rd = readRight();
        float steerErr = 0;

        bool hasL = (ld < WALL_THRESHOLD_CM * 1.5);
        bool hasR = (rd < WALL_THRESHOLD_CM * 1.5);

        if (hasL && hasR)      steerErr = ld - rd;                    // center between walls
        else if (hasL)         steerErr = ld - SIDE_CENTER_DIST;      // hold left distance
        else if (hasR)         steerErr = SIDE_CENTER_DIST - rd;      // hold right distance
        // else: no side walls → rely on encoders only

        float correction = steerKp * steerErr + steerKd * (steerErr - lastSteerErr);
        lastSteerErr = steerErr;

        // Slow down near target (trapezoidal-ish)
        long remaining = (TICKS_PER_CELL - countLeft + TICKS_PER_CELL - countRight) / 2;
        int speed = baseSpeed;
        if (remaining < TICKS_PER_CELL / 4) speed = max(MOTOR_MIN_PWM, baseSpeed / 2);

        int spdL = constrain(speed + (int)correction, 0, 255);
        int spdR = constrain(speed - (int)correction, 0, 255);

        setMotorLeft(spdL);
        setMotorRight(spdR);
        delay(10);
    }
    stopMotors();
}

void moveForwardOneCell() { moveForwardWithSteering(BASE_SPEED); }

void turnLeft90()  { moveWithPID(-TICKS_PER_90_DEG, TICKS_PER_90_DEG); }
void turnRight90() { moveWithPID( TICKS_PER_90_DEG, -TICKS_PER_90_DEG); }
void turnAround()  { moveWithPID( TICKS_PER_90_DEG * 2, -TICKS_PER_90_DEG * 2); }

// ================================================================
//  HEADING & NAVIGATION HELPERS
// ================================================================
void turnToHeading(int target) {
    int diff = (target - heading + 4) % 4;
    if      (diff == 1) turnRight90();
    else if (diff == 2) turnAround();
    else if (diff == 3) turnLeft90();
    heading = target;
}

int getBestDirection(int x, int y) {
    int bestDir = -1, bestDist = 999;
    for (int d = 0; d < 4; d++) {
        if (walls[x][y] & WALL_BITS[d]) continue;
        int nx = x + DX[d], ny = y + DY[d];
        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
        if (distances[nx][ny] < bestDist) {
            bestDist = distances[nx][ny];
            bestDir  = d;
        }
    }
    return bestDir;
}

bool isAtCenter() { return (posX == 7 || posX == 8) && (posY == 7 || posY == 8); }
bool isAtStart()  { return posX == 0 && posY == 0; }

// ================================================================
//  HIGH-LEVEL NAVIGATION
// ================================================================
void navigateToCenter() {
    Serial.println("=== EXPLORING TO CENTER ===");
    while (!isAtCenter()) {
        if (isBatteryLow()) {
            Serial.println("!!! LOW BATTERY — HALTING !!!");
            stopMotors();
            while (1) delay(1000);
        }

        visited[posX][posY] = true;
        scanAndUpdateWalls(posX, posY, heading);

        int bestDir = getBestDirection(posX, posY);
        if (bestDir == -1) {
            Serial.println("! No path — stuck!");
            stopMotors();
            return;
        }

        Serial.printf("(%d,%d) h=%d -> dir=%d  dist=%d\n",
                       posX, posY, heading, bestDir, distances[posX][posY]);

        turnToHeading(bestDir);
        moveForwardOneCell();
        posX += DX[bestDir];
        posY += DY[bestDir];
    }
    Serial.println("=== REACHED CENTER ===");
}

void navigateToStart() {
    Serial.println("=== RETURNING TO START ===");
    floodToStart();
    while (!isAtStart()) {
        scanAndUpdateWalls(posX, posY, heading);
        int bestDir = getBestDirection(posX, posY);
        if (bestDir == -1) { stopMotors(); return; }
        turnToHeading(bestDir);
        moveForwardOneCell();
        posX += DX[bestDir];
        posY += DY[bestDir];
    }
    Serial.println("=== BACK AT START ===");
}

// ================================================================
//  SPEED RUN
// ================================================================
void computeSpeedRunPath() {
    floodToCenter();
    speedRunLength = 0;
    int sx = 0, sy = 0;
    while (!((sx == 7 || sx == 8) && (sy == 7 || sy == 8))) {
        int bestDir = -1, bestDist = 999;
        for (int d = 0; d < 4; d++) {
            if (walls[sx][sy] & WALL_BITS[d]) continue;
            int nx = sx + DX[d], ny = sy + DY[d];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
            if (distances[nx][ny] < bestDist) {
                bestDist = distances[nx][ny];
                bestDir  = d;
            }
        }
        if (bestDir == -1) break;
        speedRunPath[speedRunLength++] = bestDir;
        sx += DX[bestDir];
        sy += DY[bestDir];
    }
    Serial.printf("Speed-run path: %d moves\n", speedRunLength);
}

void executeSpeedRun() {
    Serial.println("=== SPEED RUN ===");
    for (int i = 0; i < speedRunLength; i++) {
        turnToHeading(speedRunPath[i]);
        moveForwardWithSteering(SPEED_RUN_SPEED);
        posX += DX[speedRunPath[i]];
        posY += DY[speedRunPath[i]];
    }
    Serial.println("=== SPEED RUN COMPLETE ===");
}

// ================================================================
//  DEBUG UTILITIES
// ================================================================
void printMazeDistances() {
    Serial.println("\n--- Distances ---");
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_SIZE; x++)
            Serial.printf("%3d", distances[x][y]);
        Serial.println();
    }
}

void printMazeWalls() {
    Serial.println("\n--- Walls (hex) ---");
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_SIZE; x++)
            Serial.printf(" %X", walls[x][y]);
        Serial.println();
    }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Micromouse Initializing...");
    Serial.println("ESP8266 mode: IN-pin PWM, encoder A-channel only, one-pin sonar.");

    // Motor driver pins
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    analogWriteRange(PWM_MAX);
    analogWriteFreq(PWM_FREQ);
    analogWrite(IN1, 0); analogWrite(IN2, 0);
    analogWrite(IN3, 0); analogWrite(IN4, 0);


    // Encoder pins (A channel only in ESP8266 mode)
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);

    // Sonar one-pin lines idle as input; read function toggles direction.
    pinMode(SONAR_FRONT, INPUT);
    pinMode(SONAR_LEFT,  INPUT);
    pinMode(SONAR_RIGHT, INPUT);

    // Maze
    initMaze();

    Serial.printf("Battery: %.2fV\n", readBatteryVoltage());
    Serial.println("Ready — starting in 3 seconds...");
    delay(3000);
}

// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {
    // Phase 1: Explore maze to the center
    navigateToCenter();
    delay(1000);

    // Phase 2: Return to start (continue mapping walls)
    navigateToStart();
    delay(1000);

    // Phase 3: Compute shortest path & execute speed run
    computeSpeedRunPath();
    posX = 0; posY = 0; heading = 0;
    executeSpeedRun();

    // Done
    Serial.println("=== ALL DONE ===");
    printMazeDistances();
    printMazeWalls();
    while (1) delay(1000);
}
