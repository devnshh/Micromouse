/*
 * ================================================================
 *  MOVE FORWARD — ESP32 + 2 Motors + Encoders + PID + WiFi Debug
 * ================================================================
 *  Drives forward for 5 seconds with PID correction,
 *  pauses 1 second, then repeats.
 *
 *  Live encoder ticks shown on a WiFi-hosted webpage.
 *  WiFi AP:   micromouse_debug
 *  Password:  wewillwinbro
 *  Open:      http://192.168.4.1
 * ================================================================
 */

#include <WebServer.h>
#include <WiFi.h>

// ================= WiFi AP SETTINGS =================
const char *AP_SSID = "micromouse_debug";
const char *AP_PASS = "wewillwinbro";

WebServer server(80);

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
const int ENC_L_A = 4;
const int ENC_L_B = 27;
const int ENC_R_A = 14;
const int ENC_R_B = 26;

// ================= CALIBRATION =================
const int DRIVE_PWM = 138; // Base PWM for driving forward (~4V from 7.4V)

// If the bot veers RIGHT when encoder diff is 0, INCREASE this slightly
// (e.g., 1.02) If the bot veers LEFT when encoder diff is 0, DECREASE this
// slightly (e.g., 0.98)
float STEERING_OFFSET = 1.00; // Perfect balance is 1.00

// ================= PID GAINS (for straight-line correction) =================
float Kp = 2.0;  // Proportional — increase if not correcting enough
float Ki = 0.05; // Integral — fixes steady-state drift, keep small
float Kd = 1.0;  // Derivative — dampens oscillation

// ================= ENCODER STATE =================
volatile long countLeft = 0;
volatile long countRight = 0;

void IRAM_ATTR isrLeftA() { countLeft += digitalRead(ENC_L_B) ? -1 : 1; }

void IRAM_ATTR isrRightA() { countRight += digitalRead(ENC_R_B) ? -1 : 1; }

// ================= WEB SERVER HANDLERS =================

void handleData() {
  long L, R;
  noInterrupts();
  L = countLeft;
  R = countRight;
  interrupts();

  String json = "{";
  json += "\"left\":" + String(L) + ",";
  json += "\"right\":" + String(R) + ",";
  json += "\"absLeft\":" + String(abs(L)) + ",";
  json += "\"absRight\":" + String(abs(R)) + ",";
  json += "\"diff\":" + String(abs(L) - abs(R));
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Micromouse Encoders</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: 'Segoe UI', system-ui, sans-serif;
      background: #0f172a;
      color: #e2e8f0;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 24px 16px;
    }
    h1 {
      font-size: 1.5rem;
      font-weight: 700;
      margin-bottom: 6px;
      background: linear-gradient(135deg, #38bdf8, #818cf8);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
    }
    .subtitle { font-size: 0.85rem; color: #64748b; margin-bottom: 20px; }
    .status {
      font-size: 0.75rem; padding: 4px 12px; border-radius: 999px;
      margin-bottom: 16px; animation: pulse 2s infinite;
    }
    .status.ok { background: #064e3b; color: #6ee7b7; }
    .status.err { background: #7f1d1d; color: #fca5a5; animation: none; }
    @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.6} }
    .cards {
      display: flex; flex-wrap: wrap; gap: 14px;
      justify-content: center; width: 100%; max-width: 600px;
    }
    .card {
      background: #1e293b; border: 1px solid #334155;
      border-radius: 16px; padding: 18px 22px;
      min-width: 140px; flex: 1; text-align: center;
      transition: border-color 0.3s, box-shadow 0.3s;
    }
    .card:hover {
      border-color: #38bdf8;
      box-shadow: 0 0 20px rgba(56, 189, 248, 0.15);
    }
    .card .label {
      font-size: 0.75rem; text-transform: uppercase;
      letter-spacing: 1px; color: #94a3b8; margin-bottom: 6px;
    }
    .card .value { font-size: 2.2rem; font-weight: 700; }
    .card .sub { font-size: 0.75rem; color: #64748b; margin-top: 4px; }
    .card.left  .value { color: #a78bfa; }
    .card.right .value { color: #34d399; }
    .card.diff  .value { color: #38bdf8; }
    .diff-ok   .value { color: #34d399 !important; }
    .diff-warn .value { color: #fbbf24 !important; }
    .diff-bad  .value { color: #f87171 !important; }
    .log-section {
      margin-top: 20px; width: 100%; max-width: 600px;
    }
    .log-section h2 { font-size: 0.85rem; color: #94a3b8; margin-bottom: 6px; }
    #log {
      background: #1e293b; border: 1px solid #334155;
      border-radius: 12px; padding: 10px 14px;
      font-family: 'Courier New', monospace; font-size: 0.7rem;
      color: #94a3b8; max-height: 180px; overflow-y: auto; line-height: 1.5;
    }
  </style>
</head>
<body>
  <h1>Encoder Ticks</h1>
  <p class="subtitle">Live PID motor sync monitor</p>
  <div id="statusBadge" class="status ok">● Connected</div>

  <div class="cards">
    <div class="card left">
      <div class="label">Left Motor</div>
      <div class="value" id="vLeft">--</div>
      <div class="sub">raw: <span id="rawLeft">--</span></div>
    </div>
    <div class="card right">
      <div class="label">Right Motor</div>
      <div class="value" id="vRight">--</div>
      <div class="sub">raw: <span id="rawRight">--</span></div>
    </div>
    <div class="card diff" id="diffCard">
      <div class="label">Difference</div>
      <div class="value" id="vDiff">--</div>
      <div class="sub">|left| - |right|</div>
    </div>
  </div>

  <div class="log-section">
    <h2>Tick Log</h2>
    <div id="log"></div>
  </div>

  <script>
    const logEl = document.getElementById('log');
    const badge = document.getElementById('statusBadge');
    const diffCard = document.getElementById('diffCard');
    let errCount = 0;

    function addLog(msg) {
      const t = new Date().toLocaleTimeString();
      logEl.innerHTML += t + '  ' + msg + '\n';
      logEl.scrollTop = logEl.scrollHeight;
      const lines = logEl.innerHTML.split('\n');
      if (lines.length > 80) logEl.innerHTML = lines.slice(-80).join('\n');
    }

    function fetchData() {
      fetch('/data')
        .then(r => r.json())
        .then(d => {
          errCount = 0;
          badge.className = 'status ok';
          badge.textContent = '● Connected';

          document.getElementById('vLeft').textContent = d.absLeft;
          document.getElementById('vRight').textContent = d.absRight;
          document.getElementById('rawLeft').textContent = d.left;
          document.getElementById('rawRight').textContent = d.right;
          document.getElementById('vDiff').textContent = d.diff;

          // Color diff card based on sync quality
          let absDiff = Math.abs(d.diff);
          diffCard.className = 'card diff';
          if (absDiff <= 5) diffCard.classList.add('diff-ok');
          else if (absDiff <= 20) diffCard.classList.add('diff-warn');
          else diffCard.classList.add('diff-bad');

          addLog('L:' + d.absLeft + '  R:' + d.absRight + '  diff:' + d.diff);
        })
        .catch(() => {
          errCount++;
          if (errCount > 2) {
            badge.className = 'status err';
            badge.textContent = '● Disconnected';
          }
        });
    }

    setInterval(fetchData, 100);
    fetchData();
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

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
    // Handle web requests inside the loop so the page stays responsive
    server.handleClient();

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

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Move Forward Test (ESP32) ===");

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

  // Start WiFi Access Point
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.println();
  Serial.println("==============================");
  Serial.print("  WiFi AP: ");
  Serial.println(AP_SSID);
  Serial.print("  Pass:    ");
  Serial.println(AP_PASS);
  Serial.print("  IP:      ");
  Serial.println(ip);
  Serial.println("==============================");

  // Web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server running on port 80");

  Serial.println("Ready — starting in 2 seconds...");
  delay(2000);
}

// ================= MAIN LOOP =================
float integral = 0;
float lastError = 0;
bool motorsStarted = false;

void loop() {
  // Handle web requests
  server.handleClient();

  // Start motors once
  if (!motorsStarted) {
    // Reset encoder counts
    noInterrupts();
    countLeft = 0;
    countRight = 0;
    interrupts();

    // Set both motors to forward direction
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    Serial.println("Driving forward continuously (PID)...");
    motorsStarted = true;
  }

  // Read encoder counts safely
  long L, R;
  noInterrupts();
  L = countLeft;
  R = countRight;
  interrupts();

  // Use absolute values — encoders may count negative depending on wiring
  long absL = abs(L);
  long absR = abs(R);

  // Apply steering offset to trick the PID into favoring one side
  float biasedR = (float)absR * STEERING_OFFSET;

  // Error = difference in distance traveled (positive means left is ahead)
  float error = (float)absL - biasedR;

  // PID terms
  integral = constrain(integral + error, -500.0f, 500.0f);
  float derivative = error - lastError;

  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // Apply correction: slow down the faster motor, speed up the slower one
  int pwmLeft = constrain(DRIVE_PWM - (int)(correction / 2), 0, 255);
  int pwmRight = constrain(DRIVE_PWM + (int)(correction / 2), 0, 255);

  analogWrite(PWMA, pwmLeft);
  analogWrite(PWMB, pwmRight);

  delay(10); // PID update rate ~100Hz
}
