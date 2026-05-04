#include <WiFi.h>
#include <WebServer.h>

// ================= WIFI =================
const char* ssid = "Pluto";
const char* password = "12345678";

WebServer server(80);

// Keep web server responsive while motors run.
bool motionActive = false;
unsigned long motionStopAt = 0;
int currentPwm = 200;

volatile long encCountR1 = 0;
volatile long encCountR2 = 0;
volatile long encCountF1 = 0;
volatile long encCountF2 = 0;

float rpmR1 = 0.0f;
float rpmR2 = 0.0f;
float rpmF1 = 0.0f;
float rpmF2 = 0.0f;

long lastCountR1 = 0;
long lastCountR2 = 0;
long lastCountF1 = 0;
long lastCountF2 = 0;
unsigned long lastRpmMs = 0;
const unsigned long RPM_UPDATE_INTERVAL_MS = 250;

// ===================== YOUR PINS =====================
// (unchanged)
#define EN_R1 32
#define IN1_R1 33
#define IN2_R1 25
#define ENC_R1_A 35
#define ENC_R1_B 34

#define EN_R2 14
#define IN1_R2 26
#define IN2_R2 27
#define ENC_R2_A 39
#define ENC_R2_B 36

#define EN_F1 13
#define IN1_F1 2
#define IN2_F1 4
#define ENC_F1_A 16
#define ENC_F1_B 17

#define EN_F2 5
#define IN1_F2 23
#define IN2_F2 15
#define ENC_F2_A 18
#define ENC_F2_B 19

#define PPR 745

// ===================== HTML PAGE =====================
String webpage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32 Motor Control</title>
  <style>
    :root {
      --bg-a: #e8f4ff;
      --bg-b: #fff4de;
      --card: #ffffff;
      --ink: #1a2a3a;
      --muted: #5d6d7e;
      --accent: #ff7a18;
      --accent-2: #0ea5e9;
      --line: #dbe7f4;
      --radius: 18px;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 20px;
      color: var(--ink);
      font-family: "Trebuchet MS", "Segoe UI", Tahoma, sans-serif;
      background:
        radial-gradient(circle at 15% 20%, #ffffff 0 20%, transparent 45%),
        radial-gradient(circle at 85% 80%, #ffe8c9 0 16%, transparent 40%),
        linear-gradient(135deg, var(--bg-a), var(--bg-b));
    }
    .card {
      width: min(460px, 100%);
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: var(--radius);
      padding: 24px;
      box-shadow: 0 10px 30px rgba(26, 42, 58, 0.12);
    }
    h2 {
      margin: 0 0 8px;
      font-size: 1.6rem;
      letter-spacing: 0.2px;
    }
    p {
      margin: 0 0 18px;
      color: var(--muted);
      font-size: 0.95rem;
    }
    .field {
      text-align: left;
      margin-bottom: 14px;
    }
    label {
      display: block;
      margin-bottom: 6px;
      font-weight: 700;
      font-size: 0.92rem;
    }
    select, input[type="number"] {
      width: 100%;
      border: 1px solid #c7d8ea;
      border-radius: 12px;
      padding: 10px 12px;
      font-size: 1rem;
      background: #f9fcff;
      color: var(--ink);
    }
    input[type="range"] {
      width: 100%;
      accent-color: var(--accent-2);
    }
    .pwm-row {
      display: grid;
      grid-template-columns: 1fr 100px;
      gap: 10px;
      align-items: center;
    }
    .run-btn {
      width: 100%;
      border: 0;
      border-radius: 12px;
      padding: 12px;
      margin-top: 8px;
      font-size: 1rem;
      font-weight: 800;
      color: #fff;
      background: linear-gradient(90deg, var(--accent), #ff9f43);
      cursor: pointer;
    }
    .run-btn:active { transform: translateY(1px); }
    .meta {
      margin-top: 12px;
      font-size: 0.82rem;
      color: var(--muted);
      text-align: left;
    }
    .rpm-box {
      margin-top: 16px;
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 12px;
      background: #f7fbff;
    }
    .rpm-title {
      margin: 0 0 8px;
      font-size: 0.95rem;
      font-weight: 800;
      text-align: left;
    }
    .rpm-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      text-align: left;
      font-size: 0.92rem;
      color: #2a3b4c;
    }
  </style>
</head>
<body>
  <div class="card">
    <h2>Pluto Drive Control</h2>
    <p>Run motors by direction, time, and PWM speed.</p>

    <form action="/run">
      <div class="field">
        <label for="dir">Direction</label>
        <select id="dir" name="dir">
          <option value="f">Forward</option>
          <option value="b">Backward</option>
        </select>
      </div>

      <div class="field">
        <label for="time">Time (seconds)</label>
        <input id="time" type="number" name="time" value="2" min="1" max="60">
      </div>

      <div class="field">
        <label for="pwmRange">PWM (0-255)</label>
        <div class="pwm-row">
           <input id="pwmRange" type="range" min="0" max="255" value="200"
             oninput="document.getElementById('pwm').value=this.value">
           <input id="pwm" type="number" name="pwm" min="0" max="255" value="200"
             oninput="document.getElementById('pwmRange').value=this.value">
        </div>
      </div>

      <button class="run-btn" type="submit">RUN</button>
    </form>

    <div class="rpm-box">
      <div class="rpm-title">Live Wheel RPM</div>
      <div class="rpm-grid">
        <div>R1: <span id="rpmR1">0.00</span></div>
        <div>R2: <span id="rpmR2">0.00</span></div>
        <div>F1: <span id="rpmF1">0.00</span></div>
        <div>F2: <span id="rpmF2">0.00</span></div>
      </div>
    </div>

    <div class="meta">Connect to Pluto and open 192.168.4.1</div>
  </div>

  <script>
    async function updateRpm() {
      try {
        const res = await fetch('/rpm');
        const data = await res.json();
        document.getElementById('rpmR1').textContent = Number(data.r1).toFixed(2);
        document.getElementById('rpmR2').textContent = Number(data.r2).toFixed(2);
        document.getElementById('rpmF1').textContent = Number(data.f1).toFixed(2);
        document.getElementById('rpmF2').textContent = Number(data.f2).toFixed(2);
      } catch (e) {
        // Ignore transient request errors while AP reconnects.
      }
    }
    setInterval(updateRpm, 500);
    updateRpm();
  </script>
</body>
</html>
)rawliteral";

void IRAM_ATTR isrEncR1A() { encCountR1++; }
void IRAM_ATTR isrEncR2A() { encCountR2++; }
void IRAM_ATTR isrEncF1A() { encCountF1++; }
void IRAM_ATTR isrEncF2A() { encCountF2++; }

void updateRpmValues() {
  unsigned long now = millis();
  if (now - lastRpmMs < RPM_UPDATE_INTERVAL_MS) return;

  float dtMs = (float)(now - lastRpmMs);
  lastRpmMs = now;

  noInterrupts();
  long cR1 = encCountR1;
  long cR2 = encCountR2;
  long cF1 = encCountF1;
  long cF2 = encCountF2;
  interrupts();

  long dR1 = cR1 - lastCountR1;
  long dR2 = cR2 - lastCountR2;
  long dF1 = cF1 - lastCountF1;
  long dF2 = cF2 - lastCountF2;

  lastCountR1 = cR1;
  lastCountR2 = cR2;
  lastCountF1 = cF1;
  lastCountF2 = cF2;

  float scale = (60000.0f / dtMs) / (float)PPR;
  rpmR1 = dR1 * scale;
  rpmR2 = dR2 * scale;
  rpmF1 = dF1 * scale;
  rpmF2 = dF2 * scale;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // Motor pins
  int pins[] = {EN_R1,IN1_R1,IN2_R1, EN_R2,IN1_R2,IN2_R2,
                EN_F1,IN1_F1,IN2_F1, EN_F2,IN1_F2,IN2_F2};

  for(int i=0;i<12;i++) pinMode(pins[i], OUTPUT);

  pinMode(ENC_R1_A, INPUT);
  pinMode(ENC_R2_A, INPUT);
  pinMode(ENC_F1_A, INPUT);
  pinMode(ENC_F2_A, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_R1_A), isrEncR1A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R2_A), isrEncR2A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_F1_A), isrEncF1A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_F2_A), isrEncF2A, RISING);

  analogWrite(EN_R1, 0);
  analogWrite(EN_R2, 0);
  analogWrite(EN_F1, 0);
  analogWrite(EN_F2, 0);

  lastRpmMs = millis();

  allStop();

  // WiFi Access Point (ESP32 hosts the network)
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(ssid, password);

  if (apStarted) {
    Serial.println("\nAccess Point started");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("\nFailed to start Access Point");
  }

  // Routes
  server.on("/", [](){
    server.send(200, "text/html", webpage);
  });

  server.on("/run", handleRun);
  server.on("/rpm", [](){
    String json = "{\"r1\":" + String(rpmR1, 2) +
                  ",\"r2\":" + String(rpmR2, 2) +
                  ",\"f1\":" + String(rpmF1, 2) +
                  ",\"f2\":" + String(rpmF2, 2) + "}";
    server.send(200, "application/json", json);
  });

  server.begin();
}

// ===================== LOOP =====================
void loop() {
  server.handleClient();
  updateRpmValues();

  if (motionActive && millis() >= motionStopAt) {
    allStop();
    motionActive = false;
    Serial.println("Motion complete (auto stop)");
  }
}

// ===================== HANDLE COMMAND =====================
void handleRun() {
  String dir = server.arg("dir");
  int timeSec = server.arg("time").toInt();
  int pwm = server.arg("pwm").toInt();
  if (timeSec < 1) timeSec = 1;
  pwm = constrain(pwm, 0, 255);
  currentPwm = pwm;

  Serial.println("Command received");
  Serial.print("PWM: ");
  Serial.println(currentPwm);

  if(dir == "f"){
    Serial.println("FORWARD");
    setMotors(LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,currentPwm);
  }
  else {
    Serial.println("BACKWARD");
    setMotors(HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,currentPwm);
  }

  motionStopAt = millis() + ((unsigned long)timeSec * 1000UL);
  motionActive = true;

  server.send(200, "text/html", webpage);
}

// ===================== MOTOR FUNCTIONS =====================
void setMotors(int r1a,int r1b,int r2a,int r2b,int f1a,int f1b,int f2a,int f2b,int pwm){
  digitalWrite(IN1_R1,r1a); digitalWrite(IN2_R1,r1b);
  digitalWrite(IN1_R2,r2a); digitalWrite(IN2_R2,r2b);
  digitalWrite(IN1_F1,f1a); digitalWrite(IN2_F1,f1b);
  digitalWrite(IN1_F2,f2a); digitalWrite(IN2_F2,f2b);

  analogWrite(EN_R1, pwm);
  analogWrite(EN_R2, pwm);
  analogWrite(EN_F1, pwm);
  analogWrite(EN_F2, pwm);
}

void allStop(){
  analogWrite(EN_R1, 0);
  analogWrite(EN_R2, 0);
  analogWrite(EN_F1, 0);
  analogWrite(EN_F2, 0);
}