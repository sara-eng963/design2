// ===================== MOTOR =====================
#define EN_M   13
#define IN1_M  4
#define IN2_M  16
#define ENC_A  12
#define ENC_B  15

#define PPR 745

// ===================== COUNTER =====================
volatile long cM = 0;

unsigned long lastTime = 0;
long lastCount = 0;

// ===================== ISR =====================
void IRAM_ATTR encM() {
  cM += (digitalRead(ENC_A) == digitalRead(ENC_B)) ? 1 : -1;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(EN_M, OUTPUT);
  pinMode(IN1_M, OUTPUT);
  pinMode(IN2_M, OUTPUT);

  // Force OFF
  digitalWrite(EN_M, LOW);
  digitalWrite(IN1_M, LOW);
  digitalWrite(IN2_M, LOW);

  // Encoder pins
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encM, CHANGE);

  Serial.println("MOTOR TEST STARTED");
}

// ===================== LOOP =====================
void loop() {

  // ========== FORWARD ==========
  Serial.println("FORWARD");
  setMotor(LOW, HIGH);
  run(2000);

  // ========== STOP ==========
  Serial.println("STOP");
  allStop();
  delay(1000);

  // ========== BACKWARD ==========
  Serial.println("BACKWARD");
  setMotor(HIGH, LOW);
  run(2000);

  Serial.println("STOP");
  allStop();
  delay(2000);
}

// ===================== MOTOR HELPERS =====================
void setMotor(int a, int b) {
  digitalWrite(EN_M, HIGH);
  digitalWrite(IN1_M, a);
  digitalWrite(IN2_M, b);
}

void allStop() {
  digitalWrite(EN_M, LOW);
}

// ===================== RUN + RPM =====================
void run(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    rpmPrint();
  }
}

void rpmPrint() {
  unsigned long now = millis();

  if (now - lastTime >= 1000) {

    long delta = cM - lastCount;

    float rpm = abs(delta) / (float)PPR * 60;

    Serial.print("RPM: ");
    Serial.println(rpm);

    lastCount = cM;
    lastTime = now;
  }
}