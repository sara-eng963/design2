// ===================== R1 =====================
#define EN_R1 32
#define IN1_R1 33
#define IN2_R1 25

#define ENC_R1_A 35
#define ENC_R1_B 34

// ===================== R2 =====================
#define EN_R2 14
#define IN1_R2 26
#define IN2_R2 27

#define ENC_R2_A 39
#define ENC_R2_B 36

#define PPR 745

// ===================== OTHER MOTORS (OFF) =====================
#define EN_F1 13
#define IN1_F1 2
#define IN2_F1 4

#define EN_F2 5
#define IN1_F2 23
#define IN2_F2 15

// ===================== ENCODERS =====================
volatile long countR1 = 0;
volatile long countR2 = 0;

unsigned long lastTime = 0;
long lastR1 = 0;
long lastR2 = 0;

// ===================== ISR =====================
void IRAM_ATTR encR1() {
  if (digitalRead(ENC_R1_A) == digitalRead(ENC_R1_B)) countR1++;
  else countR1--;
}

void IRAM_ATTR encR2() {
  if (digitalRead(ENC_R2_A) == digitalRead(ENC_R2_B)) countR2++;
  else countR2--;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // ---- Motor pins ----
  pinMode(EN_R1, OUTPUT); pinMode(IN1_R1, OUTPUT); pinMode(IN2_R1, OUTPUT);
  pinMode(EN_R2, OUTPUT); pinMode(IN1_R2, OUTPUT); pinMode(IN2_R2, OUTPUT);

  pinMode(EN_F1, OUTPUT); pinMode(IN1_F1, OUTPUT); pinMode(IN2_F1, OUTPUT);
  pinMode(EN_F2, OUTPUT); pinMode(IN1_F2, OUTPUT); pinMode(IN2_F2, OUTPUT);

  // ---- FORCE ALL OFF ----
  digitalWrite(EN_F1, LOW);
  digitalWrite(EN_F2, LOW);
  digitalWrite(EN_R1, LOW);
  digitalWrite(EN_R2, LOW);

  digitalWrite(IN1_R1, LOW); digitalWrite(IN2_R1, LOW);
  digitalWrite(IN1_R2, LOW); digitalWrite(IN2_R2, LOW);

  digitalWrite(IN1_F1, LOW); digitalWrite(IN2_F1, LOW);
  digitalWrite(IN1_F2, LOW); digitalWrite(IN2_F2, LOW);

  // ---- Encoder pins ----
  pinMode(ENC_R1_A, INPUT);
  pinMode(ENC_R1_B, INPUT);
  pinMode(ENC_R2_A, INPUT);
  pinMode(ENC_R2_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_R1_A), encR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R2_A), encR2, CHANGE);

  Serial.println("R1 + R2 TEST STARTED");
}

// ===================== LOOP =====================
void loop() {

  // -------- FORWARD --------
  Serial.println("FORWARD");

  // R1
  digitalWrite(EN_R1, HIGH);
  digitalWrite(IN1_R1, LOW);
  digitalWrite(IN2_R1, HIGH);

  // R2
  digitalWrite(EN_R2, HIGH);
  digitalWrite(IN1_R2, LOW);
  digitalWrite(IN2_R2, HIGH);

  runTest(2000);

  // -------- STOP --------
  Serial.println("STOP");
  digitalWrite(EN_R1, LOW);
  digitalWrite(EN_R2, LOW);
  delay(1000);

  // -------- BACKWARD --------
  Serial.println("BACKWARD");

  digitalWrite(EN_R1, HIGH);
  digitalWrite(IN1_R1, HIGH);
  digitalWrite(IN2_R1, LOW);

  digitalWrite(EN_R2, HIGH);
  digitalWrite(IN1_R2, HIGH);
  digitalWrite(IN2_R2, LOW);

  runTest(2000);

  // -------- STOP --------
  Serial.println("STOP");
  digitalWrite(EN_R1, LOW);
  digitalWrite(EN_R2, LOW);
  delay(2000);
}

// ===================== RPM LOOP =====================
void runTest(unsigned long duration) {
  unsigned long start = millis();

  while (millis() - start < duration) {
    calculateRPM();
  }
}

// ===================== RPM =====================
void calculateRPM() {
  unsigned long now = millis();

  if (now - lastTime >= 1000) {

    long dR1 = countR1 - lastR1;
    long dR2 = countR2 - lastR2;

    float rpmR1 = abs(dR1) / (float)PPR * 60.0;
    float rpmR2 = abs(dR2) / (float)PPR * 60.0;

    Serial.print("R1 RPM: ");
    Serial.print(rpmR1);
    Serial.print(" | R2 RPM: ");
    Serial.println(rpmR2);

    lastR1 = countR1;
    lastR2 = countR2;
    lastTime = now;
  }
}