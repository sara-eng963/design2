// -------- MOTOR 1 --------
#define EN1 5
#define IN1_1 23
#define IN2_1 15

#define ENC1_A 18
#define ENC1_B 19

// -------- MOTOR 2 --------
#define EN2 13
#define IN1_2 2
#define IN2_2 4

#define ENC2_A 16
#define ENC2_B 17

#define PPR 745

// Encoder counts
volatile long count1 = 0;
volatile long count2 = 0;

// Timing
unsigned long lastTime = 0;
long lastCount1 = 0;
long lastCount2 = 0;

// -------- INTERRUPTS --------
void IRAM_ATTR handleEnc1() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) {
    count1++;
  } else {
    count1--;
  }
}

void IRAM_ATTR handleEnc2() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) {
    count2++;
  } else {
    count2--;
  }
}

void setup() {
  Serial.begin(115200);

  // Motor 1
  pinMode(EN1, OUTPUT);
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);

  // Motor 2
  pinMode(EN2, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);

  // Encoders
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), handleEnc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), handleEnc2, CHANGE);

  Serial.println("Dual Motor RPM Test Starting...");
}

void loop() {
  // ---- FORWARD ----
  Serial.println("FORWARD");

  // Motor 1
  digitalWrite(EN1, HIGH);
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);

  // Motor 2
  digitalWrite(EN2, HIGH);
  digitalWrite(IN1_2, LOW);
  digitalWrite(IN2_2, HIGH);

  unsigned long start = millis();
  while (millis() - start < 2000) {
    calculateRPM();
  }

  // ---- STOP ----
  Serial.println("STOP");
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  delay(1000);

  // ---- BACKWARD ----
  Serial.println("BACKWARD");

  // Motor 1
  digitalWrite(EN1, HIGH);
  digitalWrite(IN1_1, HIGH);
  digitalWrite(IN2_1, LOW);

  // Motor 2
  digitalWrite(EN2, HIGH);
  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);

  start = millis();
  while (millis() - start < 2000) {
    calculateRPM();
  }

  // ---- STOP ----
  Serial.println("STOP");
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  delay(1000);
}

void calculateRPM() {
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= 1000) {
    long c1 = count1;
    long c2 = count2;

    long d1 = c1 - lastCount1;
    long d2 = c2 - lastCount2;

    float rpm1 = abs(d1) / (float)PPR * 60.0;
    float rpm2 = abs(d2) / (float)PPR * 60.0;

    Serial.print("M1 RPM: ");
    Serial.print(rpm1);
    Serial.print(" | M2 RPM: ");
    Serial.println(rpm2);

    lastCount1 = c1;
    lastCount2 = c2;
    lastTime = currentTime;
  }
}