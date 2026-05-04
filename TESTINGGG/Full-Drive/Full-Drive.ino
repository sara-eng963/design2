
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

// ===================== F1 =====================
#define EN_F1 13
#define IN1_F1 2
#define IN2_F1 4
#define ENC_F1_A 16
#define ENC_F1_B 17

// ===================== F2 =====================
#define EN_F2 5
#define IN1_F2 23
#define IN2_F2 15
#define ENC_F2_A 18
#define ENC_F2_B 19

#define PPR 745

// ===================== COUNTERS =====================
volatile long cR1 = 0;
volatile long cR2 = 0;
volatile long cF1 = 0;
volatile long cF2 = 0;

unsigned long lastTime = 0;
long lR1 = 0, lR2 = 0, lF1 = 0, lF2 = 0;

// ===================== ISRs =====================
void IRAM_ATTR encR1(){ cR1 += (digitalRead(ENC_R1_A)==digitalRead(ENC_R1_B)) ? 1 : -1; }
void IRAM_ATTR encR2(){ cR2 += (digitalRead(ENC_R2_A)==digitalRead(ENC_R2_B)) ? 1 : -1; }
void IRAM_ATTR encF1(){ cF1 += (digitalRead(ENC_F1_A)==digitalRead(ENC_F1_B)) ? 1 : -1; }
void IRAM_ATTR encF2(){ cF2 += (digitalRead(ENC_F2_A)==digitalRead(ENC_F2_B)) ? 1 : -1; }

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // ---- Motor pins ----
  int pins[] = {EN_R1,IN1_R1,IN2_R1, EN_R2,IN1_R2,IN2_R2,
                EN_F1,IN1_F1,IN2_F1, EN_F2,IN1_F2,IN2_F2};

  for(int i=0;i<12;i++) pinMode(pins[i], OUTPUT);

  // ---- FORCE ALL MOTORS OFF ----
  digitalWrite(EN_R1,LOW); digitalWrite(EN_R2,LOW);
  digitalWrite(EN_F1,LOW); digitalWrite(EN_F2,LOW);

  digitalWrite(IN1_R1,LOW); digitalWrite(IN2_R1,LOW);
  digitalWrite(IN1_R2,LOW); digitalWrite(IN2_R2,LOW);
  digitalWrite(IN1_F1,LOW); digitalWrite(IN2_F1,LOW);
  digitalWrite(IN1_F2,LOW); digitalWrite(IN2_F2,LOW);

  // ---- Encoder pins ----
  pinMode(ENC_R1_A, INPUT); pinMode(ENC_R1_B, INPUT);
  pinMode(ENC_R2_A, INPUT); pinMode(ENC_R2_B, INPUT);
  pinMode(ENC_F1_A, INPUT); pinMode(ENC_F1_B, INPUT);
  pinMode(ENC_F2_A, INPUT); pinMode(ENC_F2_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_R1_A), encR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R2_A), encR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_F1_A), encF1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_F2_A), encF2, CHANGE);

  Serial.println("FULL DRIVETRAIN TEST STARTED");
}

// ===================== LOOP =====================
void loop() {

  // ========== FORWARD ==========
  Serial.println("FORWARD");

  setMotors(LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH);
  run(2000);

  // ========== STOP ==========
  Serial.println("STOP");
  allStop();
  delay(1000);

  // ========== BACKWARD ==========
  Serial.println("BACKWARD");
  setMotors(HIGH,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW);

  run(2000);

  Serial.println("STOP");
  allStop();
  delay(2000);
}

// ===================== MOTOR HELPERS =====================
void setMotors(int r1a,int r1b,int r2a,int r2b,int f1a,int f1b,int f2a,int f2b){
  digitalWrite(EN_R1,HIGH); digitalWrite(IN1_R1,r1a); digitalWrite(IN2_R1,r1b);
  digitalWrite(EN_R2,HIGH); digitalWrite(IN1_R2,r2a); digitalWrite(IN2_R2,r2b);
  digitalWrite(EN_F1,HIGH); digitalWrite(IN1_F1,f1a); digitalWrite(IN2_F1,f1b);
  digitalWrite(EN_F2,HIGH); digitalWrite(IN1_F2,f2a); digitalWrite(IN2_F2,f2b);
}

void allStop(){
  digitalWrite(EN_R1,LOW);
  digitalWrite(EN_R2,LOW);
  digitalWrite(EN_F1,LOW);
  digitalWrite(EN_F2,LOW);
}

// ===================== RUN + RPM =====================
void run(unsigned long duration){
  unsigned long start = millis();
  while(millis()-start < duration){
    rpmPrint();
  }
}

void rpmPrint(){
  unsigned long now = millis();

  if(now-lastTime >= 1000){

    long dR1=cR1-lR1, dR2=cR2-lR2;
    long dF1=cF1-lF1, dF2=cF2-lF2;

    Serial.print("R1:");
    Serial.print(abs(dR1)/ (float)PPR * 60);

    Serial.print(" | R2:");
    Serial.print(abs(dR2)/ (float)PPR * 60);

    Serial.print(" | F1:");
    Serial.print(abs(dF1)/ (float)PPR * 60);

    Serial.print(" | F2:");
    Serial.println(abs(dF2)/ (float)PPR * 60);

    lR1=cR1; lR2=cR2; lF1=cF1; lF2=cF2;
    lastTime=now;
  }
}