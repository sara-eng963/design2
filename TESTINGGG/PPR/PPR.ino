#include <Arduino.h>

// ============================================================
// ESP32 Quadrature Encoder Test
// ------------------------------------------------------------
// Purpose:
// - Measure real encoder PPR by turning one wheel by hand.
// - Verify encoder direction before doing PID control.
//
// Counting method used here:
// - Interrupt only on Channel A
// - Trigger only on the RISING edge of Channel A
// - Inside the ISR, read Channel B to decide direction
// - Forward rotation increases the count
// - Backward rotation decreases the count
//
// Serial commands:
// - r  -> reset encoderCount to zero
// - s  -> swap encoder direction in software
// ============================================================

// ------------------------------------------------------------
// Encoder pin options on your ESP32
// ------------------------------------------------------------
// R1 encoder
const int R1_ENC_A = 35;  // Yellow wire, input-only pin
const int R1_ENC_B = 34;  // Green wire, input-only pin

// R2 encoder
const int R2_ENC_A = 39;  // Yellow wire, input-only pin (VN)
const int R2_ENC_B = 36;  // Green wire, input-only pin (VP)

// F1 encoder
const int F1_ENC_A = 16;  // Yellow wire
const int F1_ENC_B = 17;  // Green wire

// F2 encoder
const int F2_ENC_A = 18;  // Yellow wire
const int F2_ENC_B = 19;  // Green wire

// ------------------------------------------------------------
// Motor driver pins
// These are set to a safe state at startup so the robot does
// not move unexpectedly when everything is connected.
// ------------------------------------------------------------
const int R1_EN = 32;
const int R1_IN1 = 33;
const int R1_IN2 = 25;

const int R2_EN = 14;
const int R2_IN1 = 26;
const int R2_IN2 = 27;

const int F1_EN = 13;
const int F1_IN1 = 2;
const int F1_IN2 = 4;

const int F2_EN = 5;
const int F2_IN1 = 23;
const int F2_IN2 = 15;

// ------------------------------------------------------------
// Choose ONE encoder to test here
// Change these two lines if you want to test a different wheel.
// ------------------------------------------------------------
const int ENC_A = F2_ENC_A;
const int ENC_B = F2_ENC_B;
const char *ENCODER_NAME = "F2";

// Signed encoder count.
// Volatile is required because the value changes inside the ISR.
volatile long encoderCount = 0;

// If true, software direction is flipped.
// This lets you correct direction without rewiring A/B.
bool swapDirection = false;

// Used to print every 200 ms.
unsigned long lastPrintTime = 0;

void initializeMotorsSafe() {
  const int enablePins[] = {R1_EN, R2_EN, F1_EN, F2_EN};
  const int directionPins[] = {
    R1_IN1, R1_IN2,
    R2_IN1, R2_IN2,
    F1_IN1, F1_IN2,
    F2_IN1, F2_IN2
  };

  for (int i = 0; i < 4; i++) {
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW);
  }

  for (int i = 0; i < 8; i++) {
    pinMode(directionPins[i], OUTPUT);
    digitalWrite(directionPins[i], LOW);
  }
}

// ------------------------------------------------------------
// Interrupt Service Routine
// ------------------------------------------------------------
// Keep this extremely short:
// - Read Channel B
// - Update encoderCount
// - Do not print
// - Do not use delay
// ------------------------------------------------------------
void IRAM_ATTR handleEncoderA() {
  int channelBState = digitalRead(ENC_B);

  if (channelBState == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Prints the instructions once at startup.
void printHelp() {
  Serial.println();
  Serial.println("ESP32 single encoder PPR test");
  Serial.print("Testing encoder: ");
  Serial.println(ENCODER_NAME);
  Serial.print("ENC_A pin: ");
  Serial.println(ENC_A);
  Serial.print("ENC_B pin: ");
  Serial.println(ENC_B);
  Serial.println();
  Serial.println("Serial commands:");
  Serial.println("  r = reset encoder count to zero");
  Serial.println("  s = swap encoder direction in software");
  Serial.println();
  Serial.println("Manual PPR test steps:");
  Serial.println("1. Reset count.");
  Serial.println("2. Rotate wheel exactly 10 revolutions by hand.");
  Serial.println("3. Compute PPR = abs(count) / 10.");
  Serial.println("4. Spin motor forward and confirm count increases.");
  Serial.println("5. If direction is reversed, press 's'.");
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  // Force all motor control pins to a safe startup state.
  // IN1 and IN2 are LOW, and EN is also held LOW as extra protection.
  initializeMotorsSafe();

  // Encoder pins are inputs.
  // GPIO 34, 35, 36, and 39 are input-only pins on ESP32.
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  // Attach interrupt only on Channel A, rising edge only.
  attachInterrupt(digitalPinToInterrupt(ENC_A), handleEncoderA, RISING);

  printHelp();
}

void loop() {
  // ----------------------------------------------------------
  // Handle serial commands
  // ----------------------------------------------------------
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'r' || command == 'R') {
      noInterrupts();
      encoderCount = 0;
      interrupts();

      Serial.println("encoderCount reset to zero.");
    }
    else if (command == 's' || command == 'S') {
      swapDirection = !swapDirection;

      Serial.print("swapDirection is now: ");
      if (swapDirection) {
        Serial.println("ON");
      } else {
        Serial.println("OFF");
      }
    }
  }

  // ----------------------------------------------------------
  // Print encoder count every 200 ms
  // ----------------------------------------------------------
  unsigned long now = millis();

  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;

    long countCopy;

    // Copy shared value safely.
    noInterrupts();
    countCopy = encoderCount;
    interrupts();

    // Apply software direction swap only in the main loop.
    if (swapDirection) {
      countCopy = -countCopy;
    }

    Serial.print("encoderCount = ");
    Serial.println(countCopy);
  }
}
