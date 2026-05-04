#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// One-Wheel-At-A-Time Velocity Control Test for ESP32
// ------------------------------------------------------------
// Architecture overview (important):
// - This project now runs on FreeRTOS tasks instead of putting everything
//   in loop().
// - controlLoopTask (high priority): runs every CONTROL_PERIOD_MS with
//   vTaskDelayUntil(), reads millis()-based dt, computes speed, runs PID/open-loop,
//   and applies PWM.
// - webServerTask (lower priority): services HTTP requests from the GUI.
// - Shared variables (targetRpm, measuredRpm, gains, etc.) are protected by
//   a mutex so web requests and control updates cannot race each other.
//
// Why this matters:
// - Deterministic control timing gives more repeatable PID behavior.
// - Separating control from web handling prevents slow network requests from
//   jittering the control loop.
// - Mutex-protected state prevents partial updates and inconsistent snapshots.
//
// Quick user manual:
// - Connect to Wi-Fi SSID Pluto with password 12345678.
// - Open http://192.168.4.1/ in a browser.
// - Select one wheel, select mode, then apply settings.
// - In OPEN mode, the value field is signed PWM.
// - In CLOSED mode, the value field is signed target RPM.
// - If positive PWM gives negative RPM, press Toggle Motor Inversion.
// - Use the live graph to compare target RPM and measured RPM.
// - Stop all motors or reset encoder counts from the page when needed.
//
// Recommended workflow:
// 1. Select a wheel.
// 2. Set mode OPEN, then run value 100 and confirm the measured RPM is positive.
// 3. If RPM is negative, toggle motor inversion and test again.
// 4. Find the minimum PWM that starts motion.
// 5. Try value 255 with the wheel lifted to estimate max RPM.
// 6. Set mode CLOSED, then use kp 1.0, ki 0, and value 50.
// 7. Raise kp until response is good, then add ki slowly.
// 8. Repeat for +RPM and -RPM, then move to the next wheel.
//
// Purpose:
// - Tune one wheel at a time before full mecanum control.
// - Test open-loop PWM direction and motor response.
// - Test closed-loop PID velocity control.
//
// Encoder method:
// - Channel A rising edge only
// - Read Channel B inside the ISR for direction
// - Signed encoder counts
// - Encoder inversion is applied inside the ISR/source update
//
// Open-loop vs closed-loop:
// - OPEN mode sends the signed PWM directly to the motor driver.
// - CLOSED mode measures RPM every control cycle and adjusts PWM using PID.
//
// RPM calculation:
// - Every control interval, delta encoder counts are converted to RPM using:
//   rpm = (deltaCount / 374.0) * (60.0 / dt_seconds)
// - Because encoder counts are signed, RPM is also signed.
//
// Graph usage for tuning:
// - The graph plots target RPM and measured RPM over time.
// - If Kp is too low, measured RPM rises too slowly and stays far from target.
// - If Kp is too high, measured RPM overshoots or oscillates strongly.
// - If Ki is too high, the response can drift, overshoot, or oscillate over time.
// - Kd helps damp sudden changes and can reduce overshoot when tuned carefully.
// ============================================================

const char *WIFI_SSID = "Pluto";
const char *WIFI_PASSWORD = "12345678";

WebServer server(80);

enum WheelIndex {
  // These fixed IDs let the code refer to wheels by number internally
  // while still printing readable names like r1 and f2 to Serial.
  WHEEL_R1 = 0,
  WHEEL_R2 = 1,
  WHEEL_F1 = 2,
  WHEEL_F2 = 3,
  WHEEL_COUNT = 4
};

struct WheelConfig {
  // Human-readable wheel name used in Serial commands and status prints.
  const char *name;

  // Motor driver pins for this wheel.
  // pwmPin controls speed.
  // in1Pin and in2Pin control direction through the L298N.
  uint8_t pwmPin;
  uint8_t in1Pin;
  uint8_t in2Pin;

  // Encoder pins for this wheel.
  // We trigger only from encAPin and read encBPin inside the ISR.
  uint8_t encAPin;
  uint8_t encBPin;
};

// One table holds the full wiring for all four wheels.
// This makes the rest of the code cleaner because each helper function
// can work with a wheel index instead of separate pin variables.
const WheelConfig wheels[WHEEL_COUNT] = {
  {"r1", 32, 33, 25, 35, 34},
  {"r2", 14, 26, 27, 39, 36},
  {"f1", 13, 2, 4, 16, 17},
  {"f2", 5, 23, 15, 18, 19}
};

// Raw encoder counts updated inside interrupts.
// volatile is required because the ISR changes these values asynchronously.
volatile long encoderCounts[WHEEL_COUNT] = {0, 0, 0, 0};

// Last sampled count for each wheel.
// These are used to compute deltaCount every control period.
long lastEncoderCounts[WHEEL_COUNT] = {0, 0, 0, 0};

// These flags fix encoder sign at the source level.
// If true, the ISR reverses the step before storing it.
const bool invertEncoder[WHEEL_COUNT] = {
  true,   // R1
  false,  // R2
  true,   // F1
  false   // F2
};

// These flags are separate from encoder inversion.
// They flip motor direction only, not encoder counting direction.
// Use the m command if positive PWM produces negative RPM.
bool invertMotor[WHEEL_COUNT] = {
  true,   // R1
  true,   // R2
  true,   // F1
  true    // F2
};

const float PPR = 374.0f;
const float WHEEL_DIAMETER_M = 0.097f;

// Circumference converts wheel RPM into linear ground speed in m/s.
const float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * 3.14159265358979323846f;

// Control loop timing.
// Every 50 ms we compute RPM and update the motor command.
// The web page polls status often, but the control loop timing stays fixed.
const unsigned long CONTROL_PERIOD_MS = 50;
const float CONTROL_PERIOD_SECONDS = CONTROL_PERIOD_MS / 1000.0f;

// analogWrite range for the motor PWM command.
const int PWM_MAX = 255;

// minimumPwmByWheel compensates for L298N deadband per wheel.
// Small nonzero PWM values often do not move the motor at all.
const int minimumPwmByWheel[WHEEL_COUNT] = {
  110,  // R1
  135,  // R2
  125,  // F1
  110   // F2
};

const float TARGET_RPM_LIMIT = 110.0f;

// PID gains. Kd defaults to 0 so the controller behaves as PI until derivative is needed.
float kp = 5.0f;
float ki = 0.30f;
float kd = 0.0f;

// The integral accumulates error over time and is anti-windup clamped.
float integralTerm = 0.0f;
const float INTEGRAL_LIMIT = 300.0f;

// Derivative state: the previous error is needed each cycle to compute the derivative.
float previousError = 0.0f;
float derivativeTerm = 0.0f;

// Actual measured dt from the last control cycle in seconds, stored for status reporting.
float currentDtSeconds = CONTROL_PERIOD_SECONDS;

// Last raw encoder delta count, stored for status reporting.
long lastRawDeltaCount = 0;

// Runtime state for the currently selected wheel and current test mode.
WheelIndex selectedWheel = WHEEL_R1;
float targetRpm = 0.0f;
float measuredRpm = 0.0f;
float linearSpeedMps = 0.0f;
float controlError = 0.0f;
float pidOutputUnclamped = 0.0f;
int finalPwmCommand = 0;
int openLoopPwmCommand = 0;
int appliedPwmCommand = 0;
bool runEnabled = false;
bool sampleReady = false;
float openLoopAverageRpm = 0.0f;
float openLoopRpmSum = 0.0f;
unsigned long openLoopAverageSamples = 0;
float openLoopAverageSeconds = 0.0f;

// Snapshot of one fully completed control cycle.
// Web status and logger read only from this snapshot to avoid mixed-cycle values.
struct CycleSnapshot {
  bool sampleReady;
  String wheelName;
  String modeText;
  float targetRpm;
  float measuredRpm;
  float linearSpeedMps;
  float error;
  float integralTerm;
  float derivativeTerm;
  float outputPwm;
  int finalPwm;
  int appliedPwm;
  float kp;
  float ki;
  float kd;
  float dtSeconds;
  long deltaCount;
  int minPwm;
  bool motorInverted;
  long encoderCount;
  int openLoopCommandPwm;
  float openLoopAverageRpm;
  unsigned long openLoopAverageSamples;
  float openLoopAverageSeconds;
};

CycleSnapshot latestCycleSnapshot;
String lastStatusJsonCache = "{}";

void publishCycleSnapshot();
void prepareRunInitialization();

unsigned long lastControlTime = 0;

TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t webTaskHandle = nullptr;
SemaphoreHandle_t stateMutex = nullptr;

// Try to lock shared state for a short time.
// We use a timeout instead of waiting forever to avoid deadlocks that can
// freeze the system if something unexpected happens.
bool lockState(TickType_t timeoutTicks = pdMS_TO_TICKS(20)) {
  return (stateMutex != nullptr) && (xSemaphoreTake(stateMutex, timeoutTicks) == pdTRUE);
}

// Release shared-state lock. Always call after a successful lockState().
void unlockState() {
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

enum ControlMode {
  // MODE_OPEN sends the PWM directly.
  // MODE_CLOSED uses PI control to chase a target RPM.
  MODE_OPEN = 0,
  MODE_CLOSED = 1
};

ControlMode controlMode = MODE_OPEN;

// Small helper so all PWM output passes through one place.
// Right now this uses analogWrite because that matches your setup.
void writePwmDuty(uint8_t pin, uint32_t duty) {
  analogWrite(pin, duty);
}

void resetMeasuredState() {
  measuredRpm = 0.0f;
  linearSpeedMps = 0.0f;
  controlError = 0.0f;
}

void resetOpenLoopAverage() {
  openLoopAverageRpm = 0.0f;
  openLoopRpmSum = 0.0f;
  openLoopAverageSamples = 0;
  openLoopAverageSeconds = 0.0f;
}

void setStoppedMode() {
  // Keep mode OPEN while stopped so no PID action runs accidentally.
  controlMode = MODE_OPEN;
  targetRpm = 0.0f;
  openLoopPwmCommand = 0;
  resetOpenLoopAverage();
}

// When target or wheel changes, clear stored integral action so the new test
// starts cleanly instead of carrying old error history into it.
void resetIntegral() {
  integralTerm = 0.0f;
  pidOutputUnclamped = 0.0f;
  finalPwmCommand = 0;
  previousError = 0.0f;
  derivativeTerm = 0.0f;
}

// Prepare a clean run start so first reported sample comes from a full control cycle.
// This function assumes the state mutex is already locked by the caller.
void prepareRunInitialization() {
  stopAllMotorHardware();
  resetIntegral();
  appliedPwmCommand = 0;
  lastRawDeltaCount = 0;
  resetMeasuredState();
  sampleReady = false;

  // Align deltaCount baseline with the encoder count at start time.
  long startCount;
  noInterrupts();
  startCount = encoderCounts[selectedWheel];
  interrupts();
  lastEncoderCounts[selectedWheel] = startCount;

  resetOpenLoopAverage();

  // Publish a non-ready snapshot immediately so the logger waits until the
  // next full control cycle completes.
  publishCycleSnapshot();
}

// Publish a coherent snapshot only after one full control cycle is complete.
// This function assumes the state mutex is already locked by the caller.
void publishCycleSnapshot() {
  latestCycleSnapshot.sampleReady = sampleReady;
  latestCycleSnapshot.wheelName = jsonEscape(wheels[selectedWheel].name);
  latestCycleSnapshot.modeText = String(controlMode == MODE_CLOSED ? "closed" : "open");
  latestCycleSnapshot.targetRpm = targetRpm;
  latestCycleSnapshot.measuredRpm = measuredRpm;
  latestCycleSnapshot.linearSpeedMps = linearSpeedMps;
  latestCycleSnapshot.error = controlError;
  latestCycleSnapshot.integralTerm = integralTerm;
  latestCycleSnapshot.derivativeTerm = derivativeTerm;
  latestCycleSnapshot.outputPwm = pidOutputUnclamped;
  latestCycleSnapshot.finalPwm = finalPwmCommand;
  latestCycleSnapshot.appliedPwm = appliedPwmCommand;
  latestCycleSnapshot.kp = kp;
  latestCycleSnapshot.ki = ki;
  latestCycleSnapshot.kd = kd;
  latestCycleSnapshot.dtSeconds = currentDtSeconds;
  latestCycleSnapshot.deltaCount = lastRawDeltaCount;
  latestCycleSnapshot.minPwm = minimumPwmByWheel[selectedWheel];
  latestCycleSnapshot.motorInverted = invertMotor[selectedWheel];
  latestCycleSnapshot.encoderCount = readSelectedEncoderCount();
  latestCycleSnapshot.openLoopCommandPwm = openLoopPwmCommand;
  latestCycleSnapshot.openLoopAverageRpm = openLoopAverageRpm;
  latestCycleSnapshot.openLoopAverageSamples = openLoopAverageSamples;
  latestCycleSnapshot.openLoopAverageSeconds = openLoopAverageSeconds;
}

// Clear encoder counts and the last sampled values used for RPM calculation.
// The count reset is protected because the ISR can update counts at any time.
void resetEncoderTracking() {
  noInterrupts();
  for (int index = 0; index < WHEEL_COUNT; index++) {
    encoderCounts[index] = 0;
  }
  interrupts();

  for (int index = 0; index < WHEEL_COUNT; index++) {
    lastEncoderCounts[index] = 0;
  }

  resetMeasuredState();
}

// Hard-stop one wheel by forcing both direction pins LOW and PWM to zero.
// This is the safe idle state for the L298N driver.
void stopWheelHardware(WheelIndex wheel) {
  digitalWrite(wheels[wheel].in1Pin, LOW);
  digitalWrite(wheels[wheel].in2Pin, LOW);
  writePwmDuty(wheels[wheel].pwmPin, 0);
}

// Stop every wheel in hardware.
// This is used before enabling one wheel so only a single motor can move.
void stopAllMotorHardware() {
  for (int index = 0; index < WHEEL_COUNT; index++) {
    stopWheelHardware((WheelIndex)index);
  }
}

// Stop all motors and also clear the currently remembered open-loop command.
void stopAllMotors() {
  // This function updates commanded state AND hardware output.
  // Keep these together so status reflects what is physically applied.
  stopAllMotorHardware();
  openLoopPwmCommand = 0;
  appliedPwmCommand = 0;
}

// Keep signed PWM commands inside the legal analogWrite range.
int clampSignedPwm(int pwmValue) {
  if (pwmValue > PWM_MAX) {
    return PWM_MAX;
  }
  if (pwmValue < -PWM_MAX) {
    return -PWM_MAX;
  }
  return pwmValue;
}

// Apply one signed PWM command to the selected wheel.
// No deadband compensation. If PI output is 20, apply exactly PWM 20.
// Positive and negative values choose opposite motor directions.
// Before enabling one wheel we stop all hardware first so no other wheel moves.
void applyMotorCommand(WheelIndex wheel, int commandedPwm) {
  // Step 1: clamp to legal signed command range.
  int safeCommand = clampSignedPwm(commandedPwm);

  // invertMotor changes only motor direction wiring convention.
  // It does not affect encoder sign.
  int hardwareCommand = invertMotor[wheel] ? -safeCommand : safeCommand;

  // Step 2: stop all wheel hardware first.
  // This guarantees only one wheel is driven at a time in this test utility.
  stopAllMotorHardware();

  // Step 3: map signed command to H-bridge direction pins + PWM magnitude.
  if (hardwareCommand > 0) {
    digitalWrite(wheels[wheel].in1Pin, HIGH);
    digitalWrite(wheels[wheel].in2Pin, LOW);
    writePwmDuty(wheels[wheel].pwmPin, hardwareCommand);
  } else if (hardwareCommand < 0) {
    digitalWrite(wheels[wheel].in1Pin, LOW);
    digitalWrite(wheels[wheel].in2Pin, HIGH);
    writePwmDuty(wheels[wheel].pwmPin, abs(hardwareCommand));
  } else {
    stopWheelHardware(wheel);
  }

  appliedPwmCommand = safeCommand;
}

// Core encoder update rule used by all four wheels.
// We trigger on Channel A rising edge only.
// At that instant, Channel B tells us whether to add or subtract one count.
// If a wheel needs encoder inversion, the sign is fixed here before storage.
void IRAM_ATTR updateEncoderCount(WheelIndex wheel) {
  // Direction decode from quadrature B at A rising edge.
  // This is a 1x decode method (less ISR load, enough for velocity testing).
  int step = digitalRead(wheels[wheel].encBPin) == HIGH ? 1 : -1;

  if (invertEncoder[wheel]) {
    step = -step;
  }

  encoderCounts[wheel] += step;
}

// Tiny wheel-specific ISR wrappers.
// They stay short and just forward the update to the shared helper.
void IRAM_ATTR handleEncoderR1() {
  updateEncoderCount(WHEEL_R1);
}

void IRAM_ATTR handleEncoderR2() {
  updateEncoderCount(WHEEL_R2);
}

void IRAM_ATTR handleEncoderF1() {
  updateEncoderCount(WHEEL_F1);
}

void IRAM_ATTR handleEncoderF2() {
  updateEncoderCount(WHEEL_F2);
}

typedef void (*EncoderIsr)();

const EncoderIsr encoderHandlers[WHEEL_COUNT] = {
  handleEncoderR1,
  handleEncoderR2,
  handleEncoderF1,
  handleEncoderF2
};

const char *modeName() {
  return (controlMode == MODE_CLOSED) ? "CLOSED" : "OPEN";
}

String jsonEscape(const char *text) {
  return String(text);
}

long readSelectedEncoderCount() {
  long countCopy;

  // Snapshot volatile value atomically with interrupts off.
  noInterrupts();
  countCopy = encoderCounts[selectedWheel];
  interrupts();

  return countCopy;
}

String buildStatusJson() {
  // Read from latest full-cycle snapshot only.
  CycleSnapshot snapshotCopy;
  if ((stateMutex != nullptr) && (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)) {
    snapshotCopy = latestCycleSnapshot;
    unlockState();
  } else {
    return lastStatusJsonCache;
  }

  String json = "{";
  json += "\"sampleReady\":" + String(snapshotCopy.sampleReady ? "true" : "false") + ",";
  json += "\"wheel\":\"" + snapshotCopy.wheelName + "\",";
  json += "\"mode\":\"" + snapshotCopy.modeText + "\",";
  json += "\"targetRpm\":" + String(snapshotCopy.targetRpm, 2) + ",";
  json += "\"measuredRpm\":" + String(snapshotCopy.measuredRpm, 2) + ",";
  json += "\"linearSpeedMps\":" + String(snapshotCopy.linearSpeedMps, 4) + ",";
  json += "\"error\":" + String(snapshotCopy.error, 2) + ",";
  json += "\"integralTerm\":" + String(snapshotCopy.integralTerm, 4) + ",";
  json += "\"derivativeTerm\":" + String(snapshotCopy.derivativeTerm, 4) + ",";
  json += "\"outputPwm\":" + String(snapshotCopy.outputPwm, 2) + ",";
  json += "\"finalPwm\":" + String(snapshotCopy.finalPwm) + ",";
  json += "\"appliedPwm\":" + String(snapshotCopy.appliedPwm) + ",";
  json += "\"kp\":" + String(snapshotCopy.kp, 3) + ",";
  json += "\"ki\":" + String(snapshotCopy.ki, 4) + ",";
  json += "\"kd\":" + String(snapshotCopy.kd, 4) + ",";
  json += "\"dtSeconds\":" + String(snapshotCopy.dtSeconds, 4) + ",";
  json += "\"deltaCount\":" + String(snapshotCopy.deltaCount) + ",";
  json += "\"minPwm\":" + String(snapshotCopy.minPwm) + ",";
  json += "\"motorInverted\":" + String(snapshotCopy.motorInverted ? "true" : "false") + ",";
  json += "\"encoderCount\":" + String(snapshotCopy.encoderCount) + ",";
  json += "\"openLoopCommandPwm\":" + String(snapshotCopy.openLoopCommandPwm) + ",";
  json += "\"openLoopAverageRpm\":" + String(snapshotCopy.openLoopAverageRpm, 2) + ",";
  json += "\"openLoopAverageSamples\":" + String(snapshotCopy.openLoopAverageSamples) + ",";
  json += "\"openLoopAverageSeconds\":" + String(snapshotCopy.openLoopAverageSeconds, 2);
  json += "}";
  lastStatusJsonCache = json;
  return json;
}

const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Pluto Wheel Velocity Test</title>
  <style>
    :root {
      --bg-top: #edf6ff;
      --bg-bottom: #fff5ea;
      --card: rgba(255,255,255,0.92);
      --ink: #1b2f3b;
      --muted: #62727d;
      --line: #d7e5ee;
      --accent: #e66b2f;
      --accent-soft: #fde7dd;
      --accent-2: #0b8fbf;
      --good: #1f9d62;
      --warn: #cf7a1a;
      --shadow: 0 18px 50px rgba(27, 47, 59, 0.12);
      --radius: 20px;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: "Trebuchet MS", "Segoe UI", Tahoma, sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at 10% 10%, rgba(255,255,255,0.95) 0 18%, transparent 42%),
        radial-gradient(circle at 88% 78%, rgba(255,225,191,0.95) 0 14%, transparent 38%),
        linear-gradient(140deg, var(--bg-top), var(--bg-bottom));
      padding: 20px;
    }
    .shell {
      max-width: 1180px;
      margin: 0 auto;
      display: grid;
      gap: 18px;
    }
    .hero, .panel {
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: var(--radius);
      box-shadow: var(--shadow);
      backdrop-filter: blur(8px);
    }
    .hero {
      padding: 24px;
    }
    .hero h1 {
      margin: 0 0 8px;
      font-size: clamp(1.8rem, 4vw, 2.6rem);
      letter-spacing: 0.4px;
    }
    .hero-top {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 8px;
    }
    .connection-pill {
      display: inline-flex;
      align-items: center;
      gap: 10px;
      padding: 10px 14px;
      border-radius: 999px;
      border: 1px solid var(--line);
      background: #f7fbff;
      color: var(--ink);
      font-size: 0.92rem;
      font-weight: 700;
      white-space: nowrap;
    }
    .connection-pill::before {
      content: "";
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: #9aaab5;
      box-shadow: 0 0 0 4px rgba(154, 170, 181, 0.16);
    }
    .connection-pill.connected {
      background: #edf9f2;
      color: #1f7a4f;
    }
    .connection-pill.connected::before {
      background: var(--good);
      box-shadow: 0 0 0 4px rgba(31, 157, 98, 0.16);
    }
    .connection-pill.disconnected {
      background: #fff0f0;
      color: #a33b3b;
    }
    .connection-pill.disconnected::before {
      background: #d65555;
      box-shadow: 0 0 0 4px rgba(214, 85, 85, 0.16);
    }
    .connection-pill.connecting {
      background: #fff6e7;
      color: var(--warn);
    }
    .connection-pill.connecting::before {
      background: var(--warn);
      box-shadow: 0 0 0 4px rgba(207, 122, 26, 0.16);
    }
    .hero p {
      margin: 0;
      color: var(--muted);
      max-width: 760px;
      line-height: 1.5;
    }
    .layout {
      display: grid;
      grid-template-columns: minmax(320px, 380px) 1fr;
      gap: 18px;
    }
    .panel {
      padding: 20px;
    }
    .section-title {
      margin: 0 0 14px;
      font-size: 1rem;
      letter-spacing: 0.4px;
      text-transform: uppercase;
      color: var(--muted);
    }
    .field {
      margin-bottom: 14px;
    }
    label {
      display: block;
      font-weight: 700;
      margin-bottom: 6px;
      font-size: 0.95rem;
    }
    select, input[type="number"] {
      width: 100%;
      padding: 11px 12px;
      border-radius: 12px;
      border: 1px solid #c8d7e2;
      background: #fbfdff;
      color: var(--ink);
      font-size: 1rem;
    }
    .grid-two {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }
    .grid-three {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 12px;
    }
    .button-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 16px;
    }
    button {
      border: 0;
      border-radius: 14px;
      padding: 12px 14px;
      font-size: 0.98rem;
      font-weight: 800;
      cursor: pointer;
      transition: transform 0.15s ease, box-shadow 0.15s ease;
    }
    button:hover { transform: translateY(-1px); }
    .primary {
      background: linear-gradient(90deg, var(--accent), #ef8e3d);
      color: #fff;
      box-shadow: 0 10px 24px rgba(230, 107, 47, 0.25);
    }
    .secondary {
      background: linear-gradient(90deg, var(--accent-2), #25a6d4);
      color: #fff;
      box-shadow: 0 10px 24px rgba(11, 143, 191, 0.22);
    }
    .neutral {
      background: #f3f8fb;
      color: var(--ink);
      border: 1px solid var(--line);
    }
    .danger {
      background: linear-gradient(90deg, #d65555, #ed7462);
      color: #fff;
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
      margin-bottom: 16px;
    }
    .metric {
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 14px;
      background: #fbfdff;
    }
    .metric .label {
      font-size: 0.8rem;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.4px;
      margin-bottom: 6px;
    }
    .metric .value {
      font-size: 1.35rem;
      font-weight: 800;
    }
    .wide-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
      margin-top: 8px;
    }
    .detail {
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      background: #fff;
      min-height: 82px;
    }
    .detail .label {
      font-size: 0.82rem;
      color: var(--muted);
      margin-bottom: 6px;
      text-transform: uppercase;
      letter-spacing: 0.4px;
    }
    .detail .value {
      font-size: 1.1rem;
      font-weight: 700;
      word-break: break-word;
    }
    .chart-wrap {
      margin-top: 18px;
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 14px;
      background: linear-gradient(180deg, #ffffff, #f9fcff);
    }
    .chart-wrap h3 {
      margin: 0 0 6px;
      font-size: 1.05rem;
    }
    .chart-wrap p {
      margin: 0 0 12px;
      color: var(--muted);
      font-size: 0.9rem;
      line-height: 1.45;
    }
    canvas {
      width: 100%;
      height: 300px;
      display: block;
      border-radius: 12px;
      background: #fff;
      border: 1px solid var(--line);
    }
    .note {
      margin-top: 14px;
      padding: 12px 14px;
      border-radius: 14px;
      background: var(--accent-soft);
      color: #8a4b18;
      font-size: 0.92rem;
      line-height: 1.45;
    }
    .action-status {
      margin-top: 12px;
      padding: 12px 14px;
      border-radius: 14px;
      border: 1px solid var(--line);
      background: #f7fbff;
      color: var(--ink);
      font-size: 0.92rem;
      min-height: 48px;
      display: flex;
      align-items: center;
    }
    .action-status.busy {
      background: #eef7ff;
      color: #0b5d7c;
    }
    .action-status.success {
      background: #edf9f2;
      color: #1f7a4f;
    }
    .action-status.error {
      background: #fff0f0;
      color: #a33b3b;
    }
    .footer-note {
      font-size: 0.82rem;
      color: var(--muted);
      margin-top: 14px;
    }
    .logger-toolbar {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin: 12px 0;
    }
    .logger-meta {
      font-size: 0.84rem;
      color: var(--muted);
      margin-bottom: 8px;
    }
    textarea {
      width: 100%;
      min-height: 220px;
      resize: vertical;
      border-radius: 14px;
      border: 1px solid var(--line);
      background: #fbfdff;
      color: var(--ink);
      padding: 12px;
      font: 0.92rem/1.45 Consolas, "Courier New", monospace;
    }
    @media (max-width: 900px) {
      .layout { grid-template-columns: 1fr; }
      .status-grid { grid-template-columns: repeat(2, 1fr); }
      .hero-top { flex-direction: column; align-items: flex-start; }
    }
    @media (max-width: 640px) {
      body { padding: 12px; }
      .button-row, .grid-two, .grid-three, .wide-grid, .status-grid { grid-template-columns: 1fr; }
      .panel, .hero { padding: 16px; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <section class="hero">
      <div class="hero-top">
        <h1>Pluto Wheel Velocity Test</h1>
        <div id="connectionStatus" class="connection-pill connecting">Checking ESP32 link...</div>
      </div>
      <p>Use this page to tune one wheel at a time. Open mode sends signed PWM directly. Closed mode uses PI control to chase a target RPM every 50 ms. The graph compares target RPM and measured RPM so you can see whether gains are too weak, too aggressive, or drifting.</p>
    </section>

    <div class="layout">
      <section class="panel">
        <h2 class="section-title">Control Panel</h2>

        <div class="field">
          <label for="wheel">Wheel</label>
          <select id="wheel">
            <option value="r1">R1</option>
            <option value="r2">R2</option>
            <option value="f1">F1</option>
            <option value="f2">F2</option>
          </select>
        </div>

        <div class="field">
          <label for="mode">Mode</label>
          <select id="mode" onchange="updateValueLabel()">
            <option value="open">OPEN LOOP</option>
            <option value="closed">CLOSED LOOP</option>
          </select>
        </div>

        <div class="field">
          <label id="valueLabel" for="valueInput">Open-loop PWM (-255 to +255)</label>
          <input id="valueInput" type="number" value="0" step="1">
        </div>

        <div class="grid-three">
          <div class="field">
            <label for="kpInput">Kp</label>
            <input id="kpInput" type="number" value="5.0" step="0.01">
          </div>
          <div class="field">
            <label for="kiInput">Ki</label>
            <input id="kiInput" type="number" value="0.30" step="0.001">
          </div>
          <div class="field">
            <label for="kdInput">Kd</label>
            <input id="kdInput" type="number" value="0.0" step="0.001">
          </div>
        </div>

        <div class="field">
          <label for="minPwmInput">Minimum PWM</label>
          <input id="minPwmInput" type="number" value="80" min="0" max="255" step="1">
        </div>

        <div class="button-row">
          <button id="applyButton" class="primary" onclick="applySettings(false)">Apply + Load Value</button>
          <button id="startButton" class="secondary" onclick="applySettings(true)">Start Current Value</button>
          <button class="danger" onclick="callEndpoint('/stop')">Stop All Motors</button>
          <button class="neutral" onclick="callEndpoint('/reset')">Reset Encoder Counts</button>
          <button class="neutral" onclick="callEndpoint('/toggleMotorInvert')">Toggle Motor Inversion</button>
          <button class="neutral" onclick="refreshStatus(true)">Refresh Now</button>
        </div>

        <div id="actionStatus" class="action-status">Ready. Choose a wheel, choose a mode, type a value, then press Apply + Load Value or Start Current Value.</div>

        <div class="note">
          Tuning guide: If measured RPM rises too slowly, increase Kp. If it overshoots or oscillates hard, lower Kp. If it settles below target, increase Ki slowly. If it begins to wander or oscillate over time, Ki is probably too high.
        </div>

        <div class="footer-note">ESP32 access point: <strong>Pluto</strong> | password: <strong>12345678</strong> | local page: <strong>192.168.4.1</strong></div>
      </section>

      <section class="panel">
        <h2 class="section-title">Live Status</h2>

        <div class="status-grid">
          <div class="metric"><div class="label">Selected Wheel</div><div class="value" id="wheelValue">r1</div></div>
          <div class="metric"><div class="label">Mode</div><div class="value" id="modeValue">open</div></div>
          <div class="metric"><div class="label">Encoder Count</div><div class="value" id="encoderCountValue">0</div></div>
        </div>

        <div class="wide-grid">
          <div class="detail"><div class="label">Target RPM</div><div class="value" id="targetRpmValue">0.00</div></div>
          <div class="detail"><div class="label">Measured RPM</div><div class="value" id="measuredRpmValue">0.00</div></div>
          <div class="detail"><div class="label">Linear Speed</div><div class="value" id="linearSpeedValue">0.0000 m/s</div></div>
          <div class="detail"><div class="label">Error</div><div class="value" id="errorValue">0.00</div></div>
          <div class="detail"><div class="label">Integral Term</div><div class="value" id="integralTermValue">0.0000</div></div>
          <div class="detail"><div class="label">Derivative Term</div><div class="value" id="derivativeTermValue">0.0000</div></div>
          <div class="detail"><div class="label">PID Output (Unclamped)</div><div class="value" id="pidOutputValue">0.00</div></div>
          <div class="detail"><div class="label">Final PWM (Clamped)</div><div class="value" id="finalPwmValue">0</div></div>
          <div class="detail"><div class="label">Applied PWM</div><div class="value" id="appliedPwmValue">0</div></div>
          <div class="detail"><div class="label">dt (s)</div><div class="value" id="dtSecondsValue">0.0500</div></div>
          <div class="detail"><div class="label">Delta Count</div><div class="value" id="deltaCountValue">0</div></div>
          <div class="detail"><div class="label">Open-Loop Avg RPM</div><div class="value" id="openLoopAverageRpmValue">0.00</div></div>
          <div class="detail"><div class="label">Average Window</div><div class="value" id="openLoopAverageWindowValue">0 samples / 0.00 s</div></div>
          <div class="detail"><div class="label">Kp / Ki / Kd</div><div class="value" id="gainsValue">5.000 / 0.3000 / 0.0000</div></div>
          <div class="detail"><div class="label">Min PWM (fixed/wheel)</div><div class="value" id="minPwmValue">110</div></div>
          <div class="detail"><div class="label">Motor Inversion</div><div class="value" id="motorInvertValue">true</div></div>
        </div>

        <div class="chart-wrap">
          <h3>Target vs Measured RPM</h3>
          <p>The graph keeps about the last 20 seconds of data. Target RPM is the orange line and measured RPM is the blue line. Use it to see lag, overshoot, and oscillation clearly while tuning.</p>
          <canvas id="rpmChart"></canvas>
          <div id="graphModeNote" class="footer-note">Closed-loop mode shows target RPM against measured RPM. In open-loop mode there is no target RPM to plot.</div>
        </div>

        <div class="chart-wrap">
          <h3>Run Logger</h3>
          <p>The logger starts when you apply/start a run and stops when you press Stop All Motors. It saves plain text so you can copy it directly into your notes.</p>
          <div id="loggerMeta" class="logger-meta">Logger idle.</div>
          <div class="logger-toolbar">
            <button class="neutral" onclick="copyLogText()">Copy Log</button>
            <button class="neutral" onclick="clearLogText()">Clear Log Box</button>
          </div>
          <textarea id="logOutput" readonly spellcheck="false"></textarea>
        </div>
      </section>
    </div>
  </div>

  <script>
    const chartCanvas = document.getElementById('rpmChart');
    const chartContext = chartCanvas.getContext('2d');
    const graphSamples = [];
    const maxGraphPoints = 100;
    const pollIntervalMs = 200;
    let lastStatusSuccessTime = 0;
    const dirtyFields = new Set();
    let loggingActive = false;
    let logStartTime = 0;
    let logLineCount = 0;

    function getLogOutput() {
      return document.getElementById('logOutput');
    }

    function setLoggerMeta(message) {
      document.getElementById('loggerMeta').textContent = message;
    }

    function appendLogLine(line) {
      const logOutput = getLogOutput();
      logOutput.value += line + '\n';
      logOutput.scrollTop = logOutput.scrollHeight;
    }

    function formatLogSnapshot(data, elapsedSeconds) {
      return `t=${elapsedSeconds.toFixed(2)}s | wheel=${String(data.wheel).toUpperCase()} | targetRPM=${Number(data.targetRpm).toFixed(2)} | measuredRPM=${Number(data.measuredRpm).toFixed(2)} | error=${Number(data.error).toFixed(2)} | integral=${Number(data.integralTerm).toFixed(4)} | derivative=${Number(data.derivativeTerm).toFixed(4)} | outputPWM=${Number(data.outputPwm).toFixed(2)} | finalPWM=${data.finalPwm} | appliedPWM=${data.appliedPwm} | dt=${Number(data.dtSeconds).toFixed(4)}s | deltaCount=${data.deltaCount} | encoderCount=${data.encoderCount} | kp=${Number(data.kp).toFixed(3)} | ki=${Number(data.ki).toFixed(4)} | kd=${Number(data.kd).toFixed(4)}`;
    }

    function startLogging(data) {
      loggingActive = true;
      logStartTime = Date.now();
      logLineCount = 0;
      appendLogLine('=== Velocity Test Run Started ===');
      appendLogLine(`wheel=${String(data.wheel).toUpperCase()} | mode=${String(data.mode).toUpperCase()} | targetRPM=${Number(data.targetRpm).toFixed(2)} | kp=${Number(data.kp).toFixed(3)} | ki=${Number(data.ki).toFixed(4)} | kd=${Number(data.kd).toFixed(4)}`);
      appendLogLine('waiting for first complete control cycle...');
      appendLogLine('--- samples ---');
      setLoggerMeta(`Logging ${String(data.wheel).toUpperCase()}...`);
    }

    function stopLogging() {
      if (!loggingActive) {
        setLoggerMeta('Logger idle.');
        return;
      }

      const elapsedSeconds = (Date.now() - logStartTime) / 1000;
      appendLogLine(`=== Velocity Test Run Stopped | duration=${elapsedSeconds.toFixed(2)}s | samples=${logLineCount} ===`);
      appendLogLine('');
      loggingActive = false;
      setLoggerMeta('Logger idle.');
    }

    async function copyLogText() {
      const logOutput = getLogOutput();
      try {
        await navigator.clipboard.writeText(logOutput.value);
        setActionStatus('Log copied to clipboard.', 'success');
      } catch (error) {
        logOutput.focus();
        logOutput.select();
        setActionStatus('Clipboard copy failed. The log box is selected so you can copy manually.', 'error');
      }
    }

    function clearLogText() {
      getLogOutput().value = '';
      setLoggerMeta(loggingActive ? 'Logging in progress...' : 'Logger idle.');
    }

    function markDirty(elementId) {
      dirtyFields.add(elementId);
    }

    function clearDirty(elementId) {
      dirtyFields.delete(elementId);
    }

    function isFieldLocked(elementId) {
      return isUserEditing(elementId) || dirtyFields.has(elementId);
    }

    function wireDirtyTracking(elementId) {
      const element = document.getElementById(elementId);
      element.addEventListener('input', () => markDirty(elementId));
      element.addEventListener('change', () => markDirty(elementId));
    }

    function setConnectionState(state, detail) {
      const pill = document.getElementById('connectionStatus');
      const labels = {
        connected: 'Connected to ESP32',
        disconnected: 'Disconnected from ESP32',
        connecting: 'Checking ESP32 link...'
      };
      pill.className = 'connection-pill ' + state;
      pill.textContent = detail ? `${labels[state]} - ${detail}` : labels[state];
    }

    function setActionStatus(message, state) {
      const statusBox = document.getElementById('actionStatus');
      statusBox.textContent = message;
      statusBox.className = 'action-status' + (state ? ` ${state}` : '');
    }

    function setBusyState(isBusy) {
      document.getElementById('applyButton').disabled = isBusy;
      document.getElementById('startButton').disabled = isBusy;
    }

    function updateGraphModeNote(mode) {
      const note = document.getElementById('graphModeNote');
      if (mode === 'closed') {
        note.textContent = 'Closed-loop mode: the orange line is the RPM target and the blue line is the measured RPM.';
      } else {
        note.textContent = 'Open-loop mode: there is no target RPM, because you are commanding PWM directly. Use Open-Loop Avg RPM to record the average measured speed for each PWM setting.';
      }
    }

    function updateValueLabel() {
      const mode = document.getElementById('mode').value;
      const label = document.getElementById('valueLabel');
      if (mode === 'closed') {
        label.textContent = 'Closed-loop target RPM (negative or positive)';
      } else {
        label.textContent = 'Open-loop PWM (-255 to +255)';
      }
    }

    async function callEndpoint(path) {
      try {
        setActionStatus('Sending command...', 'busy');
        const response = await fetch(path, { cache: 'no-store' });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        await refreshStatus(true);
        setConnectionState('connected');
        if (path === '/stop') {
          stopLogging();
        }
        setActionStatus('Command completed.', 'success');
      } catch (error) {
        setConnectionState('disconnected', 'request failed');
        setActionStatus('Request failed. Check Wi-Fi connection and try again.', 'error');
        console.error('Request failed:', error);
      }
    }

    async function applySettings(startAfterApply) {
      const wheel = document.getElementById('wheel').value;
      const mode = document.getElementById('mode').value;
      const value = document.getElementById('valueInput').value;
      const kp = document.getElementById('kpInput').value;
      const ki = document.getElementById('kiInput').value;
      const kd = document.getElementById('kdInput').value;
      const minPwm = document.getElementById('minPwmInput').value;
      const valueLabel = mode === 'closed' ? 'target RPM' : 'PWM';

      try {
        setBusyState(true);
        setActionStatus(`Applying ${mode} settings and loading ${valueLabel} ${value}...`, 'busy');
        let response = await fetch(`/setWheel?w=${encodeURIComponent(wheel)}`, { cache: 'no-store' });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        response = await fetch(`/setGains?kp=${encodeURIComponent(kp)}&ki=${encodeURIComponent(ki)}&kd=${encodeURIComponent(kd)}`, { cache: 'no-store' });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        response = await fetch(`/setMinPWM?value=${encodeURIComponent(minPwm)}`, { cache: 'no-store' });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }

        if (mode === 'closed') {
          // /setTarget enters closed-loop mode and sets target in one action.
          response = await fetch(`/setTarget?rpm=${encodeURIComponent(value)}`, { cache: 'no-store' });
        } else {
          // /setPWM enters open-loop mode and sets PWM in one action.
          response = await fetch(`/setPWM?value=${encodeURIComponent(value)}`, { cache: 'no-store' });
        }

        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }

        const data = await refreshStatus(true);
        setConnectionState('connected');
        clearDirty('wheel');
        clearDirty('mode');
        clearDirty('valueInput');
        clearDirty('kpInput');
        clearDirty('kiInput');
        clearDirty('kdInput');
        clearDirty('minPwmInput');
        if (data) {
          stopLogging();
          startLogging(data);
        }

        if (startAfterApply) {
          setActionStatus(`Started ${wheel.toUpperCase()} in ${mode.toUpperCase()} mode with ${valueLabel} ${value}.`, 'success');
        } else {
          setActionStatus(`Applied settings for ${wheel.toUpperCase()} and loaded ${valueLabel} ${value}.`, 'success');
        }
      } catch (error) {
        setConnectionState('disconnected', 'apply failed');
        setActionStatus('Apply failed. Check Wi-Fi connection and try again.', 'error');
        console.error('Apply failed:', error);
      } finally {
        setBusyState(false);
      }
    }

    function isUserEditing(elementId) {
      return document.activeElement === document.getElementById(elementId);
    }

    function setFieldValues(data) {
      document.getElementById('wheelValue').textContent = data.wheel;
      document.getElementById('modeValue').textContent = data.mode;
      document.getElementById('encoderCountValue').textContent = data.encoderCount;
      document.getElementById('targetRpmValue').textContent = Number(data.targetRpm).toFixed(2);
      document.getElementById('measuredRpmValue').textContent = Number(data.measuredRpm).toFixed(2);
      document.getElementById('linearSpeedValue').textContent = `${Number(data.linearSpeedMps).toFixed(4)} m/s`;
      document.getElementById('errorValue').textContent = Number(data.error).toFixed(2);
      document.getElementById('integralTermValue').textContent = Number(data.integralTerm).toFixed(4);
      document.getElementById('derivativeTermValue').textContent = Number(data.derivativeTerm).toFixed(4);
      document.getElementById('pidOutputValue').textContent = Number(data.outputPwm).toFixed(2);
      document.getElementById('finalPwmValue').textContent = data.finalPwm;
      document.getElementById('appliedPwmValue').textContent = data.appliedPwm;
      document.getElementById('dtSecondsValue').textContent = Number(data.dtSeconds).toFixed(4);
      document.getElementById('deltaCountValue').textContent = data.deltaCount;
      document.getElementById('openLoopAverageRpmValue').textContent = Number(data.openLoopAverageRpm).toFixed(2);
      document.getElementById('openLoopAverageWindowValue').textContent = `${data.openLoopAverageSamples} samples / ${Number(data.openLoopAverageSeconds).toFixed(2)} s`;
      document.getElementById('gainsValue').textContent = `${Number(data.kp).toFixed(3)} / ${Number(data.ki).toFixed(4)} / ${Number(data.kd).toFixed(4)}`;
      document.getElementById('minPwmValue').textContent = data.minPwm;
      document.getElementById('motorInvertValue').textContent = String(data.motorInverted);

      if (!isFieldLocked('wheel')) {
        document.getElementById('wheel').value = data.wheel;
      }

      if (!isFieldLocked('mode')) {
        document.getElementById('mode').value = data.mode;
      }

      if (!isFieldLocked('kpInput')) {
        document.getElementById('kpInput').value = data.kp;
      }

      if (!isFieldLocked('kiInput')) {
        document.getElementById('kiInput').value = data.ki;
      }

      if (!isFieldLocked('kdInput')) {
        document.getElementById('kdInput').value = data.kd;
      }

      if (!isFieldLocked('minPwmInput')) {
        document.getElementById('minPwmInput').value = data.minPwm;
      }

      if (!isFieldLocked('valueInput')) {
        if (data.mode === 'closed') {
          document.getElementById('valueInput').value = Number(data.targetRpm).toFixed(2);
        } else {
          document.getElementById('valueInput').value = data.appliedPwm;
        }
      }

      updateValueLabel();
      updateGraphModeNote(data.mode);
    }

    function addGraphSample(data) {
      graphSamples.push({
        mode: data.mode,
        target: data.mode === 'closed' ? Number(data.targetRpm) : null,
        measured: Number(data.measuredRpm)
      });

      if (graphSamples.length > maxGraphPoints) {
        graphSamples.shift();
      }
    }

    function drawGraph() {
      const width = chartCanvas.clientWidth;
      const height = chartCanvas.clientHeight;
      chartCanvas.width = width;
      chartCanvas.height = height;

      chartContext.clearRect(0, 0, width, height);

      if (graphSamples.length < 2) {
        chartContext.fillStyle = '#62727d';
        chartContext.font = '14px Trebuchet MS';
        chartContext.fillText('Waiting for data...', 20, 30);
        return;
      }

      let minValue = 0;
      let maxValue = 0;
      for (const sample of graphSamples) {
        minValue = Math.min(minValue, sample.measured);
        maxValue = Math.max(maxValue, sample.measured);

        if (sample.target !== null) {
          minValue = Math.min(minValue, sample.target);
          maxValue = Math.max(maxValue, sample.target);
        }
      }

      if (minValue === maxValue) {
        minValue -= 1;
        maxValue += 1;
      }

      const padding = 32;
      const graphWidth = width - padding * 2;
      const graphHeight = height - padding * 2;

      chartContext.strokeStyle = '#d7e5ee';
      chartContext.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {
        const y = padding + (graphHeight * i / 4);
        chartContext.beginPath();
        chartContext.moveTo(padding, y);
        chartContext.lineTo(width - padding, y);
        chartContext.stroke();
      }

      chartContext.fillStyle = '#62727d';
      chartContext.font = '12px Trebuchet MS';
      chartContext.fillText(maxValue.toFixed(1) + ' RPM', 6, padding + 4);
      chartContext.fillText(minValue.toFixed(1) + ' RPM', 6, height - padding + 4);

      function drawSeries(key, color) {
        chartContext.strokeStyle = color;
        chartContext.lineWidth = 2.5;
        chartContext.beginPath();

        let hasPoint = false;

        graphSamples.forEach((sample, index) => {
          const sampleValue = sample[key];
          if (sampleValue === null || Number.isNaN(sampleValue)) {
            return;
          }

          const x = padding + (graphWidth * index / (graphSamples.length - 1));
          const normalized = (sampleValue - minValue) / (maxValue - minValue);
          const y = height - padding - (normalized * graphHeight);
          if (!hasPoint) {
            chartContext.moveTo(x, y);
            hasPoint = true;
          } else {
            chartContext.lineTo(x, y);
          }
        });

        if (hasPoint) {
          chartContext.stroke();
        }
      }

      drawSeries('target', '#e66b2f');
      drawSeries('measured', '#0b8fbf');

      const latestSample = graphSamples[graphSamples.length - 1];
      const showTargetLegend = latestSample && latestSample.target !== null;

      if (showTargetLegend) {
        chartContext.fillStyle = '#e66b2f';
        chartContext.fillRect(width - 180, 16, 14, 4);
        chartContext.fillStyle = '#1b2f3b';
        chartContext.fillText('Target RPM', width - 160, 22);
      }

      chartContext.fillStyle = '#0b8fbf';
      chartContext.fillRect(showTargetLegend ? width - 90 : width - 150, 16, 14, 4);
      chartContext.fillStyle = '#1b2f3b';
      chartContext.fillText('Measured RPM', showTargetLegend ? width - 70 : width - 130, 22);
    }

    async function refreshStatus(forceDraw) {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        const data = await response.json();
        lastStatusSuccessTime = Date.now();
        setConnectionState('connected');
        setFieldValues(data);
        if (data.sampleReady) {
          addGraphSample(data);
        }
        if (loggingActive && data.sampleReady) {
          const elapsedSeconds = (Date.now() - logStartTime) / 1000;
          appendLogLine(formatLogSnapshot(data, elapsedSeconds));
          logLineCount++;
          setLoggerMeta(`Logging ${String(data.wheel).toUpperCase()} | ${logLineCount} samples`);
        } else if (loggingActive && !data.sampleReady) {
          setLoggerMeta(`Logging ${String(data.wheel).toUpperCase()} | waiting for first complete sample`);
        }
        if (forceDraw || graphSamples.length % 1 === 0) {
          drawGraph();
        }
        return data;
      } catch (error) {
        const secondsSinceLastSuccess = lastStatusSuccessTime > 0
          ? ((Date.now() - lastStatusSuccessTime) / 1000).toFixed(1)
          : null;
        setConnectionState(
          'disconnected',
          secondsSinceLastSuccess ? `last good update ${secondsSinceLastSuccess}s ago` : 'no response'
        );
        console.error('Status refresh failed:', error);
      }
    }

    updateValueLabel();
    wireDirtyTracking('wheel');
    wireDirtyTracking('mode');
    wireDirtyTracking('valueInput');
    wireDirtyTracking('kpInput');
    wireDirtyTracking('kiInput');
    wireDirtyTracking('kdInput');
    wireDirtyTracking('minPwmInput');
    refreshStatus(true);
    setInterval(() => refreshStatus(false), pollIntervalMs);
    window.addEventListener('resize', drawGraph);
  </script>
</body>
</html>
)rawliteral";

// Convert a wheel name from Serial text into the matching enum value.
WheelIndex parseWheelName(String wheelName) {
  wheelName.trim();
  wheelName.toLowerCase();

  for (int index = 0; index < WHEEL_COUNT; index++) {
    if (wheelName == wheels[index].name) {
      return (WheelIndex)index;
    }
  }

  return WHEEL_COUNT;
}

// Change the active wheel for testing.
// Whenever the active wheel changes, stop the robot and reset control state.
void selectWheel(WheelIndex newWheel) {
  if (newWheel >= WHEEL_COUNT) {
    return;
  }

  if (selectedWheel == newWheel) {
    return;
  }

  selectedWheel = newWheel;
  setStoppedMode();
  stopAllMotors();
  resetIntegral();
  resetEncoderTracking();

}

// Enter open-loop mode.
// In this mode the user directly commands the PWM and the controller does not
// try to correct the measured RPM.
void setOpenLoopPwm(int pwmValue) {
  prepareRunInitialization();
  controlMode = MODE_OPEN;
  runEnabled = true;
  targetRpm = 0.0f;
  openLoopPwmCommand = clampSignedPwm(pwmValue);

  // Open-loop must apply exactly the requested signed PWM after clamp.
  finalPwmCommand = openLoopPwmCommand;
  pidOutputUnclamped = (float)openLoopPwmCommand;

  applyMotorCommand(selectedWheel, openLoopPwmCommand);
}

// Enter closed-loop mode.
// The controller will try to drive measured RPM toward this target value.
void setTargetRpm(float newTargetRpm) {
  prepareRunInitialization();
  controlMode = MODE_CLOSED;
  runEnabled = true;
  targetRpm = constrain(newTargetRpm, -TARGET_RPM_LIMIT, TARGET_RPM_LIMIT);
  openLoopPwmCommand = 0;
}

void stopTestMotion() {
  runEnabled = false;
  setStoppedMode();
  resetIntegral();
  stopAllMotors();
}

// Stop hardware output immediately but keep the selected mode and target values.
// This lets the UI stay in CLOSED mode after pressing Stop.
void stopOutputsKeepState() {
  runEnabled = false;
  resetIntegral();
  stopAllMotorHardware();
  appliedPwmCommand = 0;
  finalPwmCommand = 0;
  pidOutputUnclamped = 0.0f;
  sampleReady = false;
}

void selectMode(String modeValue) {
  modeValue.trim();
  modeValue.toLowerCase();

  if (modeValue == "open") {
    stopTestMotion();
  } else if (modeValue == "closed") {
    stopTestMotion();
    controlMode = MODE_CLOSED;
    resetIntegral();
  }
}

void applyValueForCurrentMode(String valueText) {
  // Single entry point used by serial/web command styles.
  // Keeps mode-specific parsing logic in one place.
  if (controlMode == MODE_CLOSED) {
    setTargetRpm(valueText.toFloat());
  } else {
    setOpenLoopPwm(valueText.toInt());
  }
}

void sendOkResponse() {
  server.send(200, "text/plain", "OK");
}

void handleRoot() {
  server.send_P(200, "text/html", WEB_PAGE);
}

void handleStatus() {
  server.send(200, "application/json", buildStatusJson());
}

void handleSetWheel() {
  // Any state mutation from HTTP is mutex-protected.
  // This prevents races with controlLoopTask while it computes PID/output.
  if (server.hasArg("w")) {
    if (lockState()) {
      selectWheel(parseWheelName(server.arg("w")));
      unlockState();
    }
  }
  sendOkResponse();
}

void handleSetMode() {
  if (server.hasArg("mode")) {
    if (lockState()) {
      selectMode(server.arg("mode"));
      unlockState();
    }
  }
  sendOkResponse();
}

void handleSetPwm() {
  if (server.hasArg("value")) {
    if (lockState()) {
      setOpenLoopPwm(server.arg("value").toInt());
      unlockState();
    }
  }
  sendOkResponse();
}

void handleSetTarget() {
  if (server.hasArg("rpm")) {
    if (lockState()) {
      setTargetRpm(server.arg("rpm").toFloat());
      unlockState();
    }
  }
  sendOkResponse();
}

void handleSetGains() {
  // Update all gains inside one lock section so controller never sees
  // half-updated gains during one cycle.
  if (lockState()) {
    if (server.hasArg("kp")) {
      kp = server.arg("kp").toFloat();
    }
    if (server.hasArg("ki")) {
      ki = server.arg("ki").toFloat();
    }
    if (server.hasArg("kd")) {
      kd = server.arg("kd").toFloat();
    }
    unlockState();
  }
  sendOkResponse();
}

void handleSetMinPwm() {
  sendOkResponse();
}

void handleToggleMotorInvert() {
  // Toggling motor inversion while moving can be dangerous.
  // So we reset integral and stop outputs immediately.
  if (lockState()) {
    invertMotor[selectedWheel] = !invertMotor[selectedWheel];
    resetIntegral();
    stopAllMotors();
    unlockState();
  }
  sendOkResponse();
}

void handleReset() {
  // Reset means: stop motion + zero encoder/measurement history.
  if (lockState()) {
    stopTestMotion();
    resetEncoderTracking();
    unlockState();
  }
  sendOkResponse();
}

void handleStop() {
  if (lockState()) {
    stopOutputsKeepState();
    unlockState();
  }
  sendOkResponse();
}

void configureWebServer() {
  // Route table: each endpoint maps to one clear action.
  // Keeping routes centralized helps maintenance and debugging.
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/setWheel", handleSetWheel);
  server.on("/setMode", handleSetMode);
  server.on("/setPWM", handleSetPwm);
  server.on("/setTarget", handleSetTarget);
  server.on("/setGains", handleSetGains);
  server.on("/setMinPWM", handleSetMinPwm);
  server.on("/toggleMotorInvert", handleToggleMotorInvert);
  server.on("/reset", handleReset);
  server.on("/stop", handleStop);
  server.begin();
}

// High-priority deterministic control task.
// Key properties:
// - Uses vTaskDelayUntil() for fixed-period wakeups.
// - Uses millis() differences for real dt in seconds.
// - Runs both measurement and actuator update in one critical section so
//   each cycle is coherent.
void controlLoopTask(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

    // Required real-dt form:
    // now = millis()
    // dtSeconds = (now - lastControlTime) / 1000.0
    // lastControlTime = now
    unsigned long now = millis();
    float dtSec = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    if (dtSec <= 0.0f) {
      // Safety fallback if timer reading is abnormal.
      dtSec = CONTROL_PERIOD_SECONDS;
    }

    if (lockState()) {
      // currentDtSeconds is exposed in /status for verification and tuning.
      currentDtSeconds = dtSec;

      // Control-cycle order starts here:
      // 1) copy encoder count
      // 2) compute deltaCount
      // 3) compute measuredRPM
      updateMeasuredSpeed(currentDtSeconds);

      // Stop latch: output remains zero until next setTarget/setPWM enables run.
      if (!runEnabled) {
        applyMotorCommand(selectedWheel, 0);
        sampleReady = true;
        publishCycleSnapshot();
        unlockState();
        continue;
      }

      if (controlMode == MODE_CLOSED) {
        // 4) error
        // 5) derivative
        // 6) integral
        // 7) PID output
        // 8) clamp finalPWM
        // 9) apply motor command
        runClosedLoopControl(currentDtSeconds);
      } else {
        runOpenLoopControl();
      }

      // 10) publish/log all values from this same complete cycle.
      sampleReady = true;
      publishCycleSnapshot();

      unlockState();
    }
  }
}

// Lower-priority web task.
// It yields every 2ms to avoid starving other RTOS tasks.
// Control must always remain more deterministic than HTTP handling.
void webServerTask(void *parameter) {
  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// Configure all motor pins into a safe startup state.
// This avoids random motion when the ESP32 boots with everything connected.
void configureMotorPinsSafe() {
  for (int index = 0; index < WHEEL_COUNT; index++) {
    pinMode(wheels[index].pwmPin, OUTPUT);
    pinMode(wheels[index].in1Pin, OUTPUT);
    pinMode(wheels[index].in2Pin, OUTPUT);
    digitalWrite(wheels[index].pwmPin, LOW);
    digitalWrite(wheels[index].in1Pin, LOW);
    digitalWrite(wheels[index].in2Pin, LOW);
    writePwmDuty(wheels[index].pwmPin, 0);
  }
}

// Configure encoder inputs and attach interrupts to Channel A only.
// This matches the same counting method used in your PPR test file.
void configureEncoderPins() {
  for (int index = 0; index < WHEEL_COUNT; index++) {
    pinMode(wheels[index].encAPin, INPUT);
    pinMode(wheels[index].encBPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(wheels[index].encAPin), encoderHandlers[index], RISING);
  }
}

// Compute measured RPM for the selected wheel.
// Formula requested:
// rpm = (deltaCount / 374.0) * (60.0 / dt_seconds)
// We also convert wheel RPM to linear wheel speed in m/s.
// dtSec is the real elapsed time since the last call, measured in loop().
void updateMeasuredSpeed(float dtSec) {
  long copiedCounts[WHEEL_COUNT];

  // Copy volatile counts safely so the ISR cannot change them mid-read.
  // This creates a stable snapshot for this control cycle.
  noInterrupts();
  for (int index = 0; index < WHEEL_COUNT; index++) {
    copiedCounts[index] = encoderCounts[index];
  }
  interrupts();

  long deltaCount = copiedCounts[selectedWheel] - lastEncoderCounts[selectedWheel];
  lastEncoderCounts[selectedWheel] = copiedCounts[selectedWheel];
  lastRawDeltaCount = deltaCount;

  // Use real dt so the RPM calculation stays accurate even if the loop fires
  // slightly early or late.
  // Signed deltaCount gives signed RPM automatically.
  if (dtSec > 0.0f) {
    measuredRpm = (deltaCount / PPR) * (60.0f / dtSec);
  } else {
    measuredRpm = 0.0f;
  }
  linearSpeedMps = (measuredRpm / 60.0f) * WHEEL_CIRCUMFERENCE_M;

  if (controlMode == MODE_OPEN && openLoopPwmCommand != 0) {
    // Average measured RPM over open-loop run to build PWM->RPM tables.
    openLoopRpmSum += measuredRpm;
    openLoopAverageSamples++;
    openLoopAverageSeconds += dtSec;
    openLoopAverageRpm = openLoopRpmSum / openLoopAverageSamples;
  }
}

// Pure feedback PID controller.
// error = targetRPM - measuredRPM
// integral += error * dt
// derivative = (error - previousError) / dt
// output = Kp*error + Ki*integral + Kd*derivative
void runClosedLoopControl(float dtSec) {
  // If target is zero, stop immediately and clear all controller state.
  // This avoids integral residue causing restart jerk on next command.
  if (targetRpm == 0.0f) {
    integralTerm = 0.0f;
    previousError = 0.0f;
    derivativeTerm = 0.0f;
    controlError = 0.0f;
    pidOutputUnclamped = 0.0f;
    finalPwmCommand = 0;
    applyMotorCommand(selectedWheel, finalPwmCommand);
    return;
  }

  // Required equation: error = targetRPM - measuredRPM
  controlError = targetRpm - measuredRpm;

  // Required equation: derivative = (error - previousError) / dtSeconds
  if (dtSec > 0.0f) {
    derivativeTerm = (controlError - previousError) / dtSec;
  } else {
    derivativeTerm = 0.0f;
  }

  // Candidate integral using required form: integral += error * dtSeconds
  float candidateIntegral = integralTerm + (controlError * dtSec);
  if (candidateIntegral >  INTEGRAL_LIMIT) candidateIntegral =  INTEGRAL_LIMIT;
  if (candidateIntegral < -INTEGRAL_LIMIT) candidateIntegral = -INTEGRAL_LIMIT;

  // First compute unclamped output and then clamp.
  float candidateUnclampedOutput = (kp * controlError) + (ki * candidateIntegral) + (kd * derivativeTerm);
  int candidateFinalPwm = clampSignedPwm((int)candidateUnclampedOutput);

  // Anti-windup rule:
  // - If saturated high and error is positive, do not increase integral.
  // - If saturated low and error is negative, do not increase integral.
  bool blockIntegralGrowth = (candidateFinalPwm >= PWM_MAX  && controlError > 0.0f) ||
                             (candidateFinalPwm <= -PWM_MAX && controlError < 0.0f);

  if (!blockIntegralGrowth) {
    integralTerm = candidateIntegral;
  }

  // Recompute with the integral state actually accepted this cycle.
  pidOutputUnclamped = (kp * controlError) + (ki * integralTerm) + (kd * derivativeTerm);
  finalPwmCommand = clampSignedPwm((int)pidOutputUnclamped);
  previousError = controlError;

  // finalPWM is pure constrained PID output; no deadband/feedforward added.
  applyMotorCommand(selectedWheel, finalPwmCommand);
}

// Open-loop mode does not use feedback control.
// The exact signed PWM the user entered is applied with no deadband or feedforward.
void runOpenLoopControl() {
  controlError = 0.0f;
  pidOutputUnclamped = (float)openLoopPwmCommand;
  finalPwmCommand = openLoopPwmCommand;
  applyMotorCommand(selectedWheel, openLoopPwmCommand);
}

// Startup sequence:
// 1. Begin Serial
// 2. Force all motors off
// 3. Start the Pluto Wi-Fi access point and web server
// 4. Enable encoder inputs and interrupts
// 5. Reset controller state
void setup() {
  Serial.begin(115200);

  // Bring hardware to a safe known state before enabling Wi-Fi/interrupts.
  configureMotorPinsSafe();
  stopAllMotors();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  configureWebServer();

  // Create state mutex before launching tasks.
  stateMutex = xSemaphoreCreateMutex();

  configureEncoderPins();
  resetIntegral();
  resetEncoderTracking();

  lastControlTime = millis();

  // Control task priority is higher than web task for timing precision.
  // Both are pinned to core 1 here for simplicity; this still benefits from
  // RTOS scheduling isolation versus single-thread loop().
  xTaskCreatePinnedToCore(
    controlLoopTask,
    "ControlLoop",
    4096,
    nullptr,
    3,
    &controlTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    webServerTask,
    "WebServer",
    6144,
    nullptr,
    1,
    &webTaskHandle,
    0
  );

  Serial.println();
  Serial.println("ESP32 one-wheel velocity test web GUI");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Password: ");
  Serial.println(WIFI_PASSWORD);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Open http://192.168.4.1/");
}

// Main loop:
// - kept idle because RTOS tasks run control and web server handling.
void loop() {
  // Keep loop alive but do no work here; tasks do all runtime logic.
  vTaskDelay(pdMS_TO_TICKS(1000));
}