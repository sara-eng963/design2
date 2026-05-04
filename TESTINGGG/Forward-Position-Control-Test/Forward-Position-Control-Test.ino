#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const char *WIFI_SSID = "Pluto";
const char *WIFI_PASSWORD = "12345678";
WebServer server(80);

// ============================================================
// Forward-Position-Control-Test for ESP32
// ------------------------------------------------------------
// Goal of this file:
// - Move the robot by a signed commanded distance (for example: +0.50 forward, -0.50 backward)
// - Use a simple CASCADE controller:
//
//   Outer loop (position): distance error -> forward velocity command (m/s)
//   Inner loop (velocity): target wheel RPM -> PWM for each motor
//
// Why this structure is important:
// - Position loop should NOT output PWM directly.
// - Position loop is slower/high-level and decides "how fast forward" to go.
// - Velocity loops are low-level and decide motor PWM to hit requested RPM.
// - This separation makes tuning easier and behavior more stable.
//
// What this file intentionally does NOT include:
// - No sideways motion
// - No rotation control
// - No IMU usage
// - No trajectory planning
// - No feedforward
// - No advanced state machine
// ============================================================

// -----------------------------
// Hardware constants
// -----------------------------
const float PPR = 374.0f;
const float WHEEL_DIAMETER_M = 0.097f;
const float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * 3.14159265358979323846f;

const unsigned long CONTROL_PERIOD_MS = 50;
const float CONTROL_PERIOD_SECONDS = CONTROL_PERIOD_MS / 1000.0f;

const int PWM_MAX = 255;

// -----------------------------
// Position loop tuning
// -----------------------------
// Start position controller with P-only behavior by setting Ki_pos = 0.
// This is the safest first step for distance testing.
// - If overshoot: reduce Kp_pos or MAX_VX
// - If short travel: increase Kp_pos slightly
// - If oscillation near target: reduce Kp_pos or increase tolerance
float Kp_pos = 0.8f;
float Ki_pos = 0.0f;
const float MAX_VX = 0.25f;               // m/s clamp for outer-loop output
const float POSITION_TOLERANCE = 0.02f;  // meters
const float MIN_WHEEL_RPM = 70.0f;       // Deadzone compensation floor when motion is active

// -----------------------------
// Wheel/index definitions
// -----------------------------
enum WheelIndex {
  WHEEL_R1 = 0,
  WHEEL_R2 = 1,
  WHEEL_F1 = 2,
  WHEEL_F2 = 3,
  WHEEL_COUNT = 4
};

// Per-wheel minimum PWM values from wheel velocity test.
// These are exposed for diagnostics/tuning visibility in this file.
const int minimumPwmByWheel[WHEEL_COUNT] = {
  110,  // R1
  135,  // R2
  125,  // F1
  110   // F2
};

struct WheelConfig {
  const char *name;
  uint8_t pwmPin;
  uint8_t in1Pin;
  uint8_t in2Pin;
  uint8_t encAPin;
  uint8_t encBPin;
};

// Pinout kept exactly from Wheel-Velocity-Test.ino
const WheelConfig wheels[WHEEL_COUNT] = {
  {"r1", 32, 33, 25, 35, 34},
  {"r2", 14, 26, 27, 39, 36},
  {"f1", 13, 2, 4, 16, 17},
  {"f2", 5, 23, 15, 18, 19}
};

// Encoder inversion kept exactly from Wheel-Velocity-Test.ino
const bool invertEncoder[WHEEL_COUNT] = {
  true,
  false,
  true,
  false
};

// Motor inversion kept true for all motors
bool invertMotor[WHEEL_COUNT] = {
  true,
  true,
  true,
  true
};

// -----------------------------
// Shared state (RTOS + ISR)
// -----------------------------
volatile long encoderCounts[WHEEL_COUNT] = {0, 0, 0, 0};
long lastEncoderCounts[WHEEL_COUNT] = {0, 0, 0, 0};
long lastDeltaCounts[WHEEL_COUNT] = {0, 0, 0, 0};

float measuredRpm[WHEEL_COUNT] = {0, 0, 0, 0};
float targetRpm[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityError[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityIntegral[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityDerivative[WHEEL_COUNT] = {0, 0, 0, 0};
float previousVelocityError[WHEEL_COUNT] = {0, 0, 0, 0};
float outputPwmUnclamped[WHEEL_COUNT] = {0, 0, 0, 0};
int finalPwm[WHEEL_COUNT] = {0, 0, 0, 0};
int appliedPwm[WHEEL_COUNT] = {0, 0, 0, 0};

// Per-wheel velocity PID gains (same defaults as velocity test; can tune later)
float kpVel[WHEEL_COUNT] = {5.0f, 5.5f, 4.5f, 5.5f};
float kiVel[WHEEL_COUNT] = {1.15f, 1.15f, 1.15f, 1.30f};
float kdVel[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
const float VELOCITY_INTEGRAL_LIMIT = 300.0f;

// Position loop state
bool positionModeActive = false;
float targetDistanceM = 0.0f;
float currentDistanceM = 0.0f;
float distanceErrorM = 0.0f;
float positionIntegral = 0.0f;
float vxCommand = 0.0f;
float targetWheelRpm = 0.0f;

float currentDtSeconds = CONTROL_PERIOD_SECONDS;
unsigned long lastControlTime = 0;

SemaphoreHandle_t stateMutex = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t webTaskHandle = nullptr;

bool lockState(TickType_t timeoutTicks = pdMS_TO_TICKS(20)) {
  return (stateMutex != nullptr) && (xSemaphoreTake(stateMutex, timeoutTicks) == pdTRUE);
}

void unlockState() {
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

// -----------------------------
// Hardware helpers
// -----------------------------
void writePwmDuty(uint8_t pin, uint32_t duty) {
  analogWrite(pin, duty);
}

int clampSignedPwm(int pwmValue) {
  if (pwmValue > PWM_MAX) return PWM_MAX;
  if (pwmValue < -PWM_MAX) return -PWM_MAX;
  return pwmValue;
}

void stopWheelHardware(WheelIndex wheel) {
  digitalWrite(wheels[wheel].in1Pin, LOW);
  digitalWrite(wheels[wheel].in2Pin, LOW);
  writePwmDuty(wheels[wheel].pwmPin, 0);
}

void stopAllMotorHardware() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    stopWheelHardware((WheelIndex)i);
  }
}

// Motor direction and encoder direction are separate concepts:
// - invertMotor changes how signed PWM maps to H-bridge pins
// - invertEncoder changes count sign in ISR
// They must remain independent to avoid sign confusion.
void applyMotorCommandToWheel(WheelIndex wheel, int commandedPwm) {
  int safeCommand = clampSignedPwm(commandedPwm);
  int hardwareCommand = invertMotor[wheel] ? -safeCommand : safeCommand;

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

  appliedPwm[wheel] = safeCommand;
}

void applyAllWheelMotorCommands() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    applyMotorCommandToWheel((WheelIndex)i, finalPwm[i]);
  }
}

void setAllTargetRpm(float rpmValue) {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    targetRpm[i] = rpmValue;
  }
}

void resetVelocityControllerState() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    velocityError[i] = 0.0f;
    velocityIntegral[i] = 0.0f;
    velocityDerivative[i] = 0.0f;
    previousVelocityError[i] = 0.0f;
    outputPwmUnclamped[i] = 0.0f;
    finalPwm[i] = 0;
    appliedPwm[i] = 0;
    targetRpm[i] = 0.0f;
  }
}

void resetPositionControllerState() {
  targetDistanceM = 0.0f;
  currentDistanceM = 0.0f;
  distanceErrorM = 0.0f;
  positionIntegral = 0.0f;
  vxCommand = 0.0f;
  targetWheelRpm = 0.0f;
  positionModeActive = false;
}

// -----------------------------
// Encoder ISR logic
// -----------------------------
// ISR logic kept equivalent to your working file:
// - trigger on Channel A rising edge
// - read Channel B for direction
// - apply encoder inversion before storing count
void IRAM_ATTR updateEncoderCount(WheelIndex wheel) {
  int step = (digitalRead(wheels[wheel].encBPin) == HIGH) ? 1 : -1;
  if (invertEncoder[wheel]) {
    step = -step;
  }
  encoderCounts[wheel] += step;
}

void IRAM_ATTR handleEncoderR1() { updateEncoderCount(WHEEL_R1); }
void IRAM_ATTR handleEncoderR2() { updateEncoderCount(WHEEL_R2); }
void IRAM_ATTR handleEncoderF1() { updateEncoderCount(WHEEL_F1); }
void IRAM_ATTR handleEncoderF2() { updateEncoderCount(WHEEL_F2); }

typedef void (*EncoderIsr)();
const EncoderIsr encoderHandlers[WHEEL_COUNT] = {
  handleEncoderR1,
  handleEncoderR2,
  handleEncoderF1,
  handleEncoderF2
};

String jsonEscape(const char *text) {
  return String(text);
}

// -----------------------------
// Run control / commands
// -----------------------------
void initializeNewMove(float distanceM) {
  // Safety first: stop outputs before re-arming a new move.
  stopAllMotorHardware();

  // Reset both controllers.
  resetVelocityControllerState();
  resetPositionControllerState();

  // Load new command.
  targetDistanceM = distanceM;
  positionModeActive = true;

  // Set delta-count baseline to CURRENT encoder counts.
  // This means next cycle distance starts from zero fresh.
  noInterrupts();
  for (int i = 0; i < WHEEL_COUNT; i++) {
    lastEncoderCounts[i] = encoderCounts[i];
  }
  interrupts();

  // Clear any old sampled deltas.
  for (int i = 0; i < WHEEL_COUNT; i++) {
    lastDeltaCounts[i] = 0;
  }
}

void emergencyStopAndReset() {
  // Immediate hard stop.
  stopAllMotorHardware();

  // Reset state so robot remains idle.
  resetVelocityControllerState();
  resetPositionControllerState();
}

// -----------------------------
// Control-cycle computations
// -----------------------------
// RPM formula explanation:
// deltaCounts / PPR gives revolutions in this cycle.
// Multiply by (60 / dt) to convert rev/cycle into rev/min (RPM).
void computeWheelRpmFromDelta(long deltaCount, float dtSec, float &rpmOut) {
  if (dtSec > 0.0f) {
    rpmOut = (deltaCount / PPR) * (60.0f / dtSec);
  } else {
    rpmOut = 0.0f;
  }
}

// Encoder distance conversion:
// wheelDistance = (deltaCounts / PPR) * wheelCircumference
float deltaCountsToDistanceMeters(long deltaCount) {
  return (deltaCount / PPR) * WHEEL_CIRCUMFERENCE_M;
}

void runPositionLoop(float dtSec) {
  if (!positionModeActive) {
    vxCommand = 0.0f;
    targetWheelRpm = 0.0f;
    setAllTargetRpm(0.0f);
    return;
  }

  // Outer loop: distance error -> forward velocity command.
  distanceErrorM = targetDistanceM - currentDistanceM;

  // Stop as soon as target is reached within tolerance OR crossed.
  // Crossing check is important when minimum RPM floor can step over target.
  bool reachedByTolerance = fabs(distanceErrorM) < POSITION_TOLERANCE;
  bool crossedTarget =
    (targetDistanceM >= 0.0f && currentDistanceM >= targetDistanceM) ||
    (targetDistanceM < 0.0f && currentDistanceM <= targetDistanceM);

  if (reachedByTolerance || crossedTarget) {
    setAllTargetRpm(0.0f);
    positionIntegral = 0.0f;
    positionModeActive = false;
    vxCommand = 0.0f;
    targetWheelRpm = 0.0f;
    resetVelocityControllerState();
    stopAllMotorHardware();
    return;
  }

  // Position integral starts with Ki_pos = 0.0 (P-only), but integral is
  // computed here so enabling Ki later is straightforward.
  positionIntegral += distanceErrorM * dtSec;

  vxCommand = (Kp_pos * distanceErrorM) + (Ki_pos * positionIntegral);

  // Clamp commanded forward velocity for safety and smoother behavior.
  vxCommand = constrain(vxCommand, -MAX_VX, MAX_VX);

  // Convert forward velocity (m/s) to wheel RPM.
  // For straight-line motion (forward or backward), all wheels share target RPM sign/magnitude.
  targetWheelRpm = (vxCommand / WHEEL_CIRCUMFERENCE_M) * 60.0f;

  // Enforce minimum RPM magnitude while move mode is active.
  // This avoids low-speed commands that fall into motor deadzone.
  float absTargetRpm = fabs(targetWheelRpm);
  if (absTargetRpm > 0.0f && absTargetRpm < MIN_WHEEL_RPM) {
    targetWheelRpm = (targetWheelRpm >= 0.0f) ? MIN_WHEEL_RPM : -MIN_WHEEL_RPM;
  }

  setAllTargetRpm(targetWheelRpm);

}

void runVelocityLoopForWheel(WheelIndex wheel, float dtSec) {
  // Inner wheel velocity PID:
  // error = targetRPM - measuredRPM
  // integral += error * dt
  // derivative = (error - previousError) / dt
  // outputPWM = Kp*error + Ki*integral + Kd*derivative
  // finalPWM = clamp(outputPWM, -255, 255)

  int i = (int)wheel;
  velocityError[i] = targetRpm[i] - measuredRpm[i];

  if (dtSec > 0.0f) {
    velocityDerivative[i] = (velocityError[i] - previousVelocityError[i]) / dtSec;
  } else {
    velocityDerivative[i] = 0.0f;
  }

  float candidateIntegral = velocityIntegral[i] + (velocityError[i] * dtSec);
  if (candidateIntegral > VELOCITY_INTEGRAL_LIMIT) candidateIntegral = VELOCITY_INTEGRAL_LIMIT;
  if (candidateIntegral < -VELOCITY_INTEGRAL_LIMIT) candidateIntegral = -VELOCITY_INTEGRAL_LIMIT;

  float candidateOutput =
    (kpVel[i] * velocityError[i]) +
    (kiVel[i] * candidateIntegral) +
    (kdVel[i] * velocityDerivative[i]);

  int candidateFinal = clampSignedPwm((int)candidateOutput);

  bool blockIntegralGrowth =
    (candidateFinal >= PWM_MAX && velocityError[i] > 0.0f) ||
    (candidateFinal <= -PWM_MAX && velocityError[i] < 0.0f);

  if (!blockIntegralGrowth) {
    velocityIntegral[i] = candidateIntegral;
  }

  outputPwmUnclamped[i] =
    (kpVel[i] * velocityError[i]) +
    (kiVel[i] * velocityIntegral[i]) +
    (kdVel[i] * velocityDerivative[i]);

  finalPwm[i] = clampSignedPwm((int)outputPwmUnclamped[i]);
  previousVelocityError[i] = velocityError[i];
}

// -----------------------------
// Control task (RTOS)
// -----------------------------
void controlLoopTask(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

    unsigned long now = millis();
    float dtSec = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    if (dtSec <= 0.0f) {
      dtSec = CONTROL_PERIOD_SECONDS;
    }

    if (!lockState()) {
      continue;
    }

    currentDtSeconds = dtSec;

    // ------------------------------------------------------------
    // Required control-cycle order (exact teaching sequence):
    // 1) copy encoder counts safely
    // 2) compute delta counts
    // 3) compute each wheel measured RPM
    // 4) compute current forward distance from wheel deltas
    // 5) run position loop (if active)
    // 6) run velocity PID for each wheel
    // 7) apply motor PWM
    // ------------------------------------------------------------

    long copiedCounts[WHEEL_COUNT];
    noInterrupts();
    for (int i = 0; i < WHEEL_COUNT; i++) {
      copiedCounts[i] = encoderCounts[i];
    }
    interrupts();

    float avgDeltaDistanceM = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
      long delta = copiedCounts[i] - lastEncoderCounts[i];
      lastEncoderCounts[i] = copiedCounts[i];
      lastDeltaCounts[i] = delta;

      computeWheelRpmFromDelta(delta, dtSec, measuredRpm[i]);
      avgDeltaDistanceM += deltaCountsToDistanceMeters(delta);
    }
    avgDeltaDistanceM /= (float)WHEEL_COUNT;
    currentDistanceM += avgDeltaDistanceM;

    runPositionLoop(dtSec);

    // If position mode is not active, hold motors hard-stopped and skip PID output.
    // This prevents any residual velocity-loop state from reasserting PWM and causing ringing.
    if (!positionModeActive) {
      resetVelocityControllerState();
      stopAllMotorHardware();
      unlockState();
      continue;
    }

    for (int i = 0; i < WHEEL_COUNT; i++) {
      runVelocityLoopForWheel((WheelIndex)i, dtSec);
    }

    applyAllWheelMotorCommands();

    unlockState();
  }
}

// -----------------------------
// Wi-Fi GUI interface
// -----------------------------
String buildStatusJson() {
  if (!lockState(pdMS_TO_TICKS(50))) {
    return "{\"ok\":false}";
  }

  long encoderSnapshot[WHEEL_COUNT];
  noInterrupts();
  for (int i = 0; i < WHEEL_COUNT; i++) {
    encoderSnapshot[i] = encoderCounts[i];
  }
  interrupts();

  String json = "{";
  json += "\"ok\":true,";
  json += "\"positionModeActive\":" + String(positionModeActive ? "true" : "false") + ",";
  json += "\"targetDistanceM\":" + String(targetDistanceM, 4) + ",";
  json += "\"currentDistanceM\":" + String(currentDistanceM, 4) + ",";
  json += "\"distanceErrorM\":" + String(distanceErrorM, 4) + ",";
  json += "\"positionIntegral\":" + String(positionIntegral, 4) + ",";
  json += "\"vxCommand\":" + String(vxCommand, 4) + ",";
  json += "\"targetWheelRpm\":" + String(targetWheelRpm, 3) + ",";
  json += "\"dtSeconds\":" + String(currentDtSeconds, 4) + ",";
  json += "\"kpPos\":" + String(Kp_pos, 4) + ",";
  json += "\"kiPos\":" + String(Ki_pos, 4) + ",";
  json += "\"maxVx\":" + String(MAX_VX, 4) + ",";
  json += "\"positionTolerance\":" + String(POSITION_TOLERANCE, 4) + ",";
  json += "\"kpVel\":" + String(kpVel[0], 4) + ",";
  json += "\"kiVel\":" + String(kiVel[0], 4) + ",";
  json += "\"kdVel\":" + String(kdVel[0], 4) + ",";

  json += "\"wheels\":[";
  for (int i = 0; i < WHEEL_COUNT; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"name\":\"" + jsonEscape(wheels[i].name) + "\",";
    json += "\"encoderCount\":" + String(encoderSnapshot[i]) + ",";
    json += "\"deltaCount\":" + String(lastDeltaCounts[i]) + ",";
    json += "\"measuredRpm\":" + String(measuredRpm[i], 3) + ",";
    json += "\"targetRpm\":" + String(targetRpm[i], 3) + ",";
    json += "\"errorRpm\":" + String(velocityError[i], 3) + ",";
    json += "\"integralTerm\":" + String(velocityIntegral[i], 4) + ",";
    json += "\"derivativeTerm\":" + String(velocityDerivative[i], 4) + ",";
    json += "\"outputPwm\":" + String(outputPwmUnclamped[i], 3) + ",";
    json += "\"finalPwm\":" + String(finalPwm[i]) + ",";
    json += "\"appliedPwm\":" + String(appliedPwm[i]) + ",";
    json += "\"kp\":" + String(kpVel[i], 4) + ",";
    json += "\"ki\":" + String(kiVel[i], 4) + ",";
    json += "\"kd\":" + String(kdVel[i], 4) + ",";
    json += "\"minimumPwm\":" + String(minimumPwmByWheel[i]) + ",";
    json += "\"motorInverted\":" + String(invertMotor[i] ? "true" : "false") + ",";
    json += "\"encoderInverted\":" + String(invertEncoder[i] ? "true" : "false");
    json += "}";
  }
  json += "]";
  json += "}";

  unlockState();
  return json;
}

void sendOkResponse() {
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  server.send(200, "application/json", buildStatusJson());
}

void handleMove() {
  if (!server.hasArg("distance")) {
    server.send(400, "text/plain", "Missing distance");
    return;
  }

  float distanceM = server.arg("distance").toFloat();
  if (lockState(pdMS_TO_TICKS(50))) {
    if (fabs(distanceM) < 0.0001f) {
      emergencyStopAndReset();
    } else {
      initializeNewMove(distanceM);
    }
    unlockState();
  }

  sendOkResponse();
}

void handleSetPosGains() {
  if (!lockState(pdMS_TO_TICKS(50))) {
    server.send(503, "text/plain", "State lock busy");
    return;
  }

  float newKpPos = Kp_pos;
  float newKiPos = Ki_pos;

  if (server.hasArg("kpPos")) newKpPos = server.arg("kpPos").toFloat();
  if (server.hasArg("kiPos")) newKiPos = server.arg("kiPos").toFloat();

  Kp_pos = newKpPos;
  Ki_pos = newKiPos;

  unlockState();
  sendOkResponse();
}

void handleStop() {
  if (lockState(pdMS_TO_TICKS(50))) {
    emergencyStopAndReset();
    unlockState();
  }
  sendOkResponse();
}

const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Forward Position Control Test</title>
  <style>
    :root {
      --bg: #f3f6fa;
      --panel: #ffffff;
      --ink: #173043;
      --muted: #5b6f7f;
      --line: #d7e3ec;
      --line-soft: #e2eaf0;
      --accent: #e66b2f;
      --accent-2: #2f7dbd;
      --danger: #d65555;
      --target: #2f7dbd;
      --measured: #1f9d65;
      --error: #d65555;
    }
    * { box-sizing: border-box; }
    body {
      font-family: "Trebuchet MS", "Segoe UI", sans-serif;
      margin: 0;
      color: var(--ink);
      background:
        radial-gradient(circle at 95% -10%, #dbe9f6 0%, transparent 40%),
        radial-gradient(circle at -10% 110%, #fde9dc 0%, transparent 38%),
        var(--bg);
    }
    .wrap { max-width: 1120px; margin: 0 auto; padding: 18px; }
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 16px;
      margin-bottom: 14px;
      box-shadow: 0 8px 20px rgba(23, 48, 67, 0.05);
    }
    .row { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
    .row.tight { grid-template-columns: 1fr 1fr auto; }
    input, button {
      padding: 10px;
      border-radius: 10px;
      border: 1px solid #c6d7e4;
      font-size: 0.98rem;
      width: 100%;
    }
    input:focus {
      outline: none;
      border-color: var(--accent-2);
      box-shadow: 0 0 0 2px rgba(47, 125, 189, 0.15);
    }
    button {
      cursor: pointer;
      font-weight: 700;
      transition: transform 120ms ease, filter 120ms ease;
    }
    button:hover { transform: translateY(-1px); filter: brightness(1.02); }
    .primary { background: var(--accent); color: white; border: 0; }
    .secondary { background: var(--accent-2); color: white; border: 0; }
    .danger { background: var(--danger); color: white; border: 0; }
    .hint { color: var(--muted); font-size: 0.86rem; margin-top: 8px; }
    .status-grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }
    .card { border: 1px solid var(--line); border-radius: 12px; padding: 10px; background: #fbfdff; }
    .lbl { color: var(--muted); font-size: 0.82rem; text-transform: uppercase; letter-spacing: 0.02em; }
    .val { font-size: 1.06rem; font-weight: 700; }
    .chart-wrap { border: 1px solid var(--line); border-radius: 12px; padding: 10px; background: #fcfeff; }
    canvas { width: 100%; height: 300px; display: block; }
    .legend { display: flex; gap: 12px; flex-wrap: wrap; margin-top: 8px; font-size: 0.87rem; color: var(--muted); }
    .dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; margin-right: 6px; }
    table { width: 100%; border-collapse: collapse; }
    th, td { border-bottom: 1px solid var(--line-soft); text-align: left; padding: 8px; font-size: 0.90rem; }
    .toolbar { display: flex; justify-content: space-between; gap: 10px; align-items: center; margin-bottom: 8px; flex-wrap: wrap; }
    @media (max-width: 980px) {
      .row, .row.tight, .status-grid { grid-template-columns: 1fr; }
      canvas { height: 240px; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="panel">
      <h2>Forward Position Control Test</h2>
      <div class="row">
        <input id="distanceInput" type="number" step="0.01" value="0.50" placeholder="Distance meters (+fwd, -back)">
        <button class="primary" onclick="startMove()">Move Distance</button>
        <button class="danger" onclick="stopMove()">Stop All Motors</button>
      </div>
      <div class="row" style="margin-top:10px;">
        <input id="kpPosInput" type="number" step="0.01" value="0.8" placeholder="Position Kp">
        <input id="kiPosInput" type="number" step="0.001" value="0.0" placeholder="Position Ki">
        <button class="secondary" onclick="applyPosGains()">Apply Position Gains</button>
      </div>
      <div class="hint">Use positive distance for forward and negative distance for backward. Velocity gains are kept per wheel from firmware defaults and shown read-only below.</div>
      <p id="msg">Idle</p>
    </div>

    <div class="panel">
      <div class="status-grid">
        <div class="card"><div class="lbl">Position Active</div><div class="val" id="activeVal">false</div></div>
        <div class="card"><div class="lbl">Target Distance (m)</div><div class="val" id="targetDistVal">0.0000</div></div>
        <div class="card"><div class="lbl">Current Distance (m)</div><div class="val" id="currentDistVal">0.0000</div></div>
        <div class="card"><div class="lbl">Distance Error (m)</div><div class="val" id="errorVal">0.0000</div></div>
        <div class="card"><div class="lbl">Position Integral</div><div class="val" id="posIntegralVal">0.0000</div></div>
        <div class="card"><div class="lbl">vx Command (m/s)</div><div class="val" id="vxVal">0.0000</div></div>
        <div class="card"><div class="lbl">Target Wheel RPM</div><div class="val" id="targetWheelRpmVal">0.000</div></div>
        <div class="card"><div class="lbl">dt (s)</div><div class="val" id="dtVal">0.0000</div></div>
        <div class="card"><div class="lbl">Position Gains</div><div class="val" id="posGainsVal">0.8000 / 0.0000</div></div>
      </div>
    </div>

    <div class="panel">
      <div class="toolbar">
        <h3 style="margin:0;">Position Loop Response</h3>
        <button class="secondary" style="max-width:220px;" onclick="clearGraph()">Clear Graph History</button>
      </div>
      <div class="chart-wrap">
        <canvas id="posChart" width="1040" height="320"></canvas>
        <div class="legend">
          <span><span class="dot" style="background: var(--target);"></span>Target Distance</span>
          <span><span class="dot" style="background: var(--measured);"></span>Current Distance</span>
          <span><span class="dot" style="background: var(--error);"></span>Distance Error</span>
        </div>
      </div>
    </div>

    <div class="panel">
      <h3>Per-Wheel Status</h3>
      <table>
        <thead>
          <tr>
            <th>Wheel</th><th>Enc</th><th>Delta</th><th>Measured RPM</th><th>Target RPM</th>
            <th>Error</th><th>I</th><th>D</th><th>Output PWM</th><th>Final PWM</th><th>Applied PWM</th><th>Min PWM</th>
          </tr>
        </thead>
        <tbody id="wheelBody"></tbody>
      </table>
    </div>
  </div>

  <script>
    const dirtyFields = new Set();

    const graphState = {
      t: [],
      target: [],
      current: [],
      error: [],
      maxPoints: 320,
      t0Ms: null
    };

    function markDirty(elementId) {
      dirtyFields.add(elementId);
    }

    function clearDirty(elementId) {
      dirtyFields.delete(elementId);
    }

    function isUserEditing(elementId) {
      return document.activeElement === document.getElementById(elementId);
    }

    function isFieldLocked(elementId) {
      return isUserEditing(elementId) || dirtyFields.has(elementId);
    }

    function wireDirtyTracking(elementId) {
      const element = document.getElementById(elementId);
      if (!element) return;
      element.addEventListener('input', () => markDirty(elementId));
      element.addEventListener('change', () => markDirty(elementId));
    }

    async function startMove() {
      const d = Number(document.getElementById('distanceInput').value || 0);
      clearGraph();
      await fetch(`/move?distance=${encodeURIComponent(d)}`, { cache: 'no-store' });
      document.getElementById('msg').textContent = `Move command sent: ${d.toFixed(2)} m`;
      refreshStatus();
    }

    async function applyPosGains() {
      const kpPos = document.getElementById('kpPosInput').value;
      const kiPos = document.getElementById('kiPosInput').value;
      try {
        const response = await fetch(`/setPosGains?kpPos=${encodeURIComponent(kpPos)}&kiPos=${encodeURIComponent(kiPos)}`, { cache: 'no-store' });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        clearDirty('kpPosInput');
        clearDirty('kiPosInput');
        document.getElementById('msg').textContent = `Position gains updated: Kp_pos=${kpPos}, Ki_pos=${kiPos}`;
        refreshStatus();
      } catch (e) {
        document.getElementById('msg').textContent = 'Failed to apply position gains';
        console.error('applyPosGains failed:', e);
      }
    }

    async function stopMove() {
      await fetch('/stop', { cache: 'no-store' });
      document.getElementById('msg').textContent = 'Stopped';
      refreshStatus();
    }

    function clearGraph() {
      graphState.t = [];
      graphState.target = [];
      graphState.current = [];
      graphState.error = [];
      graphState.t0Ms = null;
      drawGraph();
    }

    function pushGraphSample(data) {
      const nowMs = Date.now();
      if (graphState.t0Ms === null) graphState.t0Ms = nowMs;

      const tSec = (nowMs - graphState.t0Ms) / 1000.0;
      const target = Number(data.targetDistanceM);
      const current = Number(data.currentDistanceM);
      const error = Number(data.distanceErrorM);

      if (!Number.isFinite(target) || !Number.isFinite(current) || !Number.isFinite(error)) return;

      graphState.t.push(tSec);
      graphState.target.push(target);
      graphState.current.push(current);
      graphState.error.push(error);

      if (graphState.t.length > graphState.maxPoints) {
        graphState.t.shift();
        graphState.target.shift();
        graphState.current.shift();
        graphState.error.shift();
      }
    }

    function drawLine(ctx, xData, yData, xMin, xMax, yMin, yMax, color) {
      if (xData.length < 2) return;
      const w = ctx.canvas.width;
      const h = ctx.canvas.height;
      const left = 52;
      const right = 14;
      const top = 12;
      const bottom = 32;
      const sx = (w - left - right) / Math.max(1e-6, (xMax - xMin));
      const sy = (h - top - bottom) / Math.max(1e-6, (yMax - yMin));

      ctx.beginPath();
      for (let i = 0; i < xData.length; i++) {
        const x = left + (xData[i] - xMin) * sx;
        const y = h - bottom - (yData[i] - yMin) * sy;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    function drawGraph() {
      const c = document.getElementById('posChart');
      const ctx = c.getContext('2d');
      if (!ctx) return;

      const dpr = window.devicePixelRatio || 1;
      const cssWidth = c.clientWidth || 1040;
      const cssHeight = c.clientHeight || 300;
      const targetW = Math.floor(cssWidth * dpr);
      const targetH = Math.floor(cssHeight * dpr);
      if (c.width !== targetW || c.height !== targetH) {
        c.width = targetW;
        c.height = targetH;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

      const w = cssWidth;
      const h = cssHeight;
      const left = 52;
      const right = 14;
      const top = 12;
      const bottom = 32;

      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(0, 0, w, h);

      const t = graphState.t;
      if (t.length < 2) {
        ctx.fillStyle = '#5b6f7f';
        ctx.font = '13px Trebuchet MS';
        ctx.fillText('Graph waiting for live samples...', left, h / 2);
        return;
      }

      const allY = [...graphState.target, ...graphState.current, ...graphState.error];
      let yMin = Math.min(...allY);
      let yMax = Math.max(...allY);
      if (Math.abs(yMax - yMin) < 1e-4) {
        yMin -= 0.1;
        yMax += 0.1;
      } else {
        const pad = 0.1 * (yMax - yMin);
        yMin -= pad;
        yMax += pad;
      }
      const xMin = t[0];
      const xMax = t[t.length - 1];

      ctx.strokeStyle = '#d7e3ec';
      ctx.lineWidth = 1;
      ctx.strokeRect(left, top, w - left - right, h - top - bottom);

      const yTicks = 5;
      ctx.font = '12px Trebuchet MS';
      ctx.fillStyle = '#5b6f7f';
      for (let i = 0; i <= yTicks; i++) {
        const p = i / yTicks;
        const y = h - bottom - p * (h - top - bottom);
        const value = yMin + p * (yMax - yMin);
        ctx.beginPath();
        ctx.moveTo(left, y);
        ctx.lineTo(w - right, y);
        ctx.strokeStyle = '#eef3f8';
        ctx.stroke();
        ctx.fillText(value.toFixed(3), 6, y + 4);
      }

      ctx.fillText('Time (s)', w / 2 - 18, h - 8);
      ctx.save();
      ctx.translate(12, h / 2 + 24);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText('Distance (m)', 0, 0);
      ctx.restore();

      drawLine(ctx, t, graphState.target, xMin, xMax, yMin, yMax, '#2f7dbd');
      drawLine(ctx, t, graphState.current, xMin, xMax, yMin, yMax, '#1f9d65');
      drawLine(ctx, t, graphState.error, xMin, xMax, yMin, yMax, '#d65555');
    }

    function setText(id, value) {
      document.getElementById(id).textContent = value;
    }

    async function refreshStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        const data = await response.json();
        if (!data.ok) return;

        setText('activeVal', String(data.positionModeActive));
        setText('targetDistVal', Number(data.targetDistanceM).toFixed(4));
        setText('currentDistVal', Number(data.currentDistanceM).toFixed(4));
        setText('errorVal', Number(data.distanceErrorM).toFixed(4));
        setText('posIntegralVal', Number(data.positionIntegral).toFixed(4));
        setText('vxVal', Number(data.vxCommand).toFixed(4));
        setText('targetWheelRpmVal', Number(data.targetWheelRpm).toFixed(3));
        setText('dtVal', Number(data.dtSeconds).toFixed(4));
        setText('posGainsVal', `${Number(data.kpPos).toFixed(4)} / ${Number(data.kiPos).toFixed(4)}`);
        const kpPosEl = document.getElementById('kpPosInput');
        const kiPosEl = document.getElementById('kiPosInput');
        if (!isFieldLocked('kpPosInput')) {
          kpPosEl.value = Number(data.kpPos).toFixed(4);
        }
        if (!isFieldLocked('kiPosInput')) {
          kiPosEl.value = Number(data.kiPos).toFixed(4);
        }

        pushGraphSample(data);
        drawGraph();

        const body = document.getElementById('wheelBody');
        body.innerHTML = '';
        for (const w of data.wheels) {
          const tr = document.createElement('tr');
          tr.innerHTML = `<td>${w.name.toUpperCase()}</td><td>${w.encoderCount}</td><td>${w.deltaCount}</td><td>${Number(w.measuredRpm).toFixed(3)}</td><td>${Number(w.targetRpm).toFixed(3)}</td><td>${Number(w.errorRpm).toFixed(3)}</td><td>${Number(w.integralTerm).toFixed(3)}</td><td>${Number(w.derivativeTerm).toFixed(3)}</td><td>${Number(w.outputPwm).toFixed(3)}</td><td>${w.finalPwm}</td><td>${w.appliedPwm}</td><td>${w.minimumPwm}</td>`;
          body.appendChild(tr);
        }
      } catch (e) {
        document.getElementById('msg').textContent = 'Connection failed';
      }
    }

    window.addEventListener('resize', drawGraph);
    wireDirtyTracking('kpPosInput');
    wireDirtyTracking('kiPosInput');
    clearGraph();
    refreshStatus();
    setInterval(refreshStatus, 200);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", WEB_PAGE);
}

void configureWebServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/move", handleMove);
  server.on("/setPosGains", handleSetPosGains);
  server.on("/stop", handleStop);
  server.begin();
}

void webServerTask(void *parameter) {
  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// -----------------------------
// Setup/loop
// -----------------------------
void configureMotorPinsSafe() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(wheels[i].pwmPin, OUTPUT);
    pinMode(wheels[i].in1Pin, OUTPUT);
    pinMode(wheels[i].in2Pin, OUTPUT);
    digitalWrite(wheels[i].pwmPin, LOW);
    digitalWrite(wheels[i].in1Pin, LOW);
    digitalWrite(wheels[i].in2Pin, LOW);
    writePwmDuty(wheels[i].pwmPin, 0);
  }
}

void configureEncoderPins() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(wheels[i].encAPin, INPUT);
    pinMode(wheels[i].encBPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(wheels[i].encAPin), encoderHandlers[i], RISING);
  }
}

void setup() {
  Serial.begin(115200);

  configureMotorPinsSafe();
  stopAllMotorHardware();
  configureEncoderPins();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  configureWebServer();

  stateMutex = xSemaphoreCreateMutex();

  if (lockState(pdMS_TO_TICKS(50))) {
    resetVelocityControllerState();
    resetPositionControllerState();

    noInterrupts();
    for (int i = 0; i < WHEEL_COUNT; i++) {
      lastEncoderCounts[i] = encoderCounts[i];
      lastDeltaCounts[i] = 0;
    }
    interrupts();

    unlockState();
  }

  lastControlTime = millis();

  xTaskCreatePinnedToCore(
    controlLoopTask,
    "ControlLoop",
    6144,
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
  Serial.println("Forward-Position-Control-Test ready (Wi-Fi GUI).");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Password: ");
  Serial.println(WIFI_PASSWORD);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Open http://192.168.4.1/");
}

void loop() {
  // Runtime logic is handled by RTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
