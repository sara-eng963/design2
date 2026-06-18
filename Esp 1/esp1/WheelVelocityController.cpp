#include "WheelVelocityController.h"

#include "EncoderManager.h"
#include "MotorDriver.h"

float targetRpm[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityError[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityIntegral[WHEEL_COUNT] = {0, 0, 0, 0};
float velocityDerivative[WHEEL_COUNT] = {0, 0, 0, 0};
float previousVelocityError[WHEEL_COUNT] = {0, 0, 0, 0};
float outputPwmUnclamped[WHEEL_COUNT] = {0, 0, 0, 0};
int finalPwm[WHEEL_COUNT] = {0, 0, 0, 0};

// Per-wheel velocity PID gains in code channel order: R1, R2, F1, F2.
//
// Derivation (June 2026, VTEST + AUTO ROT data):
//   R1 (rear-right) — highest static friction, needs KP=8 to produce 280 PWM
//                      at 35 RPM step (saturates the drive) plus strong KI.
//   R2 (rear-left)  — oscillates above KP≈7–8; KP=6 is the sweet spot.
//   F1 (front-right)— low friction, erratic at KP>5; reduced vs original.
//   F2 (front-left) — similar to R2.
//   KI raised 3–5× vs original: anti-windup now prevents runaway, so integral
//   can safely overcome residual static friction without winding up.
// Values derived from Forward-Position-With-Heading-Control-Test (same hardware,
// same 50ms loop, confirmed working for straight motion):
//   R1=8.1  R2=7.6  F1=7.0  F2=7.8
// KI raised from original 0.08–0.13 to 0.25 to overcome directional friction
// asymmetry during rotation (left-wheel stall observed in-field).
// KD = 0 on purpose — both reference tests showed KD amplifies encoder noise.
float kpVel[WHEEL_COUNT] = {8.1f, 7.6f, 7.0f, 7.8f};
float kiVel[WHEEL_COUNT] = {0.25f, 0.25f, 0.20f, 0.25f};
float kdVel[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

void setAllTargetRpm(float rpmValue) {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    targetRpm[i] = rpmValue;
  }
}

void applyWheelTargetRpmTrims() {
  targetRpm[WHEEL_F1] *= TRIM_F1;
  targetRpm[WHEEL_R1] *= TRIM_R1;
  targetRpm[WHEEL_F2] *= TRIM_F2;
  targetRpm[WHEEL_R2] *= TRIM_R2;
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

void runVelocityLoopForWheel(WheelIndex wheel, float dtSec) {
  int i = (int)wheel;
  velocityError[i] = targetRpm[i] - measuredRpm[i];

  if (dtSec > 0.0f) {
    velocityDerivative[i] = (velocityError[i] - previousVelocityError[i]) / dtSec;
  } else {
    velocityDerivative[i] = 0.0f;
  }

  velocityIntegral[i] += velocityError[i] * dtSec;

  // Anti-windup: clamp integral so its PWM contribution never exceeds ±255.
  // Without this, stalled wheels wind up the integral for seconds and lurch
  // when static friction is finally overcome.
  if (kiVel[i] > 0.0f) {
    const float maxIntegral = 255.0f / kiVel[i];
    if (velocityIntegral[i] >  maxIntegral) velocityIntegral[i] =  maxIntegral;
    if (velocityIntegral[i] < -maxIntegral) velocityIntegral[i] = -maxIntegral;
  }

  outputPwmUnclamped[i] =
    (kpVel[i] * velocityError[i]) +
    (kiVel[i] * velocityIntegral[i]) +
    (kdVel[i] * velocityDerivative[i]);

  int finalPwmCommand = (int)outputPwmUnclamped[i];

  if (targetRpm[i] > 0.0f && finalPwmCommand < 0) {
    finalPwmCommand = 0;
  }
  if (targetRpm[i] < 0.0f && finalPwmCommand > 0) {
    finalPwmCommand = 0;
  }
  if (targetRpm[i] == 0.0f) {
    finalPwmCommand = 0;
  }

  finalPwm[i] = clampSignedPwm(finalPwmCommand);
  previousVelocityError[i] = velocityError[i];
}
