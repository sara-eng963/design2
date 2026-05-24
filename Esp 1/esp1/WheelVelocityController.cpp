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

float kpVel[WHEEL_COUNT] = {5.0f, 5.5f, 4.5f, 5.5f};
float kiVel[WHEEL_COUNT] = {1.15f, 1.15f, 1.15f, 1.30f};
float kdVel[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

float clampWheelTargetRpm(float rpm) {
  if (rpm > MAX_WHEEL_TARGET_RPM) return MAX_WHEEL_TARGET_RPM;
  if (rpm < -MAX_WHEEL_TARGET_RPM) return -MAX_WHEEL_TARGET_RPM;
  return rpm;
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

void runVelocityLoopForWheel(WheelIndex wheel, float dtSec) {
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