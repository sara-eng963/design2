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

float kpVel[WHEEL_COUNT] = {6.5f, 6.5f, 7.0f, 6.5f};
float kiVel[WHEEL_COUNT] = {0.15f, 0.15f, 0.15f, 0.15f};
float kdVel[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

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

  velocityIntegral[i] += velocityError[i] * dtSec;

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
