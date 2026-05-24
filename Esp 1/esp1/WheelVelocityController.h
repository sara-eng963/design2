#pragma once

#include "HardwareConfig.h"

extern float targetRpm[WHEEL_COUNT];
extern float velocityError[WHEEL_COUNT];
extern float velocityIntegral[WHEEL_COUNT];
extern float velocityDerivative[WHEEL_COUNT];
extern float previousVelocityError[WHEEL_COUNT];
extern float outputPwmUnclamped[WHEEL_COUNT];
extern int finalPwm[WHEEL_COUNT];

extern float kpVel[WHEEL_COUNT];
extern float kiVel[WHEEL_COUNT];
extern float kdVel[WHEEL_COUNT];

constexpr float MAX_WHEEL_TARGET_RPM = 220.0f;
constexpr float VELOCITY_INTEGRAL_LIMIT = 300.0f;

float clampWheelTargetRpm(float rpm);
void setAllTargetRpm(float rpmValue);
void resetVelocityControllerState();
void runVelocityLoopForWheel(WheelIndex wheel, float dtSec);