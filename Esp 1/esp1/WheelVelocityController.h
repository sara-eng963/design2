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

void setAllTargetRpm(float rpmValue);
void resetVelocityControllerState();
void runVelocityLoopForWheel(WheelIndex wheel, float dtSec);