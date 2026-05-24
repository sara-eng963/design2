#pragma once

#include <Arduino.h>

#include "HardwareConfig.h"

extern int appliedPwm[WHEEL_COUNT];

void configureMotorPinsSafe();
void stopWheelHardware(WheelIndex wheel);
void stopAllMotorHardware();
int clampSignedPwm(int pwmValue);
void applyMotorCommandToWheel(WheelIndex wheel, int commandedPwm);
void applyAllWheelMotorCommands(const int commandedPwm[WHEEL_COUNT]);