#pragma once

#include <Arduino.h>

#include "HardwareConfig.h"

extern volatile long encoderCounts[WHEEL_COUNT];
extern long lastEncoderCounts[WHEEL_COUNT];
extern long lastDeltaCounts[WHEEL_COUNT];
extern float measuredRpm[WHEEL_COUNT];

void configureEncoderPins();
void computeWheelRpmFromDelta(long deltaCount, float dtSec, float &rpmOut);