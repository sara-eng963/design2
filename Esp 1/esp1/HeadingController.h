#pragma once

#include <cstdint>

#include "WheelVelocityController.h"

extern float Kp_heading_rpm;
extern float Ki_heading_rpm_per_rad_s;
extern bool headingControlEnabled;
extern bool headingHoldActive;
extern bool invertHeadingCorrection;

extern float turnCorrectionRPM;
extern float leftRpmComposed;
extern float rightRpmComposed;
extern float targetHeadingRad;
extern float headingErrorRad;
extern float headingIntegralRadS;
extern float Kp_rotate_rpm;
extern float Ki_rotate_rpm_per_rad_s;
extern float rotateIntegralRadS;
extern float HEADING_TOLERANCE_DEG;
extern bool invertRotateDirection;

void resetHeadingControllerState();
void runHeadingLoopAndComposeWheelTargets(
    bool positionModeActive,
    bool moveForwardMode,
    float currentDtSeconds,
    float baseForwardRpm,
    float currentHeadingRad);
void runRotateLoopAndComposeWheelTargets(float currentHeadingRad);
float headingToleranceRad();
