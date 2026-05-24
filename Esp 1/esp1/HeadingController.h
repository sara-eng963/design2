#pragma once

#include <cstdint>

#include "WheelVelocityController.h"

extern float Kp_heading_rpm;
extern float Ki_heading_rpm_per_rad_s;
extern float MAX_TURN_CORRECTION_RPM;
extern bool headingControlEnabled;
extern bool invertHeadingCorrection;

extern float turnCorrectionRPM;
extern float leftRpmComposed;
extern float rightRpmComposed;
extern float targetHeadingRad;
extern float headingErrorRad;
extern float headingIntegralRadS;
extern float Kp_rotate_rpm;
extern float MAX_ROTATE_RPM;
extern float MIN_ROTATE_RPM;
extern float HEADING_TOLERANCE_DEG;
extern bool invertRotateDirection;

constexpr float HEADING_INTEGRAL_LIMIT_RAD_S = 2.0f;

void resetHeadingControllerState();
void runHeadingLoopAndComposeWheelTargets(
    bool positionModeActive,
    bool moveForwardMode,
    float currentDtSeconds,
    float baseForwardRpm,
    float currentHeadingRad);
void runRotateLoopAndComposeWheelTargets(float currentHeadingRad);
float headingToleranceRad();