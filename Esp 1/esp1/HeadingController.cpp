#include "HeadingController.h"

float Kp_heading_rpm = 60.0f;
float Ki_heading_rpm_per_rad_s = 2.0f;
float MAX_TURN_CORRECTION_RPM = 30.0f;
bool headingControlEnabled = true;
bool invertHeadingCorrection = true;

float turnCorrectionRPM = 0.0f;
float leftRpmComposed = 0.0f;
float rightRpmComposed = 0.0f;
float targetHeadingRad = 0.0f;
float headingErrorRad = 0.0f;
float headingIntegralRadS = 0.0f;
float Kp_rotate_rpm = 10.0f;
float MAX_ROTATE_RPM = 120.0f;
float MIN_ROTATE_RPM = 60.0f;
float HEADING_TOLERANCE_DEG = 1.0f;
bool invertRotateDirection = true;

namespace {

float wrapAngleRadLocal(float angle) {
  while (angle > PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;
  return angle;
}

}  // namespace

void resetHeadingControllerState() {
  turnCorrectionRPM = 0.0f;
  leftRpmComposed = 0.0f;
  rightRpmComposed = 0.0f;
  targetHeadingRad = 0.0f;
  headingErrorRad = 0.0f;
  headingIntegralRadS = 0.0f;
}

float headingToleranceRad() {
  return HEADING_TOLERANCE_DEG * (PI_F / 180.0f);
}

void runHeadingLoopAndComposeWheelTargets(
    bool positionModeActive,
    bool moveForwardMode,
    float currentDtSeconds,
    float baseForwardRpm,
    float currentHeadingRad) {
  turnCorrectionRPM = 0.0f;
  leftRpmComposed = 0.0f;
  rightRpmComposed = 0.0f;

  if (!positionModeActive) {
    headingIntegralRadS = 0.0f;
    setAllTargetRpm(0.0f);
    return;
  }

  if (moveForwardMode) {
    if (headingControlEnabled) {
      headingErrorRad = wrapAngleRadLocal(targetHeadingRad - currentHeadingRad);

      headingIntegralRadS += headingErrorRad * currentDtSeconds;
      if (headingIntegralRadS > HEADING_INTEGRAL_LIMIT_RAD_S) headingIntegralRadS = HEADING_INTEGRAL_LIMIT_RAD_S;
      if (headingIntegralRadS < -HEADING_INTEGRAL_LIMIT_RAD_S) headingIntegralRadS = -HEADING_INTEGRAL_LIMIT_RAD_S;

      turnCorrectionRPM =
        (Kp_heading_rpm * headingErrorRad) +
        (Ki_heading_rpm_per_rad_s * headingIntegralRadS);
      turnCorrectionRPM = constrain(
        turnCorrectionRPM,
        -MAX_TURN_CORRECTION_RPM,
        MAX_TURN_CORRECTION_RPM
      );

      if (invertHeadingCorrection) {
        turnCorrectionRPM = -turnCorrectionRPM;
      }
    } else {
      headingErrorRad = 0.0f;
      headingIntegralRadS = 0.0f;
      turnCorrectionRPM = 0.0f;
    }

    const float leftRpm = clampWheelTargetRpm(baseForwardRpm - turnCorrectionRPM);
    const float rightRpm = clampWheelTargetRpm(baseForwardRpm + turnCorrectionRPM);
    leftRpmComposed = leftRpm;
    rightRpmComposed = rightRpm;

    targetRpm[WHEEL_F1] = leftRpm;
    targetRpm[WHEEL_R1] = leftRpm;
    targetRpm[WHEEL_F2] = rightRpm;
    targetRpm[WHEEL_R2] = rightRpm;
    return;
  }

  setAllTargetRpm(0.0f);
  turnCorrectionRPM = 0.0f;
}

void runRotateLoopAndComposeWheelTargets(float currentHeadingRad) {
  headingIntegralRadS = 0.0f;
  headingErrorRad = wrapAngleRadLocal(targetHeadingRad - currentHeadingRad);
  const float headingErrorDeg = fabs(headingErrorRad) * (180.0f / PI_F);

  float rotateCommandRpm = 0.0f;
  if (headingErrorDeg <= HEADING_TOLERANCE_DEG) {
    rotateCommandRpm = 0.0f;
  } else if (headingErrorDeg < 20.0f) {
    rotateCommandRpm = (headingErrorRad >= 0.0f) ? MIN_ROTATE_RPM : -MIN_ROTATE_RPM;
    rotateCommandRpm = constrain(rotateCommandRpm, -MAX_ROTATE_RPM, MAX_ROTATE_RPM);
  } else {
    rotateCommandRpm = Kp_rotate_rpm * headingErrorRad;
    rotateCommandRpm = constrain(rotateCommandRpm, -MAX_ROTATE_RPM, MAX_ROTATE_RPM);
    if (fabs(rotateCommandRpm) < MIN_ROTATE_RPM) {
      rotateCommandRpm = (rotateCommandRpm >= 0.0f) ? MIN_ROTATE_RPM : -MIN_ROTATE_RPM;
    }
    rotateCommandRpm = constrain(rotateCommandRpm, -MAX_ROTATE_RPM, MAX_ROTATE_RPM);
  }

  if (invertRotateDirection) {
    rotateCommandRpm = -rotateCommandRpm;
  }

  turnCorrectionRPM = rotateCommandRpm;
  leftRpmComposed = clampWheelTargetRpm(-rotateCommandRpm);
  rightRpmComposed = clampWheelTargetRpm(rotateCommandRpm);

  targetRpm[WHEEL_F1] = leftRpmComposed;
  targetRpm[WHEEL_R1] = leftRpmComposed;
  targetRpm[WHEEL_F2] = rightRpmComposed;
  targetRpm[WHEEL_R2] = rightRpmComposed;
}