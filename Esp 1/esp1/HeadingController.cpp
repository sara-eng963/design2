#include "HeadingController.h"

float Kp_heading_rpm = 220.0f;
float Ki_heading_rpm_per_rad_s = 30.0f;
bool headingControlEnabled = true;
bool headingHoldActive = false;
bool invertHeadingCorrection = true;

float turnCorrectionRPM = 0.0f;
float leftRpmComposed = 0.0f;
float rightRpmComposed = 0.0f;
float targetHeadingRad = 0.0f;
float headingErrorRad = 0.0f;
float headingIntegralRadS = 0.0f;
float Kp_rotate_rpm = 23.0f;
float Ki_rotate_rpm_per_rad_s = 5.0f;
float rotateIntegralRadS = 0.0f;
float HEADING_TOLERANCE_DEG = 1.1f;
bool invertRotateDirection = true;

extern float currentDtSeconds;

namespace {

constexpr float HEADING_HOLD_TOLERANCE_DEG = 1.0f;
constexpr float HEADING_HOLD_TOLERANCE_RAD =
  HEADING_HOLD_TOLERANCE_DEG * (PI_F / 180.0f);

float wrapAngleRadLocal(float angle) {
  // Remember requested direction before wrapping.
  const float originalAngle = angle;

  while (angle > PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;

  constexpr float DIRECTION_TOLERANCE_RAD = 3.0f * (PI_F / 180.0f);

  if (fabs(fabs(angle) - PI_F) <= DIRECTION_TOLERANCE_RAD) {
    // Near 180 degrees, preserve the original requested direction.
    angle = (originalAngle >= 0.0f) ? fabs(angle) : -fabs(angle);
  }

  return angle;
}

}  // namespace

void resetHeadingControllerState() {
  headingHoldActive = false;
  turnCorrectionRPM = 0.0f;
  leftRpmComposed = 0.0f;
  rightRpmComposed = 0.0f;
  targetHeadingRad = 0.0f;
  headingErrorRad = 0.0f;
  headingIntegralRadS = 0.0f;
  rotateIntegralRadS = 0.0f;
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

  if (!positionModeActive && !headingHoldActive) {
    headingIntegralRadS = 0.0f;
    setAllTargetRpm(0.0f);
    return;
  }

  if (moveForwardMode || headingHoldActive) {
    if (headingControlEnabled) {
      headingErrorRad = wrapAngleRadLocal(targetHeadingRad - currentHeadingRad);

      if (fabs(headingErrorRad) <= HEADING_HOLD_TOLERANCE_RAD) {
        headingIntegralRadS = 0.0f;
        turnCorrectionRPM = 0.0f;
      } else {
        headingIntegralRadS += headingErrorRad * currentDtSeconds;

        turnCorrectionRPM =
          (Kp_heading_rpm * headingErrorRad) +
          (Ki_heading_rpm_per_rad_s * headingIntegralRadS);

        if (invertHeadingCorrection) {
          turnCorrectionRPM = -turnCorrectionRPM;
        }
      }
    } else {
      headingErrorRad = 0.0f;
      headingIntegralRadS = 0.0f;
      turnCorrectionRPM = 0.0f;
    }

    const float leftRpm = baseForwardRpm - turnCorrectionRPM;
    const float rightRpm = baseForwardRpm + turnCorrectionRPM;
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
  headingErrorRad = wrapAngleRadLocal(targetHeadingRad - currentHeadingRad);
  rotateIntegralRadS += headingErrorRad * currentDtSeconds;
  const float headingErrorDeg = fabs(headingErrorRad) * (180.0f / PI_F);

  float rotateCommandRpm =
    (Kp_rotate_rpm * headingErrorRad) +
    (Ki_rotate_rpm_per_rad_s * rotateIntegralRadS);

  if (headingErrorDeg <= HEADING_TOLERANCE_DEG) {
    rotateCommandRpm = 0.0f;
  }

  if (invertRotateDirection) {
    rotateCommandRpm = -rotateCommandRpm;
  }

  turnCorrectionRPM = rotateCommandRpm;
  leftRpmComposed = -rotateCommandRpm;
  rightRpmComposed = rotateCommandRpm;

  targetRpm[WHEEL_F1] = leftRpmComposed;
  targetRpm[WHEEL_R1] = leftRpmComposed;
  targetRpm[WHEEL_F2] = rightRpmComposed;
  targetRpm[WHEEL_R2] = rightRpmComposed;
}
