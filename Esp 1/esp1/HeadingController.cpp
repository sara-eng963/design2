#include "HeadingController.h"

#include "MotorDriver.h"

float Kp_heading_rpm = 260.0f;
float Ki_heading_rpm_per_rad_s = 0.0f;
float Kd_heading_rpm_per_rad_s = 25.0f;

float FINAL_HEADING_KP = 600.0f;
float FINAL_HEADING_KI = 0.0f;
float FINAL_HEADING_KD = 0.0f;
bool headingWithinTolerance = false;
bool headingControlEnabled = true;
bool headingHoldActive = false;
bool invertHeadingCorrection = true;

float turnCorrectionRPM = 0.0f;
float leftRpmComposed = 0.0f;
float rightRpmComposed = 0.0f;
float targetHeadingRad = 0.0f;
float headingErrorRad = 0.0f;
float headingIntegralRadS = 0.0f;
float previousHeadingErrorRad = 0.0f;
float Kp_rotate_rpm = 100.0f;
float Ki_rotate_rpm_per_rad_s = 5.0f;
float MAX_ROTATE_RPM = 90.0f;
float rotateIntegralRadS = 0.0f;
float HEADING_TOLERANCE_DEG = 1.1f;
bool invertRotateDirection = true;

extern float currentDtSeconds;

namespace {

// Rotate-only wheel magnitude factors.
// Actual physical mapping:
// F1 = front right
// R1 = rear right
// F2 = front left
// R2 = rear left
//
// Residual after ROTATE 90 then ROTATE 0:
// shifted left 0.04 m and forward 0.02 m.
//
// Small follow-up correction:
// keep reducing front contribution to cut forward drift, but ease left-side
// dominance back toward the right side to reduce left drift.
constexpr float ROTATE_TRIM_F1 = 0.89f;  // front right
constexpr float ROTATE_TRIM_R1 = 1.06f;  // rear right
constexpr float ROTATE_TRIM_F2 = 0.94f;  // front left
constexpr float ROTATE_TRIM_R2 = 1.11f;  // rear left

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
  previousHeadingErrorRad = 0.0f;
  headingWithinTolerance = false;
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

  if (!headingControlEnabled || !headingHoldActive) {
    headingWithinTolerance = false;
    headingIntegralRadS = 0.0f;
    turnCorrectionRPM = 0.0f;
    setAllTargetRpm(0.0f);
    return;
  }

  if (moveForwardMode || headingHoldActive) {
    headingErrorRad = wrapAngleRadLocal(targetHeadingRad - currentHeadingRad);

    float headingErrorRateRadS = 0.0f;
    if (currentDtSeconds > 0.0f) {
      headingErrorRateRadS = (headingErrorRad - previousHeadingErrorRad) / currentDtSeconds;
    }

    headingWithinTolerance = fabs(headingErrorRad) <= headingToleranceRad();

    if (headingWithinTolerance) {
      headingIntegralRadS = 0.0f;
      turnCorrectionRPM = 0.0f;
      previousHeadingErrorRad = headingErrorRad;

      leftRpmComposed = baseForwardRpm;
      rightRpmComposed = baseForwardRpm;

      if (!positionModeActive) {
        leftRpmComposed = 0.0f;
        rightRpmComposed = 0.0f;
        setAllTargetRpm(0.0f);
        resetVelocityControllerState();
        stopAllMotorHardware();
        return;
      }

      targetRpm[WHEEL_F1] = baseForwardRpm;
      targetRpm[WHEEL_R1] = baseForwardRpm;
      targetRpm[WHEEL_F2] = baseForwardRpm;
      targetRpm[WHEEL_R2] = baseForwardRpm;
      applyWheelTargetRpmTrims();
      return;
    }

    headingIntegralRadS += headingErrorRad * currentDtSeconds;

    // Near-zone threshold shared by all heading contexts.
    const float headingNearZoneRad = (HEADING_TOLERANCE_DEG * 8.0f) * (PI_F / 180.0f);
    const bool inNearZone = fabs(headingErrorRad) <= headingNearZoneRad;

    if (!positionModeActive || inNearZone) {
      // Final heading hold OR near-zone during MOVE: weaker final controller, no max clamp.
      turnCorrectionRPM =
        (FINAL_HEADING_KP * headingErrorRad) +
        (FINAL_HEADING_KI * headingIntegralRadS) +
        (FINAL_HEADING_KD * headingErrorRateRadS);
      if (inNearZone) headingIntegralRadS = 0.0f;  // reset integral when near target
    } else {
      turnCorrectionRPM =
        (Kp_heading_rpm * headingErrorRad) +
        (Ki_heading_rpm_per_rad_s * headingIntegralRadS) +
        (Kd_heading_rpm_per_rad_s * headingErrorRateRadS);
    }

    previousHeadingErrorRad = headingErrorRad;

    if (invertHeadingCorrection) {
      turnCorrectionRPM = -turnCorrectionRPM;
    }

    const float leftRpm = baseForwardRpm - turnCorrectionRPM;
    const float rightRpm = baseForwardRpm + turnCorrectionRPM;
    leftRpmComposed = leftRpm;
    rightRpmComposed = rightRpm;

    targetRpm[WHEEL_F1] = leftRpm;
    targetRpm[WHEEL_R1] = leftRpm;
    targetRpm[WHEEL_F2] = rightRpm;
    targetRpm[WHEEL_R2] = rightRpm;
    applyWheelTargetRpmTrims();
    return;
  }

  setAllTargetRpm(0.0f);
  headingWithinTolerance = false;
  turnCorrectionRPM = 0.0f;
}

void runRotateLoopAndComposeWheelTargets(float currentHeadingRad) {
  headingErrorRad = wrapAngleRadLocal(targetHeadingRad - currentHeadingRad);
  rotateIntegralRadS += headingErrorRad * currentDtSeconds;
  const float headingErrorDeg = headingErrorRad * (180.0f / PI_F);
  const float absErrDeg = fabsf(headingErrorDeg);
  const float finalZoneDeg = max(HEADING_TOLERANCE_DEG * 8.0f, 3.0f);

  float rotateCommandRpm;

  if (absErrDeg <= HEADING_TOLERANCE_DEG) {
    rotateCommandRpm = 0.0f;
    rotateIntegralRadS = 0.0f;
    targetRpm[WHEEL_F1] = 0.0f;
    targetRpm[WHEEL_R1] = 0.0f;
    targetRpm[WHEEL_F2] = 0.0f;
    targetRpm[WHEEL_R2] = 0.0f;
  } else if (absErrDeg <= finalZoneDeg) {
    // Weak final correction only. No final max clamp and no minimum RPM.
    rotateCommandRpm = FINAL_HEADING_KP * headingErrorRad;
  } else {
    rotateCommandRpm =
      (Kp_rotate_rpm * headingErrorRad) +
      (Ki_rotate_rpm_per_rad_s * rotateIntegralRadS);
    rotateCommandRpm = constrain(rotateCommandRpm, -MAX_ROTATE_RPM, MAX_ROTATE_RPM);
  }

  if (invertRotateDirection) {
    rotateCommandRpm = -rotateCommandRpm;
  }

  turnCorrectionRPM = rotateCommandRpm;
  leftRpmComposed = -rotateCommandRpm;
  rightRpmComposed = rotateCommandRpm;

  // Keep signs unchanged because ROTATE already turns in the correct direction.
  // Only trim magnitudes to reduce translation during rotation.
  targetRpm[WHEEL_F1] = leftRpmComposed * ROTATE_TRIM_F1;   // actual front right
  targetRpm[WHEEL_R1] = leftRpmComposed * ROTATE_TRIM_R1;   // actual rear right
  targetRpm[WHEEL_F2] = rightRpmComposed * ROTATE_TRIM_F2;  // actual front left
  targetRpm[WHEEL_R2] = rightRpmComposed * ROTATE_TRIM_R2;  // actual rear left

  applyWheelTargetRpmTrims();
}
