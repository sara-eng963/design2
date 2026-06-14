#include "PositionController.h"

#include <Arduino.h>
#include <cmath>

#include "HeadingController.h"
#include "MotorDriver.h"
#include "WheelVelocityController.h"

float Kp_pos = 1.2f;
float Ki_pos = 0.0f;
constexpr float MAX_VX = 0.5f;

bool positionModeActive = false;
MotionMode motionMode = MODE_IDLE;
MotionMode motionResultMode = MODE_IDLE;
MotionFaultCode motionFaultCode = MOTION_FAULT_NONE;
MotionResultCode motionResultCode = MOTION_RESULT_NONE;

float targetDistanceM = 0.0f;
float currentDistanceM = 0.0f;
float distanceErrorM = 0.0f;
float positionIntegral = 0.0f;

float baseForwardRpm = 0.0f;
float rawBaseForwardRpm = 0.0f;
std::uint32_t motionStartMs = 0U;

void resetPositionControllerState() {
  targetDistanceM = 0.0f;
  currentDistanceM = 0.0f;
  distanceErrorM = 0.0f;
  positionIntegral = 0.0f;
  positionModeActive = false;
  motionMode = MODE_IDLE;
  motionStartMs = 0U;
}

void clearMotionFault() {
  motionFaultCode = MOTION_FAULT_NONE;
}

void clearMotionResult() {
  motionResultCode = MOTION_RESULT_NONE;
  motionResultMode = MODE_IDLE;
}

void beginMotionTimingWindow(std::uint32_t nowMs) {
  motionStartMs = nowMs;
  clearMotionFault();
  clearMotionResult();
}

void requestMotionResult(MotionResultCode result, MotionMode mode, MotionFaultCode fault) {
  if (motionResultCode != MOTION_RESULT_NONE) {
    return;
  }

  motionResultCode = result;
  motionResultMode = mode;
  motionFaultCode = fault;
}

bool motionIsActive() {
  return positionModeActive && (motionMode != MODE_IDLE);
}

float deltaCountsToDistanceMeters(long deltaCount) {
  return (deltaCount / PPR) * WHEEL_CIRCUMFERENCE_M;
}

const char *modeToString(MotionMode mode) {
  switch (mode) {
    case MODE_MOVE_FORWARD: return "MOVE_FORWARD";
    case MODE_ROTATE_TO_HEADING: return "ROTATE_TO_HEADING";
    default: return "IDLE";
  }
}

const char *motionFaultToString(MotionFaultCode fault) {
  switch (fault) {
    case MOTION_FAULT_TIMEOUT: return "TIMEOUT";
    case MOTION_FAULT_IMU: return "IMU";
    default: return "NONE";
  }
}

void runPositionLoop(float dtSec, std::uint32_t nowMs) {
  if (!positionModeActive || motionMode != MODE_MOVE_FORWARD) {
    baseForwardRpm = 0.0f;
    rawBaseForwardRpm = 0.0f;
    return;
  }

  if ((motionStartMs != 0U) && ((nowMs - motionStartMs) >= MOVE_TIMEOUT_MS)) {
    baseForwardRpm = 0.0f;
    rawBaseForwardRpm = 0.0f;
    turnCorrectionRPM = 0.0f;
    requestMotionResult(MOTION_RESULT_FAULT, MODE_MOVE_FORWARD, MOTION_FAULT_TIMEOUT);
    return;
  }

  distanceErrorM = targetDistanceM - currentDistanceM;

  const bool reachedByTolerance = fabs(distanceErrorM) <= POSITION_TOLERANCE;
  if (reachedByTolerance) {
    setAllTargetRpm(0.0f);
    baseForwardRpm = 0.0f;
    rawBaseForwardRpm = 0.0f;
    turnCorrectionRPM = 0.0f;
    positionIntegral = 0.0f;
    headingErrorRad = 0.0f;
    headingIntegralRadS = 0.0f;
    resetVelocityControllerState();
    stopAllMotorHardware();
    requestMotionResult(MOTION_RESULT_DONE, MODE_MOVE_FORWARD, MOTION_FAULT_NONE);
    return;
  }

  positionIntegral += distanceErrorM * dtSec;

  float vxCommand = (Kp_pos * distanceErrorM) + (Ki_pos * positionIntegral);
  vxCommand = constrain(vxCommand, -MAX_VX, MAX_VX);

  rawBaseForwardRpm = (vxCommand / WHEEL_CIRCUMFERENCE_M) * 60.0f;

  baseForwardRpm = rawBaseForwardRpm;
}
