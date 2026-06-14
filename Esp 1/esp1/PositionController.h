#pragma once

#include <cstdint>

#include "HardwareConfig.h"

extern float Kp_pos;
extern float Ki_pos;

constexpr float POSITION_TOLERANCE = 0.005f;
constexpr std::uint32_t MOVE_TIMEOUT_MS = 250000000U;
constexpr std::uint32_t ROTATE_TIMEOUT_MS = 150000000U;

extern bool positionModeActive;

enum MotionMode {
  MODE_IDLE = 0,
  MODE_MOVE_FORWARD = 1,
  MODE_ROTATE_TO_HEADING = 2
};

enum MotionFaultCode {
  MOTION_FAULT_NONE = 0,
  MOTION_FAULT_TIMEOUT = 1,
  MOTION_FAULT_IMU = 2
};

enum MotionResultCode {
  MOTION_RESULT_NONE = 0,
  MOTION_RESULT_DONE = 1,
  MOTION_RESULT_FAULT = 2
};

extern MotionMode motionMode;
extern MotionMode motionResultMode;
extern MotionFaultCode motionFaultCode;
extern MotionResultCode motionResultCode;

extern float targetDistanceM;
extern float currentDistanceM;
extern float distanceErrorM;
extern float positionIntegral;
extern float baseForwardRpm;
extern float rawBaseForwardRpm;
extern std::uint32_t motionStartMs;

void resetPositionControllerState();
void clearMotionFault();
void clearMotionResult();
void beginMotionTimingWindow(std::uint32_t nowMs);
void requestMotionResult(MotionResultCode result, MotionMode mode, MotionFaultCode fault);
bool motionIsActive();
float deltaCountsToDistanceMeters(long deltaCount);
const char *modeToString(MotionMode mode);
const char *motionFaultToString(MotionFaultCode fault);
void runPositionLoop(float dtSec, std::uint32_t nowMs);
