#include <Arduino.h>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdint>
#include <cstdlib>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>

#include "EncoderManager.h"
#include "HardwareConfig.h"
#include "HeadingController.h"
#include "IMU.h"
#include "MotorDriver.h"
#include "PositionController.h"
#include "WheelVelocityController.h"
#include "WebControl.h"
#include "manual.h"
#include "WifiConfig.h"
#include <WiFi.h>

#ifndef ENABLE_SERIAL_TEXT_COMMANDS
#define ENABLE_SERIAL_TEXT_COMMANDS 0
#endif

#ifndef ENABLE_SERIAL_DEBUG_OUTPUT
#define ENABLE_SERIAL_DEBUG_OUTPUT 0
#endif

// ============================================================
// Forward-Position-With-Heading-Control-Test for ESP32
// ------------------------------------------------------------
// Goal of this file:
// - Move straight by a signed distance command (example: +0.50, -0.50)
// - Keep heading (yaw) close to the heading captured at move start
// - Keep architecture simple and beginner-friendly
//
// Cascade structure (important):
// 1) Position loop outputs BASE forward RPM (not PWM)
// 2) Heading loop outputs TURN correction RPM (not PWM)
// 3) Wheel velocity loops output PWM
//
// Why heading control is needed:
// - Even for straight commands, small friction differences can make robot rotate.
// - IMU yaw tells us that unwanted rotation.
// - We counter it by making left/right wheel RPM slightly different.
//
// What this file intentionally does NOT include:
// - No x/y waypoint control
// - No sideways motion
// - No full mecanum kinematics
// - No trajectory planning
// - No Kalman filter
// - No heading integral/derivative
// - No feedforward
// - No deadband compensation
// - No advanced state machine
// ============================================================

// -----------------------------
// Shared state (RTOS + ISR)
// -----------------------------

float currentHeadingRad = 0.0f;

bool imuHealthy = false;
uint32_t imuLastOkReadMs = 0;
float imuRawGyroZDegPerSec = 0.0f;
float imuCorrectedGyroZDegPerSec = 0.0f;
float imuBiasZDegPerSec = 0.0f;
float imuIntegratedYawDeg = 0.0f;
float imuZeroOffsetYawDeg = 0.0f;
const char *imuMotionState = "UNKNOWN";

float currentDtSeconds = CONTROL_PERIOD_SECONDS;
unsigned long lastControlTime = 0;

SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t microRosPublishMutex = nullptr;
SemaphoreHandle_t driveStatusQueueMutex = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t imuTaskHandle = nullptr;
TaskHandle_t microRosTaskHandle = nullptr;

String serialLineBuffer;
String lastCommandResponse = "READY";
String lastMotionReport = "NONE";

rcl_publisher_t drive_status_pub = rcl_get_zero_initialized_publisher();
rcl_subscription_t drive_cmd_sub = rcl_get_zero_initialized_subscription();
std_msgs__msg__String drive_status_msg = {};
std_msgs__msg__String drive_cmd_msg = {};
rclc_support_t support = {};
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node = rcl_get_zero_initialized_node();
rclc_executor_t executor = {};

static char drive_status_buffer[384];
static char drive_cmd_buffer[128];
constexpr std::size_t DRIVE_STATUS_QUEUE_DEPTH = 8U;
static char drive_status_queue[DRIVE_STATUS_QUEUE_DEPTH][sizeof(drive_status_buffer)];
std::size_t driveStatusQueueHead = 0U;
std::size_t driveStatusQueueTail = 0U;
std::size_t driveStatusQueueCount = 0U;

bool microRosInitialized = false;

float interruptedMoveDistM = 0.0f;
float interruptedMoveTargetM = 0.0f;
float interruptedMoveHeadingDeg = 0.0f;
bool interruptedMoveValid = false;

float lastDoneRotateYawDeg = 0.0f;
float lastDoneRotateTargetDeg = 0.0f;
float lastDoneRotateErrDeg = 0.0f;
bool lastDoneRotateSnapshotValid = false;

constexpr TickType_t MICRO_ROS_TASK_DELAY_TICKS = pdMS_TO_TICKS(2);
constexpr TickType_t MICRO_ROS_WAITING_AGENT_DELAY_TICKS = pdMS_TO_TICKS(500);
constexpr TickType_t MICRO_ROS_CONNECTED_PING_DELAY_TICKS = pdMS_TO_TICKS(200);
constexpr TickType_t MICRO_ROS_INITIAL_STARTUP_DELAY_TICKS = pdMS_TO_TICKS(2000);

enum MicroRosConnectionState {
  MICRO_ROS_WAITING_AGENT,
  MICRO_ROS_AGENT_AVAILABLE,
  MICRO_ROS_AGENT_CONNECTED,
  MICRO_ROS_AGENT_DISCONNECTED,
};

bool executeCommandLine(const String &commandLine, String &response);
bool queueDriveStatus(const String &text);
bool dequeueDriveStatus(String &text);
void clearDriveStatusQueue();
void clearInterruptedMoveState();
void captureInterruptedMoveIfNeeded();

// -----------------------------
// Utility helpers
// -----------------------------
bool lockState(TickType_t timeoutTicks = pdMS_TO_TICKS(20)) {
  return (stateMutex != nullptr) && (xSemaphoreTake(stateMutex, timeoutTicks) == pdTRUE);
}

void unlockState() {
  if (stateMutex != nullptr) {
    xSemaphoreGive(stateMutex);
  }
}

float wrapAngleRad(float angle) {
  // Keep angles inside [-PI, +PI] so the controller always takes
  // the shortest turn direction. Example: +179 deg and -179 deg are
  // close in reality, and wrapping prevents a false 358 deg error.
  while (angle > PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;
  return angle;
}

float radToDeg(float rad) {
  return rad * (180.0f / PI_F);
}

bool tryParseFloatToken(const String &text, float &value) {
  if (text.length() == 0) {
    return false;
  }

  char buffer[32];
  if (text.length() >= sizeof(buffer)) {
    return false;
  }

  text.toCharArray(buffer, sizeof(buffer));
  char *endPtr = nullptr;
  value = strtof(buffer, &endPtr);
  return (endPtr != buffer) && (*endPtr == '\0');
}

int splitArguments(const String &text, String tokens[], int maxTokens) {
  int count = 0;
  int start = 0;

  while (start < text.length() && count < maxTokens) {
    while (start < text.length() && text[start] == ' ') {
      start++;
    }
    if (start >= text.length()) {
      break;
    }

    int end = start;
    while (end < text.length() && text[end] != ' ') {
      end++;
    }

    tokens[count++] = text.substring(start, end);
    start = end + 1;
  }

  return count;
}

const char *motionResultLabel(MotionMode mode) {
  switch (mode) {
    case MODE_MOVE_FORWARD: return "MOVE";
    case MODE_ROTATE_TO_HEADING: return "ROTATE";
    default: return "IDLE";
  }
}

bool motionCommandPendingResult() {
  return motionResultCode != MOTION_RESULT_NONE;
}

void initializeRosStringBuffer(std_msgs__msg__String &msg, char *buffer, std::size_t bufferSize) {
  if (bufferSize == 0U) {
    return;
  }

  buffer[0] = '\0';
  msg.data.data = buffer;
  msg.data.size = 0U;
  msg.data.capacity = bufferSize;
}

bool publishDriveStatus(const String &text) {
  if (!microRosInitialized || (microRosPublishMutex == nullptr)) {
    return false;
  }

  if (xSemaphoreTake(microRosPublishMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    return false;
  }

  const std::size_t maxLen = sizeof(drive_status_buffer) - 1U;
  std::size_t copyLen = text.length();
  if (copyLen > maxLen) {
    copyLen = maxLen;
  }

  if (copyLen > 0U) {
    std::memcpy(drive_status_buffer, text.c_str(), copyLen);
  }
  drive_status_buffer[copyLen] = '\0';
  drive_status_msg.data.size = copyLen;

  const bool ok = (rcl_publish(&drive_status_pub, &drive_status_msg, nullptr) == RCL_RET_OK);
  xSemaphoreGive(microRosPublishMutex);
  return ok;
}

bool queueDriveStatus(const String &text) {
  if (driveStatusQueueMutex == nullptr) {
    return false;
  }

  bool queued = false;
  if (xSemaphoreTake(driveStatusQueueMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    if (driveStatusQueueCount < DRIVE_STATUS_QUEUE_DEPTH) {
      const std::size_t maxLen = sizeof(drive_status_queue[0]) - 1U;
      std::size_t copyLen = text.length();
      if (copyLen > maxLen) {
        copyLen = maxLen;
      }

      if (copyLen > 0U) {
        std::memcpy(drive_status_queue[driveStatusQueueTail], text.c_str(), copyLen);
      }
      drive_status_queue[driveStatusQueueTail][copyLen] = '\0';
      driveStatusQueueTail = (driveStatusQueueTail + 1U) % DRIVE_STATUS_QUEUE_DEPTH;
      driveStatusQueueCount++;
      queued = true;
    }
    xSemaphoreGive(driveStatusQueueMutex);
  }

  return queued;
}

bool dequeueDriveStatus(String &text) {
  if (driveStatusQueueMutex == nullptr) {
    return false;
  }

  bool dequeued = false;
  if (xSemaphoreTake(driveStatusQueueMutex, 0) == pdTRUE) {
    if (driveStatusQueueCount > 0U) {
      text = String(drive_status_queue[driveStatusQueueHead]);
      driveStatusQueueHead = (driveStatusQueueHead + 1U) % DRIVE_STATUS_QUEUE_DEPTH;
      driveStatusQueueCount--;
      dequeued = true;
    }
    xSemaphoreGive(driveStatusQueueMutex);
  }

  return dequeued;
}

void clearDriveStatusQueue() {
  if (driveStatusQueueMutex == nullptr) {
    return;
  }

  if (xSemaphoreTake(driveStatusQueueMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    driveStatusQueueHead = 0U;
    driveStatusQueueTail = 0U;
    driveStatusQueueCount = 0U;
    xSemaphoreGive(driveStatusQueueMutex);
  }
}

String rosMessageToString(const std_msgs__msg__String &msg) {
  String text;
  if ((msg.data.data == nullptr) || (msg.data.size == 0U)) {
    return text;
  }

  text.reserve(msg.data.size);
  for (std::size_t i = 0; i < msg.data.size; i++) {
    text += msg.data.data[i];
  }
  return text;
}

void driveCommandCallback(const void *msgIn) {
  const auto *msg = static_cast<const std_msgs__msg__String *>(msgIn);
  if (msg == nullptr) {
    (void)queueDriveStatus("ERR NULL_COMMAND");
    return;
  }

  String response;
  const String command = rosMessageToString(*msg);
  (void)executeCommandLine(command, response);

  if (response.length() > 0) {
    (void)queueDriveStatus(response);
  }
}

bool microRosCheckOk(rcl_ret_t rc) {
  return rc == RCL_RET_OK;
}

bool createMicroRosEntities() {
  initializeRosStringBuffer(drive_status_msg, drive_status_buffer, sizeof(drive_status_buffer));
  initializeRosStringBuffer(drive_cmd_msg, drive_cmd_buffer, sizeof(drive_cmd_buffer));

  allocator = rcl_get_default_allocator();
  executor = rclc_executor_get_zero_initialized_executor();

  if (!microRosCheckOk(rclc_support_init(&support, 0, nullptr, &allocator))) {
    return false;
  }

  if (!microRosCheckOk(rclc_node_init_default(&node, "esp_drive_node", "", &support))) {
    return false;
  }

  if (!microRosCheckOk(
        rclc_publisher_init_default(
          &drive_status_pub,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/drive_status"))) {
    return false;
  }

  if (!microRosCheckOk(
        rclc_subscription_init_default(
          &drive_cmd_sub,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/drive_cmd"))) {
    return false;
  }

  if (!microRosCheckOk(rclc_executor_init(&executor, &support.context, 1, &allocator))) {
    return false;
  }

  if (!microRosCheckOk(
        rclc_executor_add_subscription(
          &executor,
          &drive_cmd_sub,
          &drive_cmd_msg,
          &driveCommandCallback,
          ON_NEW_DATA))) {
    return false;
  }

  microRosInitialized = true;
  return true;
}

void destroyMicroRosEntities() {
  if (microRosInitialized) {
    rmw_context_t *rmwContext = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, 0);
  }

  if (drive_cmd_sub.impl != nullptr) {
    (void)rcl_subscription_fini(&drive_cmd_sub, &node);
  }
  if (drive_status_pub.impl != nullptr) {
    (void)rcl_publisher_fini(&drive_status_pub, &node);
  }
  if (executor.context != nullptr) {
    (void)rclc_executor_fini(&executor);
  }
  if (node.impl != nullptr) {
    (void)rcl_node_fini(&node);
  }
  if (support.context.impl != nullptr) {
    (void)rclc_support_fini(&support);
  }

  drive_status_pub = rcl_get_zero_initialized_publisher();
  drive_cmd_sub = rcl_get_zero_initialized_subscription();
  node = rcl_get_zero_initialized_node();
  support = {};
  executor = rclc_executor_get_zero_initialized_executor();
  microRosInitialized = false;
}

void microRosTask(void *parameter) {
  (void)parameter;

  if (microRosPublishMutex == nullptr) {
    microRosPublishMutex = xSemaphoreCreateMutex();
  }

  if (driveStatusQueueMutex == nullptr) {
    driveStatusQueueMutex = xSemaphoreCreateMutex();
  }

  set_microros_transports();
  vTaskDelay(MICRO_ROS_INITIAL_STARTUP_DELAY_TICKS);

  MicroRosConnectionState state = MICRO_ROS_WAITING_AGENT;

  while (true) {
    switch (state) {
      case MICRO_ROS_WAITING_AGENT:
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          state = MICRO_ROS_AGENT_AVAILABLE;
        } else {
          vTaskDelay(MICRO_ROS_WAITING_AGENT_DELAY_TICKS);
        }
        break;

      case MICRO_ROS_AGENT_AVAILABLE:
        if (createMicroRosEntities()) {
          clearDriveStatusQueue();
          state = MICRO_ROS_AGENT_CONNECTED;
        } else {
          destroyMicroRosEntities();
          vTaskDelay(MICRO_ROS_WAITING_AGENT_DELAY_TICKS);
          state = MICRO_ROS_WAITING_AGENT;
        }
        break;

      case MICRO_ROS_AGENT_CONNECTED: {
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
          state = MICRO_ROS_AGENT_DISCONNECTED;
          break;
        }

        String queuedStatus;
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        while (dequeueDriveStatus(queuedStatus)) {
          (void)publishDriveStatus(queuedStatus);
        }

        vTaskDelay(MICRO_ROS_TASK_DELAY_TICKS);
        break;
      }

      case MICRO_ROS_AGENT_DISCONNECTED:
        destroyMicroRosEntities();
        clearDriveStatusQueue();
        vTaskDelay(MICRO_ROS_CONNECTED_PING_DELAY_TICKS);
        state = MICRO_ROS_WAITING_AGENT;
        break;

      default:
        state = MICRO_ROS_WAITING_AGENT;
        break;
    }
  }
}

void finalizeMotionStop() {
  setAllTargetRpm(0.0f);
  resetVelocityControllerState();
  resetPositionControllerState();
  resetHeadingControllerState();
  baseForwardRpm = 0.0f;
  rawBaseForwardRpm = 0.0f;
  for (int i = 0; i < WHEEL_COUNT; i++) {
    finalPwm[i] = 0;
    appliedPwm[i] = 0;
  }
  stopAllMotorHardware();
}

void setIdleStateAndStopMotors() {
  finalizeMotionStop();
  rotateIntegralRadS = 0.0f;
}

void clearAllMotionNotifications() {
  clearMotionFault();
  clearMotionResult();
}

String buildStatusLine() {
  const float yawDeg = radToDeg(currentHeadingRad);
  const float targetHeadingDeg = radToDeg(targetHeadingRad);
  const float headingErrorDeg = radToDeg(headingErrorRad);
  const bool useInterruptedMove = interruptedMoveValid;
  const float statusDistM = useInterruptedMove ? interruptedMoveDistM : currentDistanceM;
  const float statusTargetM = useInterruptedMove ? interruptedMoveTargetM : targetDistanceM;
  const float statusHeadingDeg = useInterruptedMove ? interruptedMoveHeadingDeg : yawDeg;
  String status = "STATUS mode=";
  status += modeToString(motionMode);
  status += " interrupted=";
  status += useInterruptedMove ? "1" : "0";
  status += " dist=";
  status += String(statusDistM, 2);
  status += " target=";
  status += String(statusTargetM, 2);
  status += " err=";
  status += String(distanceErrorM, 2);
  status += " heading=";
  status += String(statusHeadingDeg, 1);
  status += " yaw=";
  status += String(yawDeg, 1);
  status += " targetHeading=";
  status += String(targetHeadingDeg, 1);
  status += " headingErr=";
  status += String(headingErrorDeg, 1);
  if (lastDoneRotateSnapshotValid) {
    status += " lastDoneYaw=";
    status += String(lastDoneRotateYawDeg, 2);
    status += " lastDoneTarget=";
    status += String(lastDoneRotateTargetDeg, 2);
    status += " lastDoneErr=";
    status += String(lastDoneRotateErrDeg, 2);
  }
  status += " baseRPM=";
  status += String(baseForwardRpm, 1);
  status += " turnRPM=";
  status += String(turnCorrectionRPM, 1);
  status += " RKP=";
  status += String(Kp_rotate_rpm, 1);
  status += " RKI=";
  status += String(Ki_rotate_rpm_per_rad_s, 2);
  status += " RTOL=";
  status += String(HEADING_TOLERANCE_DEG, 1);
  status += " active=";
  status += motionIsActive() ? "1" : "0";
  status += " headingHold=";
  status += headingHoldActive ? "1" : "0";
  status += " fault=";
  status += motionFaultToString(motionFaultCode);
  status += " ros=";
  status += microRosInitialized ? "1" : "0";
  const float rawGyroZDegPerSec = app::imuDriver().rawGyroZRadPerSec() * (180.0f / PI_F);
  const float correctedGyroZDegPerSec = app::imuDriver().correctedGyroZRadPerSec() * (180.0f / PI_F);
  const float gyroBiasZDegPerSec = app::imuDriver().gyroBiasZRadPerSec() * (180.0f / PI_F);
  status += " rawGz=";
  status += String(rawGyroZDegPerSec, 2);
  status += " corrGz=";
  status += String(correctedGyroZDegPerSec, 2);
  status += " biasZ=";
  status += String(gyroBiasZDegPerSec, 2);
  status += " imuState=";
  status += app::imuDriver().motionStateName();
  status += " dtMs=";
  status += String(app::imuDriver().lastDtMs(), 3);
  return status;
}

void announceMotionResultIfNeeded() {
  String report;

  if (!lockState(pdMS_TO_TICKS(5))) {
    return;
  }

  if (motionResultCode == MOTION_RESULT_DONE) {
    report = String("DONE ") + motionResultLabel(motionResultMode);
    if ((motionResultMode == MODE_ROTATE_TO_HEADING) && lastDoneRotateSnapshotValid) {
      report += " yaw=";
      report += String(lastDoneRotateYawDeg, 2);
      report += " target=";
      report += String(lastDoneRotateTargetDeg, 2);
      report += " err=";
      report += String(lastDoneRotateErrDeg, 2);
      report += " rtol=";
      report += String(radToDeg(headingToleranceRad()), 2);
    }
  } else if (motionResultCode == MOTION_RESULT_FAULT) {
    report = String("FAULT ") + motionFaultToString(motionFaultCode);
  }

  clearMotionResult();
  unlockState();

  if (report.length() > 0) {
    lastMotionReport = report;
    (void)queueDriveStatus(report);
  }
}

void clearInterruptedMoveState() {
  interruptedMoveDistM = 0.0f;
  interruptedMoveTargetM = 0.0f;
  interruptedMoveHeadingDeg = 0.0f;
  interruptedMoveValid = false;
}

void captureInterruptedMoveIfNeeded() {
  if (motionMode != MODE_MOVE_FORWARD) {
    return;
  }

  interruptedMoveDistM = currentDistanceM;
  interruptedMoveTargetM = targetDistanceM;
  interruptedMoveHeadingDeg = radToDeg(targetHeadingRad);
  interruptedMoveValid = true;
}

// -----------------------------
// Move start / stop
// -----------------------------
bool initializeNewMove(float distanceM, bool useManualHeading, float manualHeadingDeg) {
  // Safety first.
  stopAllMotorHardware();

  clearInterruptedMoveState();

  // Reset control states.
  resetVelocityControllerState();
  resetPositionControllerState();
  resetHeadingControllerState();
  clearAllMotionNotifications();

  currentDistanceM = 0.0f;
  targetDistanceM = distanceM;
  positionModeActive = true;
  motionMode = MODE_MOVE_FORWARD;

  // Read current yaw once at move start so IMU health is checked before enabling motion.
  currentHeadingRad = app::imuDriver().displayedYawRad();
  if (!isfinite(currentHeadingRad)) {
    imuHealthy = false;
    setIdleStateAndStopMotors();
    motionFaultCode = MOTION_FAULT_IMU;
    return false;
  }

  imuHealthy = true;

  // Heading target mode:
  // - captured: hold the yaw measured at move start
  // - manual: hold the yaw entered by the user (degrees)
  targetHeadingRad = useManualHeading
    ? wrapAngleRad(manualHeadingDeg * (PI_F / 180.0f))
    : currentHeadingRad;
  headingHoldActive = true;
  headingErrorRad = 0.0f;
  headingIntegralRadS = 0.0f;
  turnCorrectionRPM = 0.0f;

  // New distance run starts with fresh encoder baseline.
  noInterrupts();
  for (int i = 0; i < WHEEL_COUNT; i++) {
    lastEncoderCounts[i] = encoderCounts[i];
  }
  interrupts();

  for (int i = 0; i < WHEEL_COUNT; i++) {
    lastDeltaCounts[i] = 0;
  }

  // MOVE completes on the first control cycle inside the 5 mm tolerance.
  beginMotionTimingWindow(millis());
  return true;
}

bool initializeNewRotation(float targetHeadingDeg) {
  stopAllMotorHardware();
  resetVelocityControllerState();
  resetPositionControllerState();
  resetHeadingControllerState();
  clearAllMotionNotifications();

  currentHeadingRad = app::imuDriver().displayedYawRad();
  if (!isfinite(currentHeadingRad)) {
    imuHealthy = false;
    setIdleStateAndStopMotors();
    motionFaultCode = MOTION_FAULT_IMU;
    return false;
  }

  imuHealthy = true;
  positionModeActive = true;
  motionMode = MODE_ROTATE_TO_HEADING;
  targetHeadingRad = wrapAngleRad(targetHeadingDeg * (PI_F / 180.0f));
  headingErrorRad = wrapAngleRad(targetHeadingRad - currentHeadingRad);
  distanceErrorM = 0.0f;
  currentDistanceM = 0.0f;
  targetDistanceM = 0.0f;
  baseForwardRpm = 0.0f;
  rawBaseForwardRpm = 0.0f;
  rotateIntegralRadS = 0.0f;
  setAllTargetRpm(0.0f);
  // ROTATE uses heading-only completion and timeout logic, not distance.
  beginMotionTimingWindow(millis());
  return true;
}

void emergencyStopAndReset() {
  captureInterruptedMoveIfNeeded();
  clearAllMotionNotifications();
  setIdleStateAndStopMotors();
}

// Heading loop output is TURN correction RPM only (not PWM).
// Positive/negative correction is mixed with baseForwardRpm so left/right
// wheels receive slightly different RPM targets to cancel yaw drift.
// -----------------------------
// Command parser (Serial + Web)
// -----------------------------
// Supported commands:
// - MOVE <distance_m> <heading_deg>
// - ROTATE <heading_deg>
// - STOP
// - STATUS
// - HEADING ON|OFF, HKP, HKI, HINVERT
// - RKP, RKI, RTOL, RINVERT
bool executeCommandLine(const String &commandLine, String &response) {
  String cmd = commandLine;
  cmd.trim();

  if (cmd.length() == 0) {
    response = "Empty command";
    return false;
  }

  int splitPos = cmd.indexOf(' ');
  String key = (splitPos >= 0) ? cmd.substring(0, splitPos) : cmd;
  key.toUpperCase();
  String arg = (splitPos >= 0) ? cmd.substring(splitPos + 1) : "";
  arg.trim();

  if (!lockState(pdMS_TO_TICKS(50))) {
    response = "State lock busy";
    return false;
  }

  if (key == "MOVE") {
    if (motionIsActive() || motionCommandPendingResult()) {
      unlockState();
      response = "ERR BUSY";
      lastCommandResponse = response;
      return false;
    }

    String tokens[3];
    const int tokenCount = splitArguments(arg, tokens, 3);
    float distanceM = 0.0f;
    float headingDeg = 0.0f;
    if ((tokenCount != 2) || !tryParseFloatToken(tokens[0], distanceM) || !tryParseFloatToken(tokens[1], headingDeg)) {
      unlockState();
      response = "ERR FORMAT MOVE <distance_m> <heading_deg>";
      lastCommandResponse = response;
      return false;
    }

    if (fabs(distanceM) < 0.0001f) {
      emergencyStopAndReset();
      response = "STOPPED";
    } else if (!initializeNewMove(distanceM, true, headingDeg)) {
      unlockState();
      response = "FAULT IMU";
      lastCommandResponse = response;
      return false;
    } else {
      response = String("ACK MOVE distance=") + String(distanceM, 2) + " heading=" + String(headingDeg, 1);
    }
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "ROTATE") {
    if (motionIsActive() || motionCommandPendingResult()) {
      unlockState();
      response = "ERR BUSY";
      lastCommandResponse = response;
      return false;
    }

    float headingDeg = 0.0f;
    if (!tryParseFloatToken(arg, headingDeg)) {
      unlockState();
      response = "ERR FORMAT ROTATE <heading_deg>";
      lastCommandResponse = response;
      return false;
    }

    if (!initializeNewRotation(headingDeg)) {
      unlockState();
      response = "FAULT IMU";
      lastCommandResponse = response;
      return false;
    } else {
      response = String("ACK ROTATE heading=") + String(headingDeg, 1);
    }
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "STOP") {
    emergencyStopAndReset();
    unlockState();
    response = "STOPPED";
    lastCommandResponse = response;
    return true;
  }

  if (key == "STATUS") {
    response = buildStatusLine();
    unlockState();
    lastCommandResponse = response;
    return true;
  }

  if (key == "HEADING") {
    String opt = arg;
    opt.toUpperCase();
    if (opt == "ON") {
      headingControlEnabled = true;
      response = "ACK HEADING ON";
    } else if (opt == "OFF") {
      headingControlEnabled = false;
      turnCorrectionRPM = 0.0f;
      response = "ACK HEADING OFF";
    } else {
      unlockState();
      response = "ERR FORMAT HEADING ON|OFF";
      lastCommandResponse = response;
      return false;
    }
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "HKP") {
    // Beginner tuning tip:
    // - Increase hkp if robot still drifts heading.
    // - Decrease hkp if robot oscillates left/right.
    Kp_heading_rpm = arg.toFloat();
    if (Kp_heading_rpm < 0.0f) Kp_heading_rpm = 0.0f;
    response = String("ACK HKP ") + String(Kp_heading_rpm, 2);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "HKI") {
    Ki_heading_rpm_per_rad_s = arg.toFloat();
    if (Ki_heading_rpm_per_rad_s < 0.0f) Ki_heading_rpm_per_rad_s = 0.0f;
    response = String("ACK HKI ") + String(Ki_heading_rpm_per_rad_s, 2);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "HINVERT") {
    // Use hinvert if heading correction direction is reversed.
    // Example symptom: robot drifts right and controller makes it drift more right.
    invertHeadingCorrection = !invertHeadingCorrection;
    response = String("ACK HINVERT ") + (invertHeadingCorrection ? "ON" : "OFF");
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "RKP") {
    Kp_rotate_rpm = arg.toFloat();
    if (Kp_rotate_rpm < 0.0f) Kp_rotate_rpm = 0.0f;
    response = String("ACK RKP ") + String(Kp_rotate_rpm, 2);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "RKI") {
    Ki_rotate_rpm_per_rad_s = arg.toFloat();
    if (Ki_rotate_rpm_per_rad_s < 0.0f) Ki_rotate_rpm_per_rad_s = 0.0f;
    response = String("ACK RKI ") + String(Ki_rotate_rpm_per_rad_s, 2);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "RTOL") {
    HEADING_TOLERANCE_DEG = arg.toFloat();
    if (HEADING_TOLERANCE_DEG < 0.5f) HEADING_TOLERANCE_DEG = 0.5f;
    if (HEADING_TOLERANCE_DEG > 45.0f) HEADING_TOLERANCE_DEG = 45.0f;
    response = String("ACK RTOL ") + String(HEADING_TOLERANCE_DEG, 2);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "RINVERT" || key == "ROTATE_INVERT") {
    invertRotateDirection = !invertRotateDirection;
    response = String("ACK RINVERT ") + (invertRotateDirection ? "ON" : "OFF");
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "PKP") {
    Kp_pos = arg.toFloat();
    if (Kp_pos < 0.0f) Kp_pos = 0.0f;
    response = String("ACK PKP ") + String(Kp_pos, 3);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "PKI") {
    Ki_pos = arg.toFloat();
    if (Ki_pos < 0.0f) Ki_pos = 0.0f;
    response = String("ACK PKI ") + String(Ki_pos, 3);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "VKP") {
    String tokens[2];
    const int tokenCount = splitArguments(arg, tokens, 2);
    float val = 0.0f;
    int idx = -1;
    if (tokenCount == 2 && tryParseFloatToken(tokens[1], val)) {
      idx = tokens[0].toInt();
    }
    if (idx < 0 || idx > 3) {
      unlockState();
      response = "ERR FORMAT VKP <wheel_index 0-3> <value>";
      lastCommandResponse = response;
      return false;
    }
    if (val < 0.0f) val = 0.0f;
    kpVel[idx] = val;
    response = String("ACK VKP[") + String(idx) + "] " + String(val, 3);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "VKI") {
    String tokens[2];
    const int tokenCount = splitArguments(arg, tokens, 2);
    float val = 0.0f;
    int idx = -1;
    if (tokenCount == 2 && tryParseFloatToken(tokens[1], val)) {
      idx = tokens[0].toInt();
    }
    if (idx < 0 || idx > 3) {
      unlockState();
      response = "ERR FORMAT VKI <wheel_index 0-3> <value>";
      lastCommandResponse = response;
      return false;
    }
    if (val < 0.0f) val = 0.0f;
    kiVel[idx] = val;
    response = String("ACK VKI[") + String(idx) + "] " + String(val, 3);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "VKPALL") {
    float val = arg.toFloat();
    if (val < 0.0f) val = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) kpVel[i] = val;
    response = String("ACK VKPALL ") + String(val, 3);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "VKIALL") {
    float val = arg.toFloat();
    if (val < 0.0f) val = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) kiVel[i] = val;
    response = String("ACK VKIALL ") + String(val, 3);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "MANUAL") {
    String tokens[3];
    int tokenCount = splitArguments(arg, tokens, 3);
    if (tokenCount != 3) {
      response = "ERR FORMAT MANUAL <vx> <vy> <wz>";
      lastCommandResponse = response;
      unlockState();
      return false;
    }
    float vx, vy, wz;
    if (!tryParseFloatToken(tokens[0], vx) ||
        !tryParseFloatToken(tokens[1], vy) ||
        !tryParseFloatToken(tokens[2], wz)) {
      response = "ERR INVALID NUMBER";
      lastCommandResponse = response;
      unlockState();
      return false;
    }
    updateManualCommand(vx, vy, wz);
    motionMode = MODE_MANUAL_VELOCITY;
    positionModeActive = true;
    response = "ACK MANUAL";
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  unlockState();
  response = "ERR UNKNOWN_COMMAND";
  lastCommandResponse = response;
  return false;
}

void pollSerialCommands() {
#if ENABLE_SERIAL_TEXT_COMMANDS && ENABLE_SERIAL_DEBUG_OUTPUT
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLineBuffer.length() > 0) {
        String resp;
        bool ok = executeCommandLine(serialLineBuffer, resp);
        Serial.print(ok ? "[OK] " : "[ERR] ");
        Serial.println(resp);
        serialLineBuffer = "";
      }
    } else {
      serialLineBuffer += c;
      if (serialLineBuffer.length() > 120) {
        serialLineBuffer = "";
      }
    }
  }
#endif
}

void imuUpdateTask(void *parameter) {
  const TickType_t imuPeriodTicks = pdMS_TO_TICKS(app::IMU_READ_PERIOD_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&lastWakeTime, imuPeriodTicks);

    if (!lockState(pdMS_TO_TICKS(5))) {
      continue;
    }

    app::IMUState imuState;
    const bool ok = app::imuDriver().read(imuState);
    imuHealthy = ok;
    if (ok) {
      imuLastOkReadMs = millis();
      imuRawGyroZDegPerSec = app::radToDeg(app::imuDriver().rawGyroZRadPerSec());
      imuCorrectedGyroZDegPerSec = app::radToDeg(app::imuDriver().correctedGyroZRadPerSec());
      imuBiasZDegPerSec = app::radToDeg(app::imuDriver().gyroBiasZRadPerSec());
      imuIntegratedYawDeg = app::radToDeg(app::imuDriver().yawIntegratedRad());
      imuZeroOffsetYawDeg = app::radToDeg(app::imuDriver().yawZeroOffsetRad());
      imuMotionState = app::imuDriver().motionStateName();
    }

    unlockState();
  }
}

// -----------------------------
// RTOS control task
// -----------------------------
void controlLoopTask(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

    unsigned long now = millis();
    float dtSec = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;
    if (dtSec <= 0.0f) dtSec = CONTROL_PERIOD_SECONDS;

    if (!lockState()) {
      continue;
    }

    currentDtSeconds = dtSec;

    // Yaw is integrated inside the IMU update task.
    // The control loop only reads displayedYawRad() for control/debug.
    if (!imuHealthy) {
      if (motionIsActive()) {
        if (motionMode == MODE_ROTATE_TO_HEADING) {
          rotateIntegralRadS = 0.0f;
        }
        requestMotionResult(MOTION_RESULT_FAULT, motionMode, MOTION_FAULT_IMU);
      }
      setIdleStateAndStopMotors();
      unlockState();
      continue;
    }

    currentHeadingRad = app::imuDriver().displayedYawRad();
    if (!isfinite(currentHeadingRad)) {
      imuHealthy = false;
      if (motionIsActive()) {
        if (motionMode == MODE_ROTATE_TO_HEADING) {
          rotateIntegralRadS = 0.0f;
        }
        requestMotionResult(MOTION_RESULT_FAULT, motionMode, MOTION_FAULT_IMU);
      }
      setIdleStateAndStopMotors();
      unlockState();
      continue;
    }

    // Required timing order:
    // 1) Update/read IMU yaw once
    // 2) Copy encoder counts safely
    // 3) Compute measured wheel RPM
    // 4) Compute current forward distance
    // 5) Position loop => baseForwardRpm
    // 6) Heading loop => turnCorrectionRPM and per-wheel targets
    // 7) Velocity loops => PWM
    // 8) Apply PWM

    long copiedCounts[WHEEL_COUNT];
    noInterrupts();
    for (int i = 0; i < WHEEL_COUNT; i++) {
      copiedCounts[i] = encoderCounts[i];
    }
    interrupts();

    float avgDeltaDistanceM = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
      long delta = copiedCounts[i] - lastEncoderCounts[i];
      lastEncoderCounts[i] = copiedCounts[i];
      lastDeltaCounts[i] = delta;

      computeWheelRpmFromDelta(delta, dtSec, measuredRpm[i]);
      avgDeltaDistanceM += deltaCountsToDistanceMeters(delta);
    }

    avgDeltaDistanceM /= (float)WHEEL_COUNT;
    if (motionMode == MODE_MOVE_FORWARD) {
      currentDistanceM += avgDeltaDistanceM;
    }

    if (motionMode == MODE_MANUAL_VELOCITY) {
    if (checkManualWatchdog()) {
        // watchdog already stopped motors and set mode to IDLE
        unlockState();
        continue;
    }
    runManualMixer();                     // writes targetRpm[]
    for (int i = 0; i < WHEEL_COUNT; i++) {
        runVelocityLoopForWheel((WheelIndex)i, dtSec);
    }
    applyAllWheelMotorCommands(finalPwm);
    unlockState();
    continue;   // skip the rest of the autonomous logic
}

    if (motionMode == MODE_MOVE_FORWARD) {
      runPositionLoop(dtSec, now);
      if (motionResultCode != MOTION_RESULT_NONE) {
        if ((motionResultCode == MOTION_RESULT_DONE) &&
            (motionResultMode == MODE_MOVE_FORWARD) &&
            headingHoldActive) {
          positionModeActive = false;
          motionMode = MODE_IDLE;
          baseForwardRpm = 0.0f;
          rawBaseForwardRpm = 0.0f;
        } else {
          setIdleStateAndStopMotors();
          unlockState();
          continue;
        }
      }
    } else {
      baseForwardRpm = 0.0f;
      rawBaseForwardRpm = 0.0f;
      distanceErrorM = targetDistanceM - currentDistanceM;
    }

    if (!positionModeActive && !headingHoldActive) {
      // Hold hard stop when idle.
      resetVelocityControllerState();
      stopAllMotorHardware();
      unlockState();
      continue;
    }

    if (motionMode == MODE_ROTATE_TO_HEADING) {
      // Pure rotation bypasses distance control. The heading loop still feeds
      // the same wheel velocity PID, but with left/right RPM targets for an in-place turn.
      headingErrorRad = wrapAngleRad(targetHeadingRad - currentHeadingRad);

      if ((motionStartMs != 0U) && ((now - motionStartMs) >= ROTATE_TIMEOUT_MS)) {
        rotateIntegralRadS = 0.0f;
        requestMotionResult(MOTION_RESULT_FAULT, MODE_ROTATE_TO_HEADING, MOTION_FAULT_TIMEOUT);
      } else if (fabs(headingErrorRad) <= headingToleranceRad()) {
        lastDoneRotateYawDeg = radToDeg(currentHeadingRad);
        lastDoneRotateTargetDeg = radToDeg(targetHeadingRad);
        lastDoneRotateErrDeg = radToDeg(headingErrorRad);
        lastDoneRotateSnapshotValid = true;
        rotateIntegralRadS = 0.0f;
        // Demo behavior: finish immediately once we enter the heading window.
        setAllTargetRpm(0.0f);
        turnCorrectionRPM = 0.0f;
        leftRpmComposed = 0.0f;
        rightRpmComposed = 0.0f;
        requestMotionResult(MOTION_RESULT_DONE, MODE_ROTATE_TO_HEADING, MOTION_FAULT_NONE);
      }

      if (motionResultCode != MOTION_RESULT_NONE) {
        rotateIntegralRadS = 0.0f;
        setIdleStateAndStopMotors();
        unlockState();
        continue;
      }

      runRotateLoopAndComposeWheelTargets(currentHeadingRad);
    } else {
      runHeadingLoopAndComposeWheelTargets(
        positionModeActive,
        motionMode == MODE_MOVE_FORWARD,
        currentDtSeconds,
        baseForwardRpm,
        currentHeadingRad
      );
    }

    if (!positionModeActive && !headingHoldActive) {
      // Could become inactive if IMU safety stop happened.
      resetVelocityControllerState();
      stopAllMotorHardware();
      unlockState();
      continue;
    }

    for (int i = 0; i < WHEEL_COUNT; i++) {
      runVelocityLoopForWheel((WheelIndex)i, dtSec);
    }

    applyAllWheelMotorCommands(finalPwm);
    unlockState();
  }
}

// Serial commands are still available through USB serial.

// -----------------------------
// Web status JSON provider
// -----------------------------
// Called from the web server handler (loop() thread).
// Acquires stateMutex with a short timeout so it never blocks the control loop.
// JSON keys for arrays are flattened (kpVel_0 … kpVel_3) so the browser-side
// refreshStatus() function can update them with a simple id lookup.
String buildStatusJson() {
  if (!lockState(pdMS_TO_TICKS(20))) {
    return "{\"error\":\"LOCK_BUSY\"}";
  }

  const float yawDeg       = radToDeg(currentHeadingRad);
  const float tgtYawDeg    = radToDeg(targetHeadingRad);
  const float hErrDeg      = radToDeg(headingErrorRad);
  const char *faultStr     = motionFaultToString(motionFaultCode);
  const char *modeStr      = modeToString(motionMode);
  const bool  active       = motionIsActive();

  // Snapshot all values while lock is held, then build the string after unlock
  // to keep the critical section as short as possible.
  float snap_kpVel[WHEEL_COUNT], snap_kiVel[WHEEL_COUNT];
  for (int i = 0; i < WHEEL_COUNT; i++) {
    snap_kpVel[i] = kpVel[i];
    snap_kiVel[i] = kiVel[i];
  }
  const float snap_Kp_pos              = Kp_pos;
  const float snap_Ki_pos              = Ki_pos;
  const float snap_Kp_heading          = Kp_heading_rpm;
  const float snap_Ki_heading          = Ki_heading_rpm_per_rad_s;
  const bool snap_headingHoldActive    = headingHoldActive;
  const float snap_Kp_rotate           = Kp_rotate_rpm;
  const float snap_HTOL                = HEADING_TOLERANCE_DEG;
  const float snap_curDist             = currentDistanceM;
  const float snap_tgtDist             = targetDistanceM;
  const float snap_distErr             = distanceErrorM;
  const float snap_baseRpm             = baseForwardRpm;
  const float snap_turnRpm             = turnCorrectionRPM;
  // Copy lastCommandResponse safely (may contain arbitrary text – strip quotes).
  String safeLastCmd = lastCommandResponse;
  safeLastCmd.replace("\"", "'");

  unlockState();

  String j;
  j.reserve(512);
  j += "{";
  j += "\"mode\":\"";         j += modeStr;          j += "\",";
  j += "\"active\":";         j += active ? "true" : "false"; j += ",";
  j += "\"yawDeg\":";         j += String(yawDeg,    2); j += ",";
  j += "\"targetYawDeg\":";   j += String(tgtYawDeg, 2); j += ",";
  j += "\"headingErrorDeg\":";j += String(hErrDeg,   2); j += ",";
  j += "\"currentDistanceM\":"; j += String(snap_curDist, 3); j += ",";
  j += "\"targetDistanceM\":";  j += String(snap_tgtDist, 3); j += ",";
  j += "\"distanceErrorM\":";   j += String(snap_distErr, 3); j += ",";
  j += "\"baseForwardRpm\":";   j += String(snap_baseRpm, 2); j += ",";
  j += "\"turnCorrectionRPM\":";j += String(snap_turnRpm, 2); j += ",";
  j += "\"Kp_pos\":";           j += String(snap_Kp_pos,     3); j += ",";
  j += "\"Ki_pos\":";           j += String(snap_Ki_pos,     3); j += ",";
  j += "\"Kp_heading_rpm\":";   j += String(snap_Kp_heading, 3); j += ",";
  j += "\"Ki_heading_rpm_per_rad_s\":"; j += String(snap_Ki_heading, 3); j += ",";
  j += "\"headingHoldActive\":"; j += snap_headingHoldActive ? "true" : "false"; j += ",";
  j += "\"Kp_rotate_rpm\":";    j += String(snap_Kp_rotate,  3); j += ",";
  j += "\"HEADING_TOLERANCE_DEG\":"; j += String(snap_HTOL,  2); j += ",";
  for (int i = 0; i < WHEEL_COUNT; i++) {
    j += "\"kpVel_"; j += String(i); j += "\":"; j += String(snap_kpVel[i], 3); j += ",";
    j += "\"kiVel_"; j += String(i); j += "\":"; j += String(snap_kiVel[i], 3); j += ",";
  }
  j += "\"lastCommandResponse\":\""; j += safeLastCmd; j += "\",";
  j += "\"fault\":\""; j += faultStr; j += "\"";
  j += "}";
  return j;
}

// -----------------------------
// Setup/loop
// -----------------------------
/*
Testing plan:
1) Test: heading off + move 0.30
2) Test: heading on  + move 0.30
3) If correction is wrong direction, run: hinvert
4) Tune hkp: 20, 30, 40
5) Then test move 0.50

Tuning hints:
- If robot still curves, increase hkp slowly.
- If robot wiggles, decrease hkp.
- Keep robot still during IMU startup calibration.
- Gyro-integrated yaw drifts over long runs, but is acceptable for short moves.
*/

void setup() {
#if ENABLE_SERIAL_DEBUG_OUTPUT
  Serial.begin(115200);
#endif

  configureMotorPinsSafe();
  stopAllMotorHardware();
  configureEncoderPins();

  microRosPublishMutex = xSemaphoreCreateMutex();
  driveStatusQueueMutex = xSemaphoreCreateMutex();

  // IMU startup.
  // If begin fails, heading control safety behavior will stop motion when needed.
  imuHealthy = app::imuDriver().begin();

  stateMutex = xSemaphoreCreateMutex();

  initManualMode();

  if (lockState(pdMS_TO_TICKS(50))) {
    setIdleStateAndStopMotors();

    // Match the heading-control test behavior: hold the startup yaw even
    // before the first MOVE command, with zero forward RPM.
    currentHeadingRad = app::imuDriver().displayedYawRad();
    if (imuHealthy && isfinite(currentHeadingRad)) {
      targetHeadingRad = currentHeadingRad;
      headingErrorRad = 0.0f;
      headingIntegralRadS = 0.0f;
      headingHoldActive = true;
    }

    noInterrupts();
    for (int i = 0; i < WHEEL_COUNT; i++) {
      lastEncoderCounts[i] = encoderCounts[i];
      lastDeltaCounts[i] = 0;
    }
    interrupts();

    unlockState();
  }

  lastControlTime = millis();

  xTaskCreatePinnedToCore(
    imuUpdateTask,
    "IMUUpdate",
    4096,
    nullptr,
    4,
    &imuTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    controlLoopTask,
    "ControlLoop",
    6144,
    nullptr,
    3,
    &controlTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    microRosTask,
    "microRosTask",
    8192,
    nullptr,
    1,
    &microRosTaskHandle,
    0
  );

  // Connect WiFi and start the tuning dashboard web server.
  // This runs after RTOS tasks are created so the control loop is already live.
  WiFi.begin(app_wifi::kSsid, app_wifi::kPassword);
  {
    int wifiAttempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
      delay(500);
      wifiAttempts++;
    }
  }
  app::configureWebControl(executeCommandLine, buildStatusJson);
  app::beginWebControl();
}

void loop() {
#if ENABLE_SERIAL_TEXT_COMMANDS
  pollSerialCommands();
#endif
  announceMotionResultIfNeeded();
  app::pollWebControl();
  vTaskDelay(pdMS_TO_TICKS(5));
}
