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

static char drive_status_buffer[256];
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
  minimumMoveRpmActive = false;
  for (int i = 0; i < WHEEL_COUNT; i++) {
    finalPwm[i] = 0;
    appliedPwm[i] = 0;
  }
  stopAllMotorHardware();
}

void setIdleStateAndStopMotors() {
  finalizeMotionStop();
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
  status += " baseRPM=";
  status += String(baseForwardRpm, 1);
  status += " turnRPM=";
  status += String(turnCorrectionRPM, 1);
  status += " RKP=";
  status += String(Kp_rotate_rpm, 1);
  status += " RMAX=";
  status += String(MAX_ROTATE_RPM, 1);
  status += " RMIN=";
  status += String(MIN_ROTATE_RPM, 1);
  status += " RTOL=";
  status += String(HEADING_TOLERANCE_DEG, 1);
  status += " active=";
  status += motionIsActive() ? "1" : "0";
  status += " fault=";
  status += motionFaultToString(motionFaultCode);
  status += " ros=";
  status += microRosInitialized ? "1" : "0";
  return status;
}

void announceMotionResultIfNeeded() {
  String report;

  if (!lockState(pdMS_TO_TICKS(5))) {
    return;
  }

  if (motionResultCode == MOTION_RESULT_DONE) {
    report = String("DONE ") + motionResultLabel(motionResultMode);
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

  // MOVE completion uses both tolerance and a short stable-in-window timer so
  // the robot does not chatter around the finish line.
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
  minimumMoveRpmActive = false;
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
// - HEADING ON|OFF, HKP, HKI, HMAX, HINVERT
// - RKP, RMAX, RMIN, RTOL, RINVERT
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

  if (key == "HMAX") {
    // hmax limits correction strength for safety and smoothness.
    // If turns are too weak, increase hmax. If too aggressive, reduce it.
    MAX_TURN_CORRECTION_RPM = arg.toFloat();
    if (MAX_TURN_CORRECTION_RPM < 0.0f) MAX_TURN_CORRECTION_RPM = 0.0f;
    if (MAX_TURN_CORRECTION_RPM > 120.0f) MAX_TURN_CORRECTION_RPM = 120.0f;
    response = String("ACK HMAX ") + String(MAX_TURN_CORRECTION_RPM, 2);
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

  if (key == "RMAX") {
    MAX_ROTATE_RPM = arg.toFloat();
    if (MAX_ROTATE_RPM < 0.0f) MAX_ROTATE_RPM = 0.0f;
    if (MAX_ROTATE_RPM > MAX_WHEEL_TARGET_RPM) MAX_ROTATE_RPM = MAX_WHEEL_TARGET_RPM;
    response = String("ACK RMAX ") + String(MAX_ROTATE_RPM, 2);
    lastCommandResponse = response;
    unlockState();
    return true;
  }

  if (key == "RMIN") {
    MIN_ROTATE_RPM = arg.toFloat();
    if (MIN_ROTATE_RPM < 0.0f) MIN_ROTATE_RPM = 0.0f;
    if (MIN_ROTATE_RPM > MAX_WHEEL_TARGET_RPM) MIN_ROTATE_RPM = MAX_WHEEL_TARGET_RPM;
    response = String("ACK RMIN ") + String(MIN_ROTATE_RPM, 2);
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

    // Update IMU exactly once per control cycle.
    // This is the only place that advances yaw integration in runtime.
    // Web/status/getters only read already-updated state.
    if (!imuHealthy) {
      if (motionIsActive()) {
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

    if (motionMode == MODE_MOVE_FORWARD) {
      runPositionLoop(dtSec, now);
      if (motionResultCode != MOTION_RESULT_NONE) {
        setIdleStateAndStopMotors();
        unlockState();
        continue;
      }
    } else {
      baseForwardRpm = 0.0f;
      rawBaseForwardRpm = 0.0f;
      minimumMoveRpmActive = false;
      distanceErrorM = targetDistanceM - currentDistanceM;
    }

    if (!positionModeActive) {
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
        requestMotionResult(MOTION_RESULT_FAULT, MODE_ROTATE_TO_HEADING, MOTION_FAULT_TIMEOUT);
      } else if (fabs(headingErrorRad) <= headingToleranceRad()) {
        // Demo behavior: finish immediately once we enter the heading window.
        setAllTargetRpm(0.0f);
        turnCorrectionRPM = 0.0f;
        leftRpmComposed = 0.0f;
        rightRpmComposed = 0.0f;
        requestMotionResult(MOTION_RESULT_DONE, MODE_ROTATE_TO_HEADING, MOTION_FAULT_NONE);
      } else {
        completionStableSinceMs = 0U;
      }

      if (motionResultCode != MOTION_RESULT_NONE) {
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

    if (!positionModeActive) {
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
// Setup/loop
// -----------------------------
/*
Testing plan:
1) Test: heading off + move 0.30
2) Test: heading on  + move 0.30
3) If correction is wrong direction, run: hinvert
4) Tune hkp: 20, 30, 40
5) Tune hmax: 10, 15, 20
6) Then test move 0.50

Tuning hints:
- If robot still curves, increase hkp slowly.
- If robot wiggles, decrease hkp.
- If correction is too aggressive, reduce hmax.
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

  if (lockState(pdMS_TO_TICKS(50))) {
    setIdleStateAndStopMotors();

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
}

void loop() {
#if ENABLE_SERIAL_TEXT_COMMANDS
  pollSerialCommands();
#endif
  announceMotionResultIfNeeded();
  vTaskDelay(pdMS_TO_TICKS(5));
}
