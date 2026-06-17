#include <Arduino.h>
#include <ctype.h>
#include <string.h>

#include <ESP32Servo.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>

// USB Serial is used by micro-ROS as binary transport. Keep debug text off it.
#define DEBUG_PRINT(x) do {} while (0)
#define DEBUG_PRINTLN(x) do {} while (0)

// =============================================================================
// PIN CONFIGURATION
// =============================================================================

#define PWM_PIN 13
#define IN1_PIN 4
#define IN2_PIN 16

#define ENC_A_PIN 12
#define ENC_B_PIN 15

#define LIMIT_PIN 17

#define TRIG_L 22
#define ECHO_L 23

#define TRIG_F 19
#define ECHO_F 21

#define TRIG_R 5
#define ECHO_R 18

#define RED_PIN 25
#define YELLOW_PIN 26
#define GREEN_PIN 27

#define VOLTAGE_PIN 34

#define GRIPPER_SERVO_PIN 14
#define GRIPPER_OPEN_ANGLE 157
#define GRIPPER_CLOSE_ANGLE 80

#define LOCK_SERVO_PIN 33
#define LOCK_OPEN_ANGLE 0
#define LOCK_CLOSE_ANGLE 90

#define LID_SERVO_PIN 32
#define LID_OPEN_ANGLE 50
#define LID_CLOSE_ANGLE 145
#define LID_SWEEP_DELAY_MS 15

// =============================================================================
// ULTRASONIC CONFIGURATION
// =============================================================================

// Lower than 30000 to avoid blocking too long.
// 15000 us roughly covers around 2.5 m max distance.
const unsigned long TIMEOUT_US = 4000;

const float OBSTACLE_THRESHOLD_CM = 40.0;

// Time between triggering different sensors.
// Reduces ultrasonic cross-talk.
const int SENSOR_GAP_MS = 10;

// Sensor task period.
// ESP2 publishes obstacle status every 100 ms.
const int SENSOR_TASK_PERIOD_MS = 50;
const int MICROROS_PUBLISH_PERIOD_MS = 50;
constexpr TickType_t MICRO_ROS_TASK_DELAY_TICKS = pdMS_TO_TICKS(2);
constexpr TickType_t MICRO_ROS_WAITING_AGENT_DELAY_TICKS = pdMS_TO_TICKS(500);
constexpr TickType_t MICRO_ROS_CONNECTED_PING_DELAY_TICKS = pdMS_TO_TICKS(200);
constexpr TickType_t MICRO_ROS_INITIAL_STARTUP_DELAY_TICKS = pdMS_TO_TICKS(2000);

const float VOLTAGE_R1 = 30000.0;
const float VOLTAGE_R2 = 7500.0;
const float ADC_REF_VOLTAGE = 3.3;
const float ADC_MAX_VALUE = 4095.0;

const int VOLTAGE_TASK_PERIOD_MS = 2000;
const int VOLTAGE_PUBLISH_PERIOD_MS = 2000;

// Clear confirmation: obstacle clear only after this many consecutive
// unblocked cycles, but obstacle detect is immediate on first raw reading.
const int CLEAR_CONFIRM_COUNT = 2;

// =============================================================================
// SHARED OBSTACLE DATA
// =============================================================================

volatile long encoderCount = 0;

const long ZERO_TICKS = 0;
const long NINETY_DEG_TICKS = -2500;

float Kp = 1.00;
float Ki = 0.00;
float Kd = 0.00;

long targetTicks = NINETY_DEG_TICKS;

float integral = 0;
float previousError = 0;

unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL_MS = 20;

long POSITION_TOLERANCE = 15;

bool MOTOR_INVERT = true;

bool pidEnabled = false;
bool homed = false;
volatile int activePositionCmd = 999;

SemaphoreHandle_t obstacleMutex;

int latestObstacleMask = 0;

// Per-sensor blocked state and clear counters for fast-detect / stable-clear logic.
bool leftBlocked  = false;
bool frontBlocked = false;
bool rightBlocked = false;

int leftClearCounter  = 0;
int frontClearCounter = 0;
int rightClearCounter = 0;

SemaphoreHandle_t trafficMutex;
char latestTrafficCmd = 'O';
bool newTrafficCmdAvailable = false;

SemaphoreHandle_t voltageMutex;
float latestVoltage = 0.0;

SemaphoreHandle_t gripperMutex;
char latestGripperCmd[32] = "close_gripper";
bool newGripperCmdAvailable = false;

Servo gripperServo;
Servo lockServo;
Servo lidServo;
int currentLidAngle = LID_CLOSE_ANGLE;

// =============================================================================
// MICRO-ROS OBJECTS
// =============================================================================

rcl_publisher_t obstaclePublisher = rcl_get_zero_initialized_publisher();
std_msgs__msg__Int32 obstacleMsg;

rcl_publisher_t voltagePublisher = rcl_get_zero_initialized_publisher();
std_msgs__msg__Float32 voltageMsg;

rcl_publisher_t esp2StatusPublisher = rcl_get_zero_initialized_publisher();
std_msgs__msg__String esp2StatusMsg;
char esp2StatusMsgBuffer[32];

rcl_subscription_t positionCmdSubscriber = rcl_get_zero_initialized_subscription();
std_msgs__msg__Int32 positionCmdMsg;

rcl_subscription_t trafficCmdSubscriber = rcl_get_zero_initialized_subscription();
std_msgs__msg__String trafficCmdMsg;
char trafficCmdBuffer[8];

rcl_subscription_t gripperCmdSubscriber = rcl_get_zero_initialized_subscription();
std_msgs__msg__String gripperCmdMsg;
char gripperCmdBuffer[32];

rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

rclc_support_t support = {};
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node = rcl_get_zero_initialized_node();
bool microRosInitialized = false;

enum MicroRosConnectionState
{
  MICRO_ROS_WAITING_AGENT,
  MICRO_ROS_AGENT_AVAILABLE,
  MICRO_ROS_AGENT_CONNECTED,
  MICRO_ROS_AGENT_DISCONNECTED,
};

typedef struct
{
  char text[32];
} Esp2StatusEvent;

QueueHandle_t esp2StatusQueue;

// =============================================================================
// ERROR HANDLING
// =============================================================================

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

void queueEsp2Status(const char *status)
{
  if (esp2StatusQueue == NULL || status == NULL)
  {
    return;
  }

  Esp2StatusEvent event;
  strncpy(event.text, status, sizeof(event.text));
  event.text[sizeof(event.text) - 1] = '\0';

  (void)xQueueSend(esp2StatusQueue, &event, 0);
}

const char *gripperStatusForCommand(const char *cmd)
{
  if (strcmp(cmd, "open_gripper") == 0)
  {
    return "opened gripper";
  }
  else if (strcmp(cmd, "close_gripper") == 0)
  {
    return "closed gripper";
  }
  else if (strcmp(cmd, "open_lock") == 0)
  {
    return "opened lock";
  }
  else if (strcmp(cmd, "close_lock") == 0)
  {
    return "closed lock";
  }
  else if (strcmp(cmd, "open_lid") == 0)
  {
    return "opened lid";
  }
  else if (strcmp(cmd, "close_lid") == 0)
  {
    return "closed lid";
  }

  return NULL;
}

void queuePositionReachedStatus()
{
  if (activePositionCmd == 90)
  {
    queueEsp2Status("position 90 reached");
    activePositionCmd = 999;
  }
  else if (activePositionCmd == 0)
  {
    queueEsp2Status("position 0 reached");
    activePositionCmd = 999;
  }
}

void allTrafficOff()
{
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
}

void setupTrafficPins()
{
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  allTrafficOff();
}

void applyTrafficCommand(char cmd)
{
  switch (cmd)
  {
    case 'R':
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(YELLOW_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
      break;

    case 'Y':
      digitalWrite(RED_PIN, LOW);
      digitalWrite(YELLOW_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      break;

    case 'G':
      digitalWrite(RED_PIN, LOW);
      digitalWrite(YELLOW_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH);
      break;

    case 'O':
      allTrafficOff();
      break;

    default:
      break;
  }
}

float readInputVoltage()
{
  int adcValue = analogRead(VOLTAGE_PIN);
  float vout = (adcValue * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
  float vin = vout / (VOLTAGE_R2 / (VOLTAGE_R1 + VOLTAGE_R2));
  return vin;
}

void moveLidToAngle(int targetAngle)
{
  targetAngle = constrain(targetAngle, 0, 180);

  if (targetAngle > currentLidAngle)
  {
    for (int pos = currentLidAngle; pos <= targetAngle; pos++)
    {
      lidServo.write(pos);
      delay(LID_SWEEP_DELAY_MS);
    }
  }
  else
  {
    lidServo.write(targetAngle);
  }

  currentLidAngle = targetAngle;
}

void setupGripperServo()
{
  gripperServo.attach(GRIPPER_SERVO_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);

  lockServo.attach(LOCK_SERVO_PIN);
  lockServo.write(LOCK_OPEN_ANGLE);

  lidServo.attach(LID_SERVO_PIN);
  lidServo.write(LID_CLOSE_ANGLE);
  currentLidAngle = LID_CLOSE_ANGLE;
}

void applyGripperCommand(const char *cmd)
{
  if (strcmp(cmd, "open_gripper") == 0)
  {
    gripperServo.write(GRIPPER_OPEN_ANGLE);
  }
  else if (strcmp(cmd, "close_gripper") == 0)
  {
    gripperServo.write(GRIPPER_CLOSE_ANGLE);
  }
  else if (strcmp(cmd, "open_lock") == 0)
  {
    lockServo.write(LOCK_OPEN_ANGLE);
  }
  else if (strcmp(cmd, "close_lock") == 0)
  {
    lockServo.write(LOCK_CLOSE_ANGLE);
  }
  else if (strcmp(cmd, "open_lid") == 0)
  {
    moveLidToAngle(LID_OPEN_ANGLE);
  }
  else if (strcmp(cmd, "close_lid") == 0)
  {
    moveLidToAngle(LID_CLOSE_ANGLE);
  }
}

// =============================================================================
// PROTECTED MOTOR PID MODULE
// =============================================================================

void setupMotor()
{
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Same PWM style as the proven working gripper+motor sketch.
  // setup() attaches servos before this function is called.
  analogWrite(PWM_PIN, 0);

  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void setMotorPwm(int pwm)
{
  pwm = constrain(pwm, 0, 255);
  analogWrite(PWM_PIN, pwm);
}

void moveMotorPositive(int pwm)
{
  if (!MOTOR_INVERT)
  {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  }
  else
  {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }

  setMotorPwm(pwm);
}

void moveMotorNegative(int pwm)
{
  if (!MOTOR_INVERT)
  {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
  else
  {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  }

  setMotorPwm(pwm);
}

void stopMotor()
{
  setMotorPwm(0);

  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

long getEncoderCount()
{
  noInterrupts();
  long countCopy = encoderCount;
  interrupts();

  return countCopy;
}

void zeroEncoder()
{
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

void resetPID()
{
  integral = 0;
  previousError = 0;
  lastPIDTime = millis();
}

void IRAM_ATTR encoderISR()
{
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);

  if (A == B)
    encoderCount++;
  else
    encoderCount--;
}

void runPID()
{
  unsigned long now = millis();

  if (now - lastPIDTime < PID_INTERVAL_MS) return;

  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  long currentPosition = getEncoderCount();

  long error = targetTicks - currentPosition;

  if (abs(error) <= POSITION_TOLERANCE)
  {
    stopMotor();
    pidEnabled = false;
    queuePositionReachedStatus();

    DEBUG_PRINT("Target reached. Position = ");
    DEBUG_PRINTLN(currentPosition);

    return;
  }

  integral += error * dt;

  float derivative = (error - previousError) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;

  previousError = error;

  int pwm = abs(output);
  pwm = constrain(pwm, 0, 255);

  if (output > 0)
  {
    moveMotorPositive(pwm);
  }
  else
  {
    moveMotorNegative(pwm);
  }

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 200)
  {
    lastPrint = millis();

    DEBUG_PRINT("Target: ");
    DEBUG_PRINT(targetTicks);

    DEBUG_PRINT(" | Pos: ");
    DEBUG_PRINT(currentPosition);

    DEBUG_PRINT(" | Error: ");
    DEBUG_PRINT(error);

    DEBUG_PRINT(" | PID Output: ");
    DEBUG_PRINT(output);

    DEBUG_PRINT(" | PWM: ");
    DEBUG_PRINT(pwm);

    DEBUG_PRINT(" | Kp: ");
    DEBUG_PRINT(Kp);

    DEBUG_PRINT(" Ki: ");
    DEBUG_PRINT(Ki);

    DEBUG_PRINT(" Kd: ");
    DEBUG_PRINTLN(Kd);
  }
}

void checkHomeLimit()
{
  if (digitalRead(LIMIT_PIN) == HIGH)
  {
    long pos = getEncoderCount();
    bool homeTargetCompleted = false;

    if (pidEnabled && targetTicks == ZERO_TICKS)
    {
      stopMotor();
      zeroEncoder();
      resetPID();

      pidEnabled = false;
      homed = true;
      queueEsp2Status("position 0 reached");
      activePositionCmd = 999;
      homeTargetCompleted = true;

      DEBUG_PRINTLN("Home reached. Encoder zeroed.");
    }

    if (pos > 50)
    {
      stopMotor();
      pidEnabled = false;
      zeroEncoder();
      if (!homeTargetCompleted && activePositionCmd == 0)
      {
        queueEsp2Status("position 0 reached");
        activePositionCmd = 999;
      }

      DEBUG_PRINTLN("Limit safety stop. Encoder zeroed.");
    }
  }
}

void positionCmdCallback(const void *msgin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  const int32_t cmd = msg->data;

  if (cmd == 90)
  {
    targetTicks = NINETY_DEG_TICKS;
    resetPID();
    activePositionCmd = 90;
    pidEnabled = true;
  }
  else if (cmd == 0)
  {
    targetTicks = ZERO_TICKS;
    resetPID();
    activePositionCmd = 0;
    pidEnabled = true;
  }
  else if (cmd == -1)
  {
    pidEnabled = false;
    activePositionCmd = 999;
    stopMotor();
    queueEsp2Status("position stopped");
  }
  else if (cmd == -2)
  {
    zeroEncoder();
    resetPID();
    activePositionCmd = 999;
    queueEsp2Status("encoder zeroed");
  }
}

void trafficCmdCallback(const void *msgin)
{
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  if (msg->data.size == 0)
  {
    return;
  }

  const char first = msg->data.data[0];
  char normalized = 0;

  if (first == 'R' || first == 'r')
  {
    normalized = 'R';
  }
  else if (first == 'Y' || first == 'y')
  {
    normalized = 'Y';
  }
  else if (first == 'G' || first == 'g')
  {
    normalized = 'G';
  }
  else if (first == 'O' || first == 'o')
  {
    normalized = 'O';
  }
  else
  {
    DEBUG_PRINTLN("Invalid traffic command");
    return;
  }

  if (xSemaphoreTake(trafficMutex, 0) == pdTRUE)
  {
    latestTrafficCmd = normalized;
    newTrafficCmdAvailable = true;
    xSemaphoreGive(trafficMutex);
  }
}

void gripperCmdCallback(const void *msgin)
{
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  if (msg->data.size == 0)
  {
    return;
  }

  char temp[32];
  size_t copyLen = msg->data.size;

  if (copyLen >= sizeof(temp))
  {
    copyLen = sizeof(temp) - 1;
  }

  memcpy(temp, msg->data.data, copyLen);
  temp[copyLen] = '\0';

  bool validCommand =
    strcmp(temp, "open_gripper") == 0 ||
    strcmp(temp, "close_gripper") == 0 ||
    strcmp(temp, "open_lock") == 0 ||
    strcmp(temp, "close_lock") == 0 ||
    strcmp(temp, "open_lid") == 0 ||
    strcmp(temp, "close_lid") == 0;

  if (!validCommand)
  {
    return;
  }

  if (xSemaphoreTake(gripperMutex, pdMS_TO_TICKS(20)) == pdTRUE)
  {
    strncpy(latestGripperCmd, temp, sizeof(latestGripperCmd));
    latestGripperCmd[sizeof(latestGripperCmd) - 1] = '\0';
    newGripperCmdAvailable = true;
    xSemaphoreGive(gripperMutex);
  }
}

// =============================================================================
// ULTRASONIC FUNCTIONS
// =============================================================================

float readUltrasonicCM(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);

  if (duration == 0)
  {
    return -1.0;
  }

  float distanceCM = duration * 0.0343 / 2.0;
  return distanceCM;
}

float median3(float a, float b, float c)
{
  if (a > b)
  {
    float temp = a;
    a = b;
    b = temp;
  }

  if (b > c)
  {
    float temp = b;
    b = c;
    c = temp;
  }

  if (a > b)
  {
    float temp = a;
    a = b;
    b = temp;
  }

  return b;
}

bool isBlocked(float distanceCM)
{
  if (distanceCM > 0 && distanceCM < OBSTACLE_THRESHOLD_CM)
  {
    return true;
  }

  return false;
}

int generateObstacleMask(bool leftBlocked, bool frontBlocked, bool rightBlocked)
{
  int mask = 0;

  if (leftBlocked)
  {
    mask |= 1;   // bit 0: left
  }

  if (frontBlocked)
  {
    mask |= 2;   // bit 1: front
  }

  if (rightBlocked)
  {
    mask |= 4;   // bit 2: right
  }

  return mask;
}

// =============================================================================
// TASK 1: ULTRASONIC SENSOR TASK
// =============================================================================
//
// Detection philosophy:
//   Obstacle appears  → publish nonzero mask on the very first raw blocked reading.
//   Obstacle clears   → require CLEAR_CONFIRM_COUNT consecutive clear cycles before
//                       marking the sensor unblocked, preventing flicker.

void ultrasonicTask(void *parameter)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS);

  while (1)
  {
    float rawLeft = readUltrasonicCM(TRIG_L, ECHO_L);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_GAP_MS));

    float rawFront = readUltrasonicCM(TRIG_F, ECHO_F);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_GAP_MS));

    float rawRight = readUltrasonicCM(TRIG_R, ECHO_R);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_GAP_MS));

    // ----- Left -----
    if (isBlocked(rawLeft))
    {
      leftBlocked = true;
      leftClearCounter = 0;
    }
    else
    {
      leftClearCounter++;
      if (leftClearCounter >= CLEAR_CONFIRM_COUNT)
      {
        leftBlocked = false;
      }
    }

    // ----- Front -----
    if (isBlocked(rawFront))
    {
      frontBlocked = true;
      frontClearCounter = 0;
    }
    else
    {
      frontClearCounter++;
      if (frontClearCounter >= CLEAR_CONFIRM_COUNT)
      {
        frontBlocked = false;
      }
    }

    // ----- Right -----
    if (isBlocked(rawRight))
    {
      rightBlocked = true;
      rightClearCounter = 0;
    }
    else
    {
      rightClearCounter++;
      if (rightClearCounter >= CLEAR_CONFIRM_COUNT)
      {
        rightBlocked = false;
      }
    }

    int obstacleMask = generateObstacleMask(leftBlocked, frontBlocked, rightBlocked);

    if (xSemaphoreTake(obstacleMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      latestObstacleMask = obstacleMask;
      xSemaphoreGive(obstacleMutex);
    }

    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

void motorPidTask(void *parameter)
{
  const TickType_t periodTicks = pdMS_TO_TICKS(5);

  while (1)
  {
    checkHomeLimit();

    if (pidEnabled)
    {
      runPID();
    }

    vTaskDelay(periodTicks);
  }
}

void trafficLightTask(void *parameter)
{
  const TickType_t periodTicks = pdMS_TO_TICKS(25);

  while (1)
  {
    char cmdToApply = 0;

    if (xSemaphoreTake(trafficMutex, pdMS_TO_TICKS(2)) == pdTRUE)
    {
      if (newTrafficCmdAvailable)
      {
        cmdToApply = latestTrafficCmd;
        newTrafficCmdAvailable = false;
      }
      xSemaphoreGive(trafficMutex);
    }

    if (cmdToApply != 0)
    {
      applyTrafficCommand(cmdToApply);
    }

    vTaskDelay(periodTicks);
  }
}

void voltageSensorTask(void *parameter)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(VOLTAGE_TASK_PERIOD_MS);

  while (1)
  {
    float measuredVoltage = readInputVoltage();

    if (xSemaphoreTake(voltageMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      latestVoltage = measuredVoltage;
      xSemaphoreGive(voltageMutex);
    }

    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

void gripperTask(void *parameter)
{
  while (1)
  {
    char cmdToApply[32];
    bool shouldApply = false;

    if (xSemaphoreTake(gripperMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      if (newGripperCmdAvailable)
      {
        strncpy(cmdToApply, latestGripperCmd, sizeof(cmdToApply));
        cmdToApply[sizeof(cmdToApply) - 1] = '\0';
        newGripperCmdAvailable = false;
        shouldApply = true;
      }

      xSemaphoreGive(gripperMutex);
    }

    if (shouldApply)
    {
      applyGripperCommand(cmdToApply);
      vTaskDelay(pdMS_TO_TICKS(1000));
      const char *status = gripperStatusForCommand(cmdToApply);
      if (status != NULL)
      {
        queueEsp2Status(status);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =============================================================================
// TASK 2: MICRO-ROS PUBLISHER TASK
// =============================================================================

bool microRosCheckOk(rcl_ret_t rc)
{
  return rc == RCL_RET_OK;
}

void clearEsp2StatusQueue()
{
  if (esp2StatusQueue != NULL)
  {
    xQueueReset(esp2StatusQueue);
  }
}

bool createMicroRosEntities()
{
  esp2StatusMsg.data.data = esp2StatusMsgBuffer;
  esp2StatusMsg.data.size = 0;
  esp2StatusMsg.data.capacity = sizeof(esp2StatusMsgBuffer);

  trafficCmdMsg.data.data = trafficCmdBuffer;
  trafficCmdMsg.data.size = 0;
  trafficCmdMsg.data.capacity = sizeof(trafficCmdBuffer);

  gripperCmdMsg.data.data = gripperCmdBuffer;
  gripperCmdMsg.data.size = 0;
  gripperCmdMsg.data.capacity = sizeof(gripperCmdBuffer);

  allocator = rcl_get_default_allocator();
  executor = rclc_executor_get_zero_initialized_executor();

  if (!microRosCheckOk(rclc_support_init(&support, 0, NULL, &allocator)))
  {
    return false;
  }

  if (!microRosCheckOk(rclc_node_init_default(&node, "esp2_mission_hardware", "", &support)))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_publisher_init_default(
          &obstaclePublisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "/obstacle_status")))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_publisher_init_default(
          &voltagePublisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "/esp2/voltage")))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_publisher_init_default(
          &esp2StatusPublisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/esp2_status")))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_subscription_init_default(
          &positionCmdSubscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "/esp2/position_cmd")))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_subscription_init_default(
          &trafficCmdSubscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/esp2/traffic_cmd")))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_subscription_init_default(
          &gripperCmdSubscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/esp2/gripper_cmd")))
  {
    return false;
  }

  if (!microRosCheckOk(rclc_executor_init(&executor, &support.context, 3, &allocator)))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_executor_add_subscription(
          &executor,
          &positionCmdSubscriber,
          &positionCmdMsg,
          &positionCmdCallback,
          ON_NEW_DATA)))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_executor_add_subscription(
          &executor,
          &trafficCmdSubscriber,
          &trafficCmdMsg,
          &trafficCmdCallback,
          ON_NEW_DATA)))
  {
    return false;
  }

  if (!microRosCheckOk(
        rclc_executor_add_subscription(
          &executor,
          &gripperCmdSubscriber,
          &gripperCmdMsg,
          &gripperCmdCallback,
          ON_NEW_DATA)))
  {
    return false;
  }

  microRosInitialized = true;
  return true;
}

void destroyMicroRosEntities()
{
  if (microRosInitialized)
  {
    rmw_context_t *rmwContext = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, 0);
  }

  if (positionCmdSubscriber.impl != NULL)
  {
    (void)rcl_subscription_fini(&positionCmdSubscriber, &node);
  }
  if (trafficCmdSubscriber.impl != NULL)
  {
    (void)rcl_subscription_fini(&trafficCmdSubscriber, &node);
  }
  if (gripperCmdSubscriber.impl != NULL)
  {
    (void)rcl_subscription_fini(&gripperCmdSubscriber, &node);
  }
  if (obstaclePublisher.impl != NULL)
  {
    (void)rcl_publisher_fini(&obstaclePublisher, &node);
  }
  if (voltagePublisher.impl != NULL)
  {
    (void)rcl_publisher_fini(&voltagePublisher, &node);
  }
  if (esp2StatusPublisher.impl != NULL)
  {
    (void)rcl_publisher_fini(&esp2StatusPublisher, &node);
  }
  if (executor.context != NULL)
  {
    (void)rclc_executor_fini(&executor);
  }
  if (node.impl != NULL)
  {
    (void)rcl_node_fini(&node);
  }
  if (support.context.impl != NULL)
  {
    (void)rclc_support_fini(&support);
  }

  obstaclePublisher = rcl_get_zero_initialized_publisher();
  voltagePublisher = rcl_get_zero_initialized_publisher();
  esp2StatusPublisher = rcl_get_zero_initialized_publisher();
  positionCmdSubscriber = rcl_get_zero_initialized_subscription();
  trafficCmdSubscriber = rcl_get_zero_initialized_subscription();
  gripperCmdSubscriber = rcl_get_zero_initialized_subscription();
  node = rcl_get_zero_initialized_node();
  support = {};
  executor = rclc_executor_get_zero_initialized_executor();
  microRosInitialized = false;
}

void runMicroRosConnectedWork()
{
  static unsigned long lastVoltagePublish = 0;
  static unsigned long lastObstaclePublish = 0;

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

  if (millis() - lastVoltagePublish >= VOLTAGE_PUBLISH_PERIOD_MS)
  {
    lastVoltagePublish = millis();

    float voltageToPublish = 0.0;

    if (xSemaphoreTake(voltageMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      voltageToPublish = latestVoltage;
      xSemaphoreGive(voltageMutex);
    }

    voltageMsg.data = voltageToPublish;
    RCSOFTCHECK(rcl_publish(&voltagePublisher, &voltageMsg, NULL));
  }

  Esp2StatusEvent statusEvent;
  while (esp2StatusQueue != NULL && xQueueReceive(esp2StatusQueue, &statusEvent, 0) == pdTRUE)
  {
    strncpy(esp2StatusMsgBuffer, statusEvent.text, sizeof(esp2StatusMsgBuffer));
    esp2StatusMsgBuffer[sizeof(esp2StatusMsgBuffer) - 1] = '\0';
    esp2StatusMsg.data.size = strlen(esp2StatusMsgBuffer);

    RCSOFTCHECK(rcl_publish(&esp2StatusPublisher, &esp2StatusMsg, NULL));
  }

  if (millis() - lastObstaclePublish >= MICROROS_PUBLISH_PERIOD_MS)
  {
    lastObstaclePublish = millis();

    int maskToPublish = 0;

    if (xSemaphoreTake(obstacleMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      maskToPublish = latestObstacleMask;
      xSemaphoreGive(obstacleMutex);
    }

    obstacleMsg.data = maskToPublish;
    RCSOFTCHECK(rcl_publish(&obstaclePublisher, &obstacleMsg, NULL));
  }
}

void microRosTask(void *parameter)
{
  (void)parameter;

  set_microros_transports();
  vTaskDelay(MICRO_ROS_INITIAL_STARTUP_DELAY_TICKS);

  MicroRosConnectionState state = MICRO_ROS_WAITING_AGENT;

  while (true)
  {
    switch (state)
    {
      case MICRO_ROS_WAITING_AGENT:
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
        {
          state = MICRO_ROS_AGENT_AVAILABLE;
        }
        else
        {
          vTaskDelay(MICRO_ROS_WAITING_AGENT_DELAY_TICKS);
        }
        break;

      case MICRO_ROS_AGENT_AVAILABLE:
        if (createMicroRosEntities())
        {
          clearEsp2StatusQueue();
          state = MICRO_ROS_AGENT_CONNECTED;
        }
        else
        {
          destroyMicroRosEntities();
          vTaskDelay(MICRO_ROS_WAITING_AGENT_DELAY_TICKS);
          state = MICRO_ROS_WAITING_AGENT;
        }
        break;

      case MICRO_ROS_AGENT_CONNECTED:
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1))
        {
          state = MICRO_ROS_AGENT_DISCONNECTED;
          break;
        }

        runMicroRosConnectedWork();
        vTaskDelay(MICRO_ROS_TASK_DELAY_TICKS);
        break;

      case MICRO_ROS_AGENT_DISCONNECTED:
        destroyMicroRosEntities();
        clearEsp2StatusQueue();
        set_microros_transports();
        vTaskDelay(MICRO_ROS_CONNECTED_PING_DELAY_TICKS);
        state = MICRO_ROS_WAITING_AGENT;
        break;

      default:
        state = MICRO_ROS_WAITING_AGENT;
        break;
    }
  }
}

// =============================================================================
// SETUP
// =============================================================================

void setup()
{
  Serial.begin(115200);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  pinMode(VOLTAGE_PIN, INPUT);

  digitalWrite(TRIG_L, LOW);
  digitalWrite(TRIG_F, LOW);
  digitalWrite(TRIG_R, LOW);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  pinMode(LIMIT_PIN, INPUT);

  setupTrafficPins();

  // Critical order: servos first, then motor PWM.
  setupGripperServo();
  setupMotor();
  stopMotor();

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);

  if (digitalRead(LIMIT_PIN) == HIGH)
  {
    zeroEncoder();
    homed = true;
  }

  obstacleMutex = xSemaphoreCreateMutex();
  trafficMutex = xSemaphoreCreateMutex();
  voltageMutex = xSemaphoreCreateMutex();
  gripperMutex = xSemaphoreCreateMutex();
  esp2StatusQueue = xQueueCreate(10, sizeof(Esp2StatusEvent));

  xTaskCreatePinnedToCore(
    ultrasonicTask,
    "Ultrasonic Task",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    microRosTask,
    "micro-ROS Task",
    8192,
    NULL,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    motorPidTask,
    "Motor PID Task",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    trafficLightTask,
    "Traffic Light Task",
    3072,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    voltageSensorTask,
    "Voltage Sensor Task",
    3072,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    gripperTask,
    "Gripper Task",
    3072,
    NULL,
    1,
    NULL,
    1
  );
}

// =============================================================================
// LOOP
// =============================================================================

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
