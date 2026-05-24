#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// =============================================================================
// PIN CONFIGURATION
// =============================================================================

#define TRIG_L 22
#define ECHO_L 23

#define TRIG_F 19
#define ECHO_F 21

#define TRIG_R 5
#define ECHO_R 18

// =============================================================================
// ULTRASONIC CONFIGURATION
// =============================================================================

// Lower than 30000 to avoid blocking too long.
// 15000 us roughly covers around 2.5 m max distance.
const unsigned long TIMEOUT_US = 15000;

const float OBSTACLE_THRESHOLD_CM = 40.0;

// Time between triggering different sensors.
// Reduces ultrasonic cross-talk.
const int SENSOR_GAP_MS = 30;

// Sensor task period.
// ESP2 publishes obstacle status every 100 ms.
const int SENSOR_TASK_PERIOD_MS = 100;
const int MICROROS_PUBLISH_PERIOD_MS = 100;

// Clear confirmation: obstacle clear only after this many consecutive
// unblocked cycles, but obstacle detect is immediate on first raw reading.
const int CLEAR_CONFIRM_COUNT = 2;

// =============================================================================
// SHARED OBSTACLE DATA
// =============================================================================

SemaphoreHandle_t obstacleMutex;

int latestObstacleMask = 0;

// Per-sensor blocked state and clear counters for fast-detect / stable-clear logic.
bool leftBlocked  = false;
bool frontBlocked = false;
bool rightBlocked = false;

int leftClearCounter  = 0;
int frontClearCounter = 0;
int rightClearCounter = 0;

// =============================================================================
// MICRO-ROS OBJECTS
// =============================================================================

rcl_publisher_t obstaclePublisher;
std_msgs__msg__Int32 obstacleMsg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// =============================================================================
// ERROR HANDLING
// =============================================================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { errorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

void errorLoop()
{
  while (1)
  {
    delay(100);
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

// =============================================================================
// TASK 2: MICRO-ROS PUBLISHER TASK
// =============================================================================

void microRosTask(void *parameter)
{
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "esp2_mission_hardware",
    "",
    &support
  ));

  RCCHECK(rclc_publisher_init_default(
    &obstaclePublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/obstacle_status"
  ));

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(MICROROS_PUBLISH_PERIOD_MS);

  while (1)
  {
    int maskToPublish = 0;

    if (xSemaphoreTake(obstacleMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      maskToPublish = latestObstacleMask;
      xSemaphoreGive(obstacleMutex);
    }

    obstacleMsg.data = maskToPublish;

    RCSOFTCHECK(rcl_publish(&obstaclePublisher, &obstacleMsg, NULL));

    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

// =============================================================================
// SETUP
// =============================================================================

void setup()
{
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  digitalWrite(TRIG_L, LOW);
  digitalWrite(TRIG_F, LOW);
  digitalWrite(TRIG_R, LOW);

  obstacleMutex = xSemaphoreCreateMutex();

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
}

// =============================================================================
// LOOP
// =============================================================================

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}