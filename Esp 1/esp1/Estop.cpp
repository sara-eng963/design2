#include "Estop.h"

#include <cstring>

#include <std_msgs/msg/string.h>

#define E_STOP_PIN 12
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

rcl_publisher_t eStopPublisher = rcl_get_zero_initialized_publisher();
std_msgs__msg__String eStopMsg;
char eStopMsgBuffer[24];
int lastEStopState = HIGH;
bool publishInitialEStopPressed = false;

void configureEStopPin()
{
  pinMode(E_STOP_PIN, INPUT_PULLDOWN);
  lastEStopState = digitalRead(E_STOP_PIN);
  publishInitialEStopPressed = lastEStopState == LOW;
}

bool createEStopPublisher(rcl_node_t *node)
{
  eStopMsg.data.data = eStopMsgBuffer;
  eStopMsg.data.size = 0;
  eStopMsg.data.capacity = sizeof(eStopMsgBuffer);

  return rclc_publisher_init_default(
    &eStopPublisher,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/e_stop") == RCL_RET_OK;
}

void destroyEStopPublisher(rcl_node_t *node)
{
  if (eStopPublisher.impl != NULL)
  {
    (void)rcl_publisher_fini(&eStopPublisher, node);
  }
}

void resetEStopPublisher()
{
  eStopPublisher = rcl_get_zero_initialized_publisher();
}

void prepareEStopInitialPublish()
{
  lastEStopState = digitalRead(E_STOP_PIN);
  publishInitialEStopPressed = lastEStopState == LOW;
}

void pollEStopPublisher()
{
  if (publishInitialEStopPressed)
  {
    strncpy(eStopMsgBuffer, "e_stop_pressed", sizeof(eStopMsgBuffer));
    eStopMsgBuffer[sizeof(eStopMsgBuffer) - 1] = '\0';
    eStopMsg.data.size = strlen(eStopMsgBuffer);

    RCSOFTCHECK(rcl_publish(&eStopPublisher, &eStopMsg, NULL));
    publishInitialEStopPressed = false;
  }

  int currentEStopState = digitalRead(E_STOP_PIN);

  if (currentEStopState != lastEStopState)
  {
    const char *event = currentEStopState == LOW
      ? "e_stop_pressed"
      : "e_stop_released";

    strncpy(eStopMsgBuffer, event, sizeof(eStopMsgBuffer));
    eStopMsgBuffer[sizeof(eStopMsgBuffer) - 1] = '\0';
    eStopMsg.data.size = strlen(eStopMsgBuffer);

    RCSOFTCHECK(rcl_publish(&eStopPublisher, &eStopMsg, NULL));
    lastEStopState = currentEStopState;
  }
}
