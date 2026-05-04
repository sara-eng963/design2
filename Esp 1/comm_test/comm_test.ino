#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#define LED_PIN 2

rcl_publisher_t esp_publisher;
rcl_subscription_t pi_subscriber;

std_msgs__msg__String esp_msg;
std_msgs__msg__String pi_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

static char esp_msg_buffer[100];
static char pi_msg_buffer[100];

int counter = 0;

volatile bool got_pi_msg = false;
unsigned long blink_time = 0;
int blink_count = 0;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void pi_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  (void)msg;

  got_pi_msg = true;
  blink_count = 0;
  blink_time = millis();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  esp_msg.data.data = esp_msg_buffer;
  esp_msg.data.capacity = sizeof(esp_msg_buffer);
  esp_msg.data.size = 0;

  pi_msg.data.data = pi_msg_buffer;
  pi_msg.data.capacity = sizeof(pi_msg_buffer);
  pi_msg.data.size = 0;

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "esp32_bidirectional_node",
    "",
    &support
  ));

  RCCHECK(rclc_publisher_init_default(
    &esp_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp_comm_test"
  ));

  RCCHECK(rclc_subscription_init_default(
    &pi_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "pi_comm_test"
  ));

  RCCHECK(rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &pi_subscriber,
    &pi_msg,
    &pi_callback,
    ON_NEW_DATA
  ));
}

void handle_led() {
  if (!got_pi_msg) return;

  if (millis() - blink_time >= 100) {
    blink_time = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    blink_count++;

    if (blink_count >= 6) {
      got_pi_msg = false;
      blink_count = 0;
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void loop() {
  static unsigned long last_publish = 0;

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  if (millis() - last_publish >= 2000) {
    snprintf(esp_msg_buffer, sizeof(esp_msg_buffer), "hello pi %d", counter++);

    esp_msg.data.size = strlen(esp_msg_buffer);
    esp_msg.data.capacity = sizeof(esp_msg_buffer);

    RCSOFTCHECK(rcl_publish(&esp_publisher, &esp_msg, NULL));

    // heartbeat blink on every publish attempt
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);

    last_publish = millis();
  }

  handle_led();

  delay(10);
}