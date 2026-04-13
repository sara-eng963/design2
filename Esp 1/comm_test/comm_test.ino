#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#define LED_PIN 2

rcl_publisher_t esp_publisher;
std_msgs__msg__String esp_msg;
rcl_subscription_t pi_subscriber;
std_msgs__msg__String pi_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

int counter = 0;
volatile bool led_blinking = false;
unsigned long blink_start_time = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void pi_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  
  Serial.print("ESP32 received from Pi: ");
  Serial.println(msg->data.data);
  
  // DIRECT LED CONTROL - NO FLAGS, NO DELAYS
  digitalWrite(LED_PIN, HIGH);  // OFF
  delay(100);
  digitalWrite(LED_PIN, LOW);   // ON
  delay(100);
  digitalWrite(LED_PIN, HIGH);  // OFF
  delay(100);
  digitalWrite(LED_PIN, LOW);   // ON
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start with LED ON (LOW = ON for many boards)
  
  Serial.println("Testing LED - should blink 3 times");
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  Serial.println("LED test done. LED should be ON now.");
  
  set_microros_transports();
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_bidirectional_node", "", &support));
  RCCHECK(rclc_publisher_init_default(&esp_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp_comm_test"));
  RCCHECK(rclc_subscription_init_default(&pi_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "pi_comm_test"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &pi_subscriber, &pi_msg, &pi_callback, ON_NEW_DATA));
  
  Serial.println("ESP32 ready! Waiting for messages from Pi...");
}

void loop() {
  static unsigned long last_publish = 0;
  
  if (millis() - last_publish > 2000) {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "hello pi %d", counter++);
    esp_msg.data.data = buffer;
    esp_msg.data.size = strlen(buffer);
    esp_msg.data.capacity = strlen(buffer) + 1;
    
    if (rcl_publish(&esp_publisher, &esp_msg, NULL) == RCL_RET_OK) {
      Serial.print("ESP32 published: ");
      Serial.println(buffer);
    }
    last_publish = millis();
  }
  
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}