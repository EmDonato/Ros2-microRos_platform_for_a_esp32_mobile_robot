#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/Twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;  // Messaggio per Twist
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist *)msgin;

  // Stampa le componenti lineari e angolari del messaggio Twist
  Serial.print("Linear X: ");
  Serial.println(msg->linear.x);
  Serial.print("Linear Y: ");
  Serial.println(msg->linear.y);
  Serial.print("Linear Z: ");
  Serial.println(msg->linear.z);
  
  Serial.print("Angular X: ");
  Serial.println(msg->angular.x);
  Serial.print("Angular Y: ");
  Serial.println(msg->angular.y);
  Serial.print("Angular Z: ");
  Serial.println(msg->angular.z);
}

void setup() {
  set_microros_wifi_transports("AutomazioneTesisti", "nicosia456", "192.168.1.52", 8888);
  Serial.begin(115200); 

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create a node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // Create subscriber for geometry_msgs::msg::Twist
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),  // Corretto per Twist
    "cmd_vel"));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
