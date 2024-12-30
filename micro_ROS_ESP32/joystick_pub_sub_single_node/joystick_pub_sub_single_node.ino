#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>


#define PWM_PIN_SPEED 25  // Pin su cui generare il PWM
#define PWM_PIN_STEERING 32  // Pin su cui generare il PWM


#define PWM_FREQ 100  // Frequenza PWM in Hz
#define PWM_RESOLUTION 10  // Risoluzione del PWM (da 1 a 16 bit)



rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_sensor;
geometry_msgs__msg__Twist msg;  // Messaggio per Twist
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}




float mapValue(float x) {
  //map [-1,1] --> [0.1 ,0.2]
    return 0.1 + 0.05 * (x + 1);
}



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
  //Serial.print("Linear X: ");
  Serial.println(msg->linear.x);
  noInterrupts();
  ledcWrite(PWM_PIN_SPEED, mapValue(msg->linear.x) * ((1 << PWM_RESOLUTION) - 1));
  ledcWrite(PWM_PIN_STEERING, mapValue(msg->angular.z * -1) * ((1 << PWM_RESOLUTION) - 1)); 
  interrupts();
  /*
  Serial.print("Linear Y: ");
  Serial.println(msg->linear.y);
  Serial.print("Linear Z: ");
  Serial.println(msg->linear.z);
  
  Serial.print("Angular X: ");
  Serial.println(msg->angular.x);
  Serial.print("Angular Y: ");
  Serial.println(msg->angular.y);
  */
  //Serial.print("Angular Z: ");
  //Serial.println(msg->angular.z);

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_sensor, NULL));
    msg_sensor.data++;
  }
}


void setup() {

  //PWM setup
  ledcAttach(PWM_PIN_SPEED,PWM_FREQ, PWM_RESOLUTION);//motor control
  ledcAttach(PWM_PIN_STEERING,PWM_FREQ, PWM_RESOLUTION); //servo left
  //set pwm
  noInterrupts();
  ledcWrite(PWM_PIN_SPEED, 0.15* ((1 << PWM_RESOLUTION) - 1));
  ledcWrite(PWM_PIN_STEERING, 0.15* ((1 << PWM_RESOLUTION) - 1)); 
  interrupts();
  set_microros_wifi_transports("AutomazioneTesisti", "nicosia456", "192.168.0.120", 8888); // tesisti
  //set_microros_wifi_transports("FASTWEB-USA6DG", "26KCXAYRSU", "192.168.1.52", 8888); //casa
  Serial.begin(115200); 

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create a node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "sensor_publisher"));

  // Create subscriber for geometry_msgs::msg::Twist
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),  // Corretto per Twist
    "cmd_vel"));
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 2 tasks: timer + subscriber
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  msg_sensor.data = 0;
}
void loop() {


RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
