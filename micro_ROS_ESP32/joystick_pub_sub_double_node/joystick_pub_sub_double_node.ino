#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node_publisher;
rcl_node_t node_subscriber;
int pwmChannel_speed = 0;
int pwmChannel_steering = 1;

std_msgs__msg__Int32 msg_sensor;
geometry_msgs__msg__Twist msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial.print("Error: "); Serial.println(temp_rc); error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial.print("Warning: "); Serial.println(temp_rc); }}



#define PWM_PIN_SPEED 25  // Pin su cui generare il PWM
#define PWM_PIN_STEERING 32  // Pin su cui generare il PWM


#define PWM_FREQ 100  // Frequenza PWM in Hz
#define PWM_RESOLUTION 10  // Risoluzione del PWM (da 1 a 16 bit)
   
void error_loop() {
    while (1) {
        Serial.println("[ERROR] Entering error loop...");
        delay(1000);
    }
}

void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    Serial.println("[DEBUG] Subscription callback triggered");
    //Serial.print("  Received Twist - Linear X: ");
    Serial.print(msg->linear.x);
    //Serial.print(", Angular Z: ");
    Serial.println(msg->angular.z);
    noInterrupts();
    ledcWrite(pwmChannel_speed, mapValue(msg->linear.x) * ((1 << PWM_RESOLUTION) - 1));
    ledcWrite(pwmChannel_steering, mapValue(msg->angular.z * -1) * ((1 << PWM_RESOLUTION) - 1)); 
    interrupts();
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        //Serial.println("[DEBUG] Timer callback triggered");
        //Serial.println("  Publishing sensor data...");
        RCSOFTCHECK(rcl_publish(&publisher, &msg_sensor, NULL));
        msg_sensor.data++;
        //Serial.print("  Published sensor data: ");
        //Serial.println(msg_sensor.data);
    }
}



float mapValue(float x) {
  //map [-1,1] --> [0.1 ,0.2]
    return 0.1 + 0.05 * (x + 1);
}



void setup() {
    Serial.begin(115200);
    delay(2000);
    ledcSetup(pwmChannel_speed, PWM_FREQ,PWM_RESOLUTION);
    ledcSetup(pwmChannel_steering, PWM_FREQ,PWM_RESOLUTION);

    //Serial.println("[INFO] Starting micro-ROS on ESP32");
    ledcAttachPin(PWM_PIN_SPEED, pwmChannel_speed);//motor control
    ledcAttachPin(PWM_PIN_STEERING, pwmChannel_steering); //servo left
    //set pwm
    noInterrupts();
    ledcWrite(pwmChannel_speed, 0.15* ((1 << PWM_RESOLUTION) - 1));
    ledcWrite(pwmChannel_steering, 0.15* ((1 << PWM_RESOLUTION) - 1)); 
    interrupts();
    // Inizializzazione micro-ROS Wi-Fi
    //Serial.println("[INFO] Setting up micro-ROS Wi-Fi transport...");
    set_microros_wifi_transports("AutomazioneTesisti", "nicosia456", "192.168.0.120", 8888); // tesisti
    //set_microros_wifi_transports("FASTWEB-USA6DG", "26KCXAYRSU", "192.168.1.52", 8888); //casa
    Serial.println("  Wi-Fi transport setup complete");

    allocator = rcl_get_default_allocator();

    // Supporto
    //Serial.println("[INFO] Initializing support...");
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    //Serial.println("  Support initialized successfully");

    // Creazione dei nodi
    //Serial.println("[INFO] Creating nodes...");
    RCCHECK(rclc_node_init_default(&node_publisher, "micro_ros_esp32_node_publisher", "", &support));
    //Serial.println("  Publisher node created");
    RCCHECK(rclc_node_init_default(&node_subscriber, "micro_ros_esp32_node_subscriber", "", &support));
    //Serial.println("  Subscriber node created");

    // Publisher
    //Serial.println("[INFO] Initializing publisher...");
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher,
        &node_publisher,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "sensor_publisher"));
    //Serial.println("  Publisher initialized successfully");

    // Subscriber
    Serial.println("[INFO] Initializing subscriber...");
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node_subscriber,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    Serial.println("  Subscriber initialized successfully");

    // Timer
    //Serial.println("[INFO] Initializing timer...");
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    //erial.println("  Timer initialized successfully");

    // Executor
    //Serial.println("[INFO] Initializing executor...");
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    //Serial.println("  Executor initialized successfully");

    msg_sensor.data = 0;
    //Serial.println("[INFO] Setup complete");
}

void loop() {
    //Serial.println("[DEBUG] Loop iteration...");
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500)));
      // Aggiunto un piccolo ritardo per ridurre la frequenza di debug
}
