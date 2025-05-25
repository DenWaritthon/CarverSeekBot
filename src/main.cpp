#include <Arduino.h>
#include <M5StickC.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>


// Variable Micro ROS configuration
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rcl_node_t node;

rcl_timer_t timer;
rcl_publisher_t imu_publisher;

// Variable massage
sensor_msgs__msg__Imu imu_msg;

// Variable IMU
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

#define DEG2RAD M_PI/180.0
#define G2M_S2 9.81

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelData(&accX, &accY, &accZ);

    imu_msg.angular_velocity.x = gyroX * DEG2RAD;
    imu_msg.angular_velocity.y = gyroY * DEG2RAD;
    imu_msg.angular_velocity.z = gyroZ * DEG2RAD;

    imu_msg.linear_acceleration.x = accX * G2M_S2;
    imu_msg.linear_acceleration.y = accY * G2M_S2;
    imu_msg.linear_acceleration.z = accZ * G2M_S2;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);

  // Start M5StickC and Init IMU
  M5.begin();
  M5.IMU.Init();

  // Use Micro ROS Using Serial
  set_microros_serial_transports(Serial); 
  delay(2000);

  // micro-ROS app
  allocator = rcl_get_default_allocator(); 
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //create init_options

  //create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
  RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 101)); // Set domain ID

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  RCCHECK(rclc_node_init_default(&node, "M5StickC_IMU_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "M5C_IMU_publisher"));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}