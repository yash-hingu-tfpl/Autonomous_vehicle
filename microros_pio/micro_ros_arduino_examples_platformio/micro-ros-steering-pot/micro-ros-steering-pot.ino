// #include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

#define POT_PIN 1           // ESP32-S3 valid ADC pin
#define STEER_MAX 0.6f      // radians

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rcl_allocator_t allocator;

void timer_callback(rcl_timer_t *, int64_t)
{
  int raw = analogRead(POT_PIN);          // 0–4095
  float norm = (raw / 4095.0f) * 2.0f - 1.0f;
  // msg.data = norm * STEER_MAX;
  msg.data = norm * STEER_MAX;


  rcl_publish(&publisher, &msg, NULL);
}

void setup()
{
  pinMode(POT_PIN, INPUT);

  set_microros_transports();   // USB CDC

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_steering_node", "", &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/esp32/steering"
  );

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(20),   // 50 Hz
    timer_callback
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
