/******************************************************************
 *  ESP32-S3 micro-ROS Golf Cart Controller
 *  ROS 2 Humble Compatible
 ******************************************************************/

//#include <micro_ros_arduino.h>
#include <Wire.h>

/* micro-ROS */
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/* ROS messages */
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

/* ===================== PIN CONFIG ===================== */
// Rear wheel encoders
#define ENC_L_A  4
#define ENC_L_B  5
#define ENC_R_A  6
#define ENC_R_B  7

// AS5600 (I2C)
#define SDA_PIN  8
#define SCL_PIN  9
#define AS5600_ADDR 0x36

// Steering multi-turn potentiometer (ADC)
#define STEER_POT_PIN 1

// X9C103S digital potentiometer
#define DIGIPOT_CS   10
#define DIGIPOT_UD   11
#define DIGIPOT_INC  12

// Status LED
#define LED_PIN 2

/* ===================== GLOBALS ===================== */
// Encoder ticks
volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;

// Steering
int32_t steering_turns = 0;
float last_as5600_angle = 0.0;

// Digipot
int current_step = 0;

// Safety
unsigned long last_cmd_time = 0;
#define CMD_TIMEOUT_MS 500

/* ===================== micro-ROS ===================== */
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t pub_left_ticks;
rcl_publisher_t pub_right_ticks;
rcl_publisher_t pub_steering;

rcl_subscription_t sub_throttle;
rclc_executor_t executor;
rcl_timer_t timer;

/* Messages */
std_msgs__msg__Int32 msg_left_ticks;
std_msgs__msg__Int32 msg_right_ticks;
std_msgs__msg__Float32 msg_steering;

/* ===================== ENCODER ISR ===================== */
void IRAM_ATTR leftEncoderISR()
{
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B))
    left_ticks++;
  else
    left_ticks--;
}

void IRAM_ATTR rightEncoderISR()
{
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B))
    right_ticks++;
  else
    right_ticks--;
}

/* ===================== AS5600 ===================== */
float readAS5600()
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);

  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return (raw * 360.0) / 4096.0;
}

float readSteeringAngleRad()
{
  float angle = readAS5600();

  if (angle - last_as5600_angle > 180.0) steering_turns--;
  if (angle - last_as5600_angle < -180.0) steering_turns++;

  last_as5600_angle = angle;

  float total_deg = steering_turns * 360.0 + angle;
  return total_deg * DEG_TO_RAD;
}

/* ===================== X9C103S ===================== */
void digipotPulse(bool up)
{
  digitalWrite(DIGIPOT_UD, up);
  digitalWrite(DIGIPOT_CS, LOW);

  digitalWrite(DIGIPOT_INC, LOW);
  delayMicroseconds(5);
  digitalWrite(DIGIPOT_INC, HIGH);

  digitalWrite(DIGIPOT_CS, HIGH);
}

void setThrottle(float throttle)
{
  throttle = constrain(throttle, 0.0, 1.0);
  int target_step = throttle * 49;   // ONLY 50 steps

  while (current_step < target_step) {
    digipotPulse(true);
    current_step++;
  }
  while (current_step > target_step) {
    digipotPulse(false);
    current_step--;
  }
}

/* ===================== ROS CALLBACK ===================== */
void throttle_callback(const void * msg)
{
  const std_msgs__msg__Float32 * throttle =
      (const std_msgs__msg__Float32 *)msg;

  setThrottle(throttle->data);
  last_cmd_time = millis();
}

/* ===================== TIMER ===================== */
void timer_callback(rcl_timer_t *, int64_t)
{
  msg_left_ticks.data = left_ticks;
  msg_right_ticks.data = right_ticks;
  msg_steering.data = readSteeringAngleRad();

  rcl_publish(&pub_left_ticks, &msg_left_ticks, NULL);
  rcl_publish(&pub_right_ticks, &msg_right_ticks, NULL);
  rcl_publish(&pub_steering, &msg_steering, NULL);

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

/* ===================== SETUP ===================== */
void setup()
{
  pinMode(LED_PIN, OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, CHANGE);

  pinMode(DIGIPOT_CS, OUTPUT);
  pinMode(DIGIPOT_UD, OUTPUT);
  pinMode(DIGIPOT_INC, OUTPUT);
  digitalWrite(DIGIPOT_CS, HIGH);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "golf_cart_esp32", "", &support);

  rclc_publisher_init_default(
    &pub_left_ticks,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "wheel_ticks_left");

  rclc_publisher_init_default(
    &pub_right_ticks,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "wheel_ticks_right");

  rclc_publisher_init_default(
    &pub_steering,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "steering_angle_rad");

  rclc_subscription_init_default(
    &sub_throttle,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_throttle");

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(50),
    timer_callback);

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_throttle,
                                 &msg_steering,
                                 &throttle_callback,
                                 ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  setThrottle(0.0);   // SAFETY
}

/* ===================== LOOP ===================== */
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Safety timeout
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    setThrottle(0.0);
  }
}
