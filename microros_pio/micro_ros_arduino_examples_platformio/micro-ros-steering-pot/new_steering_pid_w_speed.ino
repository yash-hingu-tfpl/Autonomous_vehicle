// #include <AccelStepper.h>

// #include <X9C103S.h>

// #include <PID_v1.h>



// #include <rcl/rcl.h>

// #include <rclc/rclc.h>

// #include <rclc/executor.h>

// #include <rclc_parameter/rclc_parameter.h>

// #include <std_msgs/msg/float32.h>

// #include <std_msgs/msg/bool.h>



// // ===================== PIN CONFIGURATION =====================

// #define STEP_PIN 8

// #define DIR_PIN 9

// #define EN_PIN 3

// #define X9C_INC_PIN 6

// #define X9C_UD_PIN 7

// #define X9C_CS_PIN 16

// #define POT_PIN 14

// #define RELAY_PIN 11 // Emergency Stop Relay



// #define ADC_AVG_SAMPLES 30





// bool emergency_stop = true; // Start in SAFE state

// // ===================== CALIBRATION =====================

// float M_COEFF = -3.49048;

// float B_COEFF = 1911.00;

// double Steering_To_Wheel = 0.055556;



// // ===================== NON-BLOCKING ADC BUFFER =====================

// int adcBuffer[ADC_AVG_SAMPLES];

// int bufferIndex = 0;

// long runningSum = 0;



// // ===================== PID VARIABLES =====================

// double setpointAngle = 0;

// double currentAngle = 0;

// double pidOutput = 0;



// double Kp = 55.0;

// double Ki = 0.0;

// double Kd = 1.5;



// float Max_Speed = 4000;

// float Acceleration = 5000;





// PID steeringPID(&currentAngle, &pidOutput, &setpointAngle, Kp, Ki, Kd, DIRECT);



// AccelStepper stepper(1, STEP_PIN, DIR_PIN);

// X9C103S pot1(X9C_INC_PIN, X9C_UD_PIN, X9C_CS_PIN);



// // ===================== micro-ROS =====================

// rcl_subscription_t subscriber_steering;

// rcl_subscription_t subscriber_speed;

// rcl_publisher_t publisher;

// std_msgs__msg__Float32 msg_steering_sub;

// std_msgs__msg__Float32 msg_speed_sub;

// std_msgs__msg__Float32 msg_pub;

// rcl_subscription_t estop_sub;

// std_msgs__msg__Bool estop_msg;



// rclc_executor_t executor;

// rclc_support_t support;

// rcl_node_t node;

// rcl_allocator_t allocator;

// rclc_parameter_server_t param_server;



// // ===================== ADC HELPERS =====================

// void updateAdcBuffer() {

// // Subtract the oldest value and add the newest

// runningSum -= adcBuffer[bufferIndex];

// adcBuffer[bufferIndex] = analogRead(POT_PIN);

// runningSum += adcBuffer[bufferIndex];


// // Increment index and wrap around

// bufferIndex = (bufferIndex + 1) % ADC_AVG_SAMPLES;

// }



// double calculateAngleNonBlocking() {

// double adcAvg = (double)runningSum / ADC_AVG_SAMPLES;

// return (adcAvg - B_COEFF) / M_COEFF;

// }



// // ===================== ROS PARAMS =====================



// bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context) {

// (void) context;

// if (new_param == NULL) return false;



// if (strcmp(new_param->name.data, "kp") == 0) {

// Kp = new_param->value.double_value;

// steeringPID.SetTunings(Kp, Ki, Kd);

// }

// else if (strcmp(new_param->name.data, "ki") == 0) {

// Ki = new_param->value.double_value;

// steeringPID.SetTunings(Kp, Ki, Kd);

// }

// else if (strcmp(new_param->name.data, "kd") == 0) {

// Kd = new_param->value.double_value;

// steeringPID.SetTunings(Kp, Ki, Kd);

// }

// else if (strcmp(new_param->name.data, "m_coeff") == 0) {

// M_COEFF = (float)new_param->value.double_value;

// }

// else if (strcmp(new_param->name.data, "b_coeff") == 0) {

// B_COEFF = (float)new_param->value.double_value;

// }

// else if (strcmp(new_param->name.data, "steering_to_wheel") == 0) {

// Steering_To_Wheel = new_param->value.double_value;

// }

// else if (strcmp(new_param->name.data, "max_speed") == 0) {

// Max_Speed = (float)new_param->value.double_value;

// stepper.setMaxSpeed(Max_Speed);

// steeringPID.SetOutputLimits(-Max_Speed, Max_Speed);

// }

// else if (strcmp(new_param->name.data, "acceleration") == 0) {

// Acceleration = (float)new_param->value.double_value;

// }



// return true;

// }

// // ===================== ROS CALLBACKS =====================



// // Steering Angle Callback

// void steering_callback(const void * msgin) {

// const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

// setpointAngle = (msg->data * RAD_TO_DEG) / Steering_To_Wheel;



// // Reset PID to prevent jump

// steeringPID.SetMode(MANUAL);

// steeringPID.SetMode(AUTOMATIC);

// }



// // Speed Callback (Controls X9C Potentiometer)

// void speed_callback(const void * msgin) {

// const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;


// // Map speed value (assumed 0-100) to resistance (0-99 steps)

// int resistance = (int)msg->data;

// resistance = constrain(resistance, 0, 99);


// pot1.setResistance(resistance);

// }



// void emergency_stop_callback(const void * msgin)

// {

// const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;



// emergency_stop = msg->data;



// if (emergency_stop)

// {

// digitalWrite(RELAY_PIN, HIGH); // CUT power (change if relay logic inverted)

// digitalWrite(EN_PIN, HIGH); // Disable stepper driver

// }

// else

// {

// digitalWrite(RELAY_PIN, LOW); // Enable power

// digitalWrite(EN_PIN, LOW); // Enable driver

// }

// }


// // ===================== SETUP =====================
// void setup() {
//     // 1. Hardware Initialization
//     analogReadResolution(12);
//     pinMode(EN_PIN, OUTPUT);
//     digitalWrite(EN_PIN, LOW);
//     pinMode(RELAY_PIN, OUTPUT);
//     digitalWrite(RELAY_PIN, HIGH); 

//     int initialRead = analogRead(POT_PIN);
//     for(int i = 0; i < ADC_AVG_SAMPLES; i++) { adcBuffer[i] = initialRead; }
//     runningSum = (long)initialRead * ADC_AVG_SAMPLES;

//     // 2. Stepper & PID Initial Config
//     stepper.setMaxSpeed(Max_Speed);
//     stepper.setAcceleration(Acceleration);
//     stepper.setMinPulseWidth(2);
//     pot1.initializePot();
//     pot1.setResistance(0);
//     steeringPID.SetMode(AUTOMATIC);
//     steeringPID.SetOutputLimits(-Max_Speed, Max_Speed);
//     steeringPID.SetSampleTime(10);

//     // 3. micro-ROS Core Setup
//     set_microros_transports();
//     allocator = rcl_get_default_allocator();
    
//     // Initialize support and node
//     rclc_support_init(&support, 0, NULL, &allocator);
//     rclc_node_init_default(&node, "steering_node", "", &support);

//     // 4. Parameter Server Setup (REPLACES DEFAULT INIT)
//     // Your header uses rclc_parameter_options_t
//     rclc_parameter_options_t param_options;
//     param_options.max_params = 12;                // Enough for 8 params + buffer
//     param_options.notify_changed_over_dds = true; // Required by your header
//     param_options.allow_undeclared_parameters = false;
//     param_options.low_mem_mode = false;

//     rclc_parameter_server_init_with_option(&param_server, &node, &param_options);

//     // Add and Set parameters with explicit double casting
//     rclc_add_parameter(&param_server, "kp", RCLC_PARAMETER_DOUBLE);
//     rclc_parameter_set_double(&param_server, "kp", (double)Kp);

//     rclc_add_parameter(&param_server, "ki", RCLC_PARAMETER_DOUBLE);
//     rclc_parameter_set_double(&param_server, "ki", (double)Ki);

//     rclc_add_parameter(&param_server, "kd", RCLC_PARAMETER_DOUBLE);
//     rclc_parameter_set_double(&param_server, "kd", (double)Kd);

//     rclc_add_parameter(&param_server, "m_coeff", RCLC_PARAMETER_DOUBLE);
//     rclc_parameter_set_double(&param_server, "m_coeff", (double)M_COEFF);

//     rclc_add_parameter(&param_server, "b_coeff", RCLC_PARAMETER_DOUBLE);
//     rclc_parameter_set_double(&param_server, "b_coeff", (double)B_COEFF);

//     rclc_add_parameter(&param_server, "steering_to_wheel", RCLC_PARAMETER_DOUBLE);
//     rclc_parameter_set_double(&param_server, "steering_to_wheel", (double)Steering_To_Wheel);

//     rclc_add_parameter(&param_server, "max_speed", RCLC_PARAMETER_DOUBLE);
//     rclc_parameter_set_double(&param_server, "max_speed", (double)Max_Speed);

//     rclc_add_parameter(&param_server, "acceleration", RCLC_PARAMETER_DOUBLE);
//     rclc_parameter_set_double(&param_server, "acceleration", (double)Acceleration);

//     // 5. Initialize Subscriptions & Publisher
//     rclc_subscription_init_default(&subscriber_steering, &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/cmd_steering_angle");
//     rclc_subscription_init_default(&subscriber_speed, &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/cmd_speed");
//     rclc_subscription_init_default(&estop_sub, &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/emergency_stop");
//     rclc_publisher_init_default(&publisher, &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/feedback_steering_angle");

//     // 6. Executor Setup
//     // 3 Subscriptions + 5 Handles for the Parameter Server = 8 total
//     unsigned int num_handles = 3 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
    
//     rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    
//     // Add subs to executor
//     rclc_executor_add_subscription(&executor, &subscriber_steering, &msg_steering_sub, &steering_callback, ON_NEW_DATA);
//     rclc_executor_add_subscription(&executor, &subscriber_speed, &msg_speed_sub, &speed_callback, ON_NEW_DATA);
//     rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg, &emergency_stop_callback, ON_NEW_DATA);
    
//     // Add parameter server to executor
//     rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
// }



// // ===================== LOOP =====================

// void loop() {

// // 1️⃣ Priority 1: Update Motor (Must be called as often as possible)

// stepper.runSpeed();



// // 2️⃣ Update ADC Buffer (Very fast rolling average)

// updateAdcBuffer();



// // 3️⃣ PID Logic (100Hz)

// static unsigned long last_pid = 0;

// if (millis() - last_pid >= 10) {

// last_pid = millis();



// currentAngle = calculateAngleNonBlocking();

// steeringPID.Compute();



// // Deadzone check

// if (abs(setpointAngle - currentAngle) < 0.4) {

// stepper.setSpeed(0);

// } else {

// stepper.setSpeed(-pidOutput);

// }

// }



// // 4️⃣ ROS Executor Spin (Non-blocking)

// static unsigned long last_ros = 0;

// if (millis() - last_ros >= 2) {

// rclc_executor_spin_some(&executor, 0);

// last_ros = millis();

// }



// // 5️⃣ Feedback Publish (10Hz)

// static unsigned long last_pub = 0;

// if (millis() - last_pub > 100) {

// msg_pub.data = (float)((currentAngle * Steering_To_Wheel) / RAD_TO_DEG);

// rcl_publish(&publisher, &msg_pub, NULL);

// last_pub = millis();

// }

// }
#include <AccelStepper.h>
#include <X9C103S.h>
#include <PID_v1.h>

#include <EEPROM.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>

#define EEPROM_SIZE 512
#define PARAM_MAGIC 0xDEADBEEF

// ===================== PARAM ORDER =====================
// 0 | Kp
// 1 | Ki
// 2 | Kd
// 3 | M_COEFF
// 4 | B_COEFF
// 5 | Steering_To_Wheel
// 6 | Max_Speed
// 7 | Acceleration

// ================= PIN CONFIG =================

#define STEP_PIN 8
#define DIR_PIN 9
#define EN_PIN 3

#define X9C_INC_PIN 6
#define X9C_UD_PIN 7
#define X9C_CS_PIN 16

#define POT_PIN 14
#define RELAY_PIN 11

#define ADC_AVG_SAMPLES 30

bool emergency_stop = true;

// ================= PARAMETERS =================

double Kp = 55.0;
double Ki = 0.0;
double Kd = 1.5;

float M_COEFF = -3.49048;
float B_COEFF = 1911.0;

double Steering_To_Wheel = 0.055556;

float Max_Speed = 4000;
float Acceleration = 5000;

float param_buffer[8];

// ================= PARAM STORAGE STRUCT =================

struct SteeringParams
{
  double Kp;
  double Ki;
  double Kd;

  float M_COEFF;
  float B_COEFF;

  double Steering_To_Wheel;

  float Max_Speed;
  float Acceleration;

  uint32_t magic;
};

SteeringParams params;

// ================= ADC FILTER =================

int adcBuffer[ADC_AVG_SAMPLES];
int bufferIndex = 0;
long runningSum = 0;

// ================= PID =================

double setpointAngle = 0;
double currentAngle = 0;
double pidOutput = 0;

PID steeringPID(&currentAngle, &pidOutput, &setpointAngle, Kp, Ki, Kd, DIRECT);

// ================= STEPPER =================

AccelStepper stepper(1, STEP_PIN, DIR_PIN);
X9C103S pot1(X9C_INC_PIN, X9C_UD_PIN, X9C_CS_PIN);

// ================= microROS =================

rcl_subscription_t subscriber_steering;
rcl_subscription_t subscriber_speed;
rcl_subscription_t estop_sub;
rcl_subscription_t param_sub;

rcl_publisher_t publisher;
rcl_publisher_t param_feedback_pub;

std_msgs__msg__Float32 msg_steering_sub;
std_msgs__msg__Float32 msg_speed_sub;
std_msgs__msg__Float32 msg_pub;
std_msgs__msg__Bool estop_msg;

std_msgs__msg__Float32MultiArray param_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// ================= EEPROM FUNCTIONS =================

void save_params()
{
  params.Kp = Kp;
  params.Ki = Ki;
  params.Kd = Kd;

  params.M_COEFF = M_COEFF;
  params.B_COEFF = B_COEFF;

  params.Steering_To_Wheel = Steering_To_Wheel;

  params.Max_Speed = Max_Speed;
  params.Acceleration = Acceleration;

  params.magic = PARAM_MAGIC;

  EEPROM.put(0, params);
  EEPROM.commit();
}

void load_params()
{
  EEPROM.get(0, params);

  if (params.magic != PARAM_MAGIC)
  {
    Kp = 55.0;
    Ki = 0.0;
    Kd = 1.5;

    M_COEFF = -3.49048;
    B_COEFF = 1911.0;

    Steering_To_Wheel = 0.055556;

    Max_Speed = 4000;
    Acceleration = 5000;

    save_params();
    return;
  }

  Kp = params.Kp;
  Ki = params.Ki;
  Kd = params.Kd;

  M_COEFF = params.M_COEFF;
  B_COEFF = params.B_COEFF;

  Steering_To_Wheel = params.Steering_To_Wheel;

  Max_Speed = params.Max_Speed;
  Acceleration = params.Acceleration;
}

// ================= PARAM FEEDBACK =================

void publish_param_feedback()
{
  param_buffer[0] = Kp;
  param_buffer[1] = Ki;
  param_buffer[2] = Kd;
  param_buffer[3] = M_COEFF;
  param_buffer[4] = B_COEFF;
  param_buffer[5] = Steering_To_Wheel;
  param_buffer[6] = Max_Speed;
  param_buffer[7] = Acceleration;

  param_msg.data.data = param_buffer;
  param_msg.data.size = 8;
  param_msg.data.capacity = 8;

  rcl_publish(&param_feedback_pub, &param_msg, NULL);
}

// ================= ADC =================

void updateAdcBuffer()
{
  runningSum -= adcBuffer[bufferIndex];

  adcBuffer[bufferIndex] = analogRead(POT_PIN);

  runningSum += adcBuffer[bufferIndex];

  bufferIndex = (bufferIndex + 1) % ADC_AVG_SAMPLES;
}

double calculateAngle()
{
  double adcAvg = (double)runningSum / ADC_AVG_SAMPLES;
  return (adcAvg - B_COEFF) / M_COEFF;
}

// ================= PARAM CALLBACK =================

void params_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg->data.size < 8) return;

  bool changed = false;

  if (Kp != msg->data.data[0]) {Kp = msg->data.data[0]; changed = true;}
  if (Ki != msg->data.data[1]) {Ki = msg->data.data[1]; changed = true;}
  if (Kd != msg->data.data[2]) {Kd = msg->data.data[2]; changed = true;}
  if (M_COEFF != msg->data.data[3]) {M_COEFF = msg->data.data[3]; changed = true;}
  if (B_COEFF != msg->data.data[4]) {B_COEFF = msg->data.data[4]; changed = true;}
  if (Steering_To_Wheel != msg->data.data[5]) {Steering_To_Wheel = msg->data.data[5]; changed = true;}
  if (Max_Speed != msg->data.data[6]) {Max_Speed = msg->data.data[6]; changed = true;}
  if (Acceleration != msg->data.data[7]) {Acceleration = msg->data.data[7]; changed = true;}

  steeringPID.SetTunings(Kp, Ki, Kd);

  stepper.setMaxSpeed(Max_Speed);
  stepper.setAcceleration(Acceleration);

  if (changed) save_params();

  publish_param_feedback();
}

// ================= ROS CALLBACKS =================

void steering_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msg =
      (const std_msgs__msg__Float32 *)msgin;

  setpointAngle = (msg->data * RAD_TO_DEG) / Steering_To_Wheel;

  steeringPID.SetMode(MANUAL);
  steeringPID.SetMode(AUTOMATIC);
}

void speed_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msg =
      (const std_msgs__msg__Float32 *)msgin;

  int resistance = constrain((int)msg->data, 0, 99);

  pot1.setResistance(resistance);
}

void emergency_stop_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg =
      (const std_msgs__msg__Bool *)msgin;

  emergency_stop = msg->data;

  if (emergency_stop)
  {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(EN_PIN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(EN_PIN, LOW);
  }
}

// ================= SETUP =================

void setup()
{
  Serial.begin(115200);

  analogReadResolution(12);

  EEPROM.begin(EEPROM_SIZE);

  load_params();

  pinMode(EN_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(RELAY_PIN, HIGH);

  int initialRead = analogRead(POT_PIN);

  for (int i = 0; i < ADC_AVG_SAMPLES; i++)
    adcBuffer[i] = initialRead;

  runningSum = initialRead * ADC_AVG_SAMPLES;

  stepper.setMaxSpeed(Max_Speed);
  stepper.setAcceleration(Acceleration);
  stepper.setMinPulseWidth(2);

  pot1.initializePot();
  pot1.setResistance(0);

  steeringPID.SetMode(AUTOMATIC);
  steeringPID.SetOutputLimits(-Max_Speed, Max_Speed);
  steeringPID.SetSampleTime(10);

  set_microros_transports();

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "steering_node", "", &support);

  rclc_subscription_init_default(
      &subscriber_steering,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/cmd_steering_angle");

  rclc_subscription_init_default(
      &subscriber_speed,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/cmd_speed");

  rclc_subscription_init_default(
      &estop_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/emergency_stop");

  rclc_subscription_init_default(
      &param_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/steering_params");

  rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/feedback_steering_angle");

  rclc_publisher_init_default(
      &param_feedback_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/steering_param_feedback");

  rclc_executor_init(&executor, &support.context, 4, &allocator);

  rclc_executor_add_subscription(&executor, &subscriber_steering, &msg_steering_sub, &steering_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_speed, &msg_speed_sub, &speed_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg, &emergency_stop_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &param_sub, &param_msg, &params_callback, ON_NEW_DATA);
}

// ================= LOOP =================

void loop()
{
  stepper.runSpeed();

  updateAdcBuffer();

  static unsigned long last_pid = 0;

  if (millis() - last_pid >= 10)
  {
    last_pid = millis();

    currentAngle = calculateAngle();

    steeringPID.Compute();

    if (abs(setpointAngle - currentAngle) < 0.4)
      stepper.setSpeed(0);
    else
      stepper.setSpeed(-pidOutput);
  }

  rclc_executor_spin_some(&executor, 0);

  static unsigned long last_pub = 0;

  if (millis() - last_pub > 100)
  {
    msg_pub.data = (float)((currentAngle * Steering_To_Wheel) / RAD_TO_DEG);

    rcl_publish(&publisher, &msg_pub, NULL);

    publish_param_feedback();

    last_pub = millis();
  }
}