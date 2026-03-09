// #include <Arduino.h>
// #include <AccelStepper.h>
// #include <PID_v1.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/float32.h>

// // ===================== PIN CONFIG =====================
// #define STEP_PIN  8
// #define DIR_PIN   9
// #define EN_PIN    3
// #define POT_PIN   14

// // ===================== STEERING CALIBRATION =====================
// const float M_COEFF = -3.49048;
// const float B_COEFF = 1911.0;

// // Steering ratio (wheel angle → steering shaft)
// const double Steering_To_Wheel = 0.055556;

// // ===================== CONTROL VARIABLES =====================
// volatile double setpointAngle = 0.0;   // degrees (steering shaft)
// volatile double currentAngle  = 0.0;   // degrees
// volatile double pidOutput     = 0.0;   // stepper speed (steps/sec)

// double Kp = 50.0;
// double Ki = 0;
// double Kd = 0;

// PID steeringPID((double*)&currentAngle,
//                 (double*)&pidOutput,
//                 (double*)&setpointAngle,
//                 Kp, Ki, Kd, DIRECT);

// const int stepsPerRevolution = 200 * 16; // 3200 steps
// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// // ===================== micro-ROS GLOBALS =====================
// rcl_node_t node;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_subscription_t subscriber;
// rcl_publisher_t publisher;
// rclc_executor_t executor;

// std_msgs__msg__Float32 msg_sub;
// std_msgs__msg__Float32 msg_pub;

// // ===================== TIMER =====================
// hw_timer_t *controlTimer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// volatile uint32_t pidCounter = 0;

// // =============================================================
// //                      HELPER FUNCTIONS
// // =============================================================

// double calculateAngle()
// {
//     int adc = analogRead(POT_PIN);
//     return (double(adc) - B_COEFF) / M_COEFF;
// }

// // =============================================================
// //                   ROS SUBSCRIPTION CALLBACK
// // =============================================================

// void subscription_callback(const void * msgin)
// {
//     const std_msgs__msg__Float32 * msg =
//         (const std_msgs__msg__Float32 *)msgin;

//     // Convert wheel radians → steering shaft degrees
//     setpointAngle =
//         (msg->data * RAD_TO_DEG) / Steering_To_Wheel;
// }

// // =============================================================
// //                    REAL-TIME CONTROL ISR
// // =============================================================

// void IRAM_ATTR onControlTimer()
// {
//     portENTER_CRITICAL_ISR(&timerMux);

//     // Always generate step pulses
//     stepper.runSpeed();

//     pidCounter++;

//     // Run PID at 50Hz (every 20ms)
//     if (pidCounter >= 20)
//     {
//         pidCounter = 0;

//         currentAngle = calculateAngle();
//         steeringPID.Compute();

//         // Deadband
//         if (abs(setpointAngle - currentAngle) < 0.5)
//             stepper.setSpeed(0);
//         else
//             stepper.setSpeed(-pidOutput);
//     }

//     portEXIT_CRITICAL_ISR(&timerMux);
// }

// // =============================================================
// //                           SETUP
// // =============================================================

// void setup()
// {
//     analogReadResolution(12);

//     pinMode(EN_PIN, OUTPUT);
//     digitalWrite(EN_PIN, LOW);

//     stepper.setMaxSpeed(10000);
//     stepper.setSpeed(0);

//     steeringPID.SetMode(AUTOMATIC);
//     steeringPID.SetOutputLimits(-8000, 8000);
//     steeringPID.SetSampleTime(20);

//     // ----------- TIMER SETUP (1kHz) ------------
//     controlTimer = timerBegin(0, 80, true);
//     // 80 prescaler → 1MHz timer (1 tick = 1µs)

//     timerAttachInterrupt(controlTimer, &onControlTimer, true);
//     timerAlarmWrite(controlTimer, 1000, true);
//     // 1000 ticks = 1ms → 1kHz

//     timerAlarmEnable(controlTimer);

//     // ----------- micro-ROS SETUP ------------
//     set_microros_transports();

//     allocator = rcl_get_default_allocator();
//     rclc_support_init(&support, 0, NULL, &allocator);

//     rclc_node_init_default(
//         &node,
//         "steering_node",
//         "",
//         &support);

//     rclc_subscription_init_default(
//         &subscriber,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//         "/cmd_steering_angle");

//     rclc_publisher_init_default(
//         &publisher,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//         "/feedback_steering_angle");

//     rclc_executor_init(&executor, &support.context, 1, &allocator);

//     rclc_executor_add_subscription(
//         &executor,
//         &subscriber,
//         &msg_sub,
//         &subscription_callback,
//         ON_NEW_DATA);
// }

// // =============================================================
// //                            LOOP
// // =============================================================

// void loop()
// {
//     // 1️⃣ Handle ROS (non-blocking)
//     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

//     // 2️⃣ Publish feedback at 10Hz
//     static unsigned long last_pub = 0;
//     if (millis() - last_pub >= 100)
//     {
//         last_pub = millis();

//         msg_pub.data =
//             (float)((currentAngle * Steering_To_Wheel) / RAD_TO_DEG);

//         rcl_publish(&publisher, &msg_pub, NULL);
//     }
// }




// ----------------------------------------------------------------------------------------------------------------------------

// #include <Arduino.h>
// #include <AccelStepper.h>
// #include <PID_v1.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/float32.h>

// // ===================== PIN CONFIG =====================
// #define STEP_PIN  8
// #define DIR_PIN   9
// #define EN_PIN    3
// #define POT_PIN   14

// // ===================== STEERING CALIBRATION =====================
// const float M_COEFF = -3.49048;
// const float B_COEFF = 1911.0;

// // Steering ratio (wheel angle → steering shaft)
// const double Steering_To_Wheel = 0.055556;

// // ===================== CONTROL VARIABLES =====================
// volatile double setpointAngle = 0.0;   // shaft degrees
// volatile double currentAngle  = 0.0;   // shaft degrees
// volatile double pidOutput     = 0.0;   // stepper speed

// // 🔥 PD ONLY (No Integral → no drift)
// double Kp = 18.0;
// double Ki = 0.0;
// double Kd = 0.9;

// PID steeringPID((double*)&currentAngle,
//                 (double*)&pidOutput,
//                 (double*)&setpointAngle,
//                 Kp, Ki, Kd, DIRECT);

// // ===================== STEPPER =====================
// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// // ===================== LIMITS =====================
// const double DEAD_BAND = 0.3;       // degrees
// const double MAX_SPEED = 1200;     // steps/sec
// const double OUTPUT_SMOOTH = 0.25;  // smoothing

// double filteredOutput = 0;

// // ===================== micro-ROS =====================
// rcl_node_t node;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_subscription_t subscriber;
// rcl_publisher_t publisher;
// rclc_executor_t executor;

// std_msgs__msg__Float32 msg_sub;
// std_msgs__msg__Float32 msg_pub;

// // ===================== TIMER =====================
// hw_timer_t *controlTimer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
// volatile uint32_t pidDivider = 0;

// // =============================================================
// //                      ANGLE CALCULATION
// // =============================================================

// double calculateAngle()
// {
//     int adc = analogRead(POT_PIN);
//     return (double(adc) - B_COEFF) / M_COEFF;
// }

// // =============================================================
// //                   ROS SUBSCRIPTION CALLBACK
// // =============================================================

// void subscription_callback(const void * msgin)
// {
//     const std_msgs__msg__Float32 * msg =
//         (const std_msgs__msg__Float32 *)msgin;

//     // wheel radians → steering shaft degrees
//     setpointAngle =
//         (msg->data * RAD_TO_DEG) / Steering_To_Wheel;
// }

// // =============================================================
// //                  REAL-TIME CONTROL (1kHz)
// // =============================================================

// void IRAM_ATTR onControlTimer()
// {
//     portENTER_CRITICAL_ISR(&timerMux);

//     // Always generate step pulses
//     stepper.runSpeed();

//     pidDivider++;

//     // Run PD at 100Hz
//     if (pidDivider >= 10)
//     {
//         pidDivider = 0;

//         currentAngle = calculateAngle();
//         steeringPID.Compute();

//         double error = setpointAngle - currentAngle;

//         if (abs(error) < DEAD_BAND)
//         {
//             stepper.setSpeed(0);
//             filteredOutput = 0;
//         }
//         else
//         {
//             // Clamp speed
//             if (pidOutput > MAX_SPEED) pidOutput = MAX_SPEED;
//             if (pidOutput < -MAX_SPEED) pidOutput = -MAX_SPEED;

//             // Smooth output
//             filteredOutput =
//                 (OUTPUT_SMOOTH * pidOutput) +
//                 ((1.0 - OUTPUT_SMOOTH) * filteredOutput);

//             stepper.setSpeed(-filteredOutput);
//         }
//     }

//     portEXIT_CRITICAL_ISR(&timerMux);
// }

// // =============================================================
// //                           SETUP
// // =============================================================

// void setup()
// {
//     analogReadResolution(12);

//     pinMode(EN_PIN, OUTPUT);
//     digitalWrite(EN_PIN, LOW);

//     stepper.setMaxSpeed(MAX_SPEED);
//     stepper.setSpeed(0);

//     // Initialize PD
//     steeringPID.SetMode(AUTOMATIC);
//     steeringPID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
//     steeringPID.SetSampleTime(10);   // 100Hz
//     steeringPID.SetTunings(Kp, Ki, Kd);

//     // ===== Auto-center at startup =====
//     delay(1500);
//     currentAngle = calculateAngle();
//     setpointAngle = currentAngle;

//     // ===== 1kHz hardware timer =====
//     controlTimer = timerBegin(0, 80, true);  // 1MHz
//     timerAttachInterrupt(controlTimer, &onControlTimer, true);
//     timerAlarmWrite(controlTimer, 1000, true); // 1ms
//     timerAlarmEnable(controlTimer);

//     // ===== micro-ROS setup =====
//     set_microros_transports();

//     allocator = rcl_get_default_allocator();
//     rclc_support_init(&support, 0, NULL, &allocator);

//     rclc_node_init_default(
//         &node,
//         "steering_node",
//         "",
//         &support);

//     rclc_subscription_init_default(
//         &subscriber,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//         "/cmd_steering_angle");

//     rclc_publisher_init_default(
//         &publisher,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//         "/feedback_steering_angle");

//     rclc_executor_init(&executor, &support.context, 1, &allocator);

//     rclc_executor_add_subscription(
//         &executor,
//         &subscriber,
//         &msg_sub,
//         &subscription_callback,
//         ON_NEW_DATA);
// }

// // =============================================================
// //                            LOOP
// // =============================================================

// void loop()
// {
//     // Handle ROS (non-blocking)
//     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

//     // Publish feedback at 20Hz
//     static unsigned long last_pub = 0;

//     if (millis() - last_pub >= 50)
//     {
//         last_pub = millis();

//         msg_pub.data =
//             (float)((currentAngle * Steering_To_Wheel) / RAD_TO_DEG);

//         rcl_publish(&publisher, &msg_pub, NULL);
//     }
// }




#include <AccelStepper.h>
#include <X9C103S.h>
#include <PID_v1.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// ===================== PIN CONFIGURATION =====================
#define STEP_PIN  8   
#define DIR_PIN   9   
#define EN_PIN    3   
#define X9C_INC_PIN  6
#define X9C_UD_PIN   7
#define X9C_CS_PIN   16
#define POT_PIN 14   
#define Steering_power_pin 11   


#define ADC_AVG_SAMPLES 30   // Reduced for faster response

// ===================== CALIBRATION =====================
const float M_COEFF = -3.49048;
const float B_COEFF = 1911.00;

const double Steering_To_Wheel = 0.055556;

// ===================== PID VARIABLES =====================
double setpointAngle = 0;
double currentAngle  = 0;
double pidOutput     = 0;

double Kp = 55.0;
double Ki = 0.0;      // Removed integral (important)
double Kd = 1.5;

PID steeringPID(&currentAngle, &pidOutput, &setpointAngle, Kp, Ki, Kd, DIRECT);

AccelStepper stepper(1, STEP_PIN, DIR_PIN);
X9C103S pot1(X9C_INC_PIN, X9C_UD_PIN, X9C_CS_PIN);

// ===================== micro-ROS =====================
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg_sub;
std_msgs__msg__Float32 msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// ===================== ADC =====================
int readAdcAveraged() {
  long sum = 0;
  for (int i = 0; i < ADC_AVG_SAMPLES; i++) {
    sum += analogRead(POT_PIN);
  }
  return sum / ADC_AVG_SAMPLES;
}

double calculateAngle() {
  int adcAvg = readAdcAveraged();
  return (double(adcAvg) - B_COEFF) / M_COEFF;
}

// ===================== ROS CALLBACK =====================
void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    setpointAngle = (msg->data * RAD_TO_DEG) / Steering_To_Wheel;

    // Anti-windup reset
    steeringPID.SetMode(MANUAL);
    steeringPID.SetMode(AUTOMATIC);
}

// ===================== SETUP =====================
void setup() {

    analogReadResolution(12);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);

    pinMode(Steering_power_pin, OUTPUT);
    digitalWrite(Steering_power_pin, LOW);



    // ----- STEPPER SPEED UPGRADE -----
    stepper.setMaxSpeed(4000);        // Increased 5x
    stepper.setAcceleration(1);    // Fast acceleration
    stepper.setMinPulseWidth(2);

    pot1.initializePot();
    pot1.setResistance(40);

    steeringPID.SetMode(AUTOMATIC);
    steeringPID.SetOutputLimits(-4000, 4000);
    steeringPID.SetSampleTime(10);   // 100Hz PID

    set_microros_transports();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "steering_node", "", &support);

    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/cmd_steering_angle");

    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/feedback_steering_angle");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
}

// ===================== LOOP =====================
void loop() {

    // 1️⃣ ALWAYS RUN STEPPER FIRST
    stepper.runSpeed();

    // 2️⃣ FAST PID LOOP (100Hz)
    static unsigned long last_pid = 0;
    if (millis() - last_pid >= 10) {
        last_pid = millis();

        currentAngle = calculateAngle();
        steeringPID.Compute();

        if (abs(setpointAngle - currentAngle) < 0.4) {
            stepper.setSpeed(0);
        } else {
            stepper.setSpeed(-pidOutput);
        }
    }

    // 3️⃣ ROS Spin (Non blocking)
    static unsigned long last_ros = 0;
    if (millis() - last_ros >= 2) {
        rclc_executor_spin_some(&executor, 0);
        last_ros = millis();
    }

    // 4️⃣ Feedback Publish 10Hz
    static unsigned long last_pub = 0;
    if (millis() - last_pub > 100) {
        msg_pub.data = (float)((currentAngle * Steering_To_Wheel) / RAD_TO_DEG);
        rcl_publish(&publisher, &msg_pub, NULL);
        last_pub = millis();
    }
}