


// #include <Arduino.h>
// #include <micro_ros_arduino.h>
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
#define ADC_AVG_SAMPLES 100
// ===================== PID & RATIO CONFIG =====================
const float M_COEFF = -3.49048;
// const float B_COEFF = 1854.52;
const float B_COEFF = 1911.00;

 const double Steering_To_Wheel = 0.055556;
// Removed custom RAD_TO_DEG to avoid collision with Arduino.h
// Note: Arduino.h provides RAD_TO_DEG automatically.

double setpointAngle = 0;   // Target angle (degrees)
double currentAngle  = 0;   // Feedback angle (degrees)
double pidOutput     = 0;   

double Kp = 40.0, Ki = 0.2, Kd = 0.01; 
PID steeringPID(&currentAngle, &pidOutput, &setpointAngle, Kp, Ki, Kd, DIRECT);
bool steeringModeActive = false;
// Use '1' instead of AccelStepper::DRIVER to avoid header scope issues
AccelStepper stepper(1, STEP_PIN, DIR_PIN);
X9C103S pot1(X9C_INC_PIN, X9C_UD_PIN, X9C_CS_PIN);

// ===================== micro-ROS GLOBALS =====================
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg_sub;
std_msgs__msg__Float32 msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// ===================== HELPERS =====================

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


void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    setpointAngle = (msg->data * RAD_TO_DEG) / Steering_To_Wheel;

    steeringModeActive = true; // Make sure the motor is allowed to move!
}

void setup() {
    analogReadResolution(12);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);
    stepper.setMaxSpeed(4000);
    pot1.initializePot();
    pot1.setResistance(1);

    steeringPID.SetMode(AUTOMATIC);
    steeringPID.SetOutputLimits(-4000, 4000);
    steeringPID.SetSampleTime(20);

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

// void loop() {
//     // 1. Prioritize Stepper - Call this as often as possible
//     if (steeringModeActive) {
//         stepper.runSpeed();
//     }

//     // 2. Run PID at a fixed frequency (Independent of ROS speed)
//     static unsigned long last_pid_run = 0;
//     if (millis() - last_pid_run >= 20) { // 50Hz PID ....previously 20
//         last_pid_run = millis();
//         currentAngle = calculateAngle();
//         steeringPID.Compute();

//         if (abs(setpointAngle - currentAngle) < 0.5) {
//             stepper.setSpeed(0);
//         } else {
//             stepper.setSpeed(-pidOutput);
//         }
//     }

//     // 3. Handle ROS communication (The "Heavier" task)
//     // We use a timeout so it doesn't block the loop for too long
//     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); 

//     // 4. Publish Feedback (Lower frequency is fine)
//     static unsigned long last_pub = 0;
//     if (millis() - last_pub > 100) { // 10Hz feedback is enough for ROS
//         msg_pub.data = (float)((currentAngle * Steering_To_Wheel) / RAD_TO_DEG);
//         (void) rcl_publish(&publisher, &msg_pub, NULL);
//         last_pub = millis();
//     }
// }
void loop() {

    // === 1. ALWAYS RUN STEPPER FIRST ===
    stepper.runSpeed();

    // === 2. Run PID ===
    static unsigned long last_pid_run = 0;
    if (millis() - last_pid_run >= 20) {
        last_pid_run = millis();

        currentAngle = calculateAngle();
        steeringPID.Compute();

        if (abs(setpointAngle - currentAngle) < 0.5) {
            stepper.setSpeed(0);
        } else {
            stepper.setSpeed(-pidOutput);
        }
    }

    // === 3. ROS only occasionally ===
    static unsigned long last_ros = 0;
    if (millis() - last_ros >= 5) {   // only every 5 ms
        rclc_executor_spin_some(&executor, 0);  // zero timeout
        last_ros = millis();
    }

    // === 4. Feedback publish ===
    static unsigned long last_pub = 0;
    if (millis() - last_pub > 100) {
        msg_pub.data = (float)((currentAngle * Steering_To_Wheel) / RAD_TO_DEG);
        rcl_publish(&publisher, &msg_pub, NULL);
        last_pub = millis();
    }
}









// #include <Arduino.h>
// #include <AccelStepper.h>

// #include <X9C103S.h>
// #include <PID_v1.h>

// // ===================== PIN CONFIGURATION =====================
// // Stepper Pins (TB6600)
// #define STEP_PIN  8   
// #define DIR_PIN   9   
// #define EN_PIN    3   

// // X9C Digital Pot Pins
// #define X9C_INC_PIN  6
// #define X9C_UD_PIN   7
// #define X9C_CS_PIN   16

// // ADC Feedback Pin
// #define POT_PIN 17   

// // ===================== PID & REGRESSION CONFIG =====================
// // Linear Regression: ADC = -3.49048 * Angle + 1854.52
// const float M_COEFF = -3.49048;
// const float B_COEFF = 1854.52;

// double setpointAngle = 0;   // Target angle (degrees)
// double currentAngle  = 0;   // Feedback angle from ADC
// double pidOutput     = 0;   // Calculated speed for stepper

// // PID Tuning Parameters
// // Kp: Proportional (Strength of response)
// // Ki: Integral (Corrects steady-state error)
// // Kd: Derivative (Dampens oscillation/overshoot)
// // double Kp = 40.0, Ki = 0.2, Kd = 1.5; 
// double Kp = 40.0, Ki = 0.2, Kd = 1.5; 


// PID steeringPID(&currentAngle, &pidOutput, &setpointAngle, Kp, Ki, Kd, DIRECT);

// // =// ===================== MOTOR & POT GLOBALS =====================
// X9C103S pot1(X9C_INC_PIN, X9C_UD_PIN, X9C_CS_PIN);
// const int POT_STOP_VALUE = 1;
// int potValuePercent = POT_STOP_VALUE;

// const int stepsPerRevolution = 200 * 16; // 3200 steps
// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// bool steeringModeActive = false; // Toggle for PID control
// unsigned long lastLogMs = 0;



// // ===================== HELPERS =====================

// // Convert ADC value back to Angle: Angle = (ADC - b) / m
// double calculateAngle() {
//   int adc = analogRead(POT_PIN);
//   return (double(adc) - B_COEFF) / M_COEFF;
// }



// void emergencyStopAll(const char* reason) {
//   Serial.print("[STOP] "); Serial.println(reason);
//   steeringModeActive = false;
//   stepper.setSpeed(0);
//   stepper.stop();
//   pot1.setResistance(POT_STOP_VALUE);
// }

// // ===================== COMMAND PARSER =====================

// void processCommand(String line) {
//   line.trim();
//   if (!line.length()) return;
//   char cmd = toupper(line.charAt(0));

//   // A <angle> -> Set Target Angle for PID
//   if (cmd == 'A') {
//     line.remove(0, 1);
//     setpointAngle = line.toFloat();
//     steeringModeActive = true;
//     Serial.printf("[PID] New Target: %.2f\n", setpointAngle);
//   }
//   // P <value> -> Set Digital Pot
//   else if (cmd == 'P') {
//     line.remove(0, 1);
//     int v = line.toInt();
//     pot1.setResistance(constrain(v, 1, 100));
//     Serial.printf("[X9C] Set: %d\n", v);
//   }
//   // S -> Stop Everything
//   else if (cmd == 'S') {
//     emergencyStopAll("User Command");
//   }
// }

// // ===================== BLE CALLBACKS =====================



// // ===================== SETUP & LOOP =====================

// void setup() {
//   Serial.begin(115200);
//   analogReadResolution(12);

//   // Stepper Setup
//   pinMode(EN_PIN, OUTPUT);
//   digitalWrite(EN_PIN, LOW);
//   stepper.setMaxSpeed(3000); // Max steps per second for steering
  
//   // PID Setup
//   steeringPID.SetMode(AUTOMATIC);
//   steeringPID.SetOutputLimits(-2500, 2500); // Limits speed (Steps/Sec)
//   steeringPID.SetSampleTime(20);            // 50Hz calculation

//   // X9C Setup
//   pot1.initializePot();
//   pot1.setResistance(POT_STOP_VALUE);



//   Serial.println("System Ready. Send 'A <angle>' to steer.");
// }

// void loop() {
//   // 1. Get Current Feedback
//   currentAngle = calculateAngle();

//   // 2. Compute PID Logic
//   if (steeringModeActive) {
//     steeringPID.Compute();

//     // Deadband check: If within 0.5 degrees, stop jittering
//     if (abs(setpointAngle - currentAngle) < 0.5) {
//       stepper.setSpeed(0);
//     } else {
//       stepper.setSpeed(-pidOutput);
//     }
    
//     // Constant execution of the calculated speed
//     stepper.runSpeed();
//   }

//   // 3. Serial Debugging (Every 500ms)
//   if (millis() - lastLogMs > 500) {
//     lastLogMs = millis();
//     Serial.printf("Target: %.1f | Current: %.1f | Output Speed: %.0f\n", 
//                   setpointAngle, currentAngle, pidOutput);
    
//     if (Serial.available()) {
//       processCommand(Serial.readStringUntil('\n'));
//     }
//   }
// }