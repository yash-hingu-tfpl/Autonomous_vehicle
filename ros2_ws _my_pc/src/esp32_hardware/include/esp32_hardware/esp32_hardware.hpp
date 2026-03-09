// #pragma once

// #include <vector>
// #include <memory>

// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float32.hpp"

// namespace esp32_hardware
// {

// class ESP32Hardware : public hardware_interface::SystemInterface
// {
// public:
//   RCLCPP_SHARED_PTR_DEFINITIONS(ESP32Hardware)

//   // Lifecycle
//   hardware_interface::CallbackReturn on_init(
//     const hardware_interface::HardwareInfo & info) override;

//   hardware_interface::CallbackReturn on_activate(
//     const rclcpp_lifecycle::State & previous_state) override;

//   hardware_interface::CallbackReturn on_deactivate(
//     const rclcpp_lifecycle::State & previous_state) override;

//   // Interfaces
//   std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
//   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//   // Read / Write
//   hardware_interface::return_type read(
//     const rclcpp::Time & time,
//     const rclcpp::Duration & period) override;

//   hardware_interface::return_type write(
//     const rclcpp::Time & time,
//     const rclcpp::Duration & period) override;

// private:
//   /* ---------- ROS ---------- */
//   rclcpp::Node::SharedPtr node_;
//   rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;

//   /* ---------- Steering ---------- */
//   double steering_from_esp32_ = 0.0;
//   double steering_position_  = 0.0;

//   /* ---------- Rear wheels ---------- */
//   double rear_left_wheel_velocity_  = 0.0;
//   double rear_right_wheel_velocity_ = 0.0;

//   double rear_left_wheel_command_   = 0.0;
//   double rear_right_wheel_command_  = 0.0;


  
// };

// }  // namespace esp32_hardware


// ---------------------------------------- New Code ------------------------------
#ifndef ESP32_HARDWARE__ESP32_HARDWARE_HPP_
#define ESP32_HARDWARE__ESP32_HARDWARE_HPP_

#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32.hpp"

namespace esp32_hardware
{

class ESP32Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ESP32Hardware)

  // Lifecycle
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // IO
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ROS node
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  // ====== STATE VARIABLES ======

  // Rear wheels
  double rear_left_wheel_pos_ = 0.0;
  double rear_right_wheel_pos_ = 0.0;
  double rear_left_wheel_vel_ = 0.0;
  double rear_right_wheel_vel_ = 0.0;

  // Steering joints
  double front_left_steer_pos_ = 0.0;
  double front_right_steer_pos_ = 0.0;

  // Front wheel rotation (passive)
  double front_left_wheel_pos_ = 0.0;
  double front_right_wheel_pos_ = 0.0;

  // ====== COMMAND VARIABLES ======

  // Rear wheel velocity commands
  double rear_left_wheel_cmd_ = 0.0;
  double rear_right_wheel_cmd_ = 0.0;

  // Steering commands
  double front_left_steer_cmd_ = 0.0;
  double front_right_steer_cmd_ = 0.0;

  // ====== ESP32 INPUT ======
  double steering_from_esp32_ = 0.0;
};

}  // namespace esp32_hardware

#endif  // ESP32_HARDWARE__ESP32_HARDWARE_HPP_
