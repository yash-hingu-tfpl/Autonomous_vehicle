#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

namespace golf_cart_hardware
{

class GolfCartHardware : public hardware_interface::SystemInterface
{
public:
  // Lifecycle
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

  // Interfaces
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // Control loop
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  /* ================= ROS ================= */
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_left_ticks_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_right_ticks_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steering_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throttle_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steering_;

  /* ================= Cached sensor data ================= */
  std::atomic<int32_t> left_ticks_{0};
  std::atomic<int32_t> right_ticks_{0};
  std::atomic<double> steering_angle_rad_{0.0};

  int32_t last_left_ticks_ = 0;
  int32_t last_right_ticks_ = 0;

  /* ================= ros2_control interfaces ================= */
  double cmd_velocity_ = 0.0;   // m/s
  double cmd_steering_ = 0.0;   // rad

  double state_velocity_ = 0.0; // m/s
  double state_steering_ = 0.0; // rad

  /* ================= Parameters ================= */
  double wheel_radius_ = 0.15;     // meters
  double ticks_per_rev_ = 36.0;
  double max_speed_ = 3.0;         // m/s

  /* ================= Helpers ================= */
  double velocity_to_throttle(double velocity);
};

}  // namespace golf_cart_hardware
