#include "golf_cart_hardware/golf_cart_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace golf_cart_hardware
{

/* ================= INIT ================= */
hardware_interface::CallbackReturn
GolfCartHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  node_ = rclcpp::Node::make_shared("golf_cart_hardware_node");

  executor_ =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  /* -------- Subscriptions from ESP32 -------- */
  sub_left_ticks_ =
    node_->create_subscription<std_msgs::msg::Int32>(
      "/wheel_ticks_left", 10,
      [this](std_msgs::msg::Int32::SharedPtr msg)
      {
        left_ticks_.store(msg->data);
      });

  sub_right_ticks_ =
    node_->create_subscription<std_msgs::msg::Int32>(
      "/wheel_ticks_right", 10,
      [this](std_msgs::msg::Int32::SharedPtr msg)
      {
        right_ticks_.store(msg->data);
      });

  sub_steering_ =
    node_->create_subscription<std_msgs::msg::Float32>(
      "/steering_angle_rad", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        steering_angle_rad_.store(msg->data);
      });

  /* -------- Publishers to ESP32 -------- */
  pub_throttle_ =
    node_->create_publisher<std_msgs::msg::Float32>(
      "/cmd_throttle", 10);

  pub_steering_ =
    node_->create_publisher<std_msgs::msg::Float32>(
      "/cmd_steering", 10);

  return CallbackReturn::SUCCESS;
}

/* ================= INTERFACES ================= */
std::vector<hardware_interface::StateInterface>
GolfCartHardware::export_state_interfaces()
{
  return {
    hardware_interface::StateInterface(
      "rear_wheel_joint", hardware_interface::HW_IF_VELOCITY,
      &state_velocity_),

    hardware_interface::StateInterface(
      "steering_joint", hardware_interface::HW_IF_POSITION,
      &state_steering_)
  };
}

std::vector<hardware_interface::CommandInterface>
GolfCartHardware::export_command_interfaces()
{
  return {
    hardware_interface::CommandInterface(
      "rear_wheel_joint", hardware_interface::HW_IF_VELOCITY,
      &cmd_velocity_),

    hardware_interface::CommandInterface(
      "steering_joint", hardware_interface::HW_IF_POSITION,
      &cmd_steering_)
  };
}

/* ================= ACTIVATE ================= */
hardware_interface::CallbackReturn
GolfCartHardware::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_velocity_ = 0.0;
  cmd_steering_ = 0.0;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
GolfCartHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

/* ================= READ ================= */
hardware_interface::return_type
GolfCartHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  executor_->spin_some();

  int32_t left = left_ticks_.load();
  int32_t right = right_ticks_.load();

  int32_t delta_left = left - last_left_ticks_;
  int32_t delta_right = right - last_right_ticks_;
  int32_t delta_avg = (delta_left + delta_right) / 2;

  last_left_ticks_ = left;
  last_right_ticks_ = right;

  double wheel_rotations = delta_avg / ticks_per_rev_;
  double distance =
    2.0 * M_PI * wheel_radius_ * wheel_rotations;

  state_velocity_ = distance / period.seconds();
  state_steering_ = steering_angle_rad_.load();

  return hardware_interface::return_type::OK;
}

/* ================= WRITE ================= */
hardware_interface::return_type
GolfCartHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  std_msgs::msg::Float32 throttle_msg;
  std_msgs::msg::Float32 steering_msg;

  throttle_msg.data = velocity_to_throttle(cmd_velocity_);
  steering_msg.data = cmd_steering_;

  pub_throttle_->publish(throttle_msg);
  pub_steering_->publish(steering_msg);

  return hardware_interface::return_type::OK;
}

/* ================= HELPERS ================= */
double GolfCartHardware::velocity_to_throttle(double velocity)
{
  velocity = std::max(0.0, velocity);
  return std::min(velocity / max_speed_, 1.0);
}

} // namespace golf_cart_hardware

PLUGINLIB_EXPORT_CLASS(
  golf_cart_hardware::GolfCartHardware,
  hardware_interface::SystemInterface)
