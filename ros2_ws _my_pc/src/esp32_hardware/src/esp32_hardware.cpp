// #include "esp32_hardware/esp32_hardware.hpp"

// #include "pluginlib/class_list_macros.hpp"
// #include "std_msgs/msg/float32.hpp"

// namespace esp32_hardware
// {

// hardware_interface::CallbackReturn ESP32Hardware::on_init(
//   const hardware_interface::HardwareInfo & info)
// {
//   if (hardware_interface::SystemInterface::on_init(info) !=
//       hardware_interface::CallbackReturn::SUCCESS)
//   {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   node_ = std::make_shared<rclcpp::Node>("esp32_hardware_node");

//   steering_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
//     "/esp32/steering",
//     rclcpp::SensorDataQoS(),
//     [this](const std_msgs::msg::Float32::SharedPtr msg)
//     {
//       steering_from_esp32_ = msg->data;
//     });

//   RCLCPP_INFO(node_->get_logger(),
//               "ESP32 Hardware Interface Initialized");

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface>
// ESP32Hardware::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> states;

//   states.emplace_back(
//     "front_left_steering_joint", "position", &steering_position_);
//   states.emplace_back(
//     "front_right_steering_joint", "position", &steering_position_);

//   states.emplace_back(
//     "rear_left_wheel_joint", "velocity", &rear_left_wheel_velocity_);
//   states.emplace_back(
//     "rear_right_wheel_joint", "velocity", &rear_right_wheel_velocity_);

//   return states;
// }

// std::vector<hardware_interface::CommandInterface>
// ESP32Hardware::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> commands;

//   commands.emplace_back(
//     "rear_left_wheel_joint", "velocity", &rear_left_wheel_command_);
//   commands.emplace_back(
//     "rear_right_wheel_joint", "velocity", &rear_right_wheel_command_);

//   return commands;
// }

// hardware_interface::CallbackReturn ESP32Hardware::on_activate(
//   const rclcpp_lifecycle::State &)
// {
//   RCLCPP_INFO(node_->get_logger(), "ESP32 hardware activated");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn ESP32Hardware::on_deactivate(
//   const rclcpp_lifecycle::State &)
// {
//   RCLCPP_INFO(node_->get_logger(), "ESP32 hardware deactivated");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::return_type ESP32Hardware::read(
//   const rclcpp::Time &, const rclcpp::Duration &)
// {
//   // Read steering from ESP32
//   steering_position_ = steering_from_esp32_;

//   // Fake rear wheel feedback for now
//   rear_left_wheel_velocity_ = rear_left_wheel_command_;
//   rear_right_wheel_velocity_ = rear_right_wheel_command_;

//   // Spin subscriber
//   rclcpp::spin_some(node_);

//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type ESP32Hardware::write(
//   const rclcpp::Time &, const rclcpp::Duration &)
// {
//   // Later: send velocity commands to ESP32
//   return hardware_interface::return_type::OK;
// }

// }  // namespace esp32_hardware

// PLUGINLIB_EXPORT_CLASS(
//   esp32_hardware::ESP32Hardware,
//   hardware_interface::SystemInterface)





// ----------------------------- new code ----------------------



#include "esp32_hardware/esp32_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace esp32_hardware
{

hardware_interface::CallbackReturn ESP32Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("esp32_hardware_node");
steering_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
  "/cmd_steering_angle_", 10);
  steering_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/feedback_steering_angle",
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float32::SharedPtr msg)
    {
      steering_from_esp32_ = msg->data;
    });

  RCLCPP_INFO(node_->get_logger(), "ESP32 Hardware Interface Initialized");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ESP32Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;

  // Rear wheels
  states.emplace_back("rear_left_wheel_joint", "position", &rear_left_wheel_pos_);
  states.emplace_back("rear_left_wheel_joint", "velocity", &rear_left_wheel_vel_);

  states.emplace_back("rear_right_wheel_joint", "position", &rear_right_wheel_pos_);
  states.emplace_back("rear_right_wheel_joint", "velocity", &rear_right_wheel_vel_);

  // Steering joints
  states.emplace_back("front_left_steering_joint", "position", &front_left_steer_pos_);
  states.emplace_back("front_right_steering_joint", "position", &front_right_steer_pos_);

  // Front wheel rotation (passive)
  states.emplace_back("front_left_wheel_joint", "position", &front_left_wheel_pos_);
  states.emplace_back("front_right_wheel_joint", "position", &front_right_wheel_pos_);

  return states;
}

std::vector<hardware_interface::CommandInterface>
ESP32Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commands;

  // Rear wheel velocity commands
  commands.emplace_back(
    "rear_left_wheel_joint", "velocity", &rear_left_wheel_cmd_);
  commands.emplace_back(
    "rear_right_wheel_joint", "velocity", &rear_right_wheel_cmd_);

  // Steering commands
  commands.emplace_back(
    "front_left_steering_joint", "position", &front_left_steer_cmd_);
  commands.emplace_back(
    "front_right_steering_joint", "position", &front_right_steer_cmd_);

  return commands;
}

hardware_interface::CallbackReturn ESP32Hardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "ESP32 hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32Hardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "ESP32 hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ESP32Hardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  rclcpp::spin_some(node_);

  // Steering feedback from ESP32
  front_left_steer_pos_ = steering_from_esp32_;
  front_right_steer_pos_ = steering_from_esp32_;

  // Fake velocity feedback (replace with encoders)
  
  rear_left_wheel_vel_ = rear_left_wheel_cmd_;
  rear_right_wheel_vel_ = rear_right_wheel_cmd_;

    // NO wheel motion (temporary mode)-------------------
  rear_left_wheel_vel_  = 0.0;
  rear_right_wheel_vel_ = 0.0;


  // Integrate wheel positions
  rear_left_wheel_pos_ += rear_left_wheel_vel_ * period.seconds();
  rear_right_wheel_pos_ += rear_right_wheel_vel_ * period.seconds();

  // Front wheels rotate passively
  front_left_wheel_pos_ = rear_left_wheel_pos_;
  front_right_wheel_pos_ = rear_right_wheel_pos_;

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type ESP32Hardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // 1. Send Steering Command to ESP32
  auto steer_msg = std_msgs::msg::Float32();
  
  // Usually, front_left_steer_cmd_ and front_right_steer_cmd_ are identical 
  // for simple Ackermann simulation or handled by the controller.
  steer_msg.data = front_left_steer_cmd_; 
  
  steering_pub_->publish(steer_msg);

  // 2. Send Throttle (Digital Pot) - Example
  // You would likely publish to a /cmd_throttle topic here for the X9C
  
  return hardware_interface::return_type::OK;
}


}  // namespace esp32_hardware

PLUGINLIB_EXPORT_CLASS(
  esp32_hardware::ESP32Hardware,
  hardware_interface::SystemInterface)
