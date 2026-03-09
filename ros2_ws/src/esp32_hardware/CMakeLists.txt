cmake_minimum_required(VERSION 3.8)
project(esp32_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

# Build the hardware plugin library
add_library(${PROJECT_NAME} SHARED
  src/esp32_hardware.cpp
)

# Include directories for headers
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
)

# Export the plugin for pluginlib
pluginlib_export_plugin_description_file(
  hardware_interface esp32_hardware.xml
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin XML
install(
  FILES esp32_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
