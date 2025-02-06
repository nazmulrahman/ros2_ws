cmake_minimum_required(VERSION 3.5)
project(custom_motor_hw)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Manually include serial (since it's not a standard ROS 2 package)
include_directories(${CMAKE_SOURCE_DIR}/../serial/include)
link_directories(${CMAKE_SOURCE_DIR}/../serial/lib)

# Build the shared library
add_library(motor_hardware_interface SHARED src/motor_hardware_interface.cpp)
target_include_directories(motor_hardware_interface PRIVATE include)
target_link_libraries(motor_hardware_interface serial)

ament_target_dependencies(motor_hardware_interface hardware_interface pluginlib rclcpp)

pluginlib_export_plugin_description_file(hardware_interface custom_motor_hw.xml)

# Install
install(TARGETS motor_hardware_interface
  DESTINATION lib
)
install(FILES custom_motor_hw.xml
  DESTINATION share/custom_motor_hw
)

ament_package()
