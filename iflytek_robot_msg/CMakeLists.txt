cmake_minimum_required(VERSION 3.5)

project(iflytek_robot_msg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Temperature.srv"
  "srv/Brushes.srv"
  "srv/PumpMotor.srv"
  "srv/MainMotionMotor.srv"
  "srv/FanForce.srv"
  "srv/LdsWorkMode.srv"
  "srv/MopMotor.srv"
  "srv/CameraMotor.srv"
  "srv/ChassisSensorStatus.srv"
  "srv/SensorReset.srv"
  "srv/SensorEnable.srv"
  "srv/PowerOff.srv"
  "srv/Ota.srv"
  "srv/UniversalCmd.srv"
  "srv/WifiLed.srv"
  "srv/SensorStatus.srv"
  "srv/InquirySensorData.srv"
  "msg/SensorData.msg"
  "msg/VelocityCmd.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
