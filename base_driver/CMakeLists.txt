cmake_minimum_required(VERSION 3.5)

project(base_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iflytek_robot_msg REQUIRED)

include_directories(include)

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(sensor_app
  src/sensor_app.cpp
  src/node_thread/nodes_set.cpp
  src/uart_protocol/recv_proc_task.cpp
  src/uart_protocol/uart_synchronous.cpp
  src/uart_protocol/uart_ttys.cpp
  src/uart_protocol/comm_message.cpp
  src/ota_thread/ota.cpp)

ament_target_dependencies(sensor_app
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
  "rclcpp"
  "tf2_geometry_msgs"
  "iflytek_robot_msg")

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  sensor_app
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
