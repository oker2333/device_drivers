// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <cstdio>
#include <stdio.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <string>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "oms_lds.hpp"
#include "iflytek_robot_msg/srv/lds_work_mode.hpp"

float Min_0 = 2.8;
float Max_0 = 3.0;

float Min_1 = 5.0;
float Max_1 = 5.2;

float Min_2 = 7.0;
float Max_2 = 7.1;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
/*
  if(argc != 7)
  {
     return -1;
  }

  Min_0 = atof(argv[1]);
  Max_0 = atof(argv[2]);

  Min_1 = atof(argv[3]);
  Max_1 = atof(argv[4]);

  Min_2 = atof(argv[5]);
  Max_2 = atof(argv[6]);
*/

  printf("Min_0 = %f rad,Max_0 = %f rad,Min_1 = %f rad,Max_1 = %f rad,Min_2 = %f rad,Max_2 = %f rad\n",Max_0,Min_0,Max_1,Min_1,Max_2,Min_2);

  //lds工作模式server
  rclcpp::executors::SingleThreadedExecutor work_mode_executor;
  std::shared_ptr<rclcpp::Node> work_mode_node = rclcpp::Node::make_shared("work_mode_server");
  rclcpp::Service<iflytek_robot_msg::srv::LdsWorkMode>::SharedPtr work_mode_server =
    work_mode_node->create_service<iflytek_robot_msg::srv::LdsWorkMode>("lds_work_mode", &set_work_mode);
  work_mode_executor.add_node(work_mode_node);
  std::thread work_mode_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &work_mode_executor));

  rclcpp::executors::SingleThreadedExecutor oms_lds_executor;
  auto oms_lds_node = std::make_shared<Oms_Lds>("oms_lds_node", rclcpp::NodeOptions());
  oms_lds_executor.add_node(oms_lds_node);
  std::thread oms_lds_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &oms_lds_executor));

  oms_lds_thread.join();
  work_mode_thread.join();
  rclcpp::shutdown();
  
  return 0;
}
