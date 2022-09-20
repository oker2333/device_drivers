// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <stdio.h>
#include <sys/time.h>
#include <semaphore.h>
#include <pthread.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "iflytek_robot_msg/srv/lds_work_mode.hpp"

#define PI_HC 3.141592f

class Oms_Lds : public rclcpp::Node {
 public:
  Oms_Lds(const std::string &name, rclcpp::NodeOptions const &options);
  ~Oms_Lds();

 private:
  enum class State {
    SYNC,
    LEN,
    ROT_L,
    ROT_H,
    AGS_L,
    AGS_H,
    DATA,
    TEMP_L,
    TEMP_H,
    AGE_L,
    AGE_H,
    TIME_L,
    TIME_H,
    CRC,
  };
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::string frame_id_;
  std::string port_;
  int baud_;
  int angle_offset_;
  State state_;
  uint8_t buffer_index;
  uint8_t data_buffer[1024];
  uint8_t lds_data_[80];
  std::thread thread_;
  std::thread thread_safe;
  std::atomic<bool> canceled_;
  uint16_t speed_;
  uint8_t index;
  uint8_t len;
  uint16_t sample_point;
  float start_angle_;
  float end_angle_;
  uint8_t data_temp;
  uint16_t time_;
  uint16_t temperature_;
  uint32_t sample_point_circle;
  std::vector<float> ranges_;
  std::vector<float> intensities_;
  rclcpp::Rate rate_;
  sem_t semaphore;
  
  void reset_data();
  void lds_data_parse(uint8_t data_byte);
};

void set_work_mode(const std::shared_ptr<iflytek_robot_msg::srv::LdsWorkMode::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::LdsWorkMode::Response> response);
