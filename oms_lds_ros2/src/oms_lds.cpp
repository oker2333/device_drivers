// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <signal.h>
#include "oms_lds.hpp"
#include <rcl_interfaces/msg/parameter.hpp>
#include <cmath>
#include "uart_ttys.hpp"
#include "iflytek_robot_msg/srv/lds_work_mode.hpp"
#include <stdio.h>
#include <sys/time.h>
#include <semaphore.h>
#include <pthread.h>

constexpr uint8_t kSync = 0x54; 
constexpr uint8_t points = 0x14;

#define UART_READ_SIZE 93

#define SAMPLING_PONIT_PER_SECOND 3500.0f

bool lds_power_supply = false;
int valuefd = -1;
int32_t rotationl_speed = 0;

void exit_handler(int signo)
{
    static int iTimes = 0;
    int iMaxTimes = 1;
    if (signo == SIGINT){
        iTimes++;
        if (iTimes >= iMaxTimes){
            uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFD};
            uart_write(cmd_buf,sizeof(cmd_buf));
            usleep(5000);   //确保串口接收到指令
            write(valuefd,"0", 2);
            
            fprintf(stderr,"**** %dth Ctrl+C,quit ! \n",iTimes);
            fflush(stderr);
            exit(0);
        }
        else{
            fprintf(stderr," %dth Ctrl+c,the program will exit on %dth Ctrl+C\n",iTimes,iMaxTimes);
        }
    }
}

void InitSignal(void)
{
    struct sigaction sa,osa;
    sa.sa_handler = exit_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    int ret = sigaction(SIGINT,&sa,&osa);
    if(ret <0){
        printf("set signal ctrl+c failure!\n");
    }
}

void set_work_mode(const std::shared_ptr<iflytek_robot_msg::srv::LdsWorkMode::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::LdsWorkMode::Response> response)
{
    static int32_t last_mode = -1;
    int32_t command = request->cmd;

    if(last_mode == command)
    {
       response->success = 1;
       return;
    }
    if((command >= 0) && (command <= 3))
      last_mode = command;

    if((lds_power_supply == false) && 
        ((command == 1) || (command == 2) || (command == 3)))
    {
      lds_power_supply = true;
      write(valuefd,"1", 2);
      usleep(5000);     //确保串口接收到指令
    }

   if(command == 1)
   {
      rotationl_speed = 5.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0xF4, 0x01, 0x00, 0x00, 0x02};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 2)
   {
      rotationl_speed = 10.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0xE8, 0x03, 0x00, 0x00, 0xC2};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 3)
   {
      rotationl_speed = 15.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0xDC, 0x05, 0x00, 0x00, 0xEE};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 5)
   {
      rotationl_speed = 5.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0xF4, 0x01, 0x00, 0x00, 0x02};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 6)
   {
      rotationl_speed = 6.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0x58, 0x02, 0x00, 0x00, 0xE2};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 7)
   {
      rotationl_speed = 7.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0xBC, 0x02, 0x00, 0x00, 0x3A};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 8)
   {
      rotationl_speed = 8.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0x20, 0x03, 0x00, 0x00, 0xB5};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 9)
   {
      rotationl_speed = 9.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0x84, 0x03, 0x00, 0x00, 0xE9};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 10)
   {
      rotationl_speed = 10.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0xE8, 0x03, 0x00, 0x00, 0xC2};
      uart_write(cmd_buf,sizeof(cmd_buf));
      response->success = 1;
   }
   else if(command == 0)
   {
      lds_power_supply = false;
      rotationl_speed = 0.0f;
      uint8_t cmd_buf[] = {0x54, 0x07, 0x01, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFD};
      uart_write(cmd_buf,sizeof(cmd_buf));
      usleep(5000);   //确保串口接收到指令
      write(valuefd,"0", 2);
      response->success = 1;
   }else
   {
      response->success = 0;
      printf("lsd setwokmode error\n");
   }
}


Oms_Lds::Oms_Lds(const std::string &name, rclcpp::NodeOptions const &options)
  : Node(name, options),
    frame_id_("scan"),
    port_("/dev/tty*"),
    baud_(460800),
    angle_offset_(0),
    state_(State::SYNC),
    canceled_(false),
    speed_(0),
    start_angle_(0.0),
    end_angle_(0.0),
    index(0),
    sample_point_circle(0),
    rate_(std::chrono::milliseconds(32)) {
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

  int exportfd, directionfd;
  
  exportfd = open("/sys/class/gpio/export", O_WRONLY);
  if (exportfd < 0)
  {
      printf("Cannot open GPIO to export it\n");
      exit(1);
  }
  write(exportfd, "38", 3);
  close(exportfd);

  directionfd = open("/sys/class/gpio/gpio38/direction", O_RDWR);
  if (directionfd < 0)
  {
      printf("Cannot open GPIO direction it\n");
      exit(1);
  }
  write(directionfd, "out", 4);
  close(directionfd);

  valuefd = open("/sys/class/gpio/gpio38/value", O_RDWR);
  if (valuefd < 0)
  {
      printf("Cannot open GPIO value\n");
      exit(1);
  }

  write(valuefd,"0", 2);  //默认关闭雷达供电

  if(uart_init() < 0)
  {
     perror("open uart failed\n");
     exit(1);
  }

  InitSignal();
  sem_init(&semaphore,0,0);
  
  thread_ = std::thread{[this]() -> void {
    while (rclcpp::ok() && !canceled_.load()) {
      int len = uart_read(data_buffer,UART_READ_SIZE);
      if(len <= 0)
        continue;
       for(int i = 0;i < len;i++)
       {
          lds_data_parse(data_buffer[i]);
       }
    }
  }};
  thread_safe = std::thread{[this]() -> void {
    while (rclcpp::ok() && !canceled_.load()) {
        static unsigned long int index = 0;
        sem_wait(&semaphore);
        sensor_msgs::msg::LaserScan message;
        message.header.stamp = this->now();
        message.header.frame_id = "laser_frame";
        message.angle_increment = 2 * PI_HC / sample_point_circle;
        message.angle_min = 0;
        message.angle_max = 2 * PI_HC;
        message.scan_time = 1.0f / rotationl_speed;
        message.range_min = 0.1f;
        message.range_max = 15.0f;
        message.ranges = ranges_;
        message.intensities = intensities_;
        scan_pub_->publish(message);
        printf("Now Publish stamp = %ld index = %ld\n",message.header.stamp,index++);
        sample_point = 0;
        reset_data();
    }
  }};
}

static int point_count = 0;

uint8_t CRC_8(uint8_t *puchMsg, uint8_t usDataLen){
	
	uint8_t uCRC = 0x00;//CRC寄存器
	
	for(uint8_t num=0;num<usDataLen;num++){
		uCRC = (*puchMsg++)^uCRC;//把数据与8位的CRC寄存器的8位相异或，结果存放于CRC寄存器。
		for(uint8_t x=0;x<8;x++){	//循环8次
			if(uCRC&0x80){	//判断最低位为：“1”
				uCRC = uCRC<<1;	//先左移
				uCRC = uCRC^0x07;	//再与多项式0x07异或
			}else{	//判断最低位为：“0”
				uCRC = uCRC<<1;	//右移
			}
		}
	}
	return uCRC;//返回CRC校验值

}

extern float Max_0;
extern float Min_0;

extern float Max_1;
extern float Min_1;

extern float Max_2;
extern float Min_2;

void Oms_Lds::lds_data_parse(uint8_t data_byte)
{
   switch(state_){
      case State::SYNC:
        if(data_byte == kSync){
            index = 0;
            buffer_index = 0;
            state_ = State::LEN;
        }
      break;

      case State::LEN:
        len = data_byte;
        if(len != 0x5a)
        {
           state_ = State::SYNC;
           break;
        }
        state_ = State::ROT_L;
      break;

      case State::ROT_L:
        speed_ = data_byte;
        state_ = State::ROT_H;
      break;

      case State::ROT_H:
        speed_ |= data_byte << 8;
        state_ = State::AGS_L;
      break;

      case State::AGS_L:
        data_temp = data_byte;
        state_ = State::AGS_H;
      break;

      case State::AGS_H:
        start_angle_ = (data_temp | (data_byte << 8)) * 0.01f;
        state_ = State::DATA;
      break;

      case State::DATA:
        lds_data_[index++] = data_byte;
        
        if(index == 80)
        {
           state_ = State::TEMP_L;
        }
      break;

      case State::TEMP_L:
        data_temp = data_byte;
        state_ = State::TEMP_H;
      break;

      case State::TEMP_H:
        temperature_ = (data_temp | (data_byte << 8)) * 0.01f;
        state_ = State::AGE_L;
      break;

      case State::AGE_L:
        data_temp = data_byte;
        state_ = State::AGE_H;
      break;
 
      case State::AGE_H:
        end_angle_ = (data_temp | (data_byte << 8)) * 0.01f;
        state_ = State::TIME_L;
      break;

      case State::TIME_L:
        time_ = data_byte;
        state_ = State::TIME_H;
      break;

      case State::TIME_H:
        time_ |= data_byte << 8;
        state_ = State::CRC;
      break;

      case State::CRC:
        static int32_t last_rotationl_speed = -1;
        if(rotationl_speed == 0)
        {
           state_ = State::SYNC;
           break;
        }
        else if(last_rotationl_speed != rotationl_speed)
        {
            sample_point_circle = SAMPLING_PONIT_PER_SECOND / rotationl_speed;
            ranges_.resize(sample_point_circle);
            intensities_.resize(sample_point_circle);
            last_rotationl_speed = rotationl_speed;
        }

        if(end_angle_ < start_angle_)
        {
           end_angle_ += 360;
        }

        float angle_res = (end_angle_ - start_angle_) / (points -1); 

        for(int i = 0; i < 20; i++)
        {
            sample_point++;
            int j = 4 * i;
            uint16_t range = static_cast<uint16_t>((lds_data_[j+1] << 8) | lds_data_[j]);
            uint16_t intensity = static_cast<uint16_t>((lds_data_[j+3] << 8) | lds_data_[j+2]);

            float measured_angle = start_angle_ + angle_res * i;
            if (measured_angle > 360)
            {
                measured_angle -= 360;
            }

            uint16_t angle_index = (uint16_t)(((sample_point_circle * (360.0f - measured_angle)) / 360.0f));
            angle_index = angle_index % sample_point_circle;

            ranges_[angle_index] = static_cast<float>(range)/1000.0f;
            intensities_[angle_index] = static_cast<float>(intensity);
            if(ranges_[angle_index] < 0.1f)
            {
               ranges_[angle_index] = 0.0f;
            }else if(ranges_[angle_index] > 15.0f)
            {
               ranges_[angle_index] = 0.0f;
            }

            float curr_angle = (measured_angle + 90) * PI_HC / 180.0f;
            if( ((curr_angle < Max_0) && (curr_angle > Min_0)) || ((curr_angle < Max_1) && (curr_angle > Min_1)) || ((curr_angle < Max_2) && (curr_angle > Min_2)))
            {
               ranges_[angle_index] = 0.0f;
            }

            if(sample_point == sample_point_circle)
            {
                sem_post(&semaphore);

            }
        }

        state_ = State::SYNC;
      break;

   }
}

Oms_Lds::~Oms_Lds() {
  canceled_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void Oms_Lds::reset_data() {
  for(int i = 0;i < SAMPLING_PONIT_PER_SECOND / rotationl_speed;i++)
  {
    ranges_[i] = 0.0f;
    intensities_[i] = 0.0f;
  }
}
