#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <string>
#include <functional>
#include <chrono>
#include <iostream>   

#include "rclcpp/rclcpp.hpp"

#include "iflytek_robot_msg/msg/sensor_data.hpp"

#include "iflytek_robot_msg/srv/temperature.hpp"
#include "iflytek_robot_msg/srv/brushes.hpp"
#include "iflytek_robot_msg/srv/pump_motor.hpp"
#include "iflytek_robot_msg/srv/fan_force.hpp"
#include "iflytek_robot_msg/srv/mop_motor.hpp"
#include "iflytek_robot_msg/msg/velocity_cmd.hpp"
#include "iflytek_robot_msg/srv/camera_motor.hpp"
#include "iflytek_robot_msg/srv/universal_cmd.hpp"

#include "iflytek_robot_msg/srv/sensor_reset.hpp"
#include "iflytek_robot_msg/srv/sensor_enable.hpp"
#include "iflytek_robot_msg/srv/ota.hpp"
#include "iflytek_robot_msg/srv/wifi_led.hpp"
#include "iflytek_robot_msg/srv/sensor_status.hpp"
#include "iflytek_robot_msg/srv/inquiry_sensor_data.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

#include "visibility.h"
#include "nodes_set.hpp"
#include "uart_synchronous.hpp"
#include "comm_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/***********************************************************************************************/
/*拖布电机*/
static void control_mop_motor(const std::shared_ptr<iflytek_robot_msg::srv::MopMotor::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::MopMotor::Response> response)
{
    int16_t level = request->level;
    
    uint16_t invoke_id = find_free_invoke_id();
    
    if((level >= -10) && (level <= 10))          //关闭电机
    {
        uint8_t buffer[2] = {0};

        buffer[0] = ((int16_t)level) >> 8;
        buffer[1] = ((int16_t)level) & 0xFF;

        if(datalink_frame_send(eSerialSetMopMotorVelocityLevel,MopMotor_e,buffer,2))
        {
            response->success = !get_control_message_result(eSerialSetMopMotorVelocityLevel);
        }
        else
        {
            response->success = 0;
        }
    }
    else
    {
       response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[control_mop_motor]:request->level %d,response->success %d",request->level,response->success);
}

/*注水泵电机*/
static void control_pump_motor(const std::shared_ptr<iflytek_robot_msg::srv::PumpMotor::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::PumpMotor::Response> response)
{
    uint16_t water_amount = request->water_amount;
    uint16_t invoke_id = find_free_invoke_id();

    if((water_amount >= 0) && (water_amount <= 3))
    {
        uint8_t buffer[2] = {0};

        buffer[0] = water_amount >> 8;
        buffer[1] = water_amount & 0xFF;

        if(datalink_frame_send(eSerialSetPumpMotorVelocityLevel,PumpMotor_e,buffer,2))
        {
            response->success = !get_control_message_result(eSerialSetPumpMotorVelocityLevel);
        }
        else
        {
            response->success = 0;
        }
    }
    else{
      response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[control_pump_motor]:request->water_amount %d,response->success %d",request->water_amount,response->success);
}

/*吸尘电机*/
static void control_clean_dust_motor(const std::shared_ptr<iflytek_robot_msg::srv::FanForce::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::FanForce::Response> response)
{
    double force = request->force;
    double resp = response->success;

    uint8_t buffer[2] = {0};

    buffer[0] = ((int16_t)force) >> 8;
    buffer[1] = ((int16_t)force) & 0xFF;

    if(datalink_frame_send(eSerialSetFanVelocityLevel,FanFore_e,buffer,2))
    {
        response->success = !get_control_message_result(eSerialSetFanVelocityLevel);
    }
    else
    {
        response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[control_clean_dust_motor]:request->force %d,response->success %d",request->force,response->success);
}

static void control_brushes(const std::shared_ptr<iflytek_robot_msg::srv::Brushes::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::Brushes::Response> response)
{
    int64_t brushes_id = request->brushes_id;
    uint16_t level = request->level;

    uint16_t invoke_id = find_free_invoke_id();

    if((level >= 0) && (level <= 10))
    {
        uint8_t buffer[2] = {0};

        buffer[0] = ((int16_t)level) >> 8;
        buffer[1] = ((int16_t)level) & 0xFF;

        if(brushes_id == 0){        //中扫
            Comm_send_package(eSerialSetMainMotorVelocityLevel, buffer, 2, invoke_id);
            response->success = !get_control_message_result(eSerialSetMainMotorVelocityLevel);
        }
        else if((brushes_id == 2) || (brushes_id == 1)){   //边扫
            Comm_send_package(eSerialSetSideMotorVelocityLevel, buffer, 2, invoke_id);
            response->success = !get_control_message_result(eSerialSetSideMotorVelocityLevel);
        }

        if(!semaphore_timed_wait(brushes_e)){
            response->success = 0.0f;
        }
    }else{
        response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[control_brushes]:request->brushes_id %d,request->level %d,response->success",request->brushes_id,request->level,response->success);
}

static void set_sensor_reset(const std::shared_ptr<iflytek_robot_msg::srv::SensorReset::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::SensorReset::Response> response)
{
    int8_t BitReset = request->sensor_id;

    if(datalink_frame_send(eSerialSetSensorReset,SensorReset_e,&BitReset,1))
    {
      response->success = !get_control_message_result(eSerialSetSensorReset);
    }
    else
    {
        response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[set_sensor_reset]:request->sensor_id %d,response->success %d",request->sensor_id,response->success);
}

static void set_wifi_led(const std::shared_ptr<iflytek_robot_msg::srv::WifiLed::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::WifiLed::Response> response)
{
    uint8_t buffer[5];

    buffer[0] = request->led_type;
    buffer[1] = request->blink_num >> 8;
    buffer[2] = request->blink_num & 0xFF;
    buffer[3] = request->blink_freqency;
    buffer[4] = request->led_status;

    if(datalink_frame_send(eSerialSetWifiLedDisplay,WifiLedDisplay_e,buffer,5))
    {
      response->success = !get_control_message_result(eSerialSetWifiLedDisplay);
    }
    else
    {
        response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[set_wifi_led]:request->led_type %d,response->success %d",request->led_type,response->success);
}

static void get_inquiry_sensor_data(const std::shared_ptr<iflytek_robot_msg::srv::InquirySensorData::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::InquirySensorData::Response> response)
{
    int32_t cmd = request->cmd;

    if(datalink_frame_send(eSerialInquirySensorData,InquirySensorData_e,NULL,0))
    {
        response->sensor_data = get_inquiry_sensor_data_result(cmd);
        response->success = 1;
    }
    else
    {
        response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[get_inquiry_sensor_data]:request->cmd %d,response->sensor_data %d,response->success %d",request->cmd,response->sensor_data,response->success);
}

static void get_sensor_status(const std::shared_ptr<iflytek_robot_msg::srv::SensorStatus::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::SensorStatus::Response> response)
{
    int32_t cmd = request->cmd;

    if(datalink_frame_send(eSerialgetSensorEnableStatus,SensorStatus_e,NULL,0))
    {
        response->status = get_sensor_status_result(cmd);
        response->success = 1;
    }
    else
    {
        response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[get_sensor_status]:request->cmd %d,response->status %d,response->success %d",request->cmd,response->status,response->success);
}

static void set_ota_upgrade(const std::shared_ptr<iflytek_robot_msg::srv::Ota::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::Ota::Response> response)
{
    int32_t cmd = request->cmd;
    TypeDefCmd serial_cmd;
    Sensor_Id_t sensor_id;
    int len = 0;
    uint8_t buffer[4] = {0};

    if(cmd == 0)        //开始升级命令
    {
        serial_cmd = eSerialOTACmdUpgradeStart;
        sensor_id = OTA_Upgrade_Start_e;
    }
    else if(cmd == 2)   //升级状态命令
    {
        serial_cmd = eSerialOTACmdUpgradeStatus;
        sensor_id = OTA_Upgrade_Status_e;
        len = 1;
        buffer[0] = 0;
    }
    else if(cmd == 3)   //升级数据帧命令
    {
        serial_cmd = eSerialOTACmdUpgradeDataFrame;
        sensor_id = OTA_Upgrade_Frame_e;
        /*to do*/
    }

    if(datalink_frame_send(serial_cmd,sensor_id,buffer,len))
    {
      response->success = !get_OTA_message_result(serial_cmd);
    }
    else
    {
        response->success = 0;
    }
}

static uint16_t SensorEnable = 0x0FFF;

static void set_sensor_enabling(const std::shared_ptr<iflytek_robot_msg::srv::SensorEnable::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::SensorEnable::Response> response)
{
    uint8_t buffer[2];

    int32_t cmd = request->cmd;
    int32_t sensor_id = request->sensor_id;

    if(cmd == 0)
    {
        SensorEnable &= (~(1 << sensor_id));

        buffer[0]= (uint8_t)(SensorEnable >> 8) & 0xff;
        buffer[1]= (uint8_t)(SensorEnable & 0xff);

        if(datalink_frame_send(eSerialSetSensorOpenStatus,SensorEnabling_e,buffer,2))
        {
            response->success = !get_control_message_result(eSerialSetSensorOpenStatus);
        }
        else
        {
            response->success = 0;
        }
    }
    else if(cmd == 1)
    {
        SensorEnable |= (1 << sensor_id);

        buffer[0]= (uint8_t)(SensorEnable >> 8) & 0xff;
        buffer[1]= (uint8_t)(SensorEnable & 0xff);	

        if(datalink_frame_send(eSerialSetSensorOpenStatus,SensorEnabling_e,buffer,2))
        {
            response->success = !get_control_message_result(eSerialSetSensorOpenStatus);
        }
        else
        {
            response->success = 0;
        }
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"[set_sensor_enabling]request cmd error");
    }
}

#define DISTANCE_BETWEEN_WHEELS 240.2f   //单位mm
#define DISTANCE_WHEELS_RADIUS DISTANCE_BETWEEN_WHEELS/2

class Velocity_Subscriber : public rclcpp::Node
{
public:
  Velocity_Subscriber()
  : Node("velocity_node"),received_flag(0)
  {
    subscription_ = this->create_subscription<iflytek_robot_msg::msg::VelocityCmd>(
      "ifly_robot_velocity", rclcpp::SystemDefaultsQoS(), std::bind(&Velocity_Subscriber::topic_callback, this, _1));
    timer_ = this->create_wall_timer(300ms, std::bind(&Velocity_Subscriber::timer_callback, this));
  }

private:
  void timer_callback(void)
  {
    float l_speed,r_speed;

    get_wheel_speed(&l_speed,&r_speed);

    if((received_flag == 0) && ((l_speed != 0.0f) | (l_speed != 0.0f)))
    {
      uint8_t buffer[4] = {0};
      uint16_t invoke_id = find_free_invoke_id();
      Comm_send_package(eSerialSetVelocity, buffer, 4, invoke_id);
    }
    else if(received_flag == 1)
    {
       received_flag = 0;
    }
  }

  void topic_callback(const iflytek_robot_msg::msg::VelocityCmd::SharedPtr msg)
  {
      received_flag = 1;
      uint8_t buffer[4] = {0};
      uint8_t mode = msg->velocity_model;

      float linear_speed = 0.0f;			//线速度
      float angular_speed	= 0.0f;		    //角速度
      int16_t velocity_l = 0.0f;            //左轮速度
      int16_t velocity_r = 0.0f;            //右轮速度

      uint16_t invoke_id = find_free_invoke_id();

      if(mode == 60)                    //机器人整体速度
      {
          linear_speed = msg->entirety_vel.linear.x * 1000;
          angular_speed = msg->entirety_vel.angular.z;
          
          velocity_r = (int16_t)(angular_speed * DISTANCE_WHEELS_RADIUS + linear_speed);
          velocity_l = (int16_t)(2 * linear_speed - (angular_speed * DISTANCE_WHEELS_RADIUS + linear_speed));
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Velocity_Subscriber]:linear_speed = %f,angular_speed = %f",linear_speed,angular_speed);
      }
      else if(mode == 61)              //机器人单轮速度
      {
          velocity_l = (int16_t)(msg->left_single_wheel_vel * 1000);
          velocity_r = (int16_t)(msg->right_single_wheel_vel * 1000);
      }
      else if(mode == 63)              //紧急刹车
      {
          velocity_l = 0;
          velocity_r = 0;
      }
      else
      {
          RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"velocity mode error");
          return;
      }
      
      buffer[0] = ((int16_t)velocity_l) >> 8;
      buffer[1] = ((int16_t)velocity_l) & 0xFF;
      buffer[2] = ((int16_t)velocity_r) >> 8;
      buffer[3] = ((int16_t)velocity_r) & 0xFF;

      Comm_send_package(eSerialSetVelocity, buffer, 4, invoke_id);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Velocity_Subscriber]:mode = %d,velocity_r = %d mm/s,velocity_l = %d mm/s",mode,velocity_r,velocity_l);
  }
  uint64_t received_flag;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<iflytek_robot_msg::msg::VelocityCmd>::SharedPtr subscription_;
};

class heart_beat_Publisher : public rclcpp::Node
{
  public:
    heart_beat_Publisher()
    : Node("heart_beat_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("heart_beat", 10);
        timer_ = this->create_wall_timer(15s, std::bind(&heart_beat_Publisher::timer_callback, this));
    }

  private:
    void timer_callback(void)
    {
        bool ret = false;
        auto message = std_msgs::msg::Int64();

        if(datalink_frame_send(eSerialReceiveCmdHeartbeat,HeatBeat_e,NULL,0))
        {
            ret = !get_heart_beat_result();
        }
        message.data = ret;
        publisher_->publish(message);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[heart_beat]:result = %d",ret);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};

/************************************************************************************************/

#define MASTER_PROTOCOL 0x0100

class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("device_info")
    {
        uint8_t device_type_ = 0x01;

        uint8_t online_result_;
        uint16_t slave_protocol_;

        uint8_t buffer[3];
        buffer[0] = device_type_;
        buffer[1] = MASTER_PROTOCOL >> 8;
        buffer[2] = MASTER_PROTOCOL && 0xFF;
        
        if(datalink_frame_send(eSerialReceiveCmdOnLine,OnlineMessage_e,buffer,3))
        {
            get_online_message_result(&online_result_,&slave_protocol_);
        }

        if(device_type_ == 0x01)
        {
            device_type = "LS_MD";
        }
        else if(device_type_ == 0x02)
        {
            device_type = "LS_BASE";
        }
        else if(device_type_ == 0x02)
        {
            device_type = "LS_UNI";
        }else
        {
            device_type = "NULL";
        }
        char string_master[12] = {0};
        sprintf(string_master,"V%d.%d",(MASTER_PROTOCOL >> 8),(MASTER_PROTOCOL && 0xFF));
        master_protocol = string_master;

        if(online_result_ == 0x00)
        {
            online_result = "succeess";
        }else if(online_result_ == 0x01)
        {
            online_result = "error 1";
        }else if(online_result_ == 0x02)
        {
            online_result = "error 2";
        }else if(online_result_ == 0x03)
        {
            online_result = "error 3";
        }

        char string_slave[12] = {0};
        sprintf(string_slave,"V%d.%d",(slave_protocol_ >> 8),(slave_protocol_ && 0xFF));
        slave_protocol = string_slave;

        this->declare_parameter<std::string>("device_type", device_type);
        this->declare_parameter<std::string>("master_protocol", master_protocol);

        this->declare_parameter<std::string>("online_result", online_result);
        this->declare_parameter<std::string>("slave_protocol", slave_protocol);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"device_type = %s,master_protocol = %s,online_result = %s,slave_protocol = %s",device_type.c_str(),master_protocol.c_str(),online_result.c_str(),slave_protocol.c_str());

        /******************************************************************/
        char sn_s[20] = {0};
        uint16_t chip_id_s = 0;
        char version_s[4] = {0};

        char string_buffer[20] = {0};

        if(datalink_frame_send(eSerialReceiveCmdDevicefifo,DeviceInfo_e,NULL,0))
        {
            get_device_info_result(sn_s,&chip_id_s,version_s);
        }

        sprintf(&string_buffer[0],"%d",chip_id_s);
        sprintf(version_s,"V%d.%d.%d",version_s[2],version_s[1],version_s[0]);

        SN = sn_s;
        chip_id = string_buffer;
        slave_version = version_s;

        this->declare_parameter<std::string>("SN", SN);
        this->declare_parameter<std::string>("chip_id", chip_id);
        this->declare_parameter<std::string>("slave_version", slave_version);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"SN = %s,chip_id = %s,slave_version = %s",SN.c_str(),chip_id.c_str(),slave_version.c_str());
    }

  private:
    std::string device_type;
    std::string master_protocol;

    std::string online_result;
    std::string slave_protocol;

    std::string SN;
    std::string chip_id;
    std::string slave_version;
};

void control_camera_motor(const std::shared_ptr<iflytek_robot_msg::srv::CameraMotor::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::CameraMotor::Response> response)
{
    uint8_t cmd = request->cmd;
    if(datalink_frame_send(eSerialSetCmaeraMotorLevel,CameraMotor_e,&cmd,1))
    {
        response->success = !get_control_message_result(eSerialSetCmaeraMotorLevel);
    }
    else
    {
        response->success = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[control_camera_motor]:request->cmd = %d,response->success = %d",request->cmd,response->success);
}

void universal_control(const std::shared_ptr<iflytek_robot_msg::srv::UniversalCmd::Request> request,
          std::shared_ptr<iflytek_robot_msg::srv::UniversalCmd::Response> response)
{
    uint8_t buffer[2] ;
    TypeDefCmd eSerialSetCmd;
    Sensor_Id_t sensor_id;
    
    int32_t type = request->type;
    int32_t cmd = request->cmd;
    int32_t len = 0;

    // 0:积尘，1:补水，2:洗拖布，3:风干，4:充电，5:自清洁
    if(type == 0)
    {
        buffer[0] = cmd;
        buffer[1] = 0;
        len = 2;
        eSerialSetCmd = eSerialSetCollectDust;
        sensor_id = CollectDust_e;
    }
    else if(type == 1)
    {
        buffer[0] = cmd;
        buffer[1] = 0;
        len = 2;
        eSerialSetCmd = eSerialSetSupplyWater;
        sensor_id = SupplyWater_e;
    }
    else if(type == 2)
    {
        buffer[0] = cmd;
        buffer[1] = 0;
        len = 2;
        eSerialSetCmd = eSerialSetCleanMop;
        sensor_id = CleanMop_e;
    }
    else if(type == 3)
    {
        buffer[0] = cmd;
        buffer[1] = 0;
        len = 2;
        eSerialSetCmd = eSerialSetAirDrying;
        sensor_id = AirDrying_e;
    }
    else if(type == 4)
    {
        buffer[0] = cmd;
        buffer[1] = 0;
        len = 2;
        eSerialSetCmd = eSerialSetCharging;
        sensor_id = Charging_e;
    }
    else if(type == 5)
    {
        buffer[0] = cmd;
        buffer[1] = 0;
        len = 2;
        eSerialSetCmd = eSerialSetSelfClean;
        sensor_id = SelfClean_e;
    }
    else if(type == 6)
    {
        buffer[0] = cmd;
        len = 1;
        eSerialSetCmd = eSerialSetPwrOff;
        sensor_id = PowerOff_e;
    }
    else{
        response->success = 0;
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"universal_control error type");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[universal_control]:request->type = %d,request->cmd,response->success = %d",request->type,request->cmd,response->success);
    if(datalink_frame_send(eSerialSetCmd,sensor_id,buffer,len))
    {
        response->success = !get_control_message_result(eSerialSetCmd);
    }
    else
    {
        response->success = 0;
    }
}

/****************************************************************************************************/


class PublisherNode : public rclcpp::Node
{
    public:
      PublisherNode();
      
      static PublisherNode& GetInstance(void)
      {
            static PublisherNode m_pInstance; // 静态实例化：
            return m_pInstance;
      }

      //陀螺仪数据
      void gyroscope_callback(void);
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gyroscope_publisher;

      //里程计数据
      void optical_encoder_callback(void);
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optical_encoder_publisher;

      void optical_position_callback(void);
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optical_position_publisher;

      //传感器数据
      void sensor_data_callback(void);
      rclcpp::Publisher<iflytek_robot_msg::msg::SensorData>::SharedPtr sensor_data_publisher;

      // 为了发布tf
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher;

      float Gyro_bx = 0.0;                      // 陀螺、加计零偏
      float Gyro_by = 0.0;
      float Gyro_bz = 0.0;
      float Accel_bx = 0.0;
      float Accel_by = 0.0;
      float Accel_bz = 0.0;

      double odom_radius_l = 0.035;
      double odom_radius_r = 0.035;
      double circle_encoder = 67.2*18;
      int encoder_max = 32768;
      int encoder_mid = 16384;

      bool whether_publish_tf_ = false; // 是否发布tf
      bool whether_initial_odom_ = false;        // 是否初始化odom
      bool whether_initial_imu_ = false;        // 是否初始化odom
};


PublisherNode::PublisherNode()
: Node("publisher_node")
{
    gyroscope_publisher = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

    //轮速计
    optical_encoder_publisher = create_publisher<nav_msgs::msg::Odometry>("odom_e", 100);

    optical_position_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    sensor_data_publisher = create_publisher<iflytek_robot_msg::msg::SensorData>("sensor_data", 10);

    tf_publisher = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    this->declare_parameter<bool>("slam/parameter/whether_publish_odom_base_link_tf",whether_publish_tf_);
    this->declare_parameter<bool>("slam/parameter/whether_initial_odom",whether_initial_odom_);
    this->declare_parameter<bool>("slam/parameter/whether_initial_imu",whether_initial_imu_);
}


//里程计回调函数
// 获取左右轮脉冲数：void get_wheel_pulse(int16_t *left_wheel_pulse_data,int16_t *right_wheel_pulse_data)
void PublisherNode::optical_encoder_callback(void)
{
    static bool init_odom = false;
    static double last_time = -1;
    static int16_t last_encoder_l;
    static int16_t last_encoder_r;
    static double x_position_ = 0;
    static double y_position_ = 0;
    double static theta_caclute_ = 0.0;
    int16_t cur_encoder_l, cur_encoder_r;
    get_wheel_pulse(&cur_encoder_l, &cur_encoder_r);
    double current_time = this->now().seconds();

    // 初始化
    if( !init_odom ){
        last_encoder_l = cur_encoder_l;
        last_encoder_r = cur_encoder_r;
        last_time = current_time;
        init_odom = true;
        return;
    }

    // 计算编码器增量与时间增量
    int16_t ince_encoder_l = cur_encoder_l - last_encoder_l;
    int16_t ince_encoder_r = cur_encoder_r - last_encoder_r;
    if( ince_encoder_l > encoder_mid )
        ince_encoder_l -= encoder_max;
    else if( ince_encoder_l < -encoder_mid )
        ince_encoder_l += encoder_max;
    if( ince_encoder_r > encoder_mid )
        ince_encoder_r -= encoder_max;
    else if( ince_encoder_r < -encoder_mid )
        ince_encoder_r += encoder_max;


    // 计算左右轮平移增量
    double ince_trans_l = ince_encoder_l * 2 * 3.1415926 * odom_radius_l / circle_encoder;
    double ince_trans_r = ince_encoder_r * 2 * 3.1415926 * odom_radius_r / circle_encoder;

    x_position_ += (ince_trans_l + ince_trans_r) * 0.5 * cos(theta_caclute_);
    y_position_ += (ince_trans_l + ince_trans_r) * 0.5 * sin(theta_caclute_);
    theta_caclute_ += ( ince_trans_r - ince_trans_l ) * 1000 / DISTANCE_BETWEEN_WHEELS;

    double dt = current_time - last_time;
    double vb = (ince_trans_l + ince_trans_r) * 0.5 / dt;
    double wb = ( ince_trans_r - ince_trans_l ) * 1000 / DISTANCE_BETWEEN_WHEELS / dt;

    this->get_parameter<bool>("slam/parameter/whether_initial_odom",whether_initial_odom_);

    if(whether_initial_odom_)
    {
        x_position_ = 0.0;
        y_position_ = 0.0;
        theta_caclute_ = 0.0;
    }

    tf2::Quaternion odom_quat;
    odom_quat.setEuler(0,0,theta_caclute_);


    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vb;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = wb;
    odom.pose.pose.position.x = x_position_;
    odom.pose.pose.position.y = y_position_;
    odom.pose.pose.position.z = 0.0f;
    odom.pose.pose.orientation.x = odom_quat[0];
    odom.pose.pose.orientation.y = odom_quat[1];
    odom.pose.pose.orientation.z = odom_quat[2];
    odom.pose.pose.orientation.w = odom_quat[3];
    optical_encoder_publisher->publish(odom);

    last_time = current_time;
    last_encoder_l = cur_encoder_l;
    last_encoder_r = cur_encoder_r;
}

void PublisherNode::sensor_data_callback(void)
{
    //传感器数据publisher
    uint8_t err_num_data = 0;
    auto sensor_data = iflytek_robot_msg::msg::SensorData();
    sensor_data.tof_data = get_tof_data();
    get_wheel_current(&sensor_data.left_wheel_current,&sensor_data.right_wheel_current);
    get_station_signal(&sensor_data.station_signal_middle_left,&sensor_data.station_signal_middle_right);
    
    get_wheel_pulse(&sensor_data.left_wheel_pulse,&sensor_data.right_wheel_pulse);
    get_system_time(&sensor_data.time_ses,&sensor_data.time_nsec);

    get_wheel_speed(&sensor_data.l_speed,&sensor_data.r_speed);

    get_battery_quantity(&sensor_data.battery_quantity);

    get_drop_down(&sensor_data.floor_detect_status);

    sensor_data.crash_data = get_pressure_sensor();

    get_lift_off(&sensor_data.lift_off);

    get_dust_box(&sensor_data.dust_box_status);

    get_button(&sensor_data.button);

    get_mop_status(&sensor_data.mop_status);

    get_wall_sensor(&sensor_data.wall_detect_status);
    
    get_material(&sensor_data.material);

    get_station_key(&sensor_data.station_key);

    get_station_water_box(&sensor_data.station_water_box);

    get_station_collect_dust(&sensor_data.station_collect_dust);

    get_station_supply_water(&sensor_data.station_supply_water);

    get_station_air_drying(&sensor_data.station_air_drying);

    get_station_self_clean(&sensor_data.station_self_clean);
    
    get_err_num(&err_num_data);
    sensor_data.error_signal = (uint64_t)err_num_data;

    get_charge_signal(&sensor_data.charge_status);

    get_low_power(&sensor_data.low_power_signal);

    get_water_level(&sensor_data.water_level_signal);

    get_rag_motor_current_signal(&sensor_data.rag_motor_current_signal);

    get_temperature(&sensor_data.temperature1,&sensor_data.temperature2,&sensor_data.temperature3);

    sensor_data_publisher->publish(sensor_data);
}

void PublisherNode::optical_position_callback(void){

    static float x_position_init = 0.0;
    static float y_position_init = 0.0;
    static float theta_init = 0.0;

    static float last_x_position_ = 0.0;
    static float last_y_position_ = 0.0;
    static float last_theta_ = 0.0;
    static double last_t = -1;

    float x_position, y_position, theta;
    get_fusion_pose(&x_position,&y_position,&theta);
    double cur_t = this->now().seconds();

    if(last_t < 0){
        last_t = cur_t;
        last_x_position_ = x_position;
        last_y_position_ = y_position;
        last_theta_ = theta;
        return;
    }

    double delta_theta = theta - last_theta_;
    if(delta_theta > 3.1415926 ){
        delta_theta -= 3.1415926*2; 
    }else if( delta_theta < -3.1415926 ){
        delta_theta += 3.1415926*2;
    }

    double dt = cur_t - last_t;
    double vx = sqrt( (x_position - last_x_position_) * (x_position - last_x_position_) + (y_position - last_y_position_) * (y_position - last_y_position_) ) /dt;
    double vth = (delta_theta)/dt;


    this->get_parameter<bool>("slam/parameter/whether_initial_odom",whether_initial_odom_);
    this->get_parameter<bool>("slam/parameter/whether_publish_odom_base_link_tf",whether_publish_tf_);

    if(whether_initial_odom_)
    {
        x_position_init = x_position;
        y_position_init = y_position;
        theta_init = theta;
        this->set_parameter(rclcpp::Parameter("slam/parameter/whether_initial_odom",false));
    }

    double x_position_ = cos(-theta_init) * (x_position - x_position_init) - sin(-theta_init) * (y_position - y_position_init);
    double y_position_ = sin(-theta_init) * (x_position - x_position_init) + cos(-theta_init) * (y_position - y_position_init);
    double theta_ = theta - theta_init;
    tf2::Quaternion odom_quat;
    odom_quat.setEuler(0,0,theta_);


    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;
    odom.pose.pose.position.x = x_position_;
    odom.pose.pose.position.y = y_position_;
    odom.pose.pose.position.z = 0.0f;
    odom.pose.pose.orientation.x = odom_quat[0];
    odom.pose.pose.orientation.y = odom_quat[1];
    odom.pose.pose.orientation.z = odom_quat[2];
    odom.pose.pose.orientation.w = odom_quat[3];
    optical_position_publisher->publish(odom);

    geometry_msgs::msg::TransformStamped t;
    if(whether_publish_tf_){
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x_position_;
        t.transform.translation.y = y_position_;
        t.transform.translation.z = 0.0f;
        t.transform.rotation.x = odom_quat[0];
        t.transform.rotation.y = odom_quat[1];
        t.transform.rotation.z = odom_quat[2];
        t.transform.rotation.w = odom_quat[3];
        
        tf_publisher->sendTransform(t);
    }
    last_t = cur_t;
    last_x_position_ = x_position;
    last_y_position_ = y_position;
    last_theta_ = theta;

}

//陀螺仪回调函数
void PublisherNode::gyroscope_callback(void)
{
      float pitch, roll, yaw;
      float Accel_x, Accel_y, Accel_z;
      float Gyro_x, Gyro_y, Gyro_z;

      get_IMU_ACC_angle(&Gyro_x,&Gyro_y,&Gyro_z);
      get_IMU_ACC_linear(&Accel_x,&Accel_y,&Accel_z);
      get_IMU_Eular(&pitch,&roll,&yaw);

      static int iter = 0;
      static float Gyro_bx_sum= 0.0, Gyro_by_sum = 0.0, Gyro_bz_sum = 0.0;
      static float Accel_bx_sum= 0.0, Accel_by_sum = 0.0, Accel_bz_sum = 0.0;

      this->get_parameter<bool>("slam/parameter/whether_initial_imu",whether_initial_imu_);
      if(whether_initial_imu_)
      {
          Gyro_bx_sum += Gyro_x;
          Gyro_by_sum += Gyro_y;
          Gyro_bz_sum += Gyro_z;
          Accel_bx_sum += Accel_x;
          Accel_by_sum += Accel_y;
          Accel_bz_sum += Accel_z-1.0;
          iter++;
          if(iter>100){
              Gyro_bx = Gyro_bx_sum / iter;
              Gyro_by = Gyro_by_sum / iter;
              Gyro_bz = Gyro_bz_sum / iter;
              if( abs(pitch) < 5.0 && abs(roll) < 5.0 ){
                Accel_bx = Accel_bx_sum / iter;
                Accel_by = Accel_by_sum / iter;
                Accel_bz = Accel_bz_sum / iter;
              }
              iter = 0;
              Gyro_bx_sum = 0.0;
              Gyro_by_sum = 0.0;
              Gyro_bz_sum = 0.0;
              Accel_bx_sum = 0.0;
              Accel_by_sum = 0.0;
              Accel_bz_sum = 0.0;
              this->set_parameter(rclcpp::Parameter("slam/parameter/whether_initial_imu",false));
          }
      }

      auto imu = sensor_msgs::msg::Imu();
      imu.header.frame_id = "imu";
      imu.header.stamp = this->now();

      imu.orientation.x = roll * 3.1415 / 180;
      imu.orientation.y = pitch * 3.1415 / 180;
      imu.orientation.z = yaw * 3.1415 / 180;

      imu.angular_velocity.x = Gyro_x - Gyro_bx;
      imu.angular_velocity.y = Gyro_y - Gyro_by;
      imu.angular_velocity.z = Gyro_z - Gyro_bz;

      imu.linear_acceleration.x = Accel_x - Accel_bx;
      imu.linear_acceleration.y = Accel_y - Accel_by;
      imu.linear_acceleration.z = Accel_z - Accel_bz;

      gyroscope_publisher->publish(imu);
}


void publisher_data(void)
{
    auto& pub_ptr = PublisherNode::GetInstance();   
    while(rclcpp::ok())
    {
        pub_semaphore_wait();
        
        pub_ptr.gyroscope_callback();
        pub_ptr.optical_encoder_callback();
        pub_ptr.optical_position_callback();
        pub_ptr.sensor_data_callback();
    }
}
void ROS2_node_start(void)
{
    //（积尘、补水、洗拖布、风干、充电自清洁）服务器
    rclcpp::executors::SingleThreadedExecutor universal_executor;
    std::shared_ptr<rclcpp::Node> universal_node_s = rclcpp::Node::make_shared("universal_node");
    rclcpp::Service<iflytek_robot_msg::srv::UniversalCmd>::SharedPtr universal_server =
      universal_node_s->create_service<iflytek_robot_msg::srv::UniversalCmd>("universal", &universal_control);
    universal_executor.add_node(universal_node_s);
    std::thread universal_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &universal_executor));

    //翻转马达服务器
    rclcpp::executors::SingleThreadedExecutor camera_motor_executor;
    std::shared_ptr<rclcpp::Node> camera_motor_node_s = rclcpp::Node::make_shared("camera_motor_node");
    rclcpp::Service<iflytek_robot_msg::srv::CameraMotor>::SharedPtr camera_motor_server =
      camera_motor_node_s->create_service<iflytek_robot_msg::srv::CameraMotor>("camera_motor", &control_camera_motor);
    camera_motor_executor.add_node(camera_motor_node_s);
    std::thread camera_motor_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &camera_motor_executor));

    //联机参数服务器
    rclcpp::executors::SingleThreadedExecutor online_message_executor;
    std::shared_ptr<rclcpp::Node> online_message_node_s = std::make_shared<ParametersClass>();
    online_message_executor.add_node(online_message_node_s);
    std::thread online_message_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &online_message_executor));

    //心跳消息
    rclcpp::executors::SingleThreadedExecutor heart_beat_executor;
    std::shared_ptr<rclcpp::Node> heart_beat_node_s = std::make_shared<heart_beat_Publisher>();
    heart_beat_executor.add_node(heart_beat_node_s);
    std::thread heart_beat_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &heart_beat_executor));

    //主动轮速度subscriber
    rclcpp::executors::SingleThreadedExecutor velocity_executor;
    std::shared_ptr<rclcpp::Node> Velocity_Node_s = std::make_shared<Velocity_Subscriber>();
    velocity_executor.add_node(Velocity_Node_s);
    std::thread velocity_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &velocity_executor));

    //主动查询底盘传感器状态server
    rclcpp::executors::SingleThreadedExecutor inquiry_sensor_data_executor;
    std::shared_ptr<rclcpp::Node> inquiry_sensor_data_node = rclcpp::Node::make_shared("inquiry_sensor_data_server");
    rclcpp::Service<iflytek_robot_msg::srv::InquirySensorData>::SharedPtr inquiry_sensor_data_server =
      inquiry_sensor_data_node->create_service<iflytek_robot_msg::srv::InquirySensorData>("inquiry_sensor_data", &get_inquiry_sensor_data);
    inquiry_sensor_data_executor.add_node(inquiry_sensor_data_node);
    std::thread inquiry_sensor_data_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &inquiry_sensor_data_executor));

    //传感器使能状态server
    rclcpp::executors::SingleThreadedExecutor sensor_status_executor;
    std::shared_ptr<rclcpp::Node> sensor_status_node = rclcpp::Node::make_shared("sensor_status_server");
    rclcpp::Service<iflytek_robot_msg::srv::SensorStatus>::SharedPtr sensor_status_server =
      sensor_status_node->create_service<iflytek_robot_msg::srv::SensorStatus>("sensor_status", &get_sensor_status);
    sensor_status_executor.add_node(sensor_status_node);
    std::thread sensor_status_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &sensor_status_executor));

    //wifi指示灯server
    rclcpp::executors::SingleThreadedExecutor wifi_led_executor;
    std::shared_ptr<rclcpp::Node> wifi_led_node = rclcpp::Node::make_shared("wifi_led_server");
    rclcpp::Service<iflytek_robot_msg::srv::WifiLed>::SharedPtr wifi_led_server =
      wifi_led_node->create_service<iflytek_robot_msg::srv::WifiLed>("wifi_led", &set_wifi_led);
    wifi_led_executor.add_node(wifi_led_node);
    std::thread wifi_led_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &wifi_led_executor));

    //ota升级server
    rclcpp::executors::SingleThreadedExecutor ota_upgrade_executor;
    std::shared_ptr<rclcpp::Node> ota_upgrade_Node = rclcpp::Node::make_shared("ota_upgrade_server");
    rclcpp::Service<iflytek_robot_msg::srv::Ota>::SharedPtr ota_upgrade_server =
      ota_upgrade_Node->create_service<iflytek_robot_msg::srv::Ota>("ota_upgrade", &set_ota_upgrade);
    ota_upgrade_executor.add_node(ota_upgrade_Node);
    std::thread ota_upgrade_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &ota_upgrade_executor));

    //传感器复位server
    rclcpp::executors::SingleThreadedExecutor sensor_reset_executor;
    std::shared_ptr<rclcpp::Node> sensor_reset_Node = rclcpp::Node::make_shared("sensor_reset_server");
    rclcpp::Service<iflytek_robot_msg::srv::SensorReset>::SharedPtr sensor_reset_server =
      sensor_reset_Node->create_service<iflytek_robot_msg::srv::SensorReset>("sensor_reset", &set_sensor_reset);
    sensor_reset_executor.add_node(sensor_reset_Node);
    std::thread sensor_reset_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &sensor_reset_executor));

    //传感器使能server
    rclcpp::executors::SingleThreadedExecutor sensor_enabling_executor;
    std::shared_ptr<rclcpp::Node> sensor_enabling_node = rclcpp::Node::make_shared("sensor_enabling_server");
    rclcpp::Service<iflytek_robot_msg::srv::SensorEnable>::SharedPtr sensor_enabling_server =
      sensor_enabling_node->create_service<iflytek_robot_msg::srv::SensorEnable>("sensor_enabling", &set_sensor_enabling);
    sensor_enabling_executor.add_node(sensor_enabling_node);
    std::thread sensor_enabling_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &sensor_enabling_executor));

    //主/边刷电机
    rclcpp::executors::SingleThreadedExecutor brushes_executor;
    std::shared_ptr<rclcpp::Node> brushes_Node_s = rclcpp::Node::make_shared("brushes_node");
    rclcpp::Service<iflytek_robot_msg::srv::Brushes>::SharedPtr brushes_server =
      brushes_Node_s->create_service<iflytek_robot_msg::srv::Brushes>("brushes", &control_brushes);
    brushes_executor.add_node(brushes_Node_s);
    std::thread brushes_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &brushes_executor));

    //吸尘电机
    rclcpp::executors::SingleThreadedExecutor clean_dust_motor_executor;
    std::shared_ptr<rclcpp::Node> clean_dust_motor_Node_s = rclcpp::Node::make_shared("clean_dust_motor_node");
    rclcpp::Service<iflytek_robot_msg::srv::FanForce>::SharedPtr clean_dust_motor_server =
      clean_dust_motor_Node_s->create_service<iflytek_robot_msg::srv::FanForce>("fan_force", &control_clean_dust_motor);
    clean_dust_motor_executor.add_node(clean_dust_motor_Node_s);
    std::thread clean_dust_motor_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &clean_dust_motor_executor));

    //拖布注水泵电机
    rclcpp::executors::SingleThreadedExecutor pump_motor_executor;
    std::shared_ptr<rclcpp::Node> pump_motor_Node_s = rclcpp::Node::make_shared("pump_motor_node");
    rclcpp::Service<iflytek_robot_msg::srv::PumpMotor>::SharedPtr pump_motor_server =
      pump_motor_Node_s->create_service<iflytek_robot_msg::srv::PumpMotor>("pump_motor", &control_pump_motor);
    pump_motor_executor.add_node(pump_motor_Node_s);
    std::thread pump_motor_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &pump_motor_executor));

    //拖布电机
    rclcpp::executors::SingleThreadedExecutor mop_motor_executor;
    std::shared_ptr<rclcpp::Node> mop_motor_Node_s = rclcpp::Node::make_shared("mop_motor_node");
    rclcpp::Service<iflytek_robot_msg::srv::MopMotor>::SharedPtr mop_motor_server =
      mop_motor_Node_s->create_service<iflytek_robot_msg::srv::MopMotor>("mop_motor", &control_mop_motor);
    mop_motor_executor.add_node(mop_motor_Node_s);
    std::thread mop_motor_thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &mop_motor_executor));

    //传感器数据publisher
    std::thread publisher_thread(publisher_data);

    //等待回收线程
    publisher_thread.join();
    sensor_reset_thread.join();
    sensor_enabling_thread.join();
    wifi_led_thread.join();
    ota_upgrade_thread.join();
    brushes_thread.join();
    clean_dust_motor_thread.join();
    online_message_thread.join();
    inquiry_sensor_data_thread.join();
    sensor_status_thread.join();
    pump_motor_thread.join();
    mop_motor_thread.join();
    velocity_thread.join();
    heart_beat_thread.join();
    camera_motor_thread.join();
}











