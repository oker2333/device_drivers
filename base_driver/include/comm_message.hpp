/******************************************************************************

                  版权所有 (C), ?2019 Tanglingbin, Shenzhen Silver Star Intelligent Technology Co.,Ltd

 ******************************************************************************

  文 件 名   : Sensor_DropProc.c
  版 本 号   : 初稿
  作    者  :  
  生成日期   : 2019年1月3日
  最近修改   :
  功能描述   : SPIx驱动配置
  函数列表   :
  修改历史   :
  1.日    期   : 2019年1月3日
    作    者   :  
    修改内容     : 创建文件
    
  2.日    期   : 2019年1月21日
    作    者   :  
    修改内容     : 
******************************************************************************/
#ifndef __COMM_MESSAGE_H
#define __COMM_MESSAGE_H

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "uart_synchronous.hpp"


#define FRAME_HEAD_HIGH		0xF4
#define FRAME_HEAD_LOW  	0xF5
#define FRAME_TAIL_HIGH		0xF4
#define FRAME_TAIL_LOW  	0xFB
#define FRAME_NUM_HIGH		0x00
#define FRAME_NUM_LOW  		0x00
#define FRAME_CMD_HIGH		0xC0
#define FRAME_CMD_LOW  		0x00

#define FRAME_HEAD_SIZE		2
#define FRAME_NUM_SIZE		2		
#define FRAME_CMD_SIZE		2
#define FRAME_LEN_SIZE		2
#define FRAME_CHECK_SIZE	2				
#define FRAME_TAIL_SIZE		2
#define FRAME_SPEED_SIZE	2
#define FRAME_BACK_LENGTH_SIZE	1
#define FRAME_FORWARD_LENGTH_SIZE	4
#define FRAME_ROTATE_ANGLE_SIZE	4

#define	FRAME_DROP_SIZE 	1
#define	FRAME_WALL_SIZE 	1
#define	FRAME_BUMP_SIZE 	1
#define	FRAME_LIFTOFF_SIZE 	1
#define	FRAME_DUST_SIZE 	1
#define	FRAME_WATER_SIZE 	1

#define	FRAME_TOF_SIZE 	1
#define	FRAME_VEL_SIZE 	4
#define	FRAME_MOTOR_CURRENT_SIZE 	1
#define	FRAME_BATTERY_DISPLAY_SIZE 	1
#define	FRAME_CHARGE_STATUS_SIZE 	1
#define	FRAME_BUTTON_SIZE 	1

#define	FRAME_FIND_HOME_SIZE 	5
#define	FRAME_IMU_SIZE 	12
#define	FRAME_ENCODER_SIZE 	2

typedef enum
{
		FRAME_HEAD_OFFSET = 0,
		FRAME_LENGTH_OFFSET = 2,		
		FRAME_NUM_OFFSET = 4,
		FRAME_CMD_OFFSET = 6,			
		FRAME_DATA_OFFSET = 8,
}FRAME_OFFSET_TYPE;

union U8_TO_FLOAT
{
	float f;
	unsigned char u8[4];
};

union INT_TO_U8
{
	int16_t temp;
	uint8_t u8[2];
};

#define		REPORT_LEN			200
#define   	SYS_CMD_SIZE        2048
#define   	SYS_CMD_MIN        	12
typedef struct
{
	unsigned char buffer[SYS_CMD_SIZE];
	unsigned int head;
	unsigned int tail;		
	unsigned int len;
	//unsigned int count;	
}commParmType;

typedef enum{		

/*******************联机消息*************************/	
		eSerialReceiveCmdOnLine = 0x0001,
		eSerialReportAckOnLine	= 0x8001,
		
		eSerialReceiveCmdHeartbeat = 0x0002,
		eSerialReportAckHeartbeat = 0x8002,	
		
/*******************查询消息报文*********************/	
		eSerialReceiveCmdDevicefifo = 0x0101,
		eSerialReportAckDevicefifo = 0x8101,	

		eSerialgetSensorEnableStatus = 0x0102,
		eSerialAckSensorEnableStatus = 0x8102,	
	
		eSerialInquirySensorData = 0x0103,
		eSerialAckSensorData = 0x8103,		
	
/*******************控制消息报文*********************/	
		eSerialSetSensorOpenStatus = 0x0201,
		eSerialAckCtrlCmd	= 0x8200,
		eSerialSetVelocity = 0x0202,
		eSerialSetWifiLedDisplay = 0x0203,
		eSerialSetFanVelocityLevel = 0x0204,
		eSerialSetSensorCalibrate = 0x0205,
		eSerialSetTimingReportParam = 0x0206,
		eSerialSetPwrOff = 0x0207,
		eSerialSetCleanLedDisplay = 0x0208,
		eSerialSetMainMotorVelocityLevel = 0x0209,
		eSerialSetSideMotorVelocityLevel = 0x020A,
		eSerialSetSensorReset = 0x020B,	
		eSerialSetPumpMotorVelocityLevel = 0x020C,
		eSerialSetMopMotorVelocityLevel = 0x020D,
		eSerialSetCmaeraMotorLevel = 0x020E,
		eSerialSetCollectDust  = 0x0210,
		eSerialSetSupplyWater  = 0x0211,
		eSerialSetCleanMop  = 0x0212,
		eSerialSetAirDrying  = 0x0213,
		eSerialSetCharging  = 0x0214,
		eSerialSetSelfClean  = 0x0215,
		
/*******************定时消息报文*********************/			
		eSerialReportTimingSensor = 0x0301,

/*******************主动消息报文*********************/	
		eSerialReportBatteryDisplay = 0x0401,
		eSerialReportDropSensorData = 0x0402,
		eSerialReportBumpSensorData = 0x0403,
	
		eSerialReportLiftOffData = 0x0404,
		eSerialReportDustBoxData = 0x0405,
		eSerialReportWaterBoxData = 0x0406,

		eSerialReportButtonData 				= 0x0407,
		eSerialReportWallSensorData 			= 0x0408,
		eSerialReportErrorNum 				= 0x0409,
		eSerialReportChargeSignalData 				= 0x040A,
		eSerialReportLowPowerSignalData 				= 0x040B,
		eSerialReportMiddleCleanCurrent 				= 0x040C,
		eSerialReportMutipleSignals 				= 0x040D,
		eSerialReportFloorMaterial 				= 0x040E,
		eSerialReportMopStatus 				= 0x040F,
		eSerialReportStationKey 				= 0x0410,
		eSerialReportStationOutOfContact 				= 0x0411,
		eSerialReportStationCollectDust 				= 0x0412,
		eSerialReportStationSupplyWater 				= 0x0413,
		eSerialReportStationAirDry 				= 0x0414,
		eSerialReportStationSelfClean 				= 0x0415,
		eSerialReportBatteryTemperature 				= 0x0416,
		eSerialReportStationCollectDustSensor 				= 0x0417,
		eSerialReportStationSupplyWaterSensor 				= 0x0418,
		eSerialReportStationStatus 				= 0x0419,
		
		eSerialReportAckData = 0x8400,	     	//确认应答报文（上位机 -> 下位机）

/*******************OTA升级报文*********************/
		eSerialOTACmdUpgradeStart = 0x0500,
		eSerialOTACmdUpgradeEnd = 0x0502,
		eSerialOTACmdUpgradeDataFrame = 0x0503,
		eSerialOTACmdUpgradeAck = 0x8500
}TypeDefCmd;



#define TOF_DATA 				(1<<0)
#define MOTOR_DATA 				(1<<1)
#define DOCKING_DATA 			(1<<2)
#define IMU_DATA 				(1<<3) 
#define IMU_EULER_DATA 			(1<<4)
#define ENCODER_DATA 			(1<<5)
#define POSE_INFO  				(1<<6)
#define IS_BITMASK1_EFFECT 		(1<<7)
#define SYS_TIME_DATA	 		(1<<0)
#define WALL_DATA				(1<<1)


typedef enum
{
	COMM_STEP_START,
	COMM_STEP_RESEND,
}RESEND_SETP;

typedef struct
{
	RESEND_SETP	step;
	uint8_t reSendCounter;
	uint32_t reSendTimeCounter;
	TypeDefCmd frameCmd;
	uint8_t ackFlag;				//用于保存是否收到应答标志，重发标志
	uint8_t data[2];				//用于保存前一次和后一次数据
}activeReportSensor;

typedef enum{
	TOF_BIT,
	WHEEL_CURRENT_BIT,
	STATION_SIGNAL_BIT,
	IMU_DATA_BIT,
	IMU_EULAR_BIT,
	ENCODE_DISK_BIT,
	FUSION_POSE_BIT,
	BITS0_MAX
}BitMask0;

typedef enum{
	SYSTEM_TIME_BIT,
	INFRARED_WALL_BIT,
	WHEEL_SPEED_BIT,
	BITS1_MAX
}BitMask1;

typedef int BitMask0_t;
typedef int BitMask1_t;

extern commParmType commParmInfo;


uint16_t Comm_cmdParse(void);
void Comm_send_package(TypeDefCmd frameCmd, uint8_t* buffer, uint16_t len, uint16_t frameCmdCounter);
void USART_SEND_FUN(uint8_t* buffer, uint16_t len);
uint16_t CRC16_CCITT_FALSE(uint8_t *puchMsg, uint32_t usDataLen);

void sensorActiveReportType(void);

uint16_t reported_callback(TypeDefCmd cmd,uint8_t* buffer);
uint16_t frequently_report_package(uint8_t* buffer);
void Comm_timingReportSendProc(TypeDefCmd cmd);		//定时上报
void Comm_cmdExecute(uint8_t *sysCmdBuf,uint16_t sysCmdLen);
void Comm_reportPackageProc(uint16_t cmd, uint16_t *reportBuf, uint16_t length);

void set_control_message_result(uint16_t sysParseCmdData,uint8_t status);
uint8_t get_control_message_result(uint16_t sysParseCmdData);

void get_device_info_result(uint8_t* sn_s,uint16_t* chip_id_s,uint8_t* version_s);
void set_device_info_result(uint8_t* buffer);

void set_inspection(uint16_t right_wall_v,uint16_t left_floor_v,uint16_t left_middle_floor_v,uint16_t right_middle_floor_v,uint16_t right_floor_v,uint16_t left_behind_v,uint16_t right_behind_v);
void get_inspection(uint16_t* right_wall_v,uint16_t* left_floor_v,uint16_t* left_middle_floor_v,uint16_t* right_middle_floor_v,uint16_t* right_floor_v,uint16_t* left_behind_v,uint16_t* right_behind_v);

void set_station_signal(uint8_t left_v,uint8_t middle_left_v,uint8_t middle_right_v,uint8_t right_front_v,uint8_t right_v,uint8_t left_front_v,uint8_t behind_middle_left_v,uint8_t behind_middle_right_v);
void get_station_signal(uint8_t* middle_left_v,uint8_t* middle_right_v);

uint8_t get_heart_beat_result(void);
void set_heart_beat_result(uint8_t status);

void set_battery_quantity(uint8_t data);
void get_battery_quantity(uint8_t *sensor_data);

void set_low_power(uint8_t data);
void get_low_power(uint8_t *sensor_data);

void set_charge_signal(uint8_t data);
void get_charge_signal(uint8_t *sensor_data);

void set_pressure_sensor(uint8_t sensor_data);
uint8_t get_pressure_sensor(void);

void set_material(int8_t sensor_data);
void get_material(int8_t *sensor_data);

void set_tof_data(uint8_t data);
uint8_t get_tof_data(void);

void set_system_time(uint64_t time_ses_v,uint64_t time_nsec_v);
void get_system_time(uint64_t* time_ses_v,uint64_t* time_nsec_v);

void set_lift_off(uint8_t sensor_data);
void get_lift_off(uint8_t *sensor_data);

void set_drop_down(uint8_t sensor_data);
void get_drop_down(uint8_t *sensor_data);

void set_dust_box(uint8_t sensor_data);
void get_dust_box(uint8_t *sensor_data);

void set_water_box(uint8_t sensor_data);
void get_water_box(uint8_t *sensor_data);

void set_button(uint8_t sensor_data);
void get_button(uint8_t *sensor_data);

void set_wall_sensor(uint8_t sensor_data);
void get_wall_sensor(uint8_t *sensor_data);

void set_err_num(uint8_t sensor_data);
void get_err_num(uint8_t *sensor_data);

void set_middle_clean_current(uint8_t sensor_data);
void get_middle_clean_current(uint8_t *sensor_data);

void set_temperature(float temperature1_v,float temperature2_v,float temperature3_v);
void get_temperature(float* temperature1_v,float* temperature2_v,float* temperature3_v);

void set_rocker_switch(uint8_t sensor_data);
void get_rocker_switch(uint8_t *sensor_data);

void set_water_level(uint8_t sensor_data);
void get_water_level(uint8_t *sensor_data);

void set_rag_motor_current_signal(uint8_t sensor_data);
void get_rag_motor_current_signal(uint8_t *sensor_data);

void set_station_key(int8_t sensor_data);
void get_station_key(int8_t *sensor_data);

void set_OTA_message_result(uint8_t status);
uint8_t get_OTA_message_result(void);

void set_sensor_status_result(uint16_t status);
uint8_t get_sensor_status_result(uint16_t sysParseCmdData);

void set_inquiry_sensor_data_result(uint8_t* buffer);

void set_online_message_result(uint8_t* buffer);
void get_online_message_result(uint8_t* onlie_result_p,uint16_t* slave_protocol_version_p);

void set_wheel_current(uint16_t left,uint16_t right);
void get_wheel_current(uint16_t* left,uint16_t* right);
void set_IMU_ACC_linear(float Accel_x_data,float Accel_y_data,float Accel_z_data);
void get_IMU_ACC_linear(float* Accel_x_data,float* Accel_y_data,float* Accel_z_data);
void set_IMU_ACC_angle(float Gyro_x_data,float Gyro_y_data,float Gyro_z_data);
void get_IMU_ACC_angle(float* Gyro_x_data,float* Gyro_y_data,float* Gyro_z_data);
void set_IMU_Eular(float pitch_data,float roll_data,float yaw_data);
void get_IMU_Eular(float* pitch_data,float* roll_data,float* yaw_data);
void set_wheel_pulse(int16_t left_wheel_pulse_data,int16_t right_wheel_pulse_data);
void get_wheel_pulse(int16_t *left_wheel_pulse_data,int16_t *right_wheel_pulse_data);
void set_fusion_pose(float x_position_data,float y_position_data,float theta_data);
void get_fusion_pose(float *x_position_data,float *y_position_data,float *theta_data);
void set_wheel_speed(float l_speed_data,float r_speed_data);
void get_wheel_speed(float *l_speed_data,float *r_speed_data);
void set_mop_status(int8_t sensor_data);
void get_mop_status(int8_t *sensor_data);

void set_station_out_of_contact(int8_t sensor_data);
int8_t get_station_out_of_contact(void);
void set_station_collect_dust(int8_t sensor_data);
int8_t get_station_collect_dust(void);
void set_station_supply_water(int8_t sensor_data);
int8_t get_station_supply_water(void);
void set_station_air_dry(int8_t sensor_data);
int8_t get_station_air_dry(void);
void set_station_self_clean(int8_t sensor_data);
int8_t get_station_self_clean(void);
void set_battery_temperature(int8_t sensor_data);
int8_t get_battery_temperature(void);
void set_station_collect_dust_sensor(int8_t sensor_data);
int8_t get_station_collect_dust_sensor(void);
void set_station_supply_water_sensor(int8_t sensor_data);
int8_t get_station_supply_water_sensor(void);
void set_station_status(int8_t sensor_data);
int8_t get_station_status(void);

float u8_to_float(uint8_t* buffer);
void float_to_u8(uint8_t* buffer,float value);

bool datalink_frame_send(TypeDefCmd cmd,Sensor_Id_t id,uint8_t* buffer,uint16_t len);
#endif



