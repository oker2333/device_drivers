#include "comm_message.hpp"
#include "uart_ttys.hpp"
#include "uart_synchronous.hpp"
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"

#define DEBUG 1

int fd_ttys2 = -1;
commParmType commParmInfo;
unsigned char messageSF[2] = {0xA5,0xA5};	//start_flag

/*超时重传*/

bool datalink_frame_send(TypeDefCmd cmd,Sensor_Id_t id,uint8_t* buffer,uint16_t len)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[datalink_frame_send]cmd = 0x%x",cmd);
	uint16_t invoke_id = 0;
	bool ret = false;
	for(int i = 1;i <= RYTEIES;i++)
	{
		invoke_id = find_free_invoke_id();
		Comm_send_package(cmd, buffer, len, invoke_id);

        if(semaphore_timed_wait(id)){
			ret = true;
            break;
        }else{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"[datalink_frame_send]wait ack failed %d times",i);
		}
	}
	return ret;
}

static uint64_t control_message_result = 0;

void set_control_message_result(uint16_t sysParseCmdData, uint8_t status)
{
	uint16_t mask = 1 << (sysParseCmdData & 0xFF);
	if(status)
	{
		control_message_result = control_message_result | mask;
	}
	else
	{
		control_message_result = control_message_result & ~mask;
	}
}

uint8_t get_control_message_result(uint16_t sysParseCmdData)
{
	uint16_t mask = 1 << (sysParseCmdData & 0xFF);
	uint8_t ret = control_message_result & mask;
	return ret;
}

static uint64_t heart_beat_result = 0;

void set_heart_beat_result(uint8_t status)
{
	heart_beat_result = status;
}

uint8_t get_heart_beat_result(void)
{
	uint8_t ret = heart_beat_result;
	return ret;
}

static uint8_t online_result;
static uint16_t slave_protocol_version;

void set_online_message_result(uint8_t* buffer)
{
	online_result = buffer[0];
	slave_protocol_version = (buffer[1] << 8) + buffer[2];
}

void get_online_message_result(uint8_t* online_result_p,uint16_t* slave_protocol_version_p)
{
	*online_result_p = online_result;
	*slave_protocol_version_p = slave_protocol_version;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[original data]online_result = %d,slave_protocol_version = %d",online_result,slave_protocol_version);
}

static uint8_t sensor_data[13] = {0};

void set_inquiry_sensor_data_result(uint8_t* buffer)
{
	int i = 0;
	while(i < 13)
	{
		sensor_data[i] = buffer[i];
		i++;
	}
	set_battery_quantity(buffer[0]);
	set_drop_down(buffer[1]);
	set_pressure_sensor(buffer[2]);
	set_lift_off(buffer[3]);
	set_dust_box(buffer[4]);
	set_material(buffer[5]);
	set_charge_signal(buffer[6]);
	set_mop_status(buffer[7]);
	set_battery_temperature(buffer[8]);
	set_water_level(buffer[9]);
	set_station_collect_dust_sensor(buffer[10]);
	set_station_supply_water_sensor(buffer[11]);
	set_station_status(buffer[12]);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"battery_quantity = %d,drop_down = %d,pressure_sensor = %d,lift_off = %d,dust_box = %d,material = %d,charge_signal = %d,mop_status = %d,battery_temperature = %d,water_level = %d,station_collect_dust_sensor = %d,station_supply_water_sensor = %d,station_status = %d",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11],buffer[12]);
}

uint8_t get_inquiry_sensor_data_result(uint16_t sensor_id)
{
	return sensor_data[sensor_id];
}

static uint16_t sensor_status_result = 0;

void set_sensor_status_result(uint16_t status)
{
	sensor_status_result = status;
}

uint8_t get_sensor_status_result(uint16_t sysParseCmdData)
{
	uint16_t mask = 1 << (sysParseCmdData & 0xFF);
	uint8_t ret = sensor_status_result & mask;
	return ret;
}

static int8_t sn_length;
static int8_t sn[20] = {0};
static uint16_t chip_id;
static int8_t version[3];
static int8_t compile_date[7];
static uint32_t system_time;

void set_device_info_result(uint8_t* buffer)
{
	int i = 0,j = 0,k = 0,w = 0;

	sn_length = buffer[i++];
	while(j < sn_length)
	{
		sn[j++] = buffer[i++];
	}

	chip_id = (buffer[i++] << 8) | buffer[i++];

	while(k < 3)
	{
		version[k++] = buffer[i++];
	}
	
	while(w < 7)
	{
		compile_date[w++] = buffer[i++];
	}

	system_time = (buffer[i++] << 24) | (buffer[i++] << 16) | (buffer[i++] << 8) | (buffer[i++] << 0);

}

void get_device_info_result(uint8_t* sn_s,uint16_t* chip_id_s,uint8_t* version_s)
{
	int i = 0,j = 0;
	while(i < sn_length)
	{
		sn_s[i] = sn[i];
		i++;
	}

	*chip_id_s = chip_id;

	while(j < 3)
	{
		version_s[j] = version[j];
		j++;
	}

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[original data]chip_id = %d,sn = %s",chip_id,sn);
}

static uint64_t OTA_message_result = 0;

void set_OTA_message_result(uint8_t status)
{
	OTA_message_result = status;
}

uint8_t get_OTA_message_result(void)
{
	return OTA_message_result;
}

static uint8_t pressure_sensor_data;

void set_pressure_sensor(uint8_t sensor_data)
{
	pressure_sensor_data = sensor_data;
}

uint8_t get_pressure_sensor(void)
{
	uint8_t sensor_data = pressure_sensor_data;
	return sensor_data;
}

static uint8_t lift_off_data;

void set_lift_off(uint8_t sensor_data)
{
	lift_off_data = sensor_data;
}

void get_lift_off(uint8_t *sensor_data)
{
	*sensor_data = lift_off_data;
}

static uint8_t dust_box_data;

void set_dust_box(uint8_t sensor_data)
{
	dust_box_data = sensor_data;
}

void get_dust_box(uint8_t *sensor_data)
{
	*sensor_data = dust_box_data;
}

static uint8_t water_box_data;

void set_water_box(uint8_t sensor_data)
{
	water_box_data = sensor_data;
}

void get_water_box(uint8_t *sensor_data)
{
	*sensor_data = water_box_data;
}

static uint8_t err_num_data;

void set_err_num(uint8_t sensor_data)
{
	err_num_data = sensor_data;
}

void get_err_num(uint8_t *sensor_data)
{
	*sensor_data = err_num_data;
}

static uint8_t middle_clean_current_data;

void set_middle_clean_current(uint8_t sensor_data)
{
	middle_clean_current_data = sensor_data;
}

void get_middle_clean_current(uint8_t *sensor_data)
{
	*sensor_data = middle_clean_current_data;
}

static uint8_t rocker_switch_data;

void set_rocker_switch(uint8_t sensor_data)
{
	rocker_switch_data = sensor_data;
}

void get_rocker_switch(uint8_t *sensor_data)
{
	*sensor_data = rocker_switch_data;
}

static uint8_t water_level_data;

void set_water_level(uint8_t sensor_data)
{
	water_level_data = !sensor_data;
}

void get_water_level(uint8_t *sensor_data)
{
	*sensor_data = water_level_data;
}

static uint8_t rag_motor_current_signal_data;

void set_rag_motor_current_signal(uint8_t sensor_data)
{
	rag_motor_current_signal_data = sensor_data;
}

void get_rag_motor_current_signal(uint8_t *sensor_data)
{
	*sensor_data = rag_motor_current_signal_data;
}

static uint8_t button_data;

void set_button(uint8_t sensor_data)
{
	button_data = sensor_data;
}

void get_button(uint8_t *sensor_data)
{
	*sensor_data = button_data;
}

static int8_t material_data;

void set_material(int8_t sensor_data)
{
	material_data = sensor_data;
}

void get_material(int8_t *sensor_data)
{
	*sensor_data = material_data;
}

static int8_t station_key;

void set_station_key(int8_t sensor_data)
{
	station_key = sensor_data;
}

void get_station_key(int8_t *sensor_data)
{
	*sensor_data = station_key;
}

static uint8_t wall_sensor_data;

void set_wall_sensor(uint8_t sensor_data)
{
	wall_sensor_data = sensor_data;
}

void get_wall_sensor(uint8_t *sensor_data)
{
	*sensor_data = wall_sensor_data;
}

static uint8_t drop_down_data;

void set_drop_down(uint8_t sensor_data)
{
	drop_down_data = sensor_data;
}

void get_drop_down(uint8_t *sensor_data)
{
	*sensor_data = drop_down_data;
}

static int8_t mop_status;

void set_mop_status(int8_t sensor_data)
{
	mop_status = sensor_data;
}

void get_mop_status(int8_t *sensor_data)
{
	*sensor_data = mop_status;
}

static uint8_t battery_quantity;

void set_battery_quantity(uint8_t data)
{
	battery_quantity = data;
}

void get_battery_quantity(uint8_t *sensor_data)
{
	*sensor_data = battery_quantity;
}

static uint8_t charge_signal;

void set_charge_signal(uint8_t data)
{
	charge_signal = data;
}

void get_charge_signal(uint8_t *sensor_data)
{
	*sensor_data = charge_signal;
}

static uint8_t low_power;

void set_low_power(uint8_t data)
{
	low_power = data;
}

void get_low_power(uint8_t *sensor_data)
{
	*sensor_data = low_power;
}

static uint8_t tof_data = 0;

void set_tof_data(uint8_t data)
{
	tof_data = data;
}

uint8_t get_tof_data(void)
{
	return tof_data;
}

static uint16_t left_wheel_current = 0;
static uint16_t right_wheel_current = 0;

void set_wheel_current(uint16_t left,uint16_t right)
{
	left_wheel_current = left;
	right_wheel_current = right;
}

void get_wheel_current(uint16_t* left,uint16_t* right)
{
	*left = left_wheel_current;
	*right = right_wheel_current;
}

static uint8_t left = 0;					//左接收
static uint8_t middle_left = 0;			//中左接收
static uint8_t middle_right = 0;			//中右接收
static uint8_t right_front = 0;			//右前接收
static uint8_t right = 0;				//右接收
static uint8_t left_front = 0;			//左前接收
static uint8_t behind_middle_left = 0;	//后中左接收
static uint8_t behind_middle_right = 0;	//后中右接收

void get_station_signal(uint8_t* middle_left_v,uint8_t* middle_right_v)
{
	*middle_left_v = middle_left;
	*middle_right_v = middle_right;
}

void set_station_signal(uint8_t left_v,uint8_t middle_left_v,uint8_t middle_right_v,uint8_t right_front_v,uint8_t right_v,uint8_t left_front_v,uint8_t behind_middle_left_v,uint8_t behind_middle_right_v)
{
	left = left_v;
	middle_left = middle_left_v;
	middle_right = middle_right_v;
	right_front = right_front_v;
	right = right_v;
	left_front = left_front_v;
	behind_middle_left = behind_middle_left_v;
	behind_middle_right = behind_middle_right_v;
}

static float Accel_x = 0;	//x方向加速度
static float Accel_y = 0;	//y方向加速度
static float Accel_z = 0;	//z方向加速度

static float Gyro_x = 0;	//x方向角速度
static float Gyro_y = 0;	//y方向角速度
static float Gyro_z = 0;	//z方向角速度

static float pitch = 0;		//机器姿态角俯仰角
static float roll = 0;		//机器姿态角翻滚角
static float yaw = 0;		//机器姿态角航向角

void set_IMU_ACC_linear(float Accel_x_data,float Accel_y_data,float Accel_z_data)
{
	Accel_x = Accel_x_data;
	Accel_y = Accel_y_data;
	Accel_z = Accel_z_data;
}

void get_IMU_ACC_linear(float* Accel_x_data,float* Accel_y_data,float* Accel_z_data)
{
	*Accel_x_data = Accel_x;
	*Accel_y_data = Accel_y;
	*Accel_z_data = Accel_z;
}

void set_IMU_ACC_angle(float Gyro_x_data,float Gyro_y_data,float Gyro_z_data)
{
	Gyro_x = Gyro_x_data;
	Gyro_y = Gyro_y_data;
	Gyro_z = Gyro_z_data;
}

void get_IMU_ACC_angle(float* Gyro_x_data,float* Gyro_y_data,float* Gyro_z_data)
{
	*Gyro_x_data = Gyro_x;
	*Gyro_y_data = Gyro_y;
	*Gyro_z_data = Gyro_z;
}

void set_IMU_Eular(float pitch_data,float roll_data,float yaw_data)
{
	pitch = pitch_data;
	roll = roll_data;
	yaw = yaw_data;
}

void get_IMU_Eular(float* pitch_data,float* roll_data,float* yaw_data)
{
	*pitch_data = pitch;
	*roll_data = roll;
	*yaw_data = yaw;
}

static int16_t left_wheel_pulse = 0;
static int16_t right_wheel_pulse = 0;

void set_wheel_pulse(int16_t left_wheel_pulse_data,int16_t right_wheel_pulse_data)
{
	left_wheel_pulse = left_wheel_pulse_data;
	right_wheel_pulse = right_wheel_pulse_data;
}

void get_wheel_pulse(int16_t *left_wheel_pulse_data,int16_t *right_wheel_pulse_data)
{
	*left_wheel_pulse_data = left_wheel_pulse;
	*right_wheel_pulse_data = right_wheel_pulse;
}

static float temperature1; 
static float temperature2;
static float temperature3;

void set_temperature(float temperature1_v,float temperature2_v,float temperature3_v)
{
	temperature1 = temperature1_v; 
	temperature2 = temperature2_v;
	temperature3 = temperature3_v;
}

void get_temperature(float* temperature1_v,float* temperature2_v,float* temperature3_v)
{
	*temperature1_v = temperature1; 
	*temperature2_v = temperature2;
	*temperature3_v = temperature3;
}

static uint64_t time_ses;                                  //底盘开机后时间累计单位ses
static uint64_t time_nsec;                                 //底盘开机后时间累计单位nsec

void set_system_time(uint64_t time_ses_v,uint64_t time_nsec_v)
{
	time_ses = time_ses_v;
	time_nsec = time_nsec_v;
}

void get_system_time(uint64_t* time_ses_v,uint64_t* time_nsec_v)
{
	*time_ses_v = time_ses;
	*time_nsec_v = time_nsec;
}

static float x_position = 0;
static float y_position = 0;
static float theta = 0;

void set_fusion_pose(float x_position_data,float y_position_data,float theta_data)
{
	x_position = x_position_data;
	y_position = y_position_data;
	theta = theta_data;
}

void get_fusion_pose(float *x_position_data,float *y_position_data,float *theta_data)
{
	*x_position_data = x_position;
	*y_position_data = y_position;
	*theta_data = theta;
}

static uint16_t right_wall;
static uint16_t left_floor;
static uint16_t left_middle_floor;
static uint16_t right_middle_floor;
static uint16_t right_floor;
static uint16_t left_behind;
static uint16_t right_behind;

void set_inspection(uint16_t right_wall_v,uint16_t left_floor_v,uint16_t left_middle_floor_v,uint16_t right_middle_floor_v,uint16_t right_floor_v,uint16_t left_behind_v,uint16_t right_behind_v)
{
	right_wall = right_wall_v;
	left_floor = left_floor_v;
	left_middle_floor = left_middle_floor_v;
	right_middle_floor = right_middle_floor_v;
	right_floor = right_floor_v;
	left_behind = left_behind_v;
	right_behind = right_behind_v;
}

void get_inspection(uint16_t* right_wall_v,uint16_t* left_floor_v,uint16_t* left_middle_floor_v,uint16_t* right_middle_floor_v,uint16_t* right_floor_v,uint16_t* left_behind_v,uint16_t* right_behind_v)
{
	*right_wall_v = right_wall;
	*left_floor_v = left_floor;
	*left_middle_floor_v = left_middle_floor;
	*right_middle_floor_v = right_middle_floor;
	*right_floor_v = right_floor;
	*left_behind_v = left_behind;
	*right_behind_v = right_behind;
}

static float l_speed = 0;	//左轮速度值，单位m/s
static float r_speed = 0;	//右轮速度值，单位m/s

void set_wheel_speed(float l_speed_data,float r_speed_data)
{
	l_speed = l_speed_data;
	r_speed = r_speed_data;
}

void get_wheel_speed(float *l_speed_data,float *r_speed_data)
{
	*l_speed_data = l_speed;
	*r_speed_data = r_speed;
}

static int8_t station_out_of_contact;

void set_station_out_of_contact(int8_t sensor_data)
{
	station_out_of_contact = sensor_data;
}

int8_t get_station_out_of_contact(void)
{
	return station_out_of_contact;
}

static int8_t station_collect_dust;

void set_station_collect_dust(int8_t sensor_data)
{
	station_collect_dust = sensor_data;
}

int8_t get_station_collect_dust(void)
{
	return station_collect_dust;
}

static int8_t station_supply_water;

void set_station_supply_water(int8_t sensor_data)
{
	station_supply_water = sensor_data;
}

int8_t get_station_supply_water(void)
{
	return station_supply_water;
}

static int8_t station_air_dry;

void set_station_air_dry(int8_t sensor_data)
{
	station_air_dry = sensor_data;
}

int8_t get_station_air_dry(void)
{
	return station_air_dry;
}

static int8_t station_self_clean;

void set_station_self_clean(int8_t sensor_data)
{
	station_self_clean = sensor_data;
}

int8_t get_station_self_clean(void)
{
	return station_self_clean;
}

static int8_t battery_temperature;

void set_battery_temperature(int8_t sensor_data)
{
	battery_temperature = sensor_data;
}

int8_t get_battery_temperature(void)
{
	return battery_temperature;
}

static int8_t station_collect_dust_sensor;

void set_station_collect_dust_sensor(int8_t sensor_data)
{
	station_collect_dust_sensor = sensor_data;
}

int8_t get_station_collect_dust_sensor(void)
{
	return station_collect_dust_sensor;
}

static int8_t station_supply_water_sensor;

void set_station_supply_water_sensor(int8_t sensor_data)
{
	station_supply_water_sensor = sensor_data;
}

int8_t get_station_supply_water_sensor(void)
{
	return station_supply_water_sensor;
}

static int8_t station_status;

void set_station_status(int8_t sensor_data)
{
	station_status = sensor_data;
}

int8_t get_station_status(void)
{
	return station_status;
}


float u8_to_float(uint8_t* buffer)
{
	union U8_TO_FLOAT temp;
	for(int i = 0;i < 4;i ++)
	{
		temp.u8[i] = buffer[i];
	}
	return temp.f;
}

void float_to_u8(uint8_t* buffer,float value)
{
	union U8_TO_FLOAT temp;
	temp.f = value;
}

uint16_t data0_analysis(BitMask0_t sensor_flag, uint8_t *buffer, uint16_t index)
{
	float Accel_x, Accel_y, Accel_z;
	float Gyro_x, Gyro_y, Gyro_z;
	float pitch, roll, yaw;
	float x_position, y_position, theta;
	int16_t left_wheel_pulse, right_wheel_pulse;
	uint16_t left_wheel_current, right_wheel_current;

	switch(sensor_flag){
		case TOF_BIT:
			set_tof_data(buffer[index]);
			index += 1;
			break;
		case WHEEL_CURRENT_BIT:
			left_wheel_current = (buffer[index++] << 8) | buffer[index++];
			right_wheel_current = (buffer[index++] << 8) | buffer[index++];
			set_wheel_current(left_wheel_current,right_wheel_current);
			break;
		case STATION_SIGNAL_BIT:
			set_station_signal(buffer[index],buffer[index+1],buffer[index+2],buffer[index+3],buffer[index+4],buffer[index+5],buffer[index+6],buffer[index+7]);
			index += 8;
			break;
		case IMU_DATA_BIT:
			Accel_x = u8_to_float(&buffer[index]);
			index += 4;
			Accel_y = u8_to_float(&buffer[index]);
			index += 4;
			Accel_z = u8_to_float(&buffer[index]);
			index += 4;
			Gyro_x = u8_to_float(&buffer[index]);
			index += 4;
			Gyro_y = u8_to_float(&buffer[index]);
			index += 4;
			Gyro_z = u8_to_float(&buffer[index]);
			index += 4;
			set_IMU_ACC_linear(Accel_x,Accel_y,Accel_z);
			set_IMU_ACC_angle(Gyro_x,Gyro_y,Gyro_z);
			break;
		case IMU_EULAR_BIT:
			pitch = u8_to_float(&buffer[index]);
			index += 4;
			roll = u8_to_float(&buffer[index]);
			index += 4;
			yaw = u8_to_float(&buffer[index]);
			index += 4;
			set_IMU_Eular(pitch,roll,yaw);
			break;
		case ENCODE_DISK_BIT:
			left_wheel_pulse = (buffer[index++] << 8) | (buffer[index++] << 0);
			right_wheel_pulse = (buffer[index++] << 8) | (buffer[index++] << 0);
			set_wheel_pulse(left_wheel_pulse,right_wheel_pulse);
			break;
		case FUSION_POSE_BIT:
			x_position = u8_to_float(&buffer[index]);
			index += 4;
			y_position = u8_to_float(&buffer[index]);
			index += 4;
			theta = u8_to_float(&buffer[index]);
			index += 4;
			set_fusion_pose(x_position, y_position, theta);
			break;								
		default :
			RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"BitMask0_t error"); 
	}
	return index;
}

uint16_t data1_analysis(BitMask1_t sensor_flag, uint8_t *buffer, uint16_t index)
{
	float l_speed, r_speed;
	uint16_t right_wall,left_floor,left_middle_floor,right_middle_floor,right_floor,left_behind,right_behind;
	uint16_t left_floor_calibration,left_middle_floor_calibration,right_middle_floor_calibration,right_floor_calibration,left_behind_calibration,right_behind_calibration,lidar_behind_calibration;

	uint64_t time_ses,time_nsec;

	switch(sensor_flag){
		case SYSTEM_TIME_BIT:
			time_ses = (buffer[index++] << 56) | (buffer[index++] << 48) | (buffer[index++] << 40) | (buffer[index++] << 32) | 
					   (buffer[index++] << 24) | (buffer[index++] << 16) | (buffer[index++] << 8) | (buffer[index++] << 0);
			time_nsec = (buffer[index++] << 56) | (buffer[index++] << 48) | (buffer[index++] << 40) | (buffer[index++] << 32) | 
					    (buffer[index++] << 24) | (buffer[index++] << 16) | (buffer[index++] << 8) | (buffer[index++] << 0);
			set_system_time(time_ses,time_nsec);
			break;
		case INFRARED_WALL_BIT:
			right_wall = (buffer[index++] << 8) | buffer[index++];
			
			left_floor = (buffer[index++] << 8) | buffer[index++];
			left_middle_floor = (buffer[index++] << 8) | buffer[index++];
			right_middle_floor = (buffer[index++] << 8) | buffer[index++];
			right_floor = (buffer[index++] << 8) | buffer[index++];
			left_behind = (buffer[index++] << 8) | buffer[index++];
			right_behind = (buffer[index++] << 8) | buffer[index++];

			left_floor_calibration = (buffer[index++] << 8) | buffer[index++];
			left_middle_floor_calibration = (buffer[index++] << 8) | buffer[index++];
			right_middle_floor_calibration = (buffer[index++] << 8) | buffer[index++];
			right_floor_calibration = (buffer[index++] << 8) | buffer[index++];
			left_behind_calibration = (buffer[index++] << 8) | buffer[index++];
			right_behind_calibration = (buffer[index++] << 8) | buffer[index++];

			lidar_behind_calibration = (buffer[index++] << 8) | buffer[index++];

			set_inspection(right_wall,left_floor,left_middle_floor,right_middle_floor,right_floor,left_behind,right_behind);
			break;
		case WHEEL_SPEED_BIT:
			l_speed = u8_to_float(&buffer[index]);
			index += 4;
			r_speed = u8_to_float(&buffer[index]);
			index += 4;
			set_wheel_speed(l_speed, r_speed);
			break;
		default :
			RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"BitMask1_t error"); 
	}
	return index;	
}

void ReportTimingSensor_analysis(uint8_t *sysParseCmdBuf,uint16_t sysCmdLen)
{
	uint16_t index = 0;
	uint8_t BitMask0 = sysParseCmdBuf[8];
	uint8_t BitMask1 = sysParseCmdBuf[9];
	uint8_t *buffer = &sysParseCmdBuf[10];
	
	for(BitMask0_t sensor_bit = 0; sensor_bit < BITS0_MAX; sensor_bit++)
	{
		if(BitMask0 & (1 << sensor_bit))
		{
			index = data0_analysis(sensor_bit,buffer,index);
		}
	}

	if(0 == (BitMask0 & (1 << 7)))		//位掩码1有效位
	{
		return;
	}

	for(BitMask1_t sensor_bit = 0; sensor_bit < BITS1_MAX; sensor_bit++)
	{
		if(BitMask1 & (1 << sensor_bit))
		{
			index = data1_analysis(sensor_bit,buffer,index);
		}
	}
	

}

/**************************************************************************
 * 函数名: char* xrobot_memstr(char* full_data, int full_data_len, char* substr)
 * 描述  : 解析时删掉重复的转义
 * 形式参数 : char* full_data, int full_data_len, char* substr
 *
 * 返回值 : char 地址
 *************************************************************************/
uint8_t* xrobot_memstr(uint8_t* full_data, uint32_t full_data_len, unsigned char* substr)
{
	uint32_t i; 	
	uint32_t sublen = strlen(substr);  
	uint8_t* cur = full_data;  
	int last_possible = full_data_len - sublen + 1;  

	if(full_data == NULL || full_data_len <= 0 || substr == NULL){  
		return NULL;  
	}  
	if(*substr == '\0'){ 
		return NULL;   
	}    
	for(i = 0; i < last_possible; i++)
	{
		if(*cur == *substr)
		{
			if(memcmp(cur, substr, sublen) == 0)
			{   
				return cur;
			}       
		}       
		cur++;   
	}  
	return NULL;
}

uint16_t CRC16_CCITT_FALSE(uint8_t *puchMsg, uint32_t usDataLen)
{
	unsigned short wCRCin = 0xFFFF; //初值 0xFFFF
	unsigned short wCPoly = 0x1021; //多项式 x16+x12+x5+1
	unsigned char wChar = 0;
	while(usDataLen--)
	{
		wChar = *(puchMsg++);
		wCRCin ^= (wChar <<8);
		for(int i= 0;i < 8;i++)
		{
			if(wCRCin & 0x8000)
			{
				wCRCin = (wCRCin << 1)^ wCPoly;
			} 
			else
			{
				wCRCin = wCRCin << 1;
			}
		}
	} 
	return wCRCin;
}

/*****************************************************************************
 函 数 名: Comm_cmdParse
 功能描述  : 解析循环buff中的数据，获取一帧完整的数据
 输入参数  : void  
 输出参数  : 无
 返 回 值: 
 调用函数  : 
 被调函数  : 
*****************************************************************************/
uint16_t Comm_cmdParse()
{	
	uint16_t dataLength = 0;
	uint16_t frameLength = 0;
	uint8_t *phead = NULL;
	uint8_t parse_buf[SYS_CMD_SIZE]= {0};
	uint8_t cmd_buf[SYS_CMD_SIZE]= {0};
	uint16_t cnt  = 0,CRC_CHECK = 0;
	uint32_t buffer_tail = 0;
	buffer_tail = commParmInfo.tail;

	if(buffer_tail >= commParmInfo.head)
	{
		memset(parse_buf,0,sizeof(parse_buf));
		commParmInfo.len = buffer_tail - commParmInfo.head;
		if(commParmInfo.len < SYS_CMD_MIN)return 0;
		memcpy(parse_buf,&commParmInfo.buffer[commParmInfo.head],commParmInfo.len);

		cnt = 0;
		while((phead = xrobot_memstr(&parse_buf[cnt],(commParmInfo.len - cnt),messageSF)) != NULL)
		{
			cnt = phead - parse_buf;										//计算命令头的偏移地址	

			if(commParmInfo.len - cnt < SYS_CMD_MIN)
			{		
				commParmInfo.head += cnt;									//跳过无用数据、提高解析效率
				if(commParmInfo.head >= SYS_CMD_SIZE) 
				{
					commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
				}							
				break;
			}
			else
			{
				frameLength = (parse_buf[cnt+2]<<8)|parse_buf[cnt+3];		//帧长度/包体长度
				dataLength = frameLength + 4;								//整帧数据长度
				if(frameLength <= (commParmInfo.len - cnt - 4))				//表示有一帧完整的数据、减去头尾、至少是长度满足
				{
					memcpy(cmd_buf,&parse_buf[cnt],dataLength);	
					
					if(0x5A5A != ((cmd_buf[dataLength - 2]<<8)|cmd_buf[dataLength - 1]))	//判断帧尾
					{
#ifdef DEBUG
						RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"tail_err1 ================== 0x%x,  %d",(cmd_buf[dataLength - 2]<<8)|cmd_buf[dataLength - 1],cnt);	
#endif			
						commParmInfo.head += cnt + 2;									//跳过头
						if(commParmInfo.head >= SYS_CMD_SIZE) 
						{
							commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
						}							
						break;								
					}
					else
					{
						CRC_CHECK = CRC16_CCITT_FALSE(cmd_buf + 2,frameLength - 2);
						
						if(((cmd_buf[dataLength - 4]<<8)|cmd_buf[dataLength - 3])!= CRC_CHECK)
						{
			#ifdef DEBUG
	
							RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"CRC_Err_1 ================== %x",CRC_CHECK);
			#endif	
							commParmInfo.head += cnt + 2;									//跳过整帧数据
							if(commParmInfo.head >= SYS_CMD_SIZE) 
							{
								commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
							}							
							break;									
						}
						else														//获得正确数据帧
						{						
							Comm_cmdExecute(cmd_buf,dataLength);
							commParmInfo.head += cnt + dataLength;									//
							if(commParmInfo.head >= SYS_CMD_SIZE) 
							{
								commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
							}
							break;															
						}
					}													
				}
				else
				{
					
					commParmInfo.head += cnt;									//跳过无用数据、提高解析效率
					if(commParmInfo.head >= SYS_CMD_SIZE) 
					{
						commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
					}							
					break;							
				}										
			}					
		}
	}
	else
	{
		memset(parse_buf,0,sizeof(parse_buf));
		commParmInfo.len = SYS_CMD_SIZE - commParmInfo.head;
		if(commParmInfo.len + buffer_tail < SYS_CMD_MIN)return 0;
		memcpy(parse_buf,&commParmInfo.buffer[commParmInfo.head],commParmInfo.len);
		memcpy(&parse_buf[commParmInfo.len],commParmInfo.buffer,buffer_tail);		
	
		commParmInfo.len += buffer_tail;
		cnt = 0;

		while((phead = xrobot_memstr(&parse_buf[cnt],(commParmInfo.len - cnt),messageSF)) != NULL)
		{
			cnt = phead - parse_buf;										//计算命令头的偏移地址	
			
			if(cnt > (commParmInfo.len - 4))
			{					
				commParmInfo.head += cnt;									//跳过无用数据、提高解析效率
				if(commParmInfo.head >= SYS_CMD_SIZE) 
				{
					commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
				}							
				break;																										
			}
			else
			{
				frameLength = (parse_buf[cnt+2]<<8)|parse_buf[cnt+3];		//帧长度/包体长度
				dataLength = frameLength + 4;								//整帧数据长度
				if(frameLength <= (commParmInfo.len - cnt - 4))				//表示有一帧完整的数据、减去头尾、至少是长度满足
				{
					memset(cmd_buf,0,sizeof(cmd_buf));
					memcpy(cmd_buf,&parse_buf[cnt],dataLength);	


					if(0x5A5A != ((cmd_buf[dataLength - 2]<<8)|cmd_buf[dataLength - 1]))	//判断帧尾
					{
#ifdef DEBUG							
						RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"tail_err2 ================== %x,%d",(cmd_buf[dataLength - 2]<<8)|cmd_buf[dataLength - 1],cnt);	
#endif							
						commParmInfo.head += cnt + 2;									//跳过头
						if(commParmInfo.head >= SYS_CMD_SIZE) 
						{
							commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
						}							
						break;								
					}
					else
					{
						CRC_CHECK = CRC16_CCITT_FALSE(cmd_buf + 2,frameLength - 2);
						
						if(((cmd_buf[dataLength - 4]<<8)|cmd_buf[dataLength - 3])!= CRC_CHECK)
						{
#ifdef DEBUG						
							RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"CRC_Err_2 ================== %x",CRC_CHECK);
#endif						
							commParmInfo.head += dataLength;									//跳过整帧数据
							if(commParmInfo.head >= SYS_CMD_SIZE) 
							{
								commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
							}							
							break;									
						}
						else														//获得正确数据帧
						{
							Comm_cmdExecute(cmd_buf,dataLength);
							if(dataLength >= (SYS_CMD_SIZE - commParmInfo.head))
							{
								commParmInfo.head = dataLength - (SYS_CMD_SIZE - commParmInfo.head);
								break;
							}
							else
							{
								commParmInfo.head += cnt + dataLength;									//
								if(commParmInfo.head >= SYS_CMD_SIZE) 
								{
									commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
								}							
								break;
							}							
						

						}
					}
				
				}
				else
				{
				
					commParmInfo.head += cnt;									//跳过无用数据、提高解析效率
					if(commParmInfo.head >= SYS_CMD_SIZE) 
					{
						commParmInfo.head = commParmInfo.head - SYS_CMD_SIZE;
					}							
					break;							
				}										
			}					
		}									
	}
	
	return 0;
}


/*****************************************************************************
 函 数 名: Comm_cmdExecute
 功能描述  : 获得一帧完整数据后，根据命令字获取帧内容
 输入参数  : void  
 输出参数  : 无
 返 回 值: 
 调用函数  : 
 被调函数  : 
*****************************************************************************/
void Comm_cmdExecute(uint8_t *sysParseCmdBuf,uint16_t sysCmdLen)
{
	uint8_t buffer[10];
	int j = 0;
	int i = 0;
	
	uint16_t cmd = 0;
	uint16_t offset = 0;
	uint16_t sysParseCmdData = 0;
	uint16_t invoke_id = 0;
	if(sysCmdLen < SYS_CMD_MIN)
		return;	

	sysParseCmdData = (uint16_t)((sysParseCmdBuf[FRAME_CMD_OFFSET]<<8)|sysParseCmdBuf[FRAME_CMD_OFFSET + 1]);	//获取命令字节	
	invoke_id = (uint16_t)((sysParseCmdBuf[FRAME_NUM_OFFSET]<<8)|sysParseCmdBuf[FRAME_NUM_OFFSET + 1]);		//获取帧序号

	switch(sysParseCmdData)
	{
		/*******************联机消息*************************/
		case eSerialReportAckOnLine:
			set_online_message_result(&sysParseCmdBuf[FRAME_DATA_OFFSET]);
			semaphore_post(OnlineMessage_e);
		break;

		case eSerialReportAckHeartbeat:
			set_heart_beat_result(sysParseCmdBuf[FRAME_DATA_OFFSET]);
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"eSerialReportAckHeartbeat:status = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);
			semaphore_post(HeatBeat_e);
		break;

		/*******************查询消息报文*********************/
		case eSerialReportAckDevicefifo:
			set_device_info_result(&sysParseCmdBuf[FRAME_DATA_OFFSET]);
			semaphore_post(DeviceInfo_e);
		break;
		
		case eSerialAckSensorEnableStatus:
			set_sensor_status_result(((sysParseCmdBuf[8] << 8) | sysParseCmdBuf[9]));
			semaphore_post(SensorStatus_e);
		break;
		
		case eSerialAckSensorData:
			set_inquiry_sensor_data_result(&sysParseCmdBuf[FRAME_DATA_OFFSET]);
		break;

		/*******************控制消息报文*********************/
		case eSerialAckCtrlCmd:
			cmd = (sysParseCmdBuf[9] << 8) + sysParseCmdBuf[10];
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"eSerialAckCtrlCmd:cmd = 0x%x,status = %d",cmd,sysParseCmdBuf[8]);
			set_control_message_result(cmd,sysParseCmdBuf[8]);
			
			if((cmd == eSerialSetSideMotorVelocityLevel) || (cmd == eSerialSetMainMotorVelocityLevel))
				semaphore_post(brushes_e);
			else if(cmd == eSerialSetMopMotorVelocityLevel)
				semaphore_post(MopMotor_e);
			else if(cmd == eSerialSetSensorOpenStatus)
				semaphore_post(SensorEnabling_e);
			else if(cmd == eSerialSetPwrOff)
				semaphore_post(PowerOff_e);
			else if(cmd == eSerialSetWifiLedDisplay)
				semaphore_post(WifiLedDisplay_e);
			else if(cmd == eSerialSetCmaeraMotorLevel)
				semaphore_post(CameraMotor_e);
			else if(cmd == eSerialSetFanVelocityLevel)
				semaphore_post(FanFore_e);
			else if(cmd == eSerialSetCollectDust)
				semaphore_post(CollectDust_e);
			else if(cmd == eSerialSetSupplyWater)
				semaphore_post(SupplyWater_e);
			else if(cmd == eSerialSetCleanMop)
				semaphore_post(CleanMop_e);
			else if(cmd == eSerialSetAirDrying)
				semaphore_post(AirDrying_e);
			else if(cmd == eSerialSetCharging)
				semaphore_post(Charging_e);
			else if(cmd == eSerialSetSelfClean)
				semaphore_post(SelfClean_e);

		break;

		/*******************定时消息报文*********************/	
		case eSerialReportTimingSensor:
			ReportTimingSensor_analysis(sysParseCmdBuf,sysCmdLen);
			pub_semaphore_post();
		break;

		/*******************主动消息报文*********************/
		case eSerialReportBatteryDisplay:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"battery_quantity = %ld%%",sysParseCmdBuf[FRAME_DATA_OFFSET]);
			set_battery_quantity(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportBatteryDisplay >> 8;
            buffer[j++] = eSerialReportBatteryDisplay & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportMopStatus:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"mop_status = %ld%%",sysParseCmdBuf[FRAME_DATA_OFFSET]);
			set_mop_status(sysParseCmdBuf[FRAME_DATA_OFFSET]);
			
            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportBatteryDisplay >> 8;
            buffer[j++] = eSerialReportBatteryDisplay & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportChargeSignalData:		//接触充电座信号
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"charge_signal = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);
			set_charge_signal(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportChargeSignalData >> 8;
            buffer[j++] = eSerialReportChargeSignalData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportLowPowerSignalData:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"low_power_signal = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);
			set_low_power(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportLowPowerSignalData >> 8;
            buffer[j++] = eSerialReportLowPowerSignalData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportDropSensorData:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"drop_down_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);
			set_drop_down(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportDropSensorData >> 8;
            buffer[j++] = eSerialReportDropSensorData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportBumpSensorData:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"pressure_sensor_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_pressure_sensor(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportBumpSensorData >> 8;
            buffer[j++] = eSerialReportBumpSensorData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportLiftOffData:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"lift_off_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_lift_off(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportLiftOffData >> 8;
            buffer[j++] = eSerialReportLiftOffData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportDustBoxData:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"dust_box_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_dust_box(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportDustBoxData >> 8;
            buffer[j++] = eSerialReportDustBoxData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;


		case eSerialReportWaterBoxData:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"water_box_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_water_box(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportWaterBoxData >> 8;
            buffer[j++] = eSerialReportWaterBoxData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;
		
		case eSerialReportButtonData:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"button_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_button(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportButtonData >> 8;
            buffer[j++] = eSerialReportButtonData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportWallSensorData:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"wall_sensor_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_wall_sensor(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportWallSensorData >> 8;
            buffer[j++] = eSerialReportWallSensorData & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportErrorNum:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"err_num_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_err_num(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportErrorNum >> 8;
            buffer[j++] = eSerialReportErrorNum & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportMiddleCleanCurrent:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"middle_clean_current_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_middle_clean_current(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportMiddleCleanCurrent >> 8;
            buffer[j++] = eSerialReportMiddleCleanCurrent & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportMutipleSignals:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"multiple_signals_data = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_rocker_switch(sysParseCmdBuf[FRAME_DATA_OFFSET] & (1 << 0));
			set_water_level(sysParseCmdBuf[FRAME_DATA_OFFSET] & (1 << 1));
			set_rag_motor_current_signal(sysParseCmdBuf[FRAME_DATA_OFFSET] & (0x03 << 2));

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportMutipleSignals >> 8;
            buffer[j++] = eSerialReportMutipleSignals & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportFloorMaterial:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"floor_material = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);
			
			set_material(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportFloorMaterial >> 8;
            buffer[j++] = eSerialReportFloorMaterial & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationKey:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_key = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);
			
			set_station_key(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationKey >> 8;
            buffer[j++] = eSerialReportStationKey & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationOutOfContact:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_out_of_contact = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_station_out_of_contact(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationOutOfContact >> 8;
            buffer[j++] = eSerialReportStationOutOfContact & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationCollectDust:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_collect_dust = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_station_collect_dust(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationCollectDust >> 8;
            buffer[j++] = eSerialReportStationCollectDust & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationSupplyWater:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_supply_water = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_station_supply_water(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationSupplyWater >> 8;
            buffer[j++] = eSerialReportStationSupplyWater & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationAirDry:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_air_dry = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_station_air_dry(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationAirDry >> 8;
            buffer[j++] = eSerialReportStationAirDry & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationSelfClean:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_self_clean = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_station_self_clean(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationSelfClean >> 8;
            buffer[j++] = eSerialReportStationSelfClean & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportBatteryTemperature:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"battery_temperature = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_battery_temperature(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportBatteryTemperature >> 8;
            buffer[j++] = eSerialReportBatteryTemperature & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationCollectDustSensor:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_collect_dust_sensor = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_station_collect_dust_sensor(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationCollectDustSensor >> 8;
            buffer[j++] = eSerialReportStationCollectDustSensor & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationSupplyWaterSensor:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_supply_water_sensor = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_station_supply_water_sensor(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationSupplyWaterSensor >> 8;
            buffer[j++] = eSerialReportStationSupplyWaterSensor & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialReportStationStatus:
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"station_status = %d",sysParseCmdBuf[FRAME_DATA_OFFSET]);

			set_station_status(sysParseCmdBuf[FRAME_DATA_OFFSET]);

            buffer[j++] = 0x00;
            buffer[j++] = eSerialReportStationStatus >> 8;
            buffer[j++] = eSerialReportStationStatus & 0xFF;
            Comm_send_package(eSerialReportAckData, buffer, 3, invoke_id);
		break;

		case eSerialOTACmdUpgradeAck:
			cmd = (sysParseCmdBuf[10] << 8) + sysParseCmdBuf[11];
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"eSerialOTACmdUpgradeAck:cmd = 0x%x",cmd);
			set_OTA_message_result(sysParseCmdBuf[FRAME_DATA_OFFSET+1]);

			if(cmd == eSerialOTACmdUpgradeStart)
				semaphore_post(OTA_Upgrade_Start_e);
			else if(cmd == eSerialOTACmdUpgradeEnd)
				semaphore_post(OTA_Upgrade_End_e);
			else if(cmd == eSerialOTACmdUpgradeDataFrame)
				semaphore_post(OTA_Upgrade_Frame_e);
		break;

		default:
			break;
	}

}

/*****************************************************************************
 函 数 名: Comm_send_package
 功能描述  : 组包，根据不同的命令字，组包上传
 输入参数  : void  
 输出参数  : 无
 返 回 值: 
 调用函数  : 
 被调函数  : 
*****************************************************************************/
void Comm_send_package(TypeDefCmd frameCmd, uint8_t* buffer, uint16_t len,uint16_t frameCmdCounter)
{
	
	uint16_t check_sum = 0;
	uint16_t reported_len = 0;
	uint8_t  reported_buffer[200] = {0};
	
	reported_buffer[reported_len++]= 0xA5;	//帧头
	reported_buffer[reported_len++]= 0xA5;	
	
	reported_buffer[reported_len++]= 0;		//帧长度初值
	reported_buffer[reported_len++]= 0;
	reported_buffer[reported_len++]= (frameCmdCounter>>8)&0xff;	//帧序号
	reported_buffer[reported_len++]= (unsigned char )(frameCmdCounter&0x00ff);	
	
	reported_buffer[reported_len++]= (frameCmd>>8)&0xff;	//帧命令字
	reported_buffer[reported_len++]= (frameCmd)&0xff;	

	if(len != 0){
		memcpy(reported_buffer + 8, buffer, len);
		if(len == 0)
			return;
	}

	reported_len += len;	
	reported_buffer[2] = (uint8_t)((reported_len>>8)&0xFF);
	reported_buffer[3] = (uint8_t)(reported_len&0xFF);		//帧长度

	
	check_sum = CRC16_CCITT_FALSE(&reported_buffer[2],reported_len - 2);
	reported_buffer[reported_len++] = (check_sum>>8)&0xff;				//CRC校验
	reported_buffer[reported_len++] = (unsigned char )(check_sum&0x00ff);	
	
	reported_buffer[reported_len++]= 0x5A;			//帧尾
	reported_buffer[reported_len++]= 0x5A;	
	
	USART_SEND_FUN(reported_buffer, reported_len);
}

/*****************************************************************************
 函 数 名: USART_SEND_FUN
 功能描述  : 生成一帧完整数据后，平台发送
 输入参数  : void  
 输出参数  : 无
 返 回 值: 
 调用函数  : 
 被调函数  : 
*****************************************************************************/
void USART_SEND_FUN(uint8_t* buffer, uint16_t len)
{
	uart_write(buffer, len);
}

