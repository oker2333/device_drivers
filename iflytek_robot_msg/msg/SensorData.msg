#沿边传感器
uint8 tof_data                   #离墙距离，单位mm

#左右轮电流
uint16 left_wheel_current       #左轮电流,单位:mA
uint16 right_wheel_current      #右轮电流,单位:mA

#回充座信号
uint8 station_signal_middle_left			    #中左接收
uint8 station_signal_middle_right			    #中右接收

#上报周期内的脉冲个数
int16 left_wheel_pulse                    #左轮脉冲数据
int16 right_wheel_pulse                   #右轮脉冲数据

#底盘系统时间
uint64 time_ses                                  #底盘开机后时间累计单位ses
uint64 time_nsec                                 #底盘开机后时间累计单位nsec

#左右轮速度数据
float32 l_speed                                 #左轮速度值，单位m/s
float32 r_speed                                 #右轮速度值，单位m/s

#电量
uint8 battery_quantity                           #0~100， 表示0%~100%

#地检
# bit0:左地检
# bit1:中左地检(左前地检)
# bit2:中右地检(右前地检)
# bit3:右地检
# bit4:左后地检
# bit5:右后地检
uint8 floor_detect_status

#碰撞
# bit0: 前撞左碰撞
# bit1: 前撞右碰撞
# bit2: 雷达盖左
# bit3: 雷达盖右
# bit4: 雷达盖前
# bit5: 雷达盖顶
uint8 crash_data

#离地传感器
# bit0: 左轮离地
# bit1: 右轮离地
uint8 lift_off

#尘盒传感器
# bit0: 为1 尘盒取出， 为0 尘盒放回
uint8 dust_box_status

#按键上报
# 0x01：清扫键
# 0x02：回充键
# 0x03：长按清扫键
# 0x04：长按回充键
# 0x05：长按清扫+回充键
uint8 button

#墙检信号
# bit0: 为1 墙检触发， 为0 墙检不触发
uint8 wall_detect_status

#超声波材质传感器
# 0:地面 1:地毯
int8 material

# 错误信号
uint64 error_signal

#接触充电座信号
# Bit0:是否接触充电座
# Bit1:是否是充电状态
# Bit2:是否充电完成
uint8 charge_status

#机器低电量信号
# Bit0:是否低电回充信号
# Bit1:是否低电关机信号
uint8 low_power_signal

#左右托盘变化
#1:取下，0:安装
# bit0:左托盘
# bit1:右托盘
int8 mop_status

#水箱水位检测
# 0是水箱里无水信号
# 1是水箱里有水信号
uint8 water_level_signal

#抹布电机电流信号
# 00是抹布电机无电流
# 01是抹布电机电流正常
# 10 是抹布电机过流
# 11是抹布电机堵转
uint8 rag_motor_current_signal

#温度传感器
float32 temperature1    #温度传感器1(电池温度)
float32 temperature2    #温度传感器2(主板温度)
float32 temperature3    #温度传感器3

#基站按键
#0x01:召回/出仓按键事件
#0x02:开始/暂停按键事件
#0x03:基站清洁按键事件
#0x04:模式切换按键事件
int8 station_key

#基站水箱状态
#手动上下水水箱状态
#Bit7：清水箱不在位
#Bit6：清水箱空
#Bit5：污水箱不在位
#Bit4：污水箱满
#自动上下水水箱状态
#Bit3：清水箱不在位
#Bit2：清水箱空
#Bit1：污水箱不在位
#Bit0：污水箱满
int8 station_water_box

#集尘状态
#0x00无异常
#0x01扫地机被取出
#0x02尘袋取出
#0x03尘盒取出
#0x04频繁集尘
#0x05尘满
#0x06风道堵塞
int8 station_collect_dust

#补水状态
#0x00无异常
#0x01扫地机被取出
#0x02清洗槽被取出
#0x03清水箱被取出
#0x04清水箱空
#0x05污水箱被取出
#0x06污水箱满
#0x07注水泵异常
#0x08回收泵异常
int8 station_supply_water

#风干状态
#0x00无异常
#0x01风机开路
#0x02风机短路
#0x03温度过高
int8 station_air_drying

#自清洁状态
#0x00无异常
#0x01清水泵异常
#0x02污水泵异常
int8 station_self_clean
