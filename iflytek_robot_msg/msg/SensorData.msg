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

#主机基站失联状态
#0：通信正常
#1：通信断开
int8 station_out_of_contact

#基站集尘状态
#0：空闲/正常执行
#1：集尘完成
#2：忙，当前流程暂不能执行
#3：指令异常
#4：异常结束
int8 station_collect_dust

#基站补水状态
#0：空闲/正常执行
#1：补水完成
#2：忙，当前流程暂不能执行
#3：指令异常
#4：异常结束
int8 station_supply_water

#基站风干状态
#0：空闲/正常执行
#1：风干完成
#2：忙，当前流程暂不能执行
#3：指令异常
#4：异常结束
int8 station_air_dry

#基站自清洁状态
#0：空闲/正常执行
#1：自清洁完成
#2：忙，当前流程暂不能执行
#3：指令异常
#4：异常结束
int8 station_self_clean

#基站集尘相关传感器
#Bit0：尘袋取出
#Bit1：尘盒打开
#Bit2：频繁集尘
#Bit3：尘袋满
#Bit4：风道堵塞
#Bit5：传感器故障
int8 station_collect_dust_sensor

#基站补水相关传感器
#Bit0：清水箱不在位
#Bit1：清水箱空
#Bit2：污水箱不在位
#Bit3：污水箱满
#Bit4：清水泵异常
#Bit5：污水泵异常
#Bit6：抽水泵异常
int8 station_supply_water_sensor

#基站其它状态
#Bit0：扫地机被取出
#Bit1：银离子检测
#Bit3：托盘检测
#Bit4：溢水检测
int8 station_status
