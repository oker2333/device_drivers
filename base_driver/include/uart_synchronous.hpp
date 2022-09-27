#ifndef UART_SYNCHRONOUS_HPP_
#define UART_SYNCHRONOUS_HPP_

#include <stdint.h>
#include <stdbool.h>

#define TIMEOUT_MS 200  //20ms超时重传
#define RYTEIES 3

typedef enum workday
{
    pressure_sensor_e = 0,
    brushes_e = 1,
    clean_dust_motor_e = 2,
    MopMotor_e = 3,
    main_motion_motor_e = 4,
    PumpMotor_e = 5,
    TimingReportParam_e = 6,
    SensorReset_e = 7,
    WifiLedDisplay_e = 8,
    FanFore_e = 9,
    CollectDust_e = 10,
    DeviceInfo_e = 11,
    SensorStatus_e = 12,
    CameraMotor_e = 13,
    Charging_e = 14,
    SelfClean_e = 15,
    InquirySensorData_e = 16,
    AirDrying_e = 17,
    PowerOff_e = 18,
    OnlineMessage_e = 19,
    SensorEnabling_e = 20,
    HeatBeat_e = 21,
    CleanMop_e = 22,
    OTA_Upgrade_Start_e = 23,    //0x0500
    OTA_Upgrade_End_e = 24,   //0x0502
    OTA_Upgrade_Frame_e = 25,    //0x0503
    SupplyWater_e = 26,
    publish_immediately_e = 27,
    sensor_num_e
} Sensor_Id_t; 

uint16_t find_free_invoke_id(void);

bool semaphore_timed_wait(Sensor_Id_t id);
bool semaphore_post(Sensor_Id_t id);

void pub_sem_init(void);
void pub_semaphore_post(void);
void pub_semaphore_wait(void);
void pub_semaphore_destroy(void);

#endif
