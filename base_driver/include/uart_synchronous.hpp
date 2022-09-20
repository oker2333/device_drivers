#ifndef UART_SYNCHRONOUS_HPP_
#define UART_SYNCHRONOUS_HPP_

#include <stdint.h>
#include <stdbool.h>

#define TIMEOUT_MS 200  //20ms超时重传
#define RYTEIES 3

typedef enum workday
{
    pressure_sensor_e,
    brushes_e,
    clean_dust_motor_e,
    MopMotor_e,
    main_motion_motor_e,
    PumpMotor_e,
    TimingReportParam_e,
    SensorReset_e,
    WifiLedDisplay_e,
    FanFore_e,
    CollectDust_e,
    DeviceInfo_e,
    SensorStatus_e,
    CameraMotor_e,
    Charging_e,
    SelfClean_e,
    InquirySensorData_e,
    AirDrying_e,
    PowerOff_e,
    OnlineMessage_e,
    SensorEnabling_e,
    HeatBeat_e,
    CleanMop_e,
    OTA_Upgrade_Start_e,
    OTA_Upgrade_Status_e,
    OTA_Upgrade_Frame_e,
    SupplyWater_e,
    publish_immediately_e,
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
