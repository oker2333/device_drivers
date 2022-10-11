#ifndef OTA_HPP_
#define OTA_HPP_

#define HOST 0
#define STATION 1

#define NONE_OTA    0
#define MCU_OTA     1
#define STATION_OTA 2

void ota_task(char *file_name,int32_t ota_dev);

void set_ota_status(int32_t status);
int32_t get_ota_status(void);

#endif