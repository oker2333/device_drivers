#ifndef OTA_HPP_
#define OTA_HPP_

#define BOOT 0
#define APP  1

int ota_task(char *file_name,int32_t ota_dev);

#endif