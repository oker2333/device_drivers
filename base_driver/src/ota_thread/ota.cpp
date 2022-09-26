#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "ota.hpp"
#include "rclcpp/rclcpp.hpp"
#include "comm_message.hpp"

#define HOST 0
#define STATION 1

#define HOST_FRAME_MAX_LEN 128
#define STATION_FRAME_MAX_LEN 16

static pthread_t ota_id;

int GetFileSize(char *_pName) 
{
   int iFd = -1;
   int  iLen = 0;
   if (_pName == NULL)
  {
     return -1;
  }
  iFd = open(_pName, O_RDONLY);
  if (iFd >= 0)
  {
     iLen = lseek(iFd, 0, SEEK_END); 
     close(iFd);
    return iLen;
  }
 
 return iFd;
}

#define CMD_BUFFER_LEN 200

static void* ota_task(void *arg)
{
    sleep(3);
    int64_t read_bytes = 0;
    int64_t read_counter = 0;
    uint8_t cmd_buffer[CMD_BUFFER_LEN];
    
    int64_t file_size = GetFileSize(arg);
    if(file_size <= 0){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"get %s file size failed",arg);
        return NULL;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"now start to download %s,file size = %ld bytes",arg,file_size);

    uint8_t *ota_buff = malloc(file_size);
    if(ota_buff == NULL)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"ota_buff malloc failed");
        return NULL;
    }

    int fd = open(arg,O_RDONLY);
    if(fd < 0){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"open %s failed",arg);
    }

    while(read_bytes = read(fd,&ota_buff[read_counter],file_size))
    {
        if(read_bytes < 0)
            continue;
        read_counter += read_bytes;
        if(read_counter == file_size){
            break;
        }else if(read_counter > file_size){
            return NULL;
        }
    }
    close(fd);

    //开始升级命令
    uint8_t result = 0;
    uint8_t ota_device = HOST;
    uint16_t CRC16 = CRC16_CCITT_FALSE(ota_buff,file_size);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%s CRC16 = 0x%x",arg,CRC16);

    cmd_buffer[0] = ota_device;
    cmd_buffer[1] = (CRC16 >> 8) & 0xff;
    cmd_buffer[2] = (CRC16 >> 0) & 0xff;
    if(!datalink_frame_send(eSerialOTACmdUpgradeStart,OTA_Upgrade_Start_e,cmd_buffer,3)){
        return NULL;
    }

    uint16_t frame_index = 0;
    uint32_t offset = 0;
    uint32_t remainder = file_size;
    uint16_t frame_max_len = HOST_FRAME_MAX_LEN;
    uint16_t frame_len = frame_max_len;

    cmd_buffer[0] = ota_device;     /*升级类型*/

    while(remainder)
    {
        cmd_buffer[1] = (uint8_t)(offset >> 24) & 0xff;     /*数据序号*/
        cmd_buffer[2] = (uint8_t)(offset >> 16) & 0xff;
        cmd_buffer[3] = (uint8_t)(offset >> 8) & 0xff;
        cmd_buffer[4] = (uint8_t)(offset >> 0) & 0xff;

        cmd_buffer[5] = (uint8_t)(frame_len >> 8) & 0xff;   /*数据长度*/
        cmd_buffer[6] = (uint8_t)(frame_len >> 0) & 0xff;
        
        memcpy(&cmd_buffer[7],&ota_buff[offset],frame_len); /*数据内容*/
        
        if(datalink_frame_send(eSerialOTACmdUpgradeDataFrame,OTA_Upgrade_Frame_e,cmd_buffer,frame_len+1+4+2))
        {
            if(get_OTA_message_result())
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"[eSerialOTACmdUpgradeDataFrame] error num = %d",get_OTA_message_result());
                continue;
            }
        }
        else
        {
            return NULL;
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[frame %d]%d bytes transmitted",frame_index++,frame_len);

        offset += frame_len;
        remainder = remainder - frame_len;
        if(remainder < frame_max_len)
        {
            frame_len = remainder;
        }else{
            frame_len = frame_max_len;
        }
    }

    if(ota_buff != NULL)
    {
        free(ota_buff);
        ota_buff = NULL;
    }

    /*升级完成命令*/
    cmd_buffer[0] = ota_device;
    cmd_buffer[1] = 1;
    datalink_frame_send(eSerialOTACmdUpgradeEnd,OTA_Upgrade_End_e,cmd_buffer,2);
}

void ota_init(void)
{
    pthread_create(&ota_id,NULL,ota_task,"gd32vct6.bin");
}

void ota_deinit(void)
{
    pthread_join(ota_id,NULL);
}