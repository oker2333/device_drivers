#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "recv_proc_task.hpp"
#include "uart_ttys.hpp"
#include "comm_message.hpp"

#define BUFFER_LEN 512
#define UART_READ_SIZE    20

static pthread_t recv_proc_id;

static void* recv_proc_task(void *arg)
{
    uint32_t len;
    uint16_t read_len;
    uint8_t* buffer = NULL;
    uint32_t buffer_unused = 0;

    uint8_t buffer_s[12] = {0xff,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    Comm_send_package(eSerialSetTimingReportParam, buffer_s, sizeof(buffer_s)/sizeof(uint8_t), find_free_invoke_id());

    Comm_send_package(eSerialInquirySensorData, NULL, 0, find_free_invoke_id());    //开机后主动查询传感器

    while(rclcpp::ok())
    {
        buffer_unused = SYS_CMD_SIZE - commParmInfo.tail;

        len = (UART_READ_SIZE >= buffer_unused)?buffer_unused:UART_READ_SIZE;
        read_len = uart_read(&commParmInfo.buffer[commParmInfo.tail],len);
        if(read_len <= 0)
            continue;

        buffer = &commParmInfo.buffer[commParmInfo.tail];

        commParmInfo.tail += read_len;
        if(commParmInfo.tail >= SYS_CMD_SIZE)
            commParmInfo.tail = 0;
        Comm_cmdParse();
    }

    uart_close();

    return arg;
}

void recv_proc_init(void)
{
    int ret = uart_init();
    pub_sem_init();
    if(ret == 0)
        pthread_create(&recv_proc_id,NULL,recv_proc_task,NULL);
    else
        exit(1);
}

void recv_proc_join(void)
{
    pthread_join(recv_proc_id,NULL);
}

