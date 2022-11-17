// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <unistd.h>
#include <signal.h>

#include "recv_proc_task.hpp"

#include "ota.hpp"
#include "nodes_set.hpp"
#include "nodes_in_component.hpp"
#include "uart_synchronous.hpp"

#include "rclcpp/rclcpp.hpp"

void exit_handler(int signo)
{
    if (signo == SIGINT){
        fprintf(stderr,"Ctrl+C,quit !\n");
        fflush(stderr);
        rclcpp::shutdown();
        exit(0);
    }
}

void InitSignal(void)
{
    struct sigaction sa,osa;
    sa.sa_handler = exit_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    int ret = sigaction(SIGINT,&sa,&osa);
    if(ret <0){
        printf("set signal ctrl+c failure!\n");
    }
}

/*
usage:
  ./sensor_app + robot_app.bin ota_count /dev/ttyUSB0
*/

int main(int argc, char * argv[])
{
    InitSignal();

    if(argc != 5)
    {
        printf("usage:./sensor_app + section + file_name + ota_count + /dev/ttyUSBx\n");
        return 0;
    }

    int32_t ota_dev = atoi(argv[1]);
    char *file_name = argv[2];
    int ota_count = atoi(argv[3]);
    char *uart_dev = argv[4];

    int fail_count = 0;

    rclcpp::init(argc, argv);

    recv_proc_init(uart_dev);

    for(int i = 0;i < ota_count;i++)
    {
        if(ota_task(file_name,ota_dev) == -1)
        {
            fail_count++;
        }
    }
    printf("ota_count = %d,fail_count = %d\n",ota_count,fail_count);

    rclcpp::shutdown();

    recv_proc_join();

    return 0;
}
