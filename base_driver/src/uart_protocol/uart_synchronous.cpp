#include <stdio.h>
#include <sys/time.h>
#include <semaphore.h>
#include <pthread.h>

#include "uart_synchronous.hpp"
#include "rclcpp/rclcpp.hpp"

static pthread_mutex_t invoke_id_mutex = PTHREAD_MUTEX_INITIALIZER;

#define invoke_id_lock() pthread_mutex_lock(&invoke_id_mutex)
#define invoke_id_unlock() pthread_mutex_unlock(&invoke_id_mutex)

static pthread_mutex_t semaphore_mutex = PTHREAD_MUTEX_INITIALIZER;

#define semaphore_lock() pthread_mutex_lock(&semaphore_mutex)
#define semaphore_unlock() pthread_mutex_unlock(&semaphore_mutex)

//200ms超时重传
#define TIMEOUT_US (TIMEOUT_MS*1000)

typedef struct{
	bool occupied;
	sem_t semaphore;
}Sem_Info_t;

static Sem_Info_t sem_pool[sensor_num_e];

//用于阻塞读/写时，申请信号量
static sem_t* get_semaphore(Sensor_Id_t id)
{
	sem_t* sem_ptr = NULL;
	semaphore_lock();
	if(sem_pool[id].occupied == false)
	{
		sem_pool[id].occupied = true;
		sem_ptr = &sem_pool[id].semaphore;
		sem_init(sem_ptr,0,0);		//此信号量为当前进程的所有线程共享
	}
	semaphore_unlock();
	return sem_ptr;
}

//串口接收到数据，用于搜索对应invoke_id的信号量
static sem_t* search_semaphore(Sensor_Id_t id)
{
	sem_t* sem_ptr = NULL;
	semaphore_lock();
	if(sem_pool[id].occupied == true)
	{
		sem_ptr = &sem_pool[id].semaphore;
	}
	semaphore_unlock();
	return sem_ptr;
}

//在读取传感器数据，阻塞结束后，释放对应信号量
static void free_semaphore(Sensor_Id_t id)
{
	semaphore_lock();
	sem_pool[id].occupied = false;
	semaphore_unlock();
}

bool semaphore_timed_wait(Sensor_Id_t id)
{
	sem_t* sem_ptr = NULL;
	
	struct timeval now;		//seconds && microseconds
	struct timespec out_time;	//seconds && nanoseconds
	
	sem_ptr = get_semaphore(id);
	if(!sem_ptr){
		RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"get_semaphore failed");
		return false;
	}

	gettimeofday(&now, NULL);
	out_time.tv_sec = now.tv_sec;
	out_time.tv_nsec = (now.tv_usec + TIMEOUT_US) * 1000;
	if(out_time.tv_nsec > 999999999L){	//防止时间溢出，sem_timedwait造成Invalid argument
		out_time.tv_sec++;
		out_time.tv_nsec = out_time.tv_nsec - 999999999L;
	}
	
	if(-1 == sem_timedwait(sem_ptr, &out_time)){
		sem_destroy(sem_ptr);
		free_semaphore(id);
		perror("sem_timedwait error");
		return false;
	}

	sem_destroy(sem_ptr);
	free_semaphore(id);

	return true;
}

bool semaphore_post(Sensor_Id_t id)
{
	sem_t* sem_ptr = search_semaphore(id);
	if(sem_ptr == NULL){
		RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"semaphore posted doesn't exist");
		return false;
	}
	
	sem_post(sem_ptr);
	
	return true;
}

static uint16_t invoke_id = 0;

uint16_t find_free_invoke_id(void)
{
	invoke_id_lock();
	invoke_id++;
	invoke_id_unlock();
    return invoke_id;
}

sem_t* pub_sem_ptr = NULL;

void pub_sem_init(void)
{
	static sem_t semaphore;
	pub_sem_ptr = &semaphore;
	sem_init(pub_sem_ptr,0,0);
}

void pub_semaphore_post(void)
{
	sem_post(pub_sem_ptr);
}

void pub_semaphore_wait(void)
{
	sem_wait(pub_sem_ptr);
}

