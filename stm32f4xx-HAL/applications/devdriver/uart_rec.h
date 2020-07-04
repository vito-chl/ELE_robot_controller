#ifndef _UART_REC_H
#define _UART_REC_H

#include"rtthread.h"
#include"rtdevice.h"

#define INFOBUF_MAX_LEN 64

typedef struct UART_REC
{
	
	char* uart_name;
	rt_device_t dev;
	
	struct rt_semaphore rx_sem;
	struct rt_semaphore start_rec_sem;
	struct rt_semaphore finish_rec_sem;
	
	uint8_t rec_databuf[64];
	uint8_t rec_databytes;
}uart_rec_t;



#endif


