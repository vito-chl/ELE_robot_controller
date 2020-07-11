#ifndef _CTRLINFO_H
#define _CTRLINFO_H

#include"rtthread.h"
#include"rtdevice.h"
#include"common.h"

#define FUNCLIST_SIZE 50

struct _data_handle;
typedef struct _data_handle data_handle_t;

typedef int (*data_handle_func)(uint8_t dev_addr ,uint32_t data);

struct _data_handle{
	data_handle_func func;
	
	uint8_t dev_addr;
	
	uint8_t func_num;
};


#define SET_FORWAED_SPEED 			0
#define SET_BACKWARD_SPEED			1
#define MOVE_START 					2
#define MOVE_STOP					3
#define SET_FROWARD_DISTANCE		4
#define SET_BACKWARD_DISTANCE 		5
#define RETURN_SENSOR_INFO			6
#define RETURN_SPEED 				7
#define RETURN_MOVDIRC				8

extern data_handle_t ctrl_funclist[];

#endif
