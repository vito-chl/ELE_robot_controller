#include "ctrlinfo.h"
#include "motor.h"

#define MOTRO1_ADDR 0x11
#define MOTRO2_ADDR 0x12

#define FORWARD 0
#define BACKWARD 1

static uint8_t move_direction=FORWARD;
static int16_t move_speed=0;
static int32_t move_distance=0;
static int move_mode = UNSET_MODE;

#define BUF_DEFAULT {0x3A, 0x01, 0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xCC, 0xCC, 0x0D, 0x0A}

uint8_t return_data_buf[12] = BUF_DEFAULT;
uint8_t return_data_bufsize = 12;

/*  机器人处理函数 */

static void mode_change(int mode)
{
	if(move_mode == UNSET_MODE)
	{
		motor_init(0x11);
		motor_init(0x12);
	}
	
	if(mode == move_mode)
		return;
	
	else if(mode == SPEED_MODE)
	{
		move_mode = SPEED_MODE;
		motor_set_mode(0x11, SPEED_MODE);
		motor_set_mode(0x12, SPEED_MODE);
	}
	else if(mode == DISTANCE_MODE)
	{
		move_mode = DISTANCE_MODE;
		motor_set_mode(0x11, DISTANCE_MODE);
		motor_set_mode(0x12, DISTANCE_MODE);
	}
}

static int set_forward_speed(uint8_t dev_addr ,uint32_t data)
{
	mode_change(SPEED_MODE);
	move_direction = FORWARD;
	move_speed = (int16_t)data;
	set_speed(0x11,move_speed);
	set_speed(0x12,move_speed);
	
	return 0;
}

static int set_backward_speed(uint8_t dev_addr ,uint32_t data)
{
	mode_change(SPEED_MODE);
	move_direction = BACKWARD;
	move_speed = (int16_t)data;
	set_speed(0x11,-move_speed);
	set_speed(0x12,-move_speed);
	
	return 0;
}

static int move_start(uint8_t dev_addr ,uint32_t data)
{
	if(move_mode == SPEED_MODE)
	{
		uint8_t cmd[] = ENABLD_MOTOR_CMD;
		ctrl_motor_noret(0x11, cmd, sizeof(cmd));
		ctrl_motor_noret(0x12, cmd, sizeof(cmd));
	}
	if(move_mode == DISTANCE_MODE)
	{
		uint8_t cmd[] = START_MOVE_CMD;
		ctrl_motor_noret(0x11, cmd, sizeof(cmd));
		ctrl_motor_noret(0x12, cmd, sizeof(cmd));
	}
	return 0;
}

static int move_stop(uint8_t dev_addr ,uint32_t data)
{
	if(move_mode == SPEED_MODE)
	{
		uint8_t cmd[] = DISABLD_MOTOR_CMD;
		ctrl_motor_noret(0x11, cmd, sizeof(cmd));
		ctrl_motor_noret(0x12, cmd, sizeof(cmd));
	}
	if(move_mode == DISTANCE_MODE)
	{
		uint8_t cmd[] = DISABLD_MOTOR_CMD;
		ctrl_motor_noret(0x11, cmd, sizeof(cmd));
		ctrl_motor_noret(0x12, cmd, sizeof(cmd));
	}
	return 0;
}

static int set_frond_distance(uint8_t dev_addr ,uint32_t data)
{
	mode_change(DISTANCE_MODE);
	move_direction = FORWARD;
	move_distance = (int32_t)data;
	set_distance(MOTRO1_ADDR, data);
	set_distance(MOTRO2_ADDR, data);
	
	return 0;
}

static int set_back_distance(uint8_t dev_addr ,uint32_t data)
{
	mode_change(DISTANCE_MODE);
	move_direction = BACKWARD;
	move_distance = (int32_t)data;
	set_distance(MOTRO1_ADDR, -data);
	set_distance(MOTRO2_ADDR, -data);
	
	return 0;
}

static int return_sensor_info(uint8_t dev_addr ,uint32_t data)
{
	return 0;
}

static int return_speed_from_motor(uint8_t dev_addr ,uint32_t data)
{
	int16_t speed = read_speed(dev_addr);
	speed = speed<0 ? (-speed):speed;
	return_data_buf[3] = 2;
	return_data_buf[4] = (uint8_t)(speed>>8);
	return_data_buf[5] = (uint8_t)speed;
	
	return 1;
}

static int return_speed_from_robot(uint8_t dev_addr ,uint32_t data)
{
	int16_t speed = move_speed;
	return_data_buf[3] = 2;
	return_data_buf[4] = (uint8_t)(speed>>8);
	return_data_buf[5] = (uint8_t)speed;
	
	return 1;
}

static int return_movdirc_from_motor(uint8_t dev_addr ,uint32_t data)
{
	int16_t speed = read_speed(dev_addr);
	if(speed == 0)
		return_data_buf[4] = 0x02;
	else if(speed > 0)
		return_data_buf[4] = 0x01;
	else
		return_data_buf[4] = 0x03;
		
	return_data_buf[3] = 1;

	return 1;
}

static int return_movdirc_from_robot(uint8_t dev_addr ,uint32_t data)
{
	if(move_speed == 0)
		return_data_buf[4] = 0x02;
	else if(move_speed > 0)
		return_data_buf[4] = 0x01;
	else
		return_data_buf[4] = 0x03;
		
	return_data_buf[3] = 1;

	return 1;
}

void funclist_init()
{

	/* 设置前进速度 */
	ctrl_funclist[SET_FORWAED_SPEED].func_num = 0x01;
	ctrl_funclist[SET_FORWAED_SPEED].func = set_forward_speed;
	
	/* 设置后退速度 */
	ctrl_funclist[SET_BACKWARD_SPEED].func_num = 0x02;
	ctrl_funclist[SET_BACKWARD_SPEED].func = set_backward_speed;
	
	/* 开始运动 */
	ctrl_funclist[MOVE_START].func_num = 0x04;
	ctrl_funclist[MOVE_START].func = move_start;
	
	/* 停止运动 */
	ctrl_funclist[MOVE_STOP].func_num = 0x03;
	ctrl_funclist[MOVE_STOP].func = move_stop;
	
	/* 设置前进距离 */
	ctrl_funclist[SET_FROWARD_DISTANCE].func_num = 0x05;
	ctrl_funclist[SET_FROWARD_DISTANCE].func = set_frond_distance;
	
	/* 设置后退距离 */
	ctrl_funclist[SET_BACKWARD_DISTANCE].func_num = 0x06;
	ctrl_funclist[SET_BACKWARD_DISTANCE].func = set_back_distance;
	
	/* 返回传感器信息 */
	ctrl_funclist[RETURN_SENSOR_INFO].func_num = 0x30;
	ctrl_funclist[RETURN_SENSOR_INFO].func = (data_handle_func)0;
	
	/* 返回速度 */
	ctrl_funclist[RETURN_SPEED].func_num = 0x31;
	ctrl_funclist[RETURN_SPEED].func = return_speed_from_robot;
	
	/* 返回移动方向 */
	ctrl_funclist[RETURN_MOVDIRC].func_num = 0x32;
	ctrl_funclist[RETURN_MOVDIRC].func = return_movdirc_from_robot;
	
	for(int i=0; i<=RETURN_MOVDIRC; i++)
	{
		ctrl_funclist[i].dev_addr = 0x01;
	}
}