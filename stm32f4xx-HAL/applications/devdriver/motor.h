#ifndef _MOTOR_H
#define _MOTOR_H

#include"rtthread.h"
#include"rtdevice.h"

#define SET_OPTMODE_CMD {0x00, 0x06, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00}

#define SET_CTRMODE_POS_CMD {0x00, 0x06, 0x00, 0x25, 0x00, 0x02, 0x00, 0x00}
#define SET_CTRMODE_SPEED_CMD {0x00, 0x06, 0x00, 0x25, 0x00, 0x01, 0x00, 0x00}

#define SET_MOVMODE_ABSOLUTE_CMD {0x00, 0x06, 0x00, 0x2D, 0x00, 0x00, 0x00, 0x00}	
#define SET_MOVMODE_RELATIVE_CMD {0x00, 0x06, 0x00, 0x2D, 0x00, 0x01, 0x00, 0x00}	

#define ENABLD_MOTOR_CMD {0x00, 0x06, 0x00, 0x28, 0x00, 0x01, 0x00, 0x00}
#define DISABLD_MOTOR_CMD {0x00, 0x06, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00}

#define SET_MOVE_DISTANCE {0x00, 0x10, 0x00, 0x26, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00}
#define SET_MOVE_SPEED {0x00, 0x06, 0x00, 0xAE, 0xFF, 0xFF, 0x00, 0x00}
#define START_MOVE {0x00, 0x06, 0x00, 0x29, 0x00, 0x01, 0x00, 0x00}

#define READ_SET_DISTANCE_VALUE {0x00, 0x03, 0x00, 0xAA, 0x00, 0x02, 0x00, 0x00}
#define READ_CUR_DISTANCE_VALUE {0x00, 0x03, 0x00, 0xAC, 0x00, 0x02, 0x00, 0x00}
#define READ_CUR_SPEED_VALUE {0x00, 0x03, 0x00, 0xAF, 0x00, 0x01, 0x00, 0x00}


/* ����ָ�� */
void send_motor_cmd(uint8_t* ptr, uint8_t len);

/* �������, �����ܷ���ֵ */
int ctrl_motor_noret(uint8_t motor_id, uint8_t* cmd, uint8_t len);

/* �������, ���ܷ���ֵ u16 */
// crc ��ȷ����1 ����ȷ����0
int ctrl_motor_ret(uint8_t motor_id, uint8_t* cmd, uint8_t len, uint8_t* retptr, uint8_t* retlen);

#endif