#ifndef _CAMERA_H
#define _CAMERA_H

#include"rtthread.h"
#include"rtdevice.h"

void set_angle(int16_t x, int16_t y);
void open_motor();
void close_motor();
void close_follow();
void follow();
void to_mid();
void prevent_gyro_drift();
void set_zoom_rate(int8_t r);
void update_cmd();

#endif
