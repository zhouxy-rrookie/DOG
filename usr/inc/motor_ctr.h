#ifndef __MOTOR_CTR_H__
#define __MOTOR_CTR_H__

#include "motor_control.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>

void Set_zero(MOTOR_send* cmd);
bool Is_zero(MOTOR_send cmd);
bool Set_pos(MOTOR_send* cmd, uint8_t id, float pos, float Kp);
bool Set_vel(MOTOR_send* cmd, float w, float Kw);
bool Set_Torque(MOTOR_send* cmd, uint8_t id, float T);
bool MixControl(MOTOR_send* cmd,float T, float w, float Kw, float pos, float Kp);
extern MOTOR_recv data[8];
extern MOTOR_send cmd[8];
//void set_exp(ExpectState* exp,float x_pos, float y_pos,float x_vel,float y_vel);
void FMotor_Task(void* argument);
#endif
