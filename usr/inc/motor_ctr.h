#ifndef __MOTOR_CTR_H__
#define __MOTOR_CTR_H__

#include "motor_control.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>

void Set_zero(MOTOR_send* cmd);
bool Is_zero(MOTOR_send cmd);
bool Set_pos(MOTOR_send* cmd, uint8_t id, float pos, float Kp);
bool Set_vel(MOTOR_send* cmd, uint8_t id, float w, float Kw);
bool Set_Torque(MOTOR_send* cmd, uint8_t id, float T);

#endif
