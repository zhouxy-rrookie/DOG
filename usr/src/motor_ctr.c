#include "motor_ctr.h"
#include "remote.h"

uint32_t flag = 0;
HAL_StatusTypeDef flag_m;

MOTOR_recv data;
MOTOR_send cmd = {
	.id=0,		
	.mode=1,
	.T=0.0f,
	.W=0.0f,
	.Pos=0.0f,
	.K_P=0.0f,
	.K_W=0.0f,
};

void Set_zero(MOTOR_send* cmd)
{
	cmd->K_P = 0;
	cmd->K_W = 0;
	cmd->Pos = 0;
	cmd->T = 0;
	cmd->W = 0;
}

bool Is_zero(MOTOR_send cmd)
{
	if(cmd.K_P == 0 &&
		 cmd.K_W == 0 &&
	   cmd.Pos == 0 &&
		 cmd.T == 0   &&
		 cmd.W == 0){return true;}
	else{return false;}
}

bool Set_pos(MOTOR_send* cmd, uint8_t id, float pos, float Kp)
{
	if(!cmd) return false;
	cmd->id = id;
	cmd->Pos = pos;
	cmd->K_P = Kp;
	cmd->T = 0;
	cmd->W = 0;
	return true;
}

bool Set_vel(MOTOR_send* cmd, uint8_t id, float w, float Kw)
{
	if(!cmd) return false;
	cmd->id = id;
	cmd->W = w;
	cmd->K_W = Kw;
	cmd->K_P = 0;
	cmd->T = 0;
	return true;
}

bool Set_Torque(MOTOR_send* cmd, uint8_t id, float T)
{
	if(!cmd)return false;
	cmd->id = id;
	cmd->T = T;
	return true;
}

float data_convert1(int src, int src_low, int src_high, float dst_low, float dst_high)
{
    if (src < src_low) src = src_low;
    if (src > src_high) src = src_high;

    if (src_low == src_high) return dst_low;

    float res = ((float)(src - src_low) / (src_high - src_low)) * (dst_high - dst_low) + dst_low;

    if (res < dst_low) res = dst_low;
    if (res > dst_high) res = dst_high;

    return res;
}

float vel;
float Kw = 0.02;
float num = 200;
float pos = 3.14*6.33;
void FMotor_Task(void *argument)
{
	const TickType_t delay = pdMS_TO_TICKS(100);
  for(;;)
  {
		vel = data_convert1(Remote_Ctrl.CH2,195,1795,-40,40);
		//vel = data_convert(num,195,1795,0,500);
		Set_Torque(&cmd, 3, 0.00);
		//Set_vel(&cmd, 0, vel, Kw);
		//Set_pos(&cmd,1,pos,0.01,0.01);
    flag_m = SERVO_Send_recv(&cmd, &data);
		flag++;
		vTaskDelay(delay);
  }
}
