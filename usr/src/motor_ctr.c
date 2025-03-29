#include "motor_ctr.h"
#include "cal.h"

uint32_t flag = 0;
HAL_StatusTypeDef flag_m;

MOTOR_recv data[8];
MOTOR_send cmd[8] = { 
    {.id = 0, .mode = 1}, {.id = 1, .mode = 1},
    {.id = 2, .mode = 1}, {.id = 3, .mode = 1},
    {.id = 4, .mode = 1}, {.id = 5, .mode = 1},
    {.id = 6, .mode = 1}, {.id = 7, .mode = 1}
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

//bool Set_pos(MOTOR_send* cmd, float pos, float Kp)
//{
//	if(!cmd) return false;
//	cmd->Pos = pos;
//	cmd->K_P = Kp;
//	cmd->T = 0;
//	cmd->W = 0;
//	return true;
//}

bool Set_vel(MOTOR_send* cmd, float w, float Kw)
{
	if(!cmd) return false;
	cmd->W = w;
	cmd->K_W = Kw;
	cmd->K_P = 0;
	cmd->T = 0;
	return true;
}

//bool Set_Torque(MOTOR_send* cmd, float T)
//{
//	if(!cmd)return false;
//	cmd->T = T;
//	return true;
//}

bool MixControl(MOTOR_send* cmd,float T, float w, float Kw, float pos, float Kp)
{
	if(!cmd) return false;
	cmd->T = T;
	cmd->W = w;
	cmd->K_W = Kw;
	cmd->Pos = pos;
	cmd->K_P = Kp;
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

void set_exp(ExpectState* exp,float x_pos, float y_pos,float x_vel,float y_vel)
{
	exp->foot_pos[0] = x_pos;
	exp->foot_pos[1] = y_pos;
	exp->foot_vel[0] = x_vel;
	exp->foot_vel[1] = y_vel;
}

void set_foot_pos0(float x, float y)
{
	set_exp(&exp_sta[0],x,y,10,10);
	LinkageInverse(&leg[0],&exp_sta[0]);
	MixControl(&cmd[0],0,20,0.08,-leg[0].motor_alpha.motor_angle,0.20);
	MixControl(&cmd[1],0,20,0.08,leg[0].motor_beta.motor_angle,0.20);
	JointToMotor03(&leg[0],&cmd[0],&cmd[1]);
	SERVO_Send_recv_ch2(&cmd[0], &data[0]);
	SERVO_Send_recv_ch2(&cmd[1], &data[1]);
}

void set_foot_pos1(float x, float y)
{
	set_exp(&exp_sta[1],x,y,10,10);
	LinkageInverse(&leg[1],&exp_sta[1]);
	MixControl(&cmd[2],0,20,0.08,-leg[1].motor_alpha.motor_angle,0.20);
	MixControl(&cmd[3],0,12,0.08,leg[1].motor_beta.motor_angle,0.20);
	JointToMotor12(&leg[1],&cmd[2],&cmd[3]);
	SERVO_Send_recv_ch2(&cmd[2], &data[2]);
	SERVO_Send_recv_ch2(&cmd[3], &data[3]);
}

void set_foot_pos2(float x, float y)
{
	set_exp(&exp_sta[2],x,y,10,10);
	LinkageInverse(&leg[2],&exp_sta[2]);
	MixControl(&cmd[4],0,20,0.08,-leg[2].motor_alpha.motor_angle,0.20);
	MixControl(&cmd[5],0,20,0.08,-leg[2].motor_beta.motor_angle,0.20);
	JointToMotor12(&leg[2],&cmd[4],&cmd[5]);
	SERVO_Send_recv_ch2(&cmd[4], &data[4]);
	SERVO_Send_recv_ch2(&cmd[5], &data[5]);
}

void set_foot_pos3(float x, float y)
{
	set_exp(&exp_sta[3],x,y,10,10);
	LinkageInverse(&leg[3],&exp_sta[3]);
	MixControl(&cmd[6],0,20,0.08,leg[3].motor_alpha.motor_angle,0.20);
	MixControl(&cmd[7],0,20,0.08,-leg[3].motor_beta.motor_angle,0.20);
	JointToMotor03(&leg[3],&cmd[6],&cmd[7]);
	SERVO_Send_recv_ch2(&cmd[6], &data[6]);
	SERVO_Send_recv_ch2(&cmd[7], &data[7]);
}

void draw_line(float x0, float y0, float x1, float y1, uint32_t duration_ms) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float steps = (float)duration_ms; // ÔÚduration_msºÁÃëÄÚ»­Íê
    float x_inc = dx / steps;
    float y_inc = dy / steps;

    float x = x0;
    float y = y0;

    uint32_t start_time = uwTick;
    
    for (uint32_t i = 0; i <= (uint32_t)steps; i++) {
        while (uwTick - start_time < i) {
            vTaskDelay(1);
        }
        set_foot_pos0(x, y);
		set_foot_pos1(x, y);
		set_foot_pos2(x, y);
		set_foot_pos3(x, y);
        x += x_inc;
        y += y_inc;
    }
}

void move_bezier(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, uint32_t duration_ms) {
    uint32_t start_time = uwTick;
    uint32_t steps = duration_ms;
    float t, x, y;

    for (uint32_t i = 0; i <= steps; i++) {
        while (uwTick - start_time < i) {

        }

        t = (float)i / steps;

        x = (1 - t) * (1 - t) * (1 - t) * x0 +
            3 * (1 - t) * (1 - t) * t * x1 +
            3 * (1 - t) * t * t * x2 +
            t * t * t * x3;

        y = (1 - t) * (1 - t) * (1 - t) * y0 +
            3 * (1 - t) * (1 - t) * t * y1 +
            3 * (1 - t) * t * t * y2 +
            t * t * t * y3;

        set_foot_pos0(x, y);
		set_foot_pos1(-x, y);
		set_foot_pos2(x, y);
		set_foot_pos3(-x, y);
    }
}

float pos_x = 0;
float pos_y = 0;
void FMotor_Task(void* argument)
{
  const TickType_t delay = pdMS_TO_TICKS(10);
	
  if(Init_OK == 0)
  {
	  vTaskDelete(NULL);
  }

  for(;;)
  {
	  if(Init_OK == 1){
		  
		  
		 move_bezier(150,200,100,250,-100,250,-150,200,25);
		 move_bezier(-150,200,-100,150,100,150,150,200,50);
		 //set_foot_pos1(pos_x,pos_y);
		flag++;
		vTaskDelay(delay);
	  }
  }
}


