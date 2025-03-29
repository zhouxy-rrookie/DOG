/**
 ******************************************************************************
 * @file           cal.c
 * @brief          四足机器人四连杆腿部解算模块
 * 
 * 本文件实现了四足机器人腿部的正向与逆向运动学（FK & IK）解算，
 * 适用于采用四连杆并联腿机构的机器人。通过输入目标足端坐标，
 * 计算关节角度以驱动电机控制腿部运动。
 * 
 * ----------------------------------------------------------------------------
 * 主要功能：
 * - 计算腿部关节位置（正向运动学 FK）
 * - 解析目标坐标并求解关节角度（逆向运动学 IK）
 * - 轨迹规划辅助函数
 * - 机器人步态控制相关接口
 * 
 * ----------------------------------------------------------------------------
 * 相关参数：
 * - 关节角度 α，β，φ1，φ2
 * - 连杆长度 legs[0], legs[1], legs[2], legs[3]
 * - 足端目标坐标 (x, y)
 * 
 * ----------------------------------------------------------------------------
 * 单位：CSU EK战队
 * 作者: zhouxy
 * 版本: 1.0
 * 日期: 2025-02-19
 ******************************************************************************
 */

#include "cal.h"
//#include "motor_ctr.h"


//			     x<———————————o A
//					α		 /|\
//						β	/ | \
//					       /  |  \
//			 legs[0]	  /	  |   \ legs[2]
//						 /	y √    \
//						/ 		————o D
//			     ————B o		 φ2 /
//				     φ1	\		   /
//						 \		  /
//				  legs[1] \		 / legs[3]
//						   \	/
//                          \  /
//							 o C


//							^
//							|
//				0β 0α              1β 1α
//				
//				2α 2β              3α 3β



/**************************************运动学解算***************************************/
Leg leg[4] = {0};
ExpectState exp_sta[4] = {0};
extern MOTOR_recv data[8];
extern MOTOR_send cmd[8];

/**
 * @brief 四连杆运动学正解,输入电机转角,输出足端位置；输入电机转速,输出足端速度
 * @param Leg_Param 狗腿结构体
 * @param alpha,beta 关节角度 alpha_dot,beta_dot 电机转速
 */
void LinkageCalc(Leg* leg,ExpectState* exp)
{
	float x_B,y_B,x_C,y_C,x_D,y_D,x_dot_B,x_dot_D,y_dot_B,y_dot_D,x_dot_C,y_dot_C;
	float sin_phi1,cos_phi1,sin_phi2,cos_phi2,phi1_dot;
	float A,B,C,D;
	
	float alpha = leg->joint.joint_alpha;
	float alpha_dot = leg->joint.joint_alpha_dot;
	float beta = leg->joint.joint_beta;
	float beta_dot = leg->joint.joint_beta_dot;

	x_B = leg->rod[0]*cosf(alpha);
	y_B = leg->rod[0]*sinf(alpha);
	x_D = leg->rod[2]*cosf(beta);
	y_D = leg->rod[2]*sinf(beta);
	
	A = 2*(x_B-x_D)*leg->rod[1];
	B = 2*(y_B-y_D)*leg->rod[1];
	D = (x_B-x_D)*(x_B-x_D)+(y_B-y_D)*(y_B-y_D);
	C = D*D+leg->rod[1]*leg->rod[1]-leg->rod[3]*leg->rod[3];
	
	cos_phi1 = (-A*C-B*sqrtf(A*A+B*B-C*C))/(A*A+B*B);
	sin_phi1 = (-C-A*cos_phi1)/B;
	cos_phi2 = (x_B-x_D+leg->rod[1]*cos_phi1)/leg->rod[3];
	sin_phi2 = (y_B-y_D+leg->rod[1]*sin_phi1)/leg->rod[3];
	
	x_C = x_B+leg->rod[1]*cos_phi1;
	y_C = y_B+leg->rod[1]*sin_phi1;
	
	leg->foot.pos[0] = x_C;
	leg->foot.pos[1] = y_C;
	
	x_dot_B = -leg->rod[0]*sinf(alpha)*alpha_dot;
	y_dot_B = leg->rod[0]*cosf(alpha)*alpha_dot;
	x_dot_D = -leg->rod[2]*sinf(beta)*beta_dot;
	y_dot_D = leg->rod[2]*cosf(beta)*beta_dot;
	
	phi1_dot = ((x_dot_B-x_dot_D)*cos_phi2+(y_dot_B-y_dot_D)*sin_phi2)/leg->rod[1]*(sin_phi1*cos_phi2-sin_phi2*cos_phi1);
	
	x_dot_C = x_dot_B-leg->rod[1]*sin_phi1*phi1_dot;
	y_dot_C = y_dot_B+leg->rod[1]*cos_phi1*phi1_dot;
	
	leg->foot.vel[0] = x_dot_C;
	leg->foot.vel[1] = y_dot_C;
}

/**
 * @brief 通过角度约束选择合适的解
 * @param a1 a1 逆解算得到的两个解
 * @param limits[2] 角度约束
 */
float validateAngle(float a1, float a2, const float limits[2]) {
    float angle = a1;
    if (a1 < limits[0] || a1 > limits[1]) {
        angle = a2;
        if (a2 < limits[0] || a2 > limits[1]) {
            return limits[0]; 
        }
    }
    return angle;
}

/**
 * @brief 四连杆运动学逆解,输入足端位置,输出电机转角；输入足端速度,输出电机转速
 * @param Leg_Param 狗腿结构体
 * @param alpha,beta,电机输入角度
 */
void LinkageInverse(Leg *leg, ExpectState* exp)
{
	float alpha_A,alpha_B,alpha_C,beta_A,beta_B,beta_C;
	float alpha_theta,beta_theta;
	float alpha_1,alpha_2,beta_1,beta_2;
	float x_C = exp->foot_pos[0];
	float y_C = exp->foot_pos[1];
	float x_dot_C = exp->foot_vel[0];
	float y_dot_C = exp->foot_vel[1];
	
	alpha_A = x_C/leg->rod[0];
	alpha_B = y_C/leg->rod[0];
	alpha_C = (x_C*x_C+y_C*y_C+leg->rod[0]*leg->rod[0]-leg->rod[1]*leg->rod[1])/2/(leg->rod[0]*leg->rod[0]);
	
	beta_A = x_C/leg->rod[2];
	beta_B = y_C/leg->rod[2];
	beta_C = (x_C*x_C+y_C*y_C+leg->rod[2]*leg->rod[2]-leg->rod[3]*leg->rod[3])/2/(leg->rod[2]*leg->rod[2]);
	
	alpha_theta = atan2f(alpha_B,alpha_A);
	beta_theta = atan2f(beta_B,beta_A);
	
	alpha_1 = alpha_theta+acosf(alpha_C/(sqrtf(alpha_A*alpha_A+alpha_B*alpha_B)));
	alpha_2 = alpha_theta-acosf(alpha_C/(sqrtf(alpha_A*alpha_A+alpha_B*alpha_B)));
	beta_1 = beta_theta+acosf(beta_C/(sqrtf(beta_A*beta_A+beta_B*beta_B)));
	beta_2 = beta_theta-acosf(beta_C/(sqrtf(beta_A*beta_A+beta_B*beta_B)));
	
	leg->joint.joint_alpha = validateAngle(alpha_1,alpha_2,leg->joint.limit.alpha_limit);
	leg->joint.joint_beta = validateAngle(beta_1,beta_2,leg->joint.limit.beta_limit);
	
	float sin_alpha = sinf(leg->joint.joint_alpha);
    float cos_alpha = cosf(leg->joint.joint_alpha);
    float sin_beta = sinf(leg->joint.joint_beta);
    float cos_beta = cosf(leg->joint.joint_beta);
	
	float detJ = leg->rod[0]*leg->rod[2]*(sin_alpha*cos_beta - cos_alpha*sin_beta);
    if (fabs(detJ) < 1e-6) {
        leg->joint.joint_alpha_dot = 0;
        leg->joint.joint_beta_dot = 0;
        return;
    }
	
    leg->joint.joint_alpha_dot = ( leg->rod[2]*cos_beta*x_dot_C + leg->rod[2]*sin_beta*y_dot_C) / detJ;
    leg->joint.joint_beta_dot  = (-leg->rod[0]*cos_alpha*x_dot_C - leg->rod[0]*sin_alpha*y_dot_C) / detJ;
}

/**
 * @brief 电机角度转换为关节坐标,输入电机位置,输出关节转角，输入关节速度，输出电机速度；
		  在关节坐标系，从外侧向内看，逆时针转为正方向。从电机转子侧看，逆时针为正方向
 * @param Leg 狗腿结构体
 */
void MotorToJoint03(Leg* leg,MOTOR_recv data_alpha,MOTOR_recv data_beta)
{
	leg->motor_alpha.motor_angle = data_alpha.Pos;
	leg->motor_beta.motor_angle = data_beta.Pos;
	leg->motor_alpha.motor_velocity = data_alpha.W;
	leg->motor_beta.motor_velocity = data_beta.W; 
	
	leg->joint.joint_alpha = (leg->motor_alpha.motor_angle-leg->motor_alpha.init_pos) / Reduction_ratio - PI / 3;
	leg->joint.joint_beta = -(leg->motor_beta.motor_angle-leg->motor_beta.init_pos) / Reduction_ratio + 4 * PI / 3;
	
	leg->joint.joint_alpha_dot = leg->motor_alpha.motor_velocity;
	leg->joint.joint_beta_dot = leg->motor_beta.motor_velocity;
}

void MotorToJoint12(Leg* leg,MOTOR_recv data_alpha,MOTOR_recv data_beta)
{
	leg->motor_alpha.motor_angle = data_alpha.Pos;
	leg->motor_beta.motor_angle = data_beta.Pos;
	leg->motor_alpha.motor_velocity = data_alpha.W;
	leg->motor_beta.motor_velocity = data_beta.W; 
	
	leg->joint.joint_alpha = (leg->motor_alpha.init_pos-leg->motor_alpha.motor_angle) / Reduction_ratio - PI / 3;
	leg->joint.joint_beta = (leg->motor_beta.motor_angle-leg->motor_beta.init_pos) / Reduction_ratio + 4 * PI / 3;
	
	leg->joint.joint_alpha_dot = leg->motor_alpha.motor_velocity;
	leg->joint.joint_beta_dot = leg->motor_beta.motor_velocity;
}
/**
 * @brief 关节坐标转换为电机角度,输入关节坐标,输出电机位置，输入关节速度，输出电机速度；
 * @param Leg 狗腿结构体
 */
void JointToMotor03(Leg* leg,MOTOR_send* cmd_alpha,MOTOR_send* cmd_beta){
	leg->motor_alpha.motor_angle = (leg->joint.joint_alpha + PI /3)* Reduction_ratio + leg->motor_alpha.init_pos;
	leg->motor_beta.motor_angle = (-leg->joint.joint_beta + 4* PI / 3) * Reduction_ratio + leg->motor_beta.init_pos;
	
	leg->motor_alpha.motor_velocity = leg->joint.joint_alpha_dot;
	leg->motor_beta.motor_velocity = leg->joint.joint_beta_dot;
	
	cmd_alpha->Pos = leg->motor_alpha.motor_angle;
	cmd_beta->Pos = leg->motor_beta.motor_angle;
	cmd_alpha->W = leg->motor_alpha.motor_velocity;
	cmd_beta->W = leg->motor_beta.motor_velocity;
}

void JointToMotor12(Leg* leg,MOTOR_send* cmd_alpha,MOTOR_send* cmd_beta){
	leg->motor_alpha.motor_angle = -(leg->joint.joint_alpha + PI /3)* Reduction_ratio + leg->motor_alpha.init_pos;
	leg->motor_beta.motor_angle = (leg->joint.joint_beta - 4* PI / 3) * Reduction_ratio + leg->motor_beta.init_pos;
	
	leg->motor_alpha.motor_velocity = leg->joint.joint_alpha_dot;
	leg->motor_beta.motor_velocity = leg->joint.joint_beta_dot;
	
	cmd_alpha->Pos = leg->motor_alpha.motor_angle;
	cmd_beta->Pos = leg->motor_beta.motor_angle;
	cmd_alpha->W = leg->motor_alpha.motor_velocity;
	cmd_beta->W = leg->motor_beta.motor_velocity;
}
/**
 * @brief 将一个腿的参数映射到四条腿，直接传四个结构体可读性强。不是数组指针传不起，传四个更有性价比；
 * @param Leg 解算得到狗腿结构体。
 * @param Leg0~Leg3 四条腿
 */
void SingleToAll(Leg* leg, Leg* leg0, Leg* leg1, Leg* leg2, Leg* leg3)
{
	leg0->foot.pos[0] = leg->foot.pos[0];
	leg0->foot.pos[1] = leg->foot.pos[1];
	leg1->foot.pos[0] = -leg->foot.pos[0];
	leg1->foot.pos[1] = leg->foot.pos[1];
	leg2->foot.pos[0] = leg->foot.pos[0];
	leg2->foot.pos[1] = leg->foot.pos[1];
	leg3->foot.pos[0] = -leg->foot.pos[0];
	leg3->foot.pos[1] = leg->foot.pos[1];
	
	leg0->foot.vel[0] = leg->foot.vel[0];
	leg0->foot.vel[1] = leg->foot.vel[1];
	leg1->foot.vel[0] = -leg->foot.vel[0];
	leg1->foot.vel[1] = leg->foot.vel[1];
	leg2->foot.vel[0] = leg->foot.vel[0];
	leg2->foot.vel[1] = leg->foot.vel[1];
	leg3->foot.vel[0] = -leg->foot.vel[0];
	leg3->foot.vel[1] = leg->foot.vel[1];
	
	
}

/**
 * @brief 电机初始化，到最高点读取初始位置
 * @param Leg 狗腿结构体
 */
uint8_t Init_OK = 0;
void Motor_Init(Leg* leg0, Leg* leg1, Leg* leg2, Leg* leg3)
{
	leg0->rod[0] = 125;
	leg0->rod[1] = 250;
	leg0->rod[2] = 125;
	leg0->rod[3] = 250;
	
	leg1->rod[0] = 125;
	leg1->rod[1] = 250;
	leg1->rod[2] = 125;
	leg1->rod[3] = 250;
	
	leg2->rod[0] = 125;
	leg2->rod[1] = 250;
	leg2->rod[2] = 125;
	leg2->rod[3] = 250;
	
	leg3->rod[0] = 125;
	leg3->rod[1] = 250;
	leg3->rod[2] = 125;
	leg3->rod[3] = 250;
	
	MixControl(&cmd[0],0,-5,0.1,0,0);
	MixControl(&cmd[1],0,-5,0.1,0,0);
	MixControl(&cmd[2],0,5,0.1,0,0);
	MixControl(&cmd[3],0,5,0.1,0,0);
	MixControl(&cmd[4],0,5,0.1,0,0);
	MixControl(&cmd[5],0,5,0.1,0,0);
	MixControl(&cmd[6],0,-5,0.1,0,0);
	MixControl(&cmd[7],0,-5,0.1,0,0);
	
	leg0->joint.limit.alpha_limit[0] = -PI/6;
	leg0->joint.limit.alpha_limit[1] = PI/3;
	leg0->joint.limit.beta_limit[0] = 2*PI/3;
	leg0->joint.limit.beta_limit[1] = 4*PI/3;
	
	leg1->joint.limit.alpha_limit[0] = -PI/6;
	leg1->joint.limit.alpha_limit[1] = PI/3;
	leg1->joint.limit.beta_limit[0] = 2*PI/3;
	leg1->joint.limit.beta_limit[1] = 4*PI/3;
	
	leg2->joint.limit.alpha_limit[0] = -PI/6;
	leg2->joint.limit.alpha_limit[1] = PI/3;
	leg2->joint.limit.beta_limit[0] = 2*PI/3;
	leg2->joint.limit.beta_limit[1] = 4*PI/3;
	
	leg3->joint.limit.alpha_limit[0] = -PI/6;
	leg3->joint.limit.alpha_limit[1] = PI/3;
	leg3->joint.limit.beta_limit[0] = 2*PI/3;
	leg3->joint.limit.beta_limit[1] = 4*PI/3;
	
	
	uint8_t reach[8] = {0,0,0,0,0,0,0,0};
	uint8_t sum = 0;
	while(1)
	{
		SERVO_Send_recv_ch2(&cmd[4],&data[4]);
		SERVO_Send_recv_ch2(&cmd[5],&data[5]);
		SERVO_Send_recv_ch2(&cmd[2],&data[2]);
		SERVO_Send_recv_ch2(&cmd[3],&data[3]);
		SERVO_Send_recv_ch2(&cmd[6],&data[6]);
		SERVO_Send_recv_ch2(&cmd[7],&data[7]);
		SERVO_Send_recv_ch2(&cmd[0],&data[0]);
		SERVO_Send_recv_ch2(&cmd[1],&data[1]);
		sum = 0;
		for(int i = 0;i<8;i++)
		{
			if(reach[i] == 0 && (ABS(data[i].W)) < 1.0F){
				SERVO_Send_recv_ch2(&cmd[4],&data[4]);
				SERVO_Send_recv_ch2(&cmd[5],&data[5]);
				SERVO_Send_recv_ch2(&cmd[2],&data[2]);
				SERVO_Send_recv_ch2(&cmd[3],&data[3]);
				SERVO_Send_recv_ch2(&cmd[6],&data[6]);
				SERVO_Send_recv_ch2(&cmd[7],&data[7]);
				SERVO_Send_recv_ch2(&cmd[0],&data[0]);
				SERVO_Send_recv_ch2(&cmd[1],&data[1]);
				if(ABS(data[i].W) < 1.0F){reach[i] = 1;}
			}
			sum+=reach[i];
		}
		if(sum == 8)
		{
//			for(int i = 0; i < 8; i++)
//			{
//				Set_vel(&cmd[i],0,0);
//			}
			leg0->motor_alpha.init_pos = data[0].Pos;
			leg0->motor_beta.init_pos = data[1].Pos;
			leg1->motor_alpha.init_pos = data[2].Pos;
			leg1->motor_beta.init_pos = data[3].Pos;
			leg2->motor_alpha.init_pos = data[4].Pos;
			leg2->motor_beta.init_pos = data[5].Pos;
			leg3->motor_alpha.init_pos = data[6].Pos;
			leg3->motor_beta.init_pos = data[7].Pos;
			Init_OK = 1;
			
			return;
		}
		HAL_Delay(20);
	}
}

/**************************************VMC控制算法*****************************************/

/**
 * @brief 雅可比矩阵,输入足端力,输出电机力矩；
 * @param Leg 狗腿结构体
 * @param alpha,beta,电机输入角度
 */
void JacobiMatrix(Leg* leg, ExpectState* exp)
{
	float x_B,y_B,x_C,y_C,x_D,y_D,x_dot_B,x_dot_D,y_dot_B,y_dot_D,x_dot_C,y_dot_C;
	float sin_phi1,cos_phi1,sin_phi2,cos_phi2,phi1_dot;
	float A,B,C,D;
	
	float alpha = leg->joint.joint_alpha;
	float alpha_dot = leg->joint.joint_alpha_dot;
	float beta = leg->joint.joint_beta;
	float beta_dot = leg->joint.joint_beta_dot;
 
	x_B = leg->rod[0]*cosf(alpha);
	y_B = leg->rod[0]*sinf(alpha);
	x_D = leg->rod[2]*cosf(beta);
	y_D = leg->rod[2]*sinf(beta);
	
	A = 2*(x_B-x_D)*leg->rod[1];
	B = 2*(y_B-y_D)*leg->rod[1];
	D = (x_B-x_D)*(x_B-x_D)+(y_B-y_D)*(y_B-y_D);
	C = D*D+leg->rod[1]*leg->rod[1]-leg->rod[3]*leg->rod[3];
	
	cos_phi1 = (-A*C-B*sqrtf(A*A+B*B-C*C))/(A*A+B*B);
	sin_phi1 = (-C-A*cos_phi1)/B;
	cos_phi2 = (x_B-x_D+leg->rod[1]*cos_phi1)/leg->rod[3];
	sin_phi2 = (y_B-y_D+leg->rod[1]*sin_phi1)/leg->rod[3];
	float sin_phi12 = sin_phi1*cos_phi2-sin_phi2*cos_phi1;
	
	if(fabs(sin_phi12)<0.01){
		leg->Jacobi[0][0] = leg->rod[0]*sin_phi2*(sinf(alpha)*cos_phi1-cos(alpha)*sin_phi1)/sin_phi12;
		leg->Jacobi[0][1] = -leg->rod[2]*sin_phi1*(sinf(beta)*cos_phi2-cosf(beta)*sin_phi2)/sin_phi12;
		leg->Jacobi[1][0] = -leg->rod[0]*cos_phi2*(sinf(alpha)*cos_phi1-cosf(alpha)*sin_phi1)/sin_phi12;
		leg->Jacobi[1][1] = leg->rod[2]*cos_phi1*(sinf(beta)*cos_phi2-cosf(beta)*sin_phi2)/sin_phi12;
	}else{
		leg->Jacobi[0][0] = 0;
		leg->Jacobi[0][1] = 0;
		leg->Jacobi[1][0] = 0;
		leg->Jacobi[1][1] = 0;
	}

	leg->Jacobi_T[0][0] = leg->Jacobi[0][0];
	leg->Jacobi_T[0][1] = leg->Jacobi[1][0];
	leg->Jacobi_T[1][0] = leg->Jacobi[0][1];
	leg->Jacobi_T[1][1] = leg->Jacobi[1][1];
	 
}




/**
 * @brief 支撑相虚拟力计算
 * @param leg 腿部指针
 * @param exp 期望状态
 * @param phase 当前相位
 * @param vf 虚拟力计算结果
 */
void StanceForceCalculator(Leg* leg, ExpectState* exp, GaitPhase phase, VirtualForce* vf)
{
    float dx = exp->foot_pos[0] - leg->foot.pos[0];
    float dy = exp->foot_pos[1] - leg->foot.pos[1];
    float dvx = exp->foot_vel[0] - leg->foot.vel[0];
    float dvy = exp->foot_vel[1] - leg->foot.vel[1];
    
    vf->F_virtual[0] = VMC_KP_STANCE * dx + VMC_KD_STANCE * dvx;
    vf->F_virtual[1] = VMC_KP_STANCE * dy + VMC_KD_STANCE * dvy;
    vf->F_virtual[1] += 0.25f * BODY_MASS * GRAVITY;
    
	//因为足端没有力传感器，所以需要用阻抗控制器进行预测
    vf->F_contact[0] = 0.0f; 
    vf->F_contact[1] = 0.0f;  
    
    vf->F_resultant[0] = vf->F_virtual[0] + vf->F_contact[0];
    vf->F_resultant[1] = vf->F_virtual[1] + vf->F_contact[1];
}

/**
 * @brief 阻抗控制型接触力估计
 * @param leg 腿部指针
 * @param F_est 输出接触力估计
 */
void ImpedanceForce(Leg* leg, ExpectState* exp, float F_est[2])
{
    const float K_imp[2] = {500.0f, 500.0f}; // 虚拟刚度
    const float B_imp[2] = {50.0f, 50.0f};   // 虚拟阻尼
    
    float dx = leg->foot.pos[0] - exp->foot_pos[0];
    float dy = leg->foot.pos[1] - exp->foot_pos[1];
    float dvx = leg->foot.vel[0] - exp->foot_vel[0];
    float dvy = leg->foot.vel[1] - exp->foot_vel[1];
    
    // 虚拟力估计 F = KΔx + BΔv
    F_est[0] = K_imp[0] * dx + B_imp[0] * dvx;
    F_est[1] = K_imp[1] * dy + B_imp[1] * dvy;
}

/**
 * @brief 摆动相虚拟力计算
 * @param leg 腿部指针
 * @param exp 期望状态
 * @param vf 虚拟力计算结果
 */
void SwingTrajectoryControl(Leg* leg, ExpectState* exp, VirtualForce* vf)
{
    // 轨迹跟踪PD控制
    float dx = exp->foot_pos[0] - leg->foot.pos[0];
    float dy = exp->foot_pos[1] - leg->foot.pos[1];
    float dvx = exp->foot_vel[0] - leg->foot.vel[0];
    float dvy = exp->foot_vel[1] - leg->foot.vel[1];
    
    vf->F_resultant[0] = VMC_KP_SWING * dx + VMC_KD_SWING * dvx;
    vf->F_resultant[1] = VMC_KP_SWING * dy + VMC_KD_SWING * dvy;
}

GaitPhase gait_phase[4] = {PHASE_STANCE, PHASE_SWING, PHASE_SWING, PHASE_STANCE}; // 初始trot步态
float swing_gain[4] = {1.0f, 0.8f, 0.8f, 1.0f}; // 各腿摆动相增益

GaitPhase DetectContactPhase(Leg* leg, ExpectState* exp)
{
    static float force_threshold = 20.0f;
    static float filter_alpha = 0.2f;
    static float filtered_force = 0.0f;
    
    float F_est[2];
    ImpedanceForce(leg, exp,F_est);
    
    // 低通滤波
    filtered_force = filter_alpha * filtered_force + (1-filter_alpha) * (F_est[0] + F_est[1]);

    if(filtered_force > force_threshold) {
        return PHASE_STANCE;
    } else {
        return PHASE_SWING;
    }
}

///**
// * @brief VMC主控制器（需在实时控制循环中调用）
// * @param leg 腿部指针数组
// * @param exp_sta 期望状态数组
// * @param gait_phase 各腿当前步态相位
// */
//void VMC_Controller(Leg *leg, ExpectState *exp, GaitPhase *gait_phase,MOTOR_send *cmd,MOTOR_recv *data)
//{
//    VirtualForce vf;
//    static float torque_gain = 0.1f; // 力矩到关节角度的转换系数
//    
//    *gait_phase = DetectContactPhase(leg,exp);
//        // 根据相位选择控制模式
//    if(gait_phase == PHASE_STANCE) {
//        StanceForceCalculator(leg, exp, *gait_phase, &vf);
//        float tau[2];
//        MultiplyMatrixVector(2, 2, 1, leg->Jacobi_T, vf.F_resultant, tau);
//        tau[0] += torque_gain * (exp_sta->foot_pos[0] - leg->motor_alpha.motor_angle);
//        tau[1] += torque_gain * (exp_sta->foot_pos[1] - leg->motor_beta.motor_angle);
//            
//        MixControl(cmd, tau[0], 0, 0, 0, 0);
//        MixControl(cmd, tau[1], 0, 0, 0, 0);
//            
//    } else {
//        float swing_progress = fmod(uwTick, swing_gain[i]*1000)/1000.0f;
//            
//            // 发送位置指令
//        set_exp(&exp_sta, exp_sta->foot_pos[0], exp_sta->foot_pos[1], 0, 0);
//        LinkageInverse(leg, exp_sta);
//        MixControl(cmd, 0, 20, 0.08, -leg->motor_alpha.motor_angle, 0.20);
//        MixControl(cmd, 0, 20, 0.08, leg->motor_beta.motor_angle, 0.20);
//    }
//        
//        // 发送控制指令
//    JointToMotor03(leg, cmd, cmd);
//    SERVO_Send_recv_ch2(cmd, data);
//    SERVO_Send_recv_ch2(cmd, data);
//}










