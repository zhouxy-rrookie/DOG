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


/**
 * @brief 四连杆运动学正解,输入电机转角,输出足端位置；输入电机转速,输出足端速度
 * @param Leg_Param 狗腿结构体
 * @param alpha,beta 关节角度 alpha_dot,beta_dot 电机转速
 */
void Linkage_Calc(Leg_Param* leg,float alpha,float beta,float alpha_dot,float beta_dot)
{
	float x_B,y_B,x_C,y_C,x_D,y_D,x_dot_B,x_dot_D,y_dot_B,y_dot_D,x_dot_C,y_dot_C;
	float sin_phi1,cos_phi1,sin_phi2,cos_phi2,phi1_dot;
	float A,B,C,D;
	
	leg->state.alpha = alpha;
	leg->state.beta = beta;
	leg->state.alpha_dot = alpha_dot;
	leg->state.beta_dot = beta_dot;
	
	x_B = leg->rod[0]*cosf(leg->state.alpha);
	y_B = leg->rod[0]*sinf(leg->state.alpha);
	x_D = leg->rod[2]*cosf(leg->state.beta);
	y_D = leg->rod[2]*sinf(leg->state.beta);
	
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
	
	leg->foot.foot_pos[0] = x_C;
	leg->foot.foot_pos[1] = y_C;
	
	x_dot_B = -leg->rod[0]*sinf(leg->state.alpha)*alpha_dot;
	y_dot_B = leg->rod[0]*cosf(leg->state.alpha)*alpha_dot;
	x_dot_D = -leg->rod[2]*sinf(leg->state.beta)*beta_dot;
	y_dot_D = leg->rod[2]*cosf(leg->state.beta)*beta_dot;
	
	phi1_dot = ((x_dot_B-x_dot_D)*cos_phi2+(y_dot_B-y_dot_D)*sin_phi2)/leg->rod[1]*(sin_phi1*cos_phi2-sin_phi2*cos_phi1);
	
	x_dot_C = x_dot_B-leg->rod[1]*sin_phi1*phi1_dot;
	y_dot_C = y_dot_B+leg->rod[1]*cos_phi1*phi1_dot;
	
	leg->foot.foot_vel[0] = x_dot_C;
	leg->foot.foot_vel[1] = y_dot_C;
}

/**
 * @brief 通过角度约束选择合适的解
 * @param a1 a1 逆解算得到的两个解
 * @param limits[2] 角度约束
 */
float validate_angle(float a1, float a2, const float limits[2]) {
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
void Linkage_Inverse(Leg_Param *leg, float x_C,float y_C, float x_dot_C, float y_dot_C)
{
	float alpha_A,alpha_B,alpha_C,beta_A,beta_B,beta_C;
	float alpha_theta,beta_theta;
	float alpha_1,alpha_2,beta_1,beta_2;
	
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
	
	leg->state.alpha = validate_angle(alpha_1,alpha_2,leg->state.joint_limit[0]);
	leg->state.beta = validate_angle(beta_1,beta_2,leg->state.joint_limit[1]);
	
	float sin_alpha = sinf(leg->state.alpha);
    float cos_alpha = cosf(leg->state.alpha);
    float sin_beta = sinf(leg->state.beta);
    float cos_beta = cosf(leg->state.beta);
	
	float detJ = leg->rod[0]*leg->rod[2]*(sin_alpha*cos_beta - cos_alpha*sin_beta);
    if (fabs(detJ) < 1e-6) {
        leg->state.alpha_dot = 0;
        leg->state.beta_dot = 0;
        return;
    }
	
    leg->state.alpha_dot = ( leg->rod[2]*cos_beta*x_dot_C + leg->rod[2]*sin_beta*y_dot_C) / detJ;
    leg->state.beta_dot  = (-leg->rod[0]*cos_alpha*x_dot_C - leg->rod[0]*sin_alpha*y_dot_C) / detJ;
}

/**
 * @brief 雅可比矩阵,输入电机力矩,输出足端力；
 * @param Leg_Param 狗腿结构体
 * @param alpha,beta,电机输入角度
 */
void Jacobi_Matrix(Leg_Param *leg, float Torque_alpha, float Torque_beta)
{
	float x_B,y_B,x_C,y_C,x_D,y_D,x_dot_B,x_dot_D,y_dot_B,y_dot_D,x_dot_C,y_dot_C;
	float sin_phi1,cos_phi1,sin_phi2,cos_phi2,phi1_dot;
	float A,B,C,D;

	x_B = leg->rod[0]*cosf(leg->state.alpha);
	y_B = leg->rod[0]*sinf(leg->state.alpha);
	x_D = leg->rod[2]*cosf(leg->state.beta);
	y_D = leg->rod[2]*sinf(leg->state.beta);
	
	A = 2*(x_B-x_D)*leg->rod[1];
	B = 2*(y_B-y_D)*leg->rod[1];
	D = (x_B-x_D)*(x_B-x_D)+(y_B-y_D)*(y_B-y_D);
	C = D*D+leg->rod[1]*leg->rod[1]-leg->rod[3]*leg->rod[3];
	
	cos_phi1 = (-A*C-B*sqrtf(A*A+B*B-C*C))/(A*A+B*B);
	sin_phi1 = (-C-A*cos_phi1)/B;
	cos_phi2 = (x_B-x_D+leg->rod[1]*cos_phi1)/leg->rod[3];
	sin_phi2 = (y_B-y_D+leg->rod[1]*sin_phi1)/leg->rod[3];
	float sin_phi12 = sin_phi1*cos_phi2-sin_phi2*cos_phi1;
	
	float Jacobi[2][2];
	Jacobi[0][0] = leg->rod[0]*sin_phi2*(sinf(leg->state.alpha)*cos_phi1-cos(leg->state.alpha)*sin_phi1)/sin_phi12;
	Jacobi[0][1] = -leg->rod[2]*sin_phi1*(sinf(leg->state.beta)*cos_phi2-cosf(leg->state.beta)*sin_phi2)/sin_phi12;
	Jacobi[1][0] = -leg->rod[0]*cos_phi2*(sinf(leg->state.alpha)*cos_phi1-cosf(leg->state.alpha)*sin_phi1)/sin_phi12;
	Jacobi[1][1] = leg->rod[2]*cos_phi1*(sinf(leg->state.beta)*cos_phi2-cosf(leg->state.beta)*sin_phi2)/sin_phi12;
}






















