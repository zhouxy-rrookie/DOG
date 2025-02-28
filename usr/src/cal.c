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


Leg_Param LEG;

//				 x<———————————o O
//								α		 /|\
//									β	/ | \
//									 /	|  \
//				  legs[0]	/		|   \ legs[2]
//								 /	y	√    \
//								/ 		  ————o C
//					————A	o			   φ2 /
//					   φ1	 \		 		 /
//									\			  /
//				   legs[1] \		 / legs[3]
//										\	  /
//											o B



/**
 * @brief 四连杆运动学正解,输入电机转角,输出足端位置；输入电机转速,输出足端速度
 * @param Leg_Param 狗腿结构体
 * @param alpha,beta,电机输入角度
 */
void Linkage_Calc(Leg_Param* leg,float alpha, float beta, float alpha_w, float beta_w)
{
	float phi1_cos,phi1_sin,phi2_cos,phi2_sin;
	float xA,yA,xB,yB,xC,yC;
	float vAx,vAy,vBx,vBy,vCx,vCy;
	float fA, fB, fC, Dac_sq;
	
	leg->alpha = alpha;
	leg->beta = beta;
	
	xA = leg->legs[0] * cosf(leg->alpha);
	yA = leg->legs[0] * sinf(leg->alpha);
	xC = leg->legs[2] * cosf(leg->beta);
	yC = leg->legs[2] * sinf(leg->beta);
	
	Dac_sq = (xA - xC) * (xA - xC) + (yA - yC) * (yA - yC);
	fA = 2 * (xA - xC) * leg->legs[1];
	fB = 2 * (yA - yC) * leg->legs[1];
	fC = Dac_sq + leg->legs[1] * leg->legs[1] - leg->legs[3] * leg->legs[3];
	
	phi1_cos = (-fA*fC - fB*sqrtf(fA*fA + fB*fB - fC*fC)) / (fA*fA + fB*fB);
	phi1_sin = sqrtf(1 - phi1_cos * phi1_cos);
	phi2_cos = (xA - xC + leg->legs[1]*phi1_cos) / leg->legs[3];
	phi2_sin = (yA - yC + leg->legs[1]*phi1_sin) / leg->legs[3];
	
	xB = xA + leg->legs[1] * phi1_cos;
	yB = yA + leg->legs[1] * phi1_sin;
	
	leg->foot_pos[0] = xB;
	leg->foot_pos[1] = yB;
	
	vAx = -leg->legs[0] * sinf(leg->alpha)*alpha_w;
	vAy =  leg->legs[0] * cosf(leg->alpha)*alpha_w;
	vBx = vAx - leg->legs[1] * phi1_sin;
	vBy = vAy + leg->legs[1] * phi1_cos;
	
	leg->foot_vel[0] = vBx;
	leg->foot_vel[1] = vBy;
}

/**
 * @brief 四连杆运动学正解,输入电机转角,输出足端位置；输入电机转速,输出足端速度
 * @param Leg_Param 狗腿结构体
 * @param alpha,beta,电机输入角度
 */
int Linkage_Inverse(Leg_Param* leg, float xB, float yB, 
                   float alpha_sol[2], float beta_sol[2])
{
    const float L0 = leg->legs[0];
    const float L1 = leg->legs[1];
    const float L2 = leg->legs[2];
    const float L3 = leg->legs[3];
    
    int solution_count = 0;
    float temp_alphas[2] = {0};
    float temp_betas[2] = {0};
    int alpha_count = 0;
    int beta_count = 0;

    // 计算alpha可能解 --------------------------------------------------------
    const float C_alpha = (L0*L0 + xB*xB + yB*yB - L1*L1) / (2.0f * L0);
    const float D_alpha = xB*xB + yB*yB;
    
    if (D_alpha > 1e-6f) { // 避免除零
        const float ratio_alpha = C_alpha / sqrtf(D_alpha);
        
        if (fabsf(ratio_alpha) <= 1.0f) {
            const float theta_alpha = atan2f(yB, xB);
            const float delta_alpha = acosf(ratio_alpha);
            
            temp_alphas[0] = theta_alpha - delta_alpha;
            temp_alphas[1] = theta_alpha + delta_alpha;
            alpha_count = 2;
        }
    }

    // 计算beta可能解 ---------------------------------------------------------
    const float C_beta = (L2*L2 + xB*xB + yB*yB - L3*L3) / (2.0f * L2);
    const float D_beta = xB*xB + yB*yB;
    
    if (D_beta > 1e-6f) { // 避免除零
        const float ratio_beta = C_beta / sqrtf(D_beta);
        
        if (fabsf(ratio_beta) <= 1.0f) {
            const float theta_beta = atan2f(yB, xB);
            const float delta_beta = acosf(ratio_beta);
            
            temp_betas[0] = theta_beta - delta_beta;
            temp_betas[1] = theta_beta + delta_beta;
            beta_count = 2;
        }
    }

    // 验证解的有效性 ---------------------------------------------------------
    for (int i = 0; i < alpha_count; ++i) {
        for (int j = 0; j < beta_count; ++j) {
            // 计算对应点坐标
            const float xA = L0 * cosf(temp_alphas[i]);
            const float yA = L0 * sinf(temp_alphas[i]);
            const float xC = L2 * cosf(temp_betas[j]);
            const float yC = L2 * sinf(temp_betas[j]);
            
            // 验证连杆长度约束
            const float AB_sq = (xA - xB)*(xA - xB) + (yA - yB)*(yA - yB);
            const float CB_sq = (xC - xB)*(xC - xB) + (yC - yB)*(yC - yB);
            const float AB_err = fabsf(AB_sq - L1*L1);
            const float CB_err = fabsf(CB_sq - L3*L3);
            
            // 误差阈值（可根据需要调整）
            if (AB_err < 1e-3f && CB_err < 1e-3f) {
                alpha_sol[solution_count] = temp_alphas[i];
                beta_sol[solution_count] = temp_betas[j];
                solution_count++;
                
                // 最多返回2个有效解
                if (solution_count >= 2) return 2;
            }
        }
    }
    
    return solution_count;
}
