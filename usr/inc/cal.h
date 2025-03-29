#ifndef __CAL_H__
#define __CAL_H__

#include "math.h"
#include "dog.h"
#include "motor_ctr.h"
#include "motor_control.h"

#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

#define ABS(x)  ((x) >= 0? (x) : -(x))
								
#define Reduction_ratio  6.33f
#define PI 				 3.1415f

typedef struct {
    float motor_angle;       // 电机轴角度 
    float motor_velocity;    // 电机轴转速 
    float motor_torque;     // 电机电流   
	float init_pos;
    float temp;              
} MotorParam;

typedef struct {
    float joint_alpha;
    float joint_beta;
	float joint_alpha_dot;
	float joint_beta_dot;
    
    struct {
        float alpha_limit[2]; 
        float beta_limit[2];     
        float max_velocity; 
    } limit;
} JointState;

typedef struct {
    float pos[2];       
    float vel[2];       
    float ground_force[3];  
} FootState;

typedef struct {
	float target_angle;      
    float target_velocity; 
    float kp;                
    float kw;               
    float Target_torque;                
    float max_current;      
    float max_temp;         
} ControlParam;

typedef struct {
    union {
        uint16_t all_flags;
        struct {
            bool is_grounded    : 1;  // 触地状态
            bool is_overcurrent : 1;  // 过流标志
            bool is_overtemp    : 1;  // 过热标志
            bool is_calibrated  : 1;  // 校准状态
            uint8_t error_code  : 4;  // 错误编码
        };
    } status;
    
    uint32_t update_tick;  
} LegFlags;

typedef enum {
    CTRL_POSITION = 0,   // 纯位置控制
    CTRL_HYBRID,         // 位置-力混合控制
    CTRL_FORCE           // 纯力控制
} ControlMode;


typedef struct {
	uint8_t id;
	float rod[4];
    MotorParam    motor_alpha;   // α电机
    MotorParam    motor_beta;    // β电机
    JointState     joint;       // 关节状态
    FootState      foot;        // 足端状态
    LegFlags   system;      	// 系统状态
    ControlParam   ctrl_param;  // 控制参数
	float Jacobi[2][2];
	float Jacobi_T[2][2];
}Leg;

typedef struct {
    float alpha;             // 预期关节α角
    float beta;              // 预期关节β角 
    float alpha_vel;         // 预期α角速度
    float beta_vel;          // 预期β角速度
    float foot_pos[2];       // 预期足端坐标
    float foot_vel[2];       // 预期足端速度
	float foot_force[2];     // 预期足端受到地面的力
} ExpectState;

//力控参数
#define VMC_KP_STANCE    200.0f    // 支撑相刚度
#define VMC_KD_STANCE    20.0f     // 支撑相阻尼
#define VMC_KP_SWING     500.0f    // 摆动相刚度
#define VMC_KD_SWING     50.0f     // 摆动相阻尼
#define GRAVITY          9.8f      
#define BODY_MASS        10.0f    

typedef enum {
    PHASE_STANCE = 0,  // 支撑相
    PHASE_SWING        // 摆动相
} GaitPhase;

typedef struct {
    float F_virtual[2];    // 虚拟力/力矩 [Fx, Fy]
    float F_contact[2];    // 接触力估计
    float F_resultant[2];  // 合成输出力
} VirtualForce;


extern Leg leg[4];
extern ExpectState exp_sta[4];
extern uint8_t Init_OK;
 

void LinkageCalc(Leg* leg,ExpectState* exp);
float validateAngle(float a1, float a2, const float limits[2]);
void LinkageInverse(Leg *leg, ExpectState* exp);
void JointToMotor03(Leg* leg,MOTOR_send* cmd_alpha,MOTOR_send* cmd_beta);
void JointToMotor12(Leg* leg,MOTOR_send* cmd_alpha,MOTOR_send* cmd_beta);
void JacobiMatrix(Leg* leg, ExpectState* exp);
void MotorToJoint03(Leg* leg,MOTOR_recv data_alpha,MOTOR_recv data_beta);
void MotorToJoint12(Leg* leg,MOTOR_recv data_alpha,MOTOR_recv data_beta);
void Motor_Init(Leg* leg0, Leg* leg1, Leg* leg2, Leg* leg3);
GaitPhase DetectContactPhase(Leg* leg, ExpectState* exp);
void ImpedanceForce(Leg* leg, ExpectState* exp, float F_est[2]);
#endif
