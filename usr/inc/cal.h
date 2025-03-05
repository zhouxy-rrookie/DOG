#ifndef __CAL_H__
#define __CAL_H__

#include "math.h"
#include "dog.h"

#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

#define ABS(x)  ((x) >= 0? (x) : -(x))

//足端状态结构体
typedef struct{
	float foot_pos[2];   // 足端坐标
    float foot_vel[2];   // 足端速度
}FootState;							
// 关节状态结构体
typedef struct {
	float GO_alpha;      // 电机1角度
	float GO_beta;      // 电机2角度
    float alpha;         // 关节实际α角
    float beta;          // 关节实际β角
    float alpha_dot;     // 电机1转速
    float beta_dot;      // 电机2转速
} JointState;

// 控制参数结构体
typedef struct {
    float target_pos[2];  // 目标足端坐标 [x, y]
    float target_vel[2];  // 目标足端速度 [vx, vy]
    float kp;             // 位置控制P系数
    float kd;             // 速度控制D系数
} ControlParam;

// 状态标志结构体
typedef struct {
    bool is_grounded;    // 触地标志位
    bool is_error;       // 错误状态标志
    uint8_t error_code;  // 错误代码
} StatusFlag;

// 完整狗腿结构体
typedef struct {
    float rod[4];  // 运动学参数
    JointState     state;   // 实时状态
    ControlParam   control; // 控制参数
    StatusFlag     flags;   // 状态标志
    uint8_t        leg_id;
} Leg_Param;
#endif