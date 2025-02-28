#ifndef __DOG_H__
#define __DOG_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    SYS_INIT,   // 系统初始化
    SYS_ACT,    // 系统运行中
    SYS_ERROR   // 系统错误
} SYSTEM_STATE;

typedef enum {
    DOG_WAIT,           // 等待状态
    DOG_STANDBY,        // 备用状态
    DOG_WALK,           // 行走
    DOG_KEEP,           // 保持当前状态
    DOG_SQUAT,          // 蹲下
    DOG_STOP,           // 停止
    DOG_EMERGENCY_STOP  // 紧急停止
} DOG_STATE;

typedef bool (*PreTransitionCallback)(DOG_STATE from, DOG_STATE to); 
typedef void (*PostTransitionCallback)(DOG_STATE from, DOG_STATE to);

typedef struct {
    SYSTEM_STATE system;  // 当前系统状态
    DOG_STATE previous;   // 之前状态
    DOG_STATE current;    // 当前状态
} State;

extern State g_dog_state;

void RegisterPreTransitionCallback(PreTransitionCallback cb);
void RegisterPostTransitionCallback(PostTransitionCallback cb);

DOG_STATE Get_Current_State(void);
SYSTEM_STATE Get_System_State(void);

void TransitionState(DOG_STATE new_state);
void InitStateMachine(void);


#endif
