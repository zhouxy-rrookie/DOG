#include "dog.h"



typedef bool (*PreTransitionCallback)(DOG_STATE from, DOG_STATE to); 
typedef void (*PostTransitionCallback)(DOG_STATE from, DOG_STATE to);


State g_dog_state = { 
    .system = SYS_INIT,
    .previous = DOG_WAIT,
    .current = DOG_WAIT,
};

static PreTransitionCallback pre_cb = NULL;
static PostTransitionCallback post_cb = NULL;

void RegisterPreTransitionCallback(PreTransitionCallback cb) {
    pre_cb = cb;
}

void RegisterPostTransitionCallback(PostTransitionCallback cb) {
    post_cb = cb;
}

DOG_STATE Get_Current_State(void)
{
	return g_dog_state.current;
}

SYSTEM_STATE Get_System_State(void)
{
	return g_dog_state.system;
}

void TransitionState(DOG_STATE new_state) {
    if (g_dog_state.current == new_state) return;

    if (g_dog_state.current == DOG_EMERGENCY_STOP && 
        new_state != DOG_EMERGENCY_STOP) return;

    if (pre_cb && !pre_cb(g_dog_state.current, new_state)) {
        return;
    }

    g_dog_state.previous = g_dog_state.current;
    g_dog_state.current = new_state;

    if (post_cb) {
        post_cb(g_dog_state.previous, g_dog_state.current);
    }
}

bool PreCheck(DOG_STATE from, DOG_STATE to) {
    if (to == DOG_EMERGENCY_STOP) return false; 
    if (from == DOG_STOP && to == DOG_WALK) return false; // 不能直接从STOP进入WALK
    if (from == DOG_SQUAT && to == DOG_WALK) return false; // 不能直接从SQUAT进入WALK
    return true; 
}

void PostAction(DOG_STATE from, DOG_STATE to) {
    
}

void InitStateMachine(void) {
    RegisterPreTransitionCallback(PreCheck);
    RegisterPostTransitionCallback(PostAction);
	
	if(g_dog_state.system == SYS_INIT){g_dog_state.system = SYS_ACT;}
}
