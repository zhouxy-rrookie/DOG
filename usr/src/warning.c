#include "warning.h"



void System_Init(void)
{
	WS2812_Ctrl(0,255,0);
}
void motor_unconnected(void)
{
	WS2812_Ctrl(255,0,0);
}

