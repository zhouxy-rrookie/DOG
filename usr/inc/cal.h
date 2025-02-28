#ifndef __CAL_H__
#define __CAL_H__

#include "math.h"

#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

#define ABS(x)  ((x) >= 0? (x) : -(x))

typedef struct{
	float legs[4];
	float alpha;
	float beta;
	float foot_pos[2];
	float foot_vel[2];
}Leg_Param;

#endif