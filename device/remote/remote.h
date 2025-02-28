#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

#define SBUS_RX_BUF_NUM 50u

#define RC_FRAME_LENGTH 25u

#define RC_CH_VALUE_OFFSET 1024U

extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];



typedef  struct
{
    uint16_t CH1;  
    uint16_t CH2;  
    uint16_t CH3;  
    uint16_t CH4;
    uint16_t CH5;
    uint16_t CH6;
    uint16_t CH7;
    uint16_t CH8;
    uint16_t CH9;
    uint16_t CH10;
		uint16_t CH11;
		uint16_t CH12;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

extern Remote_Info_Typedef Remote_Ctrl;

extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl);

void BSP_USART_Init(void);

#endif
