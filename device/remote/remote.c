#include "remote.h"
#include "usart.h"
#include "cal.h"

static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t *, uint32_t );

static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){


 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxEventType = HAL_UART_RXEVENT_TC;

 huart->RxXferSize    = DataLength;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 

 HAL_DMAEx_MultiBufferStart(&hdma_uart5_rx,(uint32_t)SrcAddress,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength);
	

}


void BSP_USART_Init(void){

	USART_RxDMA_MultiBufferStart(&huart5,(uint32_t *)&(huart5.Instance->RDR),(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],36);
		
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	 if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
  {

			__HAL_DMA_DISABLE(huart->hdmarx);

			((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

      __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);
      if(Size == 0x000B && SBUS_MultiRx_Buf[1][0] == 0x0F  && SBUS_MultiRx_Buf[1][24] == 0x00)
      {
        SBUS_TO_RC(SBUS_MultiRx_Buf[0],&Remote_Ctrl);
				
      }
  }

  else
  {

			__HAL_DMA_DISABLE(huart->hdmarx);
		
       ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
		
      __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);
      if(Size == 0x000B && SBUS_MultiRx_Buf[1][0] == 0x0F  && SBUS_MultiRx_Buf[1][24] == 0x00)
      {
        SBUS_TO_RC(SBUS_MultiRx_Buf[1],&Remote_Ctrl);
      }
  }
	

  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
	 huart->RxEventType = HAL_UART_RXEVENT_TC;

  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  __HAL_DMA_ENABLE(huart->hdmarx);
}

Remote_Info_Typedef Remote_Ctrl={
	.online_cnt = 0xFAU,

	.rc_lost = true,
};

uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl)
{
    if (sbus_buf == NULL || Remote_Ctrl == NULL) return;

	Remote_Ctrl->CH1 = ((((uint16_t)sbus_buf[2] << 8) | (uint16_t)sbus_buf[1]) & 0x07FF);	//192~1792
	Remote_Ctrl->CH2 = ((int16_t)sbus_buf[ 2] >> 3 | ((int16_t)sbus_buf[ 3] << 5 )) & 0x07FF;//195~1795
	Remote_Ctrl->CH3 = ((int16_t)sbus_buf[ 3] >> 6 | ((int16_t)sbus_buf[ 4] << 2 ) | (int16_t)sbus_buf[ 5] << 10 ) & 0x07FF;//188~1788
	Remote_Ctrl->CH4 = ((int16_t)sbus_buf[ 5] >> 1 | ((int16_t)sbus_buf[ 6] << 7 )) & 0x07FF;//192~1792
	Remote_Ctrl->CH5 = ((int16_t)sbus_buf[ 6] >> 4 | ((int16_t)sbus_buf[ 7] << 4 )) & 0x07FF;
	Remote_Ctrl->CH6 = ((int16_t)sbus_buf[ 7] >> 7 | ((int16_t)sbus_buf[ 8] << 1 ) | (int16_t)sbus_buf[9] << 9 ) & 0x07FF;
	
	//下面ch7，ch8重复不知道哪个能用
	Remote_Ctrl->CH7 = (int16_t)sbus_buf[ 9] >> 5 | ((int16_t)sbus_buf[10] << 8);
//	Remote_Ctrl.CH8 = (int16_t)sbus_buf[11] >> 1 | ((int16_t)sbus_buf[12] << 7);
//	Remote_Ctrl.CH7 = ((int16_t)sbus_buf[ 9] >> 2 | ((int16_t)sbus_buf[ 10] << 6)) & 0x07FF;
	Remote_Ctrl->CH8 = ((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[ 11] << 3)) & 0x07FF;

	Remote_Ctrl->CH9 = ((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF;

	Remote_Ctrl->CH10 = ((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF;
	
		Remote_Ctrl->online_cnt = 0xFAU;

		Remote_Ctrl->rc_lost = false;
}

float data_convert(int src, int src_low, int src_high, float dst_low, float dst_high)
{
    if (src < src_low) src = src_low;
    if (src > src_high) src = src_high;

    if (src_low == src_high) return dst_low;

    float res = ((float)(src - src_low) / (src_high - src_low)) * (dst_high - dst_low) + dst_low;

    if (res < dst_low) res = dst_low;
    if (res > dst_high) res = dst_high;

    return res;
}
