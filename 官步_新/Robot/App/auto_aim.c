#include "auto_aim.h"
#include "usart.h"

#define auto_aim_rxbuf_len 20
#define auto_aim_frame_len 10

uint8_t auto_aim_rxbuf[2][auto_aim_rxbuf_len];

int16_t auto_aim_vx=0,auto_aim_vy=0;

uint32_t auto_aim_cnt=0;

void Auto_Aim_Init(void)
{
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	
	__HAL_DMA_DISABLE(&hdma_usart6_rx);
	while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
	}
	
	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
	hdma_usart6_rx.Instance->M0AR = (uint32_t)(auto_aim_rxbuf[0]);
	hdma_usart6_rx.Instance->M1AR = (uint32_t)(auto_aim_rxbuf[1]);
	hdma_usart6_rx.Instance->NDTR = auto_aim_rxbuf_len;
	
	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
	
	__HAL_DMA_ENABLE(&hdma_usart6_rx);
}

void auto_aim_rxbuf_solve(volatile const uint8_t * rxbuf,int16_t * dx,int16_t * dy)
{
	if(rxbuf[0]!=0xAA||rxbuf[1]!=auto_aim_frame_len||rxbuf[2]!=0x81)
	{
		*dx=0;
		*dy=0;
		return;
	}
	*dx=(rxbuf[4]<<8)|rxbuf[3];
	*dy=(rxbuf[6]<<8)|rxbuf[5];
	
	if(*dx==30000||*dy==20000) 
	{
		*dx=0;
		*dy=0;
	}
	
	auto_aim_cnt++;
}

void USART6_IRQHandler(void)
{
	if(huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart6);
	}
	else if(USART6->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart6);

		if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = auto_aim_rxbuf_len - hdma_usart6_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart6_rx.Instance->NDTR = auto_aim_rxbuf_len;

			//set memory buffer 1
			//设定缓冲区1
			hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if(this_time_rx_len == auto_aim_frame_len)
			{
				auto_aim_rxbuf_solve(auto_aim_rxbuf[0],&auto_aim_vx,&auto_aim_vy);
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = auto_aim_rxbuf_len - hdma_usart6_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart6_rx.Instance->NDTR = auto_aim_rxbuf_len;

			//set memory buffer 0
			//设定缓冲区0
			DMA2_Stream1->CR &= ~(DMA_SxCR_CT);

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if(this_time_rx_len == auto_aim_frame_len)
			{
				auto_aim_rxbuf_solve(auto_aim_rxbuf[1],&auto_aim_vx,&auto_aim_vy);
			}
		}
	}
}
