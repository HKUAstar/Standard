#ifndef _AUTO_AIM
#define _AUTO_AIM

#include "main.h"

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern int16_t auto_aim_vx,auto_aim_vy;

void Auto_Aim_Init(void);

#endif
