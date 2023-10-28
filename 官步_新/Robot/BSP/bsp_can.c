#include "bsp_can.h"
#include "main.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "detect_task.h"

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint8_t rx_data[8];
motor_measure_t motor_measure_chassis[4];
motor_measure_t motor_measure_gimbal[3];
uint32_t id=0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if(rx_header.StdId!=CAN_3508_M1_ID&&rx_header.StdId!=CAN_3508_M2_ID&&rx_header.StdId!=CAN_3508_M3_ID&&rx_header.StdId!=CAN_3508_M4_ID)
		id=rx_header.StdId;
	
	switch (rx_header.StdId)
	{
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_3508_M1_ID;
			
			get_motor_measure(&motor_measure_chassis[i], rx_data);
			
			detect_hook(CHASSIS_MOTOR1_TOE + i);
			break;
		}
		case CAN_6020_M1_ID:
		case CAN_6020_M2_ID:
		case CAN_2006_M1_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_6020_M1_ID;
			
			get_motor_measure(&motor_measure_gimbal[i], rx_data);
			
			break;
		}
		default:
		{
			break;
		}
	}
}
