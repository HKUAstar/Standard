#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "arm_math.h"

#define M3505_MOTOR_SPEED_PID_KP 5.0f
#define M3505_MOTOR_SPEED_PID_KI 0.01f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP 700.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.00f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 20000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1000.0f

CAN_TxHeaderTypeDef  chassis_tx_message;
uint8_t              chassis_can_send_data[8];

chassis_motor_t chassis_m3508[4];
chassis_control_t chassis_control;

static void CAN_Chassis_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//-16384,+16384
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Chassis_Motor_Init(void)
{
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	
	for(uint8_t i=0;i<4;i++)
	{
		chassis_m3508[i].speed=0;
		chassis_m3508[i].speed_set=0;
		chassis_m3508[i].give_current=0;
		
		PID_init(&chassis_m3508[i].pid,PID_POSITION,motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
}

void Chassis_Motor_Data_Update(void)
{
	for(uint8_t i=0;i<4;i++)
	{
		chassis_m3508[i].speed=motor_measure_chassis[i].speed_rpm;
	}
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

static void chassis_vector_set(void)
{
	if(rc_ctrl.rc.s[1]==RC_SW_DOWN)//stop
	{
		chassis_control.vx=0;
		chassis_control.vy=0;
		chassis_control.wz=0;
	}
	else if(rc_ctrl.rc.s[1]==RC_SW_MID)//normal move
	{
		chassis_control.chassis_follow_gimbal_angle=(float)(((uint16_t)gimbal_m6020[0].ENC_angle+(8192-7847))%8192)/8192.0f*360.0f;
		if(chassis_control.chassis_follow_gimbal_angle>180)
		{
			chassis_control.chassis_follow_gimbal_angle-=360;
		}
		PID_calc(&chassis_control.chassis_follow_gimbal_pid,chassis_control.chassis_follow_gimbal_angle,0);
		chassis_control.wz=-chassis_control.chassis_follow_gimbal_pid.out;
		
		chassis_control.vx=rc_ctrl.rc.ch[3]*4;
		chassis_control.vy=rc_ctrl.rc.ch[2]*(-4);
	}
	else if(rc_ctrl.rc.s[1]==RC_SW_UP)//rotate mode
	{
		chassis_control.chassis_follow_gimbal_angle=(float)(((uint16_t)gimbal_m6020[0].ENC_angle+(8192-7847))%8192)/8192.0f*360.0f;
		if(chassis_control.chassis_follow_gimbal_angle>180)
		{
			chassis_control.chassis_follow_gimbal_angle-=360;
		}
		
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		fp32 vx,vy;
		
		vx=rc_ctrl.rc.ch[3]*4;
		vy=rc_ctrl.rc.ch[2]*(-4);
		
		sin_yaw = arm_sin_f32(-chassis_control.chassis_follow_gimbal_angle/180.0f*3.14159f);
		cos_yaw = arm_cos_f32(-chassis_control.chassis_follow_gimbal_angle/180.0f*3.14159f);
		chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
    chassis_control.vy = -sin_yaw * vx + cos_yaw * vy;

		chassis_control.wz=15000;
	}
}

void Chassis_Task(void const * argument)
{
	Chassis_Motor_Init();
	const static fp32 chassis_follow_gimbal_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP,CHASSIS_FOLLOW_GIMBAL_PID_KI,CHASSIS_FOLLOW_GIMBAL_PID_KD};
	PID_init(&chassis_control.chassis_follow_gimbal_pid,PID_POSITION,chassis_follow_gimbal_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	
	vTaskDelay(200);	
	
	while(1)
	{
		Chassis_Motor_Data_Update();
		
		fp32 motor_speed[4];
	
		chassis_vector_set();		
		chassis_vector_to_mecanum_wheel_speed(chassis_control.vx,chassis_control.vy,chassis_control.wz,motor_speed);
		
		for(uint8_t i=0;i<4;i++)
		{
			chassis_m3508[i].speed_set=motor_speed[i];
		}
		
		for(uint8_t i=0;i<4;i++)
		{
			PID_calc(&chassis_m3508[i].pid,chassis_m3508[i].speed,chassis_m3508[i].speed_set);
			chassis_m3508[i].give_current=chassis_m3508[i].pid.out;
		}
		
		CAN_Chassis_CMD(chassis_m3508[0].give_current,chassis_m3508[1].give_current,chassis_m3508[2].give_current,chassis_m3508[3].give_current);
		vTaskDelay(2);
	}
}
