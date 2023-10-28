#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "auto_aim.h"

#include "arm_math.h"

#define YAW_MOTOR_SPEED_PID_KP 6000.0f
#define YAW_MOTOR_SPEED_PID_KI 100.0f
#define YAW_MOTOR_SPEED_PID_KD 8000.0f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 25000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define YAW_MOTOR_ANGLE_PID_KP 0.5f
#define YAW_MOTOR_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ANGLE_PID_KD 6.0f
#define YAW_MOTOR_ANGLE_PID_MAX_OUT 3.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 100.0f

#define YAW_MOTOR_AUTO_AIM_PID_KP 0.05f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define YAW_MOTOR_AUTO_AIM_PID_KD 0.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 3.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

#define PITCH_MOTOR_SPEED_PID_KP 6000.0f
#define PITCH_MOTOR_SPEED_PID_KI 100.0f
#define PITCH_MOTOR_SPEED_PID_KD 8000.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 25000.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define PITCH_MOTOR_ANGLE_PID_KP 0.5f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 6.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 6.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 100.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.01f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.0f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 0.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 3.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

CAN_TxHeaderTypeDef  gimbal_tx_message;
uint8_t              gimbal_can_send_data[8];

gimbal_motor_t gimbal_m6020[2];
uint8_t yaw_mode=0,yaw_mode_last=0;//0:speed,1:angle
uint8_t pitch_mode=0,pitch_mode_last=0;//0:speed,1:angle

int16_t auto_aim_err_yaw=0,auto_aim_err_pitch=0;

static void CAN_Gimbal_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//-30000,+30000
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = motor1 >> 8;
	gimbal_can_send_data[1] = motor1;
	gimbal_can_send_data[2] = motor2 >> 8;
	gimbal_can_send_data[3] = motor2;
	gimbal_can_send_data[4] = motor3 >> 8;
	gimbal_can_send_data[5] = motor3;
	gimbal_can_send_data[6] = motor4 >> 8;
	gimbal_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void Gimbal_Motor_Init(void)
{
	const static fp32 yaw_motor_speed_pid[3] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD};
	const static fp32 yaw_motor_angle_pid[3] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD};
	const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};
	
	const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
	const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
	const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};
	
	for(uint8_t i=0;i<2;i++)
	{
		gimbal_m6020[i].INS_speed=0;
		gimbal_m6020[i].INS_speed_set=0;
		gimbal_m6020[i].INS_angle=0;
		gimbal_m6020[i].INS_angle_set=0;
		gimbal_m6020[i].ENC_angle=0;
		gimbal_m6020[i].ENC_angle_actual=0;
		gimbal_m6020[i].ENC_angle_set=0;
		gimbal_m6020[i].give_current=0;
	}
	
	PID_init(&gimbal_m6020[0].speed_pid,PID_POSITION,yaw_motor_speed_pid,YAW_MOTOR_SPEED_PID_MAX_OUT,YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_m6020[0].angle_pid,PID_POSITION,yaw_motor_angle_pid,YAW_MOTOR_ANGLE_PID_MAX_OUT,YAW_MOTOR_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_m6020[0].auto_aim_pid,PID_POSITION,yaw_motor_auto_aim_pid,YAW_MOTOR_AUTO_AIM_PID_MAX_OUT,YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
	
	PID_init(&gimbal_m6020[1].speed_pid,PID_POSITION,pitch_motor_speed_pid,PITCH_MOTOR_SPEED_PID_MAX_OUT,PITCH_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_m6020[1].angle_pid,PID_POSITION,pitch_motor_angle_pid,PITCH_MOTOR_ANGLE_PID_MAX_OUT,PITCH_MOTOR_ANGLE_PID_MAX_IOUT);	
	PID_init(&gimbal_m6020[1].auto_aim_pid,PID_POSITION,pitch_motor_auto_aim_pid,PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT,PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);	
}

void Gimbal_Motor_Data_Update(void)
{
	//yaw
	//z2+y2
	fp32 temp;
	
	arm_sqrt_f32(bmi088_real_data.gyro[2]*bmi088_real_data.gyro[2]+bmi088_real_data.gyro[1]*bmi088_real_data.gyro[1],&temp);
	
	if(bmi088_real_data.gyro[2]<0)
	{
		temp=-temp;
	}
	gimbal_m6020[0].INS_speed=temp*0.1f+gimbal_m6020[0].INS_speed*0.9f;
	
//	gimbal_m6020[0].INS_speed=bmi088_real_data.gyro[2];
	gimbal_m6020[0].INS_angle=INS_angle_deg[0];
	gimbal_m6020[0].ENC_angle=motor_measure_gimbal[0].ecd;	
	
	//pitch
//	gimbal_m6020[1].INS_speed=(float)motor_measure_gimbal[1].speed_rpm/60.0f*2*3.14159;
	gimbal_m6020[1].INS_speed=bmi088_real_data.gyro[0]*0.1f+gimbal_m6020[1].INS_speed*0.9f;
//	gimbal_m6020[1].INS_speed=bmi088_real_data.gyro[0];
	gimbal_m6020[1].INS_angle=INS_angle_deg[2];
	gimbal_m6020[1].ENC_angle=motor_measure_gimbal[1].ecd;	
}

void Yaw_Motor_Control(void)
{
	if(rc_ctrl.rc.s[0]==RC_SW_MID&&(auto_aim_vx!=0||auto_aim_vy!=0))//auto aim test
	{
		auto_aim_err_yaw=auto_aim_vx-320;
		
		PID_calc(&gimbal_m6020[0].auto_aim_pid,auto_aim_err_yaw,0);
//		gimbal_m6020[0].INS_speed_set=0;
		gimbal_m6020[0].INS_speed_set=gimbal_m6020[0].auto_aim_pid.out;
		
		gimbal_m6020[0].INS_angle_set=gimbal_m6020[0].INS_angle;
	}
	else
	{
		yaw_mode_last=yaw_mode;
		if(rc_ctrl.rc.ch[0]>10||rc_ctrl.rc.ch[0]<-10)
		{			
			yaw_mode=0;
		}
		else
		{
			yaw_mode=1;
		}

		if(yaw_mode==0)
		{
			gimbal_m6020[0].INS_speed_set=-(float)rc_ctrl.rc.ch[0]/660.0f*3;
		}
		else if(yaw_mode==1&&yaw_mode_last==0)
		{
			gimbal_m6020[0].INS_angle_set=gimbal_m6020[0].INS_angle;
		}

		if(yaw_mode==1)
		{
			PID_calc(&gimbal_m6020[0].angle_pid,gimbal_m6020[0].INS_angle,gimbal_m6020[0].INS_angle_set);
			gimbal_m6020[0].INS_speed_set=gimbal_m6020[0].angle_pid.out;
		}
	}

	PID_calc(&gimbal_m6020[0].speed_pid,gimbal_m6020[0].INS_speed,gimbal_m6020[0].INS_speed_set);
	gimbal_m6020[0].give_current=gimbal_m6020[0].speed_pid.out;
}

void Pitch_Motor_Control(void)
{
	gimbal_m6020[1].ENC_angle_actual=(float)(((uint16_t)gimbal_m6020[1].ENC_angle+(8192-5077))%8192)/8192.0f*360.0f;
	if(gimbal_m6020[1].ENC_angle_actual>180)
	{
		gimbal_m6020[1].ENC_angle_actual-=360;
	}
	
	if(rc_ctrl.rc.s[0]==RC_SW_MID&&(auto_aim_vx!=0||auto_aim_vy!=0))//auto aim test
	{
		auto_aim_err_pitch=auto_aim_vy-240;
		
		PID_calc(&gimbal_m6020[1].auto_aim_pid,auto_aim_err_pitch,0);
//		gimbal_m6020[1].INS_speed_set=0;
		gimbal_m6020[1].INS_speed_set=gimbal_m6020[1].auto_aim_pid.out;
		
		gimbal_m6020[1].INS_angle_set=gimbal_m6020[1].INS_angle;
	}
	else
	{
		pitch_mode_last=pitch_mode;
		if(rc_ctrl.rc.ch[1]>10||rc_ctrl.rc.ch[1]<-10)
		{			
			pitch_mode=0;
		}
		else
		{
			pitch_mode=1;
		}
		
		if(pitch_mode==0)
		{
			gimbal_m6020[1].INS_speed_set=-(float)rc_ctrl.rc.ch[1]/660.0f*2;
		}
		else if(pitch_mode==1&&pitch_mode_last==0)
		{
			gimbal_m6020[1].INS_angle_set=gimbal_m6020[1].INS_angle;
			
	//		gimbal_m6020[1].ENC_angle_set=gimbal_m6020[1].ENC_angle_actual;
		}
		
		if(pitch_mode==1)
		{
			PID_calc(&gimbal_m6020[1].angle_pid,gimbal_m6020[1].INS_angle,gimbal_m6020[1].INS_angle_set);
			gimbal_m6020[1].INS_speed_set=-gimbal_m6020[1].angle_pid.out;
			
	//		PID_calc(&gimbal_m6020[1].angle_pid,gimbal_m6020[1].ENC_angle_actual,gimbal_m6020[1].ENC_angle_set);
	//		gimbal_m6020[1].INS_speed_set=gimbal_m6020[1].angle_pid.out;
		}
	}
	
	PID_calc(&gimbal_m6020[1].speed_pid,gimbal_m6020[1].INS_speed,gimbal_m6020[1].INS_speed_set);
	gimbal_m6020[1].give_current=gimbal_m6020[1].speed_pid.out;
}

uint32_t ylj=0;
void Gimbal_Task(void const * argument)
{
	Gimbal_Motor_Init();
	
	vTaskDelay(200);
	
	while(1)
	{
		Gimbal_Motor_Data_Update();

		Yaw_Motor_Control();
		Pitch_Motor_Control();
		
		CAN_Gimbal_CMD(gimbal_m6020[0].give_current,gimbal_m6020[1].give_current,shoot_m2006[0].give_current,0);
		
		ylj++;
		
		vTaskDelay(1);
	}
}
