#include "Shoot_Task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "bsp_tim.h"

#define SHOOT_MOTOR_SPEED_PID_KP 10.0f
#define SHOOT_MOTOR_SPEED_PID_KI 0.5f
#define SHOOT_MOTOR_SPEED_PID_KD 0.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_OUT 28000.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define FRIC_MOTOR_SPEED_PID_KP 5.0f
#define FRIC_MOTOR_SPEED_PID_KI 0.1f
#define FRIC_MOTOR_SPEED_PID_KD 0.0f
#define FRIC_MOTOR_SPEED_PID_MAX_OUT 500.0f
#define FRIC_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

shoot_motor_t shoot_m2006[1];
fric_motor_t fric_2212[2];
uint8_t fric_mode=0;//0:stop,1:start,2:on

void Shoot_Motor_Init(void)
{
	const static fp32 shoot_motor_speed_pid[3] = {SHOOT_MOTOR_SPEED_PID_KP, SHOOT_MOTOR_SPEED_PID_KI, SHOOT_MOTOR_SPEED_PID_KD};
	const static fp32 fric_motor_speed_pid[3] = {FRIC_MOTOR_SPEED_PID_KP, FRIC_MOTOR_SPEED_PID_KI, FRIC_MOTOR_SPEED_PID_KD};
	
	shoot_m2006[0].speed=0;
	shoot_m2006[0].speed_set=0;
	shoot_m2006[0].angle=0;
	shoot_m2006[0].angle_set=0;
	shoot_m2006[0].ENC_angle=0;
	shoot_m2006[0].give_current=0;
	
	PID_init(&shoot_m2006[0].speed_pid,PID_POSITION,shoot_motor_speed_pid,SHOOT_MOTOR_SPEED_PID_MAX_OUT,SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	
	for(uint8_t i=0;i<2;i++)
	{
		fric_2212[i].speed=0;;
  	fric_2212[i].speed_set=0;
		fric_2212[i].give_pwm=0;
		PID_init(&fric_2212[i].pid,PID_POSITION,fric_motor_speed_pid,FRIC_MOTOR_SPEED_PID_MAX_OUT,FRIC_MOTOR_SPEED_PID_MAX_IOUT);
	}	
}

void Shoot_Motor_Data_Update(void)
{
	shoot_m2006[0].speed=motor_measure_gimbal[2].speed_rpm;
	shoot_m2006[0].ENC_angle=motor_measure_gimbal[2].ecd;
//	shoot_m2006[0].angle=motor_measure_gimbal[1].ecd;	
	
	fric_2212[0].speed=fric_speed1;
	fric_2212[1].speed=fric_speed2;
	
	fric_2212[0].speed_set=40;
	fric_2212[1].speed_set=40;
}

void Dial_Motor_Control(void)
{
	shoot_m2006[0].give_current=0;
	return;
	
	if(rc_ctrl.rc.s[0]==RC_SW_UP&&fric_mode==2)
	{
		shoot_m2006[0].speed_set=-500;
	}
	else
	{
		shoot_m2006[0].speed_set=0;
	}
	
	PID_calc(&shoot_m2006[0].speed_pid,shoot_m2006[0].speed,shoot_m2006[0].speed_set);
	shoot_m2006[0].give_current=shoot_m2006[0].speed_pid.out;
}

void Fric_Motor_Control(void)
{
	TIM12->CCR1=1000;
	TIM12->CCR2=1000;
	return;
	
	if(rc_ctrl.rc.s[0]==RC_SW_UP||rc_ctrl.rc.s[0]==RC_SW_MID)
	{
		if(fric_mode==0)
		{
			fric_mode=1;
		}
		else if(fric_mode==1)
		{
			if(TIM12->CCR1>1050)
			{
				fric_mode=2;
			}
			else 
			{
				TIM12->CCR1+=10;
				TIM12->CCR2+=10;
				vTaskDelay(100);
			}
		}
		else if(fric_mode==2)
		{
			for(uint8_t i=0;i<2;i++)
			{
				PID_calc(&fric_2212[i].pid,fric_2212[i].speed,fric_2212[i].speed_set);
				fric_2212[i].give_pwm=fric_2212[i].pid.out;
			}
			
			if(fric_2212[0].give_pwm>500) fric_2212[0].give_pwm=500;
			if(fric_2212[0].give_pwm<50) fric_2212[0].give_pwm=50;
			
			if(fric_2212[1].give_pwm>500) fric_2212[1].give_pwm=500;
			if(fric_2212[1].give_pwm<50) fric_2212[1].give_pwm=50;
			
			TIM12->CCR1=fric_2212[0].give_pwm+1000;
			TIM12->CCR2=fric_2212[1].give_pwm+1000;
		}
	}
	else
	{
		fric_mode=0;
		TIM12->CCR1=1000;
		TIM12->CCR2=1000;
	}
}

void Shoot_Task(void const * argument)
{
	Shoot_Motor_Init();
	vTaskDelay(200);
	
	while(1)
	{
		Shoot_Motor_Data_Update();
				
		Fric_Motor_Control();
		Dial_Motor_Control();
		
		vTaskDelay(2);
	}
}
