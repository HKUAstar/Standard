#ifndef _SHOOT_TASK
#define _SHOOT_TASK

#include "main.h"
#include "Gimbal_Task.h"

typedef struct
{
  fp32 speed;
  fp32 speed_set;
	fp32 angle;
  fp32 angle_set;
	fp32 ENC_angle;
  int16_t give_current;
	
	pid_type_def speed_pid;
	pid_type_def angle_pid;
} shoot_motor_t;

typedef struct
{
  fp32 speed;
  fp32 speed_set;
	int16_t give_pwm;
	
	pid_type_def pid;
} fric_motor_t;

extern shoot_motor_t shoot_m2006[1];

void Shoot_Task(void const * argument);

#endif
