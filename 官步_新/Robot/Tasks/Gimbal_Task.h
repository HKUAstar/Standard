#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "struct_typedef.h"
#include "pid.h"

#define CAN_GIMBAL_ALL_ID 0x1FF
#define CAN_6020_M1_ID 0x205
#define CAN_6020_M2_ID 0x206
#define CAN_2006_M1_ID 0x207
#define GIMBAL_CAN hcan1

typedef struct
{
  fp32 INS_speed;
  fp32 INS_speed_set;
	fp32 INS_angle;
  fp32 INS_angle_set;
	fp32 ENC_angle;
	fp32 ENC_angle_actual;
	fp32 ENC_angle_set;
  int16_t give_current;
	
	pid_type_def speed_pid;
	pid_type_def angle_pid;
	pid_type_def auto_aim_pid;
} gimbal_motor_t;

extern gimbal_motor_t gimbal_m6020[2];

void Gimbal_Task(void const * argument);

#endif
