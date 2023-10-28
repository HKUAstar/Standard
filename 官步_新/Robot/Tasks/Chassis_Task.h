#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"

#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204
#define CHASSIS_CAN hcan1

#define MOTOR_DISTANCE_TO_CENTER 0.2f
#define CHASSIS_WZ_SET_SCALE 0.0f

typedef struct
{
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	
	pid_type_def pid;
}chassis_motor_t;

typedef struct
{
  fp32 vx;
	fp32 vy;
	fp32 wz;
	
  fp32 chassis_follow_gimbal_angle;	
	pid_type_def chassis_follow_gimbal_pid;
}chassis_control_t;

extern chassis_motor_t chassis_m3508[4];

void Chassis_Task(void const * argument);

#endif
