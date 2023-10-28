#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void can_filter_init(void);

extern motor_measure_t motor_measure_chassis[4];
extern motor_measure_t motor_measure_gimbal[3];

#endif
