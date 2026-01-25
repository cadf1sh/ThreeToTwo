#ifndef __MOTOR_SYSTEM_H
#define __MOTOR_SYSTEM_H

#include "motor_publicdata.h"
#include "stepper_foc.h"

extern STEPPER_FOC_STRUCT stepper_foc_ctrl;

void Motor_System_Init(void);
void Motor_System_Run(void);
#endif
