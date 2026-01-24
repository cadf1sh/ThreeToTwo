#ifndef __MOTOR_SYSTEM_H
#define __MOTOR_SYSTEM_H

#include "motor_sensoruse.h"
#include "motor_publicdata.h"
#include "stepper_foc.h"

void Motor_System_Init(void);
void Motor_System_Run(void);
void Motor_System_SetCurrentRef(float id_ref, float iq_ref);

#endif
