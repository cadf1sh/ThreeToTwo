#ifndef __STEPPER_FOC_H
#define __STEPPER_FOC_H

#include "motor_publicdata.h"

void Stepper_Foc_Init(void);
void Stepper_Foc_SetCurrentRef(void);
void Stepper_Foc_SetCurrentLimit(void);
void Stepper_Foc_Run(void);


#endif