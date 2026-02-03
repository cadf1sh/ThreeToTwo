#ifndef __STEPPER_FOC_H
#define __STEPPER_FOC_H
#include "motor_publicdata.h"
//电流开环
#define STEPPER_FOC_OPEN_LOOP_TEST 0 //电流闭环是否开启步进角验证
#define STEPPER_FOC_OPEN_LOOP_STEP 5.0f//电流闭环步数/可正负

void Stepper_Foc_Init(void);
void Stepper_Foc_SetCurrentRef(void);
void Stepper_Foc_SetCurrentLimit(void);
void Stepper_Foc_Run(void);

extern float VF_xita ;
extern u8 VF_open_finish;
#endif