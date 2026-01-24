#ifndef __STEPPER_FOC_H
#define __STEPPER_FOC_H

#include "foc_drv.h"
#include "pid_drv.h"
#include "math_drv.h"
#include "motor_publicdata.h"

typedef struct
{
  FOC_STRUCT foc;
  PID_STRUCT id_pid;
  PID_STRUCT iq_pid;
  float id_ref;
  float iq_ref;
}STEPPER_FOC_STRUCT;

void Stepper_Foc_Init(STEPPER_FOC_STRUCT *ctrl, float pwm_cycle, float pwm_limit);
void Stepper_Foc_SetCurrentRef(STEPPER_FOC_STRUCT *ctrl, float id_ref, float iq_ref);
void Stepper_Foc_Run(STEPPER_FOC_STRUCT *ctrl, float iu, float iw, float elec_angle, float bus_voltage);

#endif
