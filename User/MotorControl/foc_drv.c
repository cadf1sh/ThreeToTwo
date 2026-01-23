#include "foc_drv.h"

static float Clamp_Float(float value, float min_val, float max_val)
{
  if (value < min_val)
  {
    return min_val;
  }
  if (value > max_val)
  {
    return max_val;
  }
  return value;
}


void Pack_Transform(FOC_STRUCT *p)
{
  p->Id = (p->Ialpha * p->CosVal) + (p->Ibeta * p->SinVal);
  p->Iq = (-p->Ialpha * p->SinVal) + (p->Ibeta * p->CosVal);
}


void IPack_Transform(FOC_STRUCT *p)
{
  p->Ualpha = p->Ud * p->CosVal - p->Uq * p->SinVal;
  p->Ubeta = p->Uq * p->CosVal + p->Ud * p->SinVal;
}

void Calculate_Stepper_PWM(FOC_STRUCT *p)
{
  if (p->Ubus <= 0.0f)
  {
    p->DutyCycleA = 0;
    p->DutyCycleB = 0;
    p->DutyCycleC = 0;
    p->DutyCycleD = 0;
    return;
  }

  float limit = (p->PwmLimit > 0 && p->PwmLimit < p->PwmCycle) ? p->PwmLimit : p->PwmCycle;
  float ratio_a = Clamp_Float(p->Ualpha / p->Ubus, -1.0f, 1.0f);
  float ratio_b = Clamp_Float(p->Ubeta / p->Ubus, -1.0f, 1.0f);

  float duty_a = (0.5f + 0.5f * ratio_a) * p->PwmCycle;
  float duty_b = (0.5f + 0.5f * ratio_b) * p->PwmCycle;

  duty_a = Clamp_Float(duty_a, 0.0f, limit);
  duty_b = Clamp_Float(duty_b, 0.0f, limit);

  p->DutyCycleA = (u16)duty_a;                 // A+
  p->DutyCycleB = (u16)(p->PwmCycle - duty_a); // A-
  p->DutyCycleC = (u16)duty_b;                 // B+
  p->DutyCycleD = (u16)(p->PwmCycle - duty_b); // B-
	
}
