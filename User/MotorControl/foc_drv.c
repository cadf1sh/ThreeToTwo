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
  float ratio_a = Clamp_Float(p->Ualpha / p->Ubus, -1.0f, 1.0f);
  float ratio_b = Clamp_Float(p->Ubeta / p->Ubus, -1.0f, 1.0f);

  float pwm_center = p->PwmCycle * 0.5f;
  float pwm_amplitude = p->PwmLimit * 0.5f;

  float duty_a = pwm_center + ratio_a * pwm_amplitude;
  float duty_b = pwm_center - ratio_a * pwm_amplitude;
  float duty_c = pwm_center + ratio_b * pwm_amplitude;
  float duty_d = pwm_center - ratio_b * pwm_amplitude;

  p->DutyCycleA = (u16)Clamp_Float(duty_a, 0.0f, p->PwmCycle);
  p->DutyCycleB = (u16)Clamp_Float(duty_b, 0.0f, p->PwmCycle);
  p->DutyCycleC = (u16)Clamp_Float(duty_c, 0.0f, p->PwmCycle);
  p->DutyCycleD = (u16)Clamp_Float(duty_d, 0.0f, p->PwmCycle);

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,p->DutyCycleA);     //更新PWM比较值             
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,p->DutyCycleB);     //更新PWM比较值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,p->DutyCycleC); 		 //更新PWM比较值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,p->DutyCycleD);     //更新PWM比较值
	
//  p->DutyCycleA = 8500-(u16)duty_a;                 // A+
//  p->DutyCycleB = (u16)8500; // A-
//  p->DutyCycleC = 8500-(u16)duty_b;                 // B+
//  p->DutyCycleD = (u16)8500; // B-
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,p->DutyCycleA);     //更新PWM比较值             
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,p->DutyCycleB);     //更新PWM比较值
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,p->DutyCycleC); 		 //更新PWM比较值
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,p->DutyCycleD);     //更新PWM比较值
	
}
