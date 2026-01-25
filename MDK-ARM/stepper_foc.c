#include "stepper_foc.h"

static float Stepper_Foc_Clamp(float value, float min_val, float max_val)
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

void Stepper_Foc_Init(STEPPER_FOC_STRUCT *ctrl, float pwm_cycle, float pwm_limit)
{
  if (!ctrl)
  {
    return;
  }

  ctrl->foc.PwmCycle = pwm_cycle;
  ctrl->foc.PwmLimit = pwm_limit;
  ctrl->foc.IdLPFFactor = 0.1f;
  ctrl->foc.IqLPFFactor = 0.1f;
  ctrl->foc.IdLPF = 0.0f;
  ctrl->foc.IqLPF = 0.0f;

  ctrl->id_pid.Kp = 0.5f;
  ctrl->id_pid.Ki = 0.003f;
  ctrl->id_pid.Kd = 0.0f;
  ctrl->id_pid.OutMax = 3.0f;
  ctrl->id_pid.OutMin = -3.0f;

  ctrl->iq_pid.Kp = 0.5f;
  ctrl->iq_pid.Ki = 0.003f;
  ctrl->iq_pid.Kd = 0.0f;
  ctrl->iq_pid.OutMax = 3.0f;
  ctrl->iq_pid.OutMin = -3.0f;

  ctrl->id_ref = 0.0f;
  ctrl->iq_ref = 1.0f;
  ctrl->id_ref_limit = 3.0f;
  ctrl->iq_ref_limit = 3.0f;
}

void Stepper_Foc_SetCurrentRef(STEPPER_FOC_STRUCT *ctrl, float id_ref, float iq_ref)
{
  if (!ctrl)
  {
    return;
  }

  ctrl->id_ref = Stepper_Foc_Clamp(id_ref, -ctrl->id_ref_limit, ctrl->id_ref_limit);
  ctrl->iq_ref = Stepper_Foc_Clamp(iq_ref, -ctrl->iq_ref_limit, ctrl->iq_ref_limit);
}

void Stepper_Foc_SetCurrentLimit(STEPPER_FOC_STRUCT *ctrl, float id_limit, float iq_limit)
{
  if (!ctrl)
  {
    return;
  }

  ctrl->id_ref_limit = (id_limit > 0.0f) ? id_limit : ctrl->id_ref_limit;
  ctrl->iq_ref_limit = (iq_limit > 0.0f) ? iq_limit : ctrl->iq_ref_limit;
}

void Stepper_Foc_Run(STEPPER_FOC_STRUCT *ctrl, float iu, float iw, float elec_angle, float bus_voltage)
{
  if (!ctrl)
  {
    return;
  }

  ctrl->foc.Iu = iu;
  ctrl->foc.Iw = iw;
  ctrl->foc.Ialpha = iu;
  ctrl->foc.Ibeta = iw;
  ctrl->foc.Ubus = bus_voltage;

  Calculate_Sin_Cos(elec_angle, &ctrl->foc.SinVal, &ctrl->foc.CosVal);
  Pack_Transform(&ctrl->foc);

  ctrl->foc.IdLPF = ctrl->foc.Id * ctrl->foc.IdLPFFactor + ctrl->foc.IdLPF * (1.0f - ctrl->foc.IdLPFFactor);
  ctrl->foc.IqLPF = ctrl->foc.Iq * ctrl->foc.IqLPFFactor + ctrl->foc.IqLPF * (1.0f - ctrl->foc.IqLPFFactor);

  ctrl->id_pid.Ref = ctrl->id_ref;
  ctrl->id_pid.Fbk = ctrl->foc.IdLPF;
  PID_Control(&ctrl->id_pid);
	
  ctrl->iq_pid.Ref = ctrl->iq_ref;
  ctrl->iq_pid.Fbk = ctrl->foc.IqLPF;
  PID_Control(&ctrl->iq_pid);

  ctrl->foc.Ud = ctrl->id_pid.Out;
  ctrl->foc.Uq = ctrl->iq_pid.Out;
  IPack_Transform(&ctrl->foc);

  Calculate_Stepper_PWM(&ctrl->foc);
}
