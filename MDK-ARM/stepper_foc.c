#include "stepper_foc.h"

static float stepper_id_ref_limit = 3.0f;
static float stepper_iq_ref_limit = 3.0f;

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

void Stepper_Foc_Init(void)
{
  MC.Foc.IdLPFFactor = 0.1f;
  MC.Foc.IqLPFFactor = 0.1f;
  MC.Foc.IdLPF = 0.0f;
  MC.Foc.IqLPF = 0.0f;

  MC.IdPid.Kp = 0.2f;
  MC.IdPid.Ki = 0.002f;
  MC.IdPid.Kd = 0.0f;
  MC.IdPid.OutMax = 6.0f;
  MC.IdPid.OutMin = -6.0f;

  MC.IqPid.Kp = 0.001f;
  MC.IqPid.Ki = 0.001f;
  MC.IqPid.Kd = 0.0f;
  MC.IqPid.OutMax = 6.0f;
  MC.IqPid.OutMin = -6.0f;

  MC.IdPid.Ref = 0.0f;
  MC.IqPid.Ref = 1.0f;
  stepper_id_ref_limit = 3.0f;
  stepper_iq_ref_limit = 3.0f;
}

void Stepper_Foc_SetCurrentRef(void)
{
  MC.IdPid.Ref = Stepper_Foc_Clamp(MC.IdPid.Ref, -stepper_id_ref_limit, stepper_id_ref_limit);
  MC.IqPid.Ref = Stepper_Foc_Clamp(MC.IqPid.Ref, -stepper_iq_ref_limit, stepper_iq_ref_limit);
}

void Stepper_Foc_SetCurrentLimit(void)
{
return;
}

void Stepper_Foc_Run(void)
{

  MC.Foc.Ialpha = MC.Sample.IaReal;
  MC.Foc.Ibeta = MC.Sample.IbReal;
  MC.Foc.Ubus = MC.Sample.BusReal;
	
	MC.Encoder.ElectricalVal += 10;
  Calculate_Sin_Cos((float)MC.Encoder.ElectricalVal, &MC.Foc.SinVal, &MC.Foc.CosVal);
  Pack_Transform(&MC.Foc);

  MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1.0f - MC.Foc.IdLPFFactor);
  MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1.0f - MC.Foc.IqLPFFactor);

  MC.IdPid.Fbk = MC.Foc.IdLPF;
  PID_Control(&MC.IdPid);
	
  MC.IqPid.Fbk = MC.Foc.IqLPF;
  PID_Control(&MC.IqPid);

  MC.Foc.Ud = MC.IdPid.Out;
  MC.Foc.Uq = MC.IqPid.Out;
  IPack_Transform(&MC.Foc);

  Calculate_Stepper_PWM(&MC.Foc);
}