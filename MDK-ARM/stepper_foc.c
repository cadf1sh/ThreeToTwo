#include "stepper_foc.h"
#include "math.h"

//电流开环
#define STEPPER_FOC_OPEN_LOOP_TEST 1 //电流闭环是否开启步进角验证
#define STEPPER_FOC_OPEN_LOOP_STEP 5.0f//电流闭环步数/可正负
#define STEPPER_FOC_STARTUP_STEP 2.0f
#define STEPPER_FOC_STARTUP_LOCK_THRESHOLD 16.0f
#define STEPPER_FOC_STARTUP_LOCK_COUNT 200
#if STEPPER_FOC_OPEN_LOOP_TEST
static float stepper_open_loop_electrical = 0.0f;
#endif

static float stepper_startup_electrical = 0.0f;
static u16 stepper_startup_lock_count = 0;
static u8 stepper_startup_active = 0;
static u8 stepper_last_run_mode = 0;

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

static float Stepper_Foc_AdvanceElectrical(float electrical, float step, float max_val)
{
  electrical += step;
  if (electrical >= max_val)
  {
    electrical -= max_val;
  }
  if (electrical < 0.0f)
  {
    electrical += max_val;
  }
  return electrical;
}

static float Stepper_Foc_ElectricalDiff(float a, float b, float max_val)
{
  float diff = a - b;
  if (diff > max_val * 0.5f)
  {
    diff -= max_val;
  }
  else if (diff < -max_val * 0.5f)
  {
    diff += max_val;
  }
  return diff;
}

void Stepper_Foc_Init(void)
{
  MC.Foc.IdLPFFactor = 0.1f;
  MC.Foc.IqLPFFactor = 0.1f;
  MC.Foc.IdLPF = 0.0f;
  MC.Foc.IqLPF = 0.0f;

  MC.IdPid.Kp = 0.001f;
  MC.IdPid.Ki = 0.001f;
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
  MC.Foc.Ubus = MC.Sample.BusReal;
  if (stepper_last_run_mode != MC.Motor.RunMode)
  {
    if (MC.Motor.RunMode == SPEED_CURRENT_LOOP)
    {
      stepper_startup_active = 1;
      stepper_startup_lock_count = 0;
      stepper_startup_electrical = (float)MC.Encoder.ElectricalVal;
    }
    stepper_last_run_mode = MC.Motor.RunMode;
  }
  switch (MC.Motor.RunMode)
  {
			case ENCODER_CALIB:
			{
				MC.Foc.Uq = 0.0f;
				MC.Foc.SinVal = 0;
				MC.Foc.CosVal = 0;
								if (MC.Encoder.CalibFlag == 0)
				{
					MC.Foc.Ud += 0.0001f;
					MC.Foc.SinVal =1;
					if (MC.Foc.Ud >= MC.Identify.VoltageSet[1])
					{
						MC.Foc.Ud = 0.0f;
						MC.Foc.SinVal = 0;
						MC.Encoder.CalibFlag = 1;
					}
				}
				else if (MC.Encoder.CalibFlag == 1)
				{
					MC.Foc.Ud += 0.0001f;
					MC.Foc.CosVal =1;
					if (MC.Foc.Ud >= MC.Identify.VoltageSet[1])
					{
						MC.Encoder.CalibOffset = MC.Encoder.EncoderVal;
						MC.Encoder.CalibFlag = 0;
						MC.Foc.Ud = 0.0f;
						MC.Foc.CosVal = 0;
						MC.Motor.RunMode = SPEED_CURRENT_LOOP;
					}
				}
			}break;
						
			case CURRENT_CLOSE_LOOP:
			{
			#if STEPPER_FOC_OPEN_LOOP_TEST
				stepper_open_loop_electrical += STEPPER_FOC_OPEN_LOOP_STEP;
				if (stepper_open_loop_electrical >= MC.Encoder.EncoderValMax)
				{
					stepper_open_loop_electrical -= MC.Encoder.EncoderValMax;
				}
				if (stepper_open_loop_electrical < 0.0f)
				{
					stepper_open_loop_electrical += MC.Encoder.EncoderValMax;
				}
				Calculate_Sin_Cos(stepper_open_loop_electrical, &MC.Foc.SinVal, &MC.Foc.CosVal);
			#else
				Calculate_Sin_Cos((float)MC.Encoder.ElectricalVal, &MC.Foc.SinVal, &MC.Foc.CosVal);
			#endif			
			MC.Foc.Ialpha = MC.Sample.IaReal;
      MC.Foc.Ibeta = MC.Sample.IbReal;
      Pack_Transform(&MC.Foc);

      MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1.0f - MC.Foc.IdLPFFactor);
      MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1.0f - MC.Foc.IqLPFFactor);

      MC.IdPid.Fbk = MC.Foc.IdLPF;
      PID_Control(&MC.IdPid);

      MC.IqPid.Fbk = MC.Foc.IqLPF;
      PID_Control(&MC.IqPid);

      MC.Foc.Ud = MC.IdPid.Out;
      MC.Foc.Uq = MC.IqPid.Out;
			}break;
			
			case SPEED_CURRENT_LOOP:
			{

			MC.Foc.Ialpha = MC.Sample.IaReal;
      MC.Foc.Ibeta = MC.Sample.IbReal;
      Pack_Transform(&MC.Foc);

      MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1.0f - MC.Foc.IdLPFFactor);
      MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1.0f - MC.Foc.IqLPFFactor);

      MC.IdPid.Fbk = MC.Foc.IdLPF;
      PID_Control(&MC.IdPid);

      MC.IqPid.Fbk = MC.Foc.IqLPF;
      PID_Control(&MC.IqPid);

      MC.Foc.Ud = MC.IdPid.Out;
      MC.Foc.Uq = MC.IqPid.Out;
			}break;
			
			case POS_SPEED_CURRENT_LOOP:
			{
      
			Calculate_Sin_Cos((float)MC.Encoder.ElectricalVal, &MC.Foc.SinVal, &MC.Foc.CosVal);			
			MC.Foc.Ialpha = MC.Sample.IaReal;
      MC.Foc.Ibeta = MC.Sample.IbReal;
      Pack_Transform(&MC.Foc);

      MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1.0f - MC.Foc.IdLPFFactor);
      MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1.0f - MC.Foc.IqLPFFactor);

      MC.IdPid.Fbk = MC.Foc.IdLPF;
      PID_Control(&MC.IdPid);

      MC.IqPid.Fbk = MC.Foc.IqLPF;
      PID_Control(&MC.IqPid);

      MC.Foc.Ud = MC.IdPid.Out;
      MC.Foc.Uq = MC.IqPid.Out;
			}break;

  }

	IPack_Transform(&MC.Foc);
  Calculate_Stepper_PWM(&MC.Foc);
	}