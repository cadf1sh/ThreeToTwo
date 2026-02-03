#include "stepper_foc.h"
#include "math.h"

#if STEPPER_FOC_OPEN_LOOP_TEST
static float stepper_open_loop_electrical = 0.0f;
#endif

#define VF_EXIT_SPEED_LOOP_AIM      (500.0f)   // 真实速度阈值：按你的单位调（先给200）
#define VF_EXIT_CNT_TH        (20)      // 连续满足阈值次数：约等于稳定时间（按速度环执行频率调）
static uint16_t vf_exit_cnt = 0;
u8 VF_open_finish = 0;
float VF_xita = 0;
float VF_xita_rate = 300;
float VF_xita_rate_last = 300;

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

	float vxrp_error = 0;
	float vxrp_offset = 0;
	float vxrp_error_last = 0;
	float vxrp_error_lim = 0;
	float vxrp_addout = 0;
	float vxrp_kp = 0.001f;
	float vxrp_kd = 0.0005f;
	static void VF_xita_rate_pid()
	{
		vxrp_error = MC.SpdPid.Out;
		vxrp_addout = vxrp_kp*vxrp_error+vxrp_kd*(vxrp_error-vxrp_error_last);
		vxrp_error_last =vxrp_error;
	}
	
void Stepper_Foc_Init(void)
{

}

void Stepper_Foc_SetCurrentRef(void)
{
  MC.IdPid.Ref = Stepper_Foc_Clamp(MC.IdPid.Ref, -MC.IdPid.Ref_lim, MC.IdPid.Ref_lim);
  MC.IqPid.Ref = Stepper_Foc_Clamp(MC.IqPid.Ref, -MC.IdPid.Ref_lim, MC.IdPid.Ref_lim);
}

void Stepper_Foc_SetCurrentLimit(void)
{
return;
}

void Stepper_Foc_Run(void)
{
  MC.Foc.Ubus = MC.Sample.BusReal;

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
      MC.IqPid.Ref = 0.5f;
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
//			MC.Speed.SpeedCalculateCnt++;  			
//			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //每SPEED_DIVISION_FACTOR次 执行一次速度闭环
//			{
//			MC.Speed.SpeedCalculateCnt=0;
//			MC.Speed.ElectricalPosThis = MC.Encoder.ElectricalVal;         //获取当前电角度
//			Calculate_Speed(&MC.Speed);                                  	 //根据当前电角度和上次电角度计算电角速度
//			MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
//				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波	
//			MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;	

//			MC.SpdPid.Ref = (MC.Speed.MechanicalSpeedSet/4);
//			MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //反馈速度值	
//			if(MC.SpdPid.Fbk > -2000 && MC.SpdPid.Fbk < 2000) 
//			{
//				MC.SpdPid.Kp = MC.SpdPid.KpMax;
//			}
//			else
//			{
//				MC.SpdPid.Kp = MC.SpdPid.KpMin;
//			}	
//					
//			PID_Control(&MC.SpdPid);
//			MC.IqPid.Ref = MC.SpdPid.Out;
//			
//			}
			Calculate_Sin_Cos((float)MC.Encoder.ElectricalVal, &MC.Foc.SinVal, &MC.Foc.CosVal);	
//			MC.Foc.Ialpha = MC.Sample.IaReal;
//      MC.Foc.Ibeta = MC.Sample.IbReal;
//      Pack_Transform(&MC.Foc);

//      MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1.0f - MC.Foc.IdLPFFactor);
//      MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1.0f - MC.Foc.IqLPFFactor);

//      MC.IdPid.Fbk = MC.Foc.IdLPF;
//      PID_Control(&MC.IdPid);

//      MC.IqPid.Fbk = MC.Foc.IqLPF;
//      PID_Control(&MC.IqPid);

//      MC.Foc.Ud = MC.IdPid.Out;
////      MC.Foc.Uq = MC.IqPid.Out;
//			MC.Foc.SinVal=0, MC.Foc.CosVal=1;
      MC.Foc.Ud = 3;
      MC.Foc.Uq = 0;
			}break;
			
			case POS_SPEED_CURRENT_LOOP:
			{
      MC.IqPid.Ref = 0.5f;
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

	
	
	
	
