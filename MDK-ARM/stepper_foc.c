#include "stepper_foc.h"

//电流开环
#define STEPPER_FOC_OPEN_LOOP_TEST 1 //电流闭环是否开启步进角验证
#define STEPPER_FOC_OPEN_LOOP_STEP 5.0f//电流闭环步数/可正负
#if STEPPER_FOC_OPEN_LOOP_TEST
static float stepper_open_loop_electrical = 0.0f;
#endif

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
			MC.Speed.SpeedCalculateCnt++;  			
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //每SPEED_DIVISION_FACTOR次 执行一次速度闭环
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.Encoder.ElectricalVal;         //获取当前电角度
				Calculate_Speed(&MC.Speed);                                  	 //根据当前电角度和上次电角度计算电角速度
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波	
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
				
				if(MC.Speed.MechanicalSpeedSet != MC.Speed.MechanicalSpeedSetLast)               //给定了新的目标速度
				{                                                      						
					MC.TAccDec.StartSpeed = MC.Speed.MechanicalSpeedSetLast * MC.Encoder.PolePairs;//设置初速度
					MC.TAccDec.EndSpeed   = MC.Speed.MechanicalSpeedSet     * MC.Encoder.PolePairs;//设置末速度
					T_Shaped_Acc_Dec(&MC.TAccDec);                                                 //T形加减速计算
					if(MC.TAccDec.FinishFlag == 1)                                                 //执行完加减速
					{
						MC.Speed.MechanicalSpeedSetLast = MC.Speed.MechanicalSpeedSet;               //更新上次目标速度
						MC.TAccDec.FinishFlag = 0;
					}					
				}		
				MC.SpdPid.Ref = MC.TAccDec.SpeedOut;					                 //获得目标值   
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //反馈速度值	
				if(MC.SpdPid.Fbk > -2000 && MC.SpdPid.Fbk < 2000) 
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMax;
				}
				else
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMin;
				}			
				PID_Control(&MC.SpdPid);                            					 //速度闭环
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}
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
			
			case POS_SPEED_CURRENT_LOOP:
			{
			MC.Position.PosCalculateCnt++;
			MC.Speed.SpeedCalculateCnt++;			
			if(MC.Position.PosCalculateCnt >= POS_DIVISION_FACTOR)           //POS_DIVISION_FACTOR 执行一次位置闭环
			{											
				MC.Position.PosCalculateCnt = 0;			
				MC.Position.ElectricalPosThis = MC.Encoder.ElectricalVal;			 //获取当前位置
				Calculate_Position(&MC.Position);                              //计算总位置
				MC.PosPid.Fbk = MC.Position.ElectricalPosSum;								   //反馈实际位置
				MC.PosPid.Ref = MC.Position.MechanicalPosSet * POLEPAIRS;			 //给定目标位置
				MC.Position.MechanicalPosRaw = MC.Position.ElectricalPosSum / POLEPAIRS;
				PID_Control(&MC.PosPid);                                       //位置闭环
			}
					
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)					 //SPEED_DIVISION_FACTOR 执行一次速度闭环
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.Encoder.ElectricalVal;
				Calculate_Speed(&MC.Speed);                                    //计算速度
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / POLEPAIRS;	
				
				MC.SpdPid.Ref = MC.PosPid.Out;					
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //反馈速度值	
				if(MC.SpdPid.Fbk > -2000 && MC.SpdPid.Fbk < 2000) 
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMax;
				}
				else
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMin;
				}							
				PID_Control(&MC.SpdPid);                            					 //速度闭环
				MC.IqPid.Ref = MC.SpdPid.Out;										
			}        
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