/**
  ******************************************************************************
  * ?: 
  *     : ?
  *     : V1.0
  * д: 
  *     : 
  ******************************************************************************
  */
/* ?? ----------------------------------------------------------------*/

#include "motor_sensoruse.h"

/**
  * :ип 
  * :
  * ?:
  * ?    : 
  */
void Sensoruse_Control()
{
#if MOTOR_TWO_PHASE_STEPPER
	MC.Encoder.ElectricalSpdSet = MC.Speed.MechanicalSpeedSet; // Stepper: use speed set as electrical speed command
	Electrical_Angle_Generator(&MC.Encoder);
	Calculate_Sin_Cos(MC.Encoder.ElectricalValSet,&MC.Foc.SinVal,&MC.Foc.CosVal);
#else
	Calculate_Sin_Cos(MC.Encoder.ElectricalVal,&MC.Foc.SinVal,&MC.Foc.CosVal);	
#endif
	switch(MC.Motor.RunMode)
	{
		case ENCODER_CALIB:                                                //У?
		{	
			if(MC.Encoder.CalibFlag == 0)                                    //?ζλ90
			{
				MC.Foc.Ud += 0.0001f;
				MC.Foc.Uq = 0;
				MC.Foc.SinVal = 1;                                             //??90????1
				MC.Foc.CosVal = 0;                                             //??90????0		 								
				if(MC.Foc.Ud >= MC.Identify.VoltageSet[1])                     //У???????
				{
					MC.Foc.Ud = 0;
					MC.Encoder.CalibFlag = 1;                                    //?ζλ?е?
				}				
			} 	
			
			if(MC.Encoder.CalibFlag == 1)                                    //?ζλ0
			{
				MC.Foc.Ud += 0.0001f;
				MC.Foc.Uq = 0;
				MC.Foc.SinVal = 0;                                             //??0???0
				MC.Foc.CosVal = 1;                                             //??0???1		 				                  
				
				if(MC.Foc.Ud >= MC.Identify.VoltageSet[1])                     //У???????
				{
					MC.Encoder.CalibOffset = MC.Encoder.EncoderVal;              //??
					MC.Encoder.CalibFlag = 0;
					MC.Foc.Ud = 0;
					MC.Motor.RunMode = POS_SPEED_CURRENT_LOOP;                   //?У???
				}	  
			}
	  IPack_Transform(&MC.Foc);                                          //PACK任			
		}break;				
		
		case CURRENT_OPEN_LOOP:                                            //Uq??
		{   	
			IPack_Transform(&MC.Foc);                                        //PACK任
		}break;	
		
		case CURRENT_CLOSE_LOOP:                                           //?Iq_ref?
		{						
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;
#if MOTOR_TWO_PHASE_STEPPER
			TwoPhase_CurrentToAlphaBeta(&MC.Foc);                           // Stepper: IA/IB -> Ialpha/Ibeta
#else
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);                                        // Clark任
#endif
		
			Pack_Transform(&MC.Foc);                                         //??任
			
			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id??
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq?? 

			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;			
			PID_Control(&MC.IqPid);                                          //Iq?
			PID_Control(&MC.IdPid);                                          //Id?		


			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;
      IPack_Transform(&MC.Foc);                                        //PACK任			
		}break;	
		
		case SPEED_CURRENT_LOOP:
		{		
			MC.Speed.SpeedCalculateCnt++;  			
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //?SPEED_DIVISION_FACTOR ?????
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.Encoder.ElectricalVal;         //???
				Calculate_Speed(&MC.Speed);                                  	 //?????ε???
@@ -97,52 +108,57 @@ void Sensoruse_Control()
				if(MC.Speed.MechanicalSpeedSet != MC.Speed.MechanicalSpeedSetLast)               //???
				{                                                      						
					MC.TAccDec.StartSpeed = MC.Speed.MechanicalSpeedSetLast * MC.Encoder.PolePairs;//ó?
					MC.TAccDec.EndSpeed   = MC.Speed.MechanicalSpeedSet     * MC.Encoder.PolePairs;//??
					T_Shaped_Acc_Dec(&MC.TAccDec);                                                 //Tμ??
					if(MC.TAccDec.FinishFlag == 1)                                                 //??
					{
						MC.Speed.MechanicalSpeedSetLast = MC.Speed.MechanicalSpeedSet;               //???
						MC.TAccDec.FinishFlag = 0;
					}					
				}		
				MC.SpdPid.Ref = MC.TAccDec.SpeedOut;					                 //??   
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //??	
				if(MC.SpdPid.Fbk > -2000 && MC.SpdPid.Fbk < 2000) 
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMax;
				}
				else
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMin;
				}			
				PID_Control(&MC.SpdPid);                            					 //???
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;
#if MOTOR_TWO_PHASE_STEPPER
			TwoPhase_CurrentToAlphaBeta(&MC.Foc);                           // Stepper: IA/IB -> Ialpha/Ibeta
#else
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);                                        // Clark任
#endif
	
			Pack_Transform(&MC.Foc);             														 //??任

			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id??
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq?? 

			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;		
		
			PID_Control(&MC.IqPid);               													 //Iq?
			PID_Control(&MC.IdPid);              														 //Id?

			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;
    	IPack_Transform(&MC.Foc);            														 //PACK任			
		}break;	
		
		case POS_SPEED_CURRENT_LOOP:
		{
			MC.Position.PosCalculateCnt++;
			MC.Speed.SpeedCalculateCnt++;			
			if(MC.Position.PosCalculateCnt >= POS_DIVISION_FACTOR)           //POS_DIVISION_FACTOR ??λ??
			{											
				MC.Position.PosCalculateCnt = 0;			
				MC.Position.ElectricalPosThis = MC.Encoder.ElectricalVal;			 //??λ
@@ -154,53 +170,60 @@ void Sensoruse_Control()
			}
					
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)					 //SPEED_DIVISION_FACTOR ?????
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.Encoder.ElectricalVal;
				Calculate_Speed(&MC.Speed);                                    //?
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//??
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / POLEPAIRS;	
				
				MC.SpdPid.Ref = MC.PosPid.Out;					
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //??	
				if(MC.SpdPid.Fbk > -2000 && MC.SpdPid.Fbk < 2000) 
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMax;
				}
				else
				{
					MC.SpdPid.Kp = MC.SpdPid.KpMin;
				}							
				PID_Control(&MC.SpdPid);                            					 //???
				MC.IqPid.Ref = MC.SpdPid.Out;										
			}                                                   
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;
#if MOTOR_TWO_PHASE_STEPPER
			TwoPhase_CurrentToAlphaBeta(&MC.Foc);                           // Stepper: IA/IB -> Ialpha/Ibeta
#else
			MC.Foc.Iv = MC.Sample.IvReal;			
			Clark_Transform(&MC.Foc);                                        // Clark任
#endif
	
			Pack_Transform(&MC.Foc);                                         //??任

			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id?? 
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq?? 
			
			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;			
			PID_Control(&MC.IqPid);                                          //Iq?
			PID_Control(&MC.IdPid);                                          //Id?			

			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;
			IPack_Transform(&MC.Foc);                                        //PACK任
		}break;	
	}
	
	MC.Foc.Ubus = MC.Sample.BusReal;									
#if MOTOR_TWO_PHASE_STEPPER
  Calculate_HBridgePWM(&MC.Foc);								
#else
  Calculate_SVPWM(&MC.Foc);								
#endif
}
