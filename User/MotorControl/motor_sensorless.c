/**
  ******************************************************************************
  * 文件名程: 
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/

#include "motor_sensorless.h"

/**
  * 函数功能:无感控制 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Sensorless_Control()
{
	static u8  step = 0;                	 	  //в
	static u8  errorcnt = 0;               	  //??
	static u16 runtim = 0;                 	  //?
  static u16 comparecnt = 0;              	//??		
	static u16 successcnt = 0;             		//?
	static u16 errortimout = 0;            	  //?
	switch(MC.Motor.RunMode)
	{
		case STRONG_DRAG_CURRENT_OPEN:             //?         
		{	
			MC.Foc.Ud = 0.6f;                        //?????
			MC.Encoder.ElectricalSpdSet = 1000;      //?????RPM
			Electrical_Angle_Generator(&MC.Encoder); //趨?????
			Calculate_Sin_Cos(MC.Encoder.ElectricalValSet,&MC.Foc.SinVal,&MC.Foc.CosVal);	//?		
			IPack_Transform(&MC.Foc);                //PACK任	
		}	
    break;	
		
		case STRONG_DRAG_CURRENT_CLOSE:            //??         
		{	
			MC.IdPid.Ref = 1;                        //?????
			MC.Encoder.ElectricalSpdSet = 1000;      //?????RPM
			Electrical_Angle_Generator(&MC.Encoder); //趨?????
			Calculate_Sin_Cos(MC.Encoder.ElectricalValSet,&MC.Foc.SinVal,&MC.Foc.CosVal); //?		
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;			
			MC.Foc.Ialpha = MC.Foc.Iu;
			MC.Foc.Ibeta = MC.Foc.Iw;
			
			Pack_Transform(&MC.Foc);                 //??任

			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id??
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq?? 

			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;			
			PID_Control(&MC.IqPid);                  //Iq?
			PID_Control(&MC.IdPid);                  //Id?			


			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;	 
      IPack_Transform(&MC.Foc);                //PACK任		
		}	
    break;
		
		case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:     //?+? ???         
		{
			if(step == 0)                         //? 
			{
				MC.IqPid.Ref = 0;                       
				MC.IdPid.Ref = 0;   
				MC.SpdPid.Integrate = 0;
				MC.Encoder.ElectricalValSet = 0;
				MC.Speed.MechanicalSpeedSetLast = 0;
				MC.TAccDec.FinishFlag = 0;
				MC.TAccDec.SpeedOut = 0;
				MC.TAccDec.StartSpeed = 0;
				MC.TAccDec.EndSpeed = 0;
        MC.SPLL.WeForeLPF = 0;
        MC.SPLL.IPart = 0;
				if(MC.Speed.MechanicalSpeedSet >= 100 / POLEPAIRS || MC.Speed.MechanicalSpeedSet <= -100 / POLEPAIRS)
				{
          step = 1;				                    //л?						
				}
			}			
			else if(step == 1)                    //?? ? 
			{
				MC.IqPid.Ref = 0;
				MC.IdPid.Ref = 5;                   //d?
				MC.Encoder.ElectricalSpdSet = MC.Speed.MechanicalSpeedSet * POLEPAIRS;
			  Electrical_Angle_Generator(&MC.Encoder);
			  Calculate_Sin_Cos(MC.Encoder.ElectricalValSet,&MC.Foc.SinVal,&MC.Foc.CosVal);					
								
				if(MC.Encoder.ElectricalSpdSet >= 0) MC.SPLL.Dir = -1;
        if(MC.Encoder.ElectricalSpdSet < 0)  MC.SPLL.Dir =  1;
				if(MC.Encoder.ElectricalValSet > 500 && MC.Encoder.ElectricalValSet < 3500)	
				{
					if(MC.SPLL.ETheta > 500 && MC.SPLL.ETheta < 3500)          //?б?
					{
						comparecnt++;
						if(MC.Encoder.ElectricalValSet - MC.SPLL.ETheta <= MC.Encoder.EncoderValMax * 0.1f) //?????С?%10
						{
							if(MC.Encoder.ElectricalValSet -MC.SPLL.ETheta >= -MC.Encoder.EncoderValMax * 0.1f)
							{
								successcnt++;							
							}
						}
					}			
				}	
				if(comparecnt >= 300)                     //α?
				{
					if(successcnt >= comparecnt * 0.9f)     //??80????
					{
            step = 2;                             //л?	
            MC.IdPid.Ref = 0;						
						MC.Encoder.ElectricalValSet = 0;
					}
					successcnt = 0;
					comparecnt = 0;                    //ж
				}
        errortimout++;
        if(errortimout >= 40000)             //??л?,??
				{
					step = 0;                          //??
					errortimout = 0;
					errorcnt++;					
				}					
			}			
			else if(step == 2)                     //й?? 
			{
				MC.Speed.SpeedCalculateCnt++;				
				if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //?SPEED_DIVISION_FACTOR ?????
				{
					MC.Speed.SpeedCalculateCnt = 0;
					MC.Speed.ElectricalPosThis = MC.SPLL.ETheta;                   //??? ????
					Calculate_Speed(&MC.Speed);                                  	 //?????ε???
					MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
																			+ MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//??	
					MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
					
					if(MC.Speed.MechanicalSpeedSet >= 0 && MC.Speed.MechanicalSpeedSet <= 800) MC.Speed.MechanicalSpeedSet =  800; //???
					if(MC.Speed.MechanicalSpeedSet <= 0 && MC.Speed.MechanicalSpeedSet >= -800)MC.Speed.MechanicalSpeedSet = -800; //???
					
					if(MC.Speed.MechanicalSpeedSet != MC.Speed.MechanicalSpeedSetLast)               //μ??
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
					MC.SpdPid.Kp = MC.SpdPid.KpMin;	
					PID_Control(&MC.SpdPid);                            					 //???
					MC.IqPid.Ref = MC.SpdPid.Out;	
				}					
        Calculate_Sin_Cos(MC.SPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);
				if(runtim <  20000) 
				{
				  runtim++;
				}
				if(runtim >= 20000)                   //?1S?
				{
					if(MC.Speed.ElectricalSpeedLPF <= 1000 && MC.Speed.ElectricalSpeedLPF >= -1000)   //??????
					{
						step = 0;                         //
						runtim = 0;
						errorcnt++;
					}				
				}
			}	
			
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;			
			MC.Foc.Ialpha = MC.Foc.Iu;
			MC.Foc.Ibeta = MC.Foc.Iw;
		
			Pack_Transform(&MC.Foc);               //??任

			MC.Foc.IdLPF = MC.Foc.Id * MC.Foc.IdLPFFactor + MC.Foc.IdLPF * (1 - MC.Foc.IdLPFFactor); //Id??
			MC.Foc.IqLPF = MC.Foc.Iq * MC.Foc.IqLPFFactor + MC.Foc.IqLPF * (1 - MC.Foc.IqLPFFactor); //Iq?? 

			MC.IqPid.Fbk = MC.Foc.IqLPF;
			MC.IdPid.Fbk = MC.Foc.IdLPF;			
			PID_Control(&MC.IqPid);               //Iq?
			PID_Control(&MC.IdPid);               //Id?

			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out;
	    	IPack_Transform(&MC.Foc);             //PACK任				
      
			
			MC.SMO.Rs = MC.Identify.Rs;
			MC.SMO.Ld = MC.Identify.Ld;
			MC.SMO.Ialpha = MC.Foc.Ialpha;
			MC.SMO.Ibeta  = MC.Foc.Ibeta;
			MC.SMO.Ualpha = MC.Foc.Ualpha;
			MC.SMO.Ubeta  = MC.Foc.Ubeta;
			SMO_Calculate(&MC.SMO);	
			MC.SPLL.Ain = MC.SMO.EalphaForeLPF;
			MC.SPLL.Bin = MC.SMO.EbetaForeLPF;			
			PLL_Calculate(&MC.SPLL);			  
			Calculate_Sin_Cos(MC.SPLL.ETheta,&MC.SPLL.SinVal,&MC.SPLL.CosVal);	
		}	
    break;
			
		case HFI_CURRENT_CLOSE:                    //??? HFI?У???        
		{						
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;			
			MC.Foc.Ialpha = MC.Foc.Iu;
			MC.Foc.Ibeta = MC.Foc.Iw;
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);		
			Pack_Transform(&MC.Foc);                 //??任
			
			MC.HFI.Id = MC.Foc.Id;
			MC.HFI.Iq = MC.Foc.Iq;							
			MC.HFI.Ialpha = MC.Foc.Ialpha;
			MC.HFI.Ibeta  = MC.Foc.Ibeta;  	
			HFI_Calculate(&MC.HFI);
      if(MC.HFI.NSDFlag == 0)
			{
				MC.IdPid.Ref = MC.HFI.IdRef;
			}	
      if(MC.HFI.NSDOut == 1)
			{
				MC.HFI.NSDOut = 0;
				MC.HPLL.ThetaFore += ONE_PI;
				if(MC.HPLL.ThetaFore > TWO_PI) 
				{
					MC.HPLL.ThetaFore -= TWO_PI;  	      //???  0-2
				}				
			}				
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.HPLL.SinVal,&MC.HPLL.CosVal);				
			MC.HPLL.Ain = MC.HFI.IbetaOut;
			MC.HPLL.Bin = -MC.HFI.IalphaOut;
			PLL_Calculate(&MC.HPLL);							
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);              //Iq?
			PID_Control(&MC.IdPid);              //Id?			
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);            //PACK任		
		}	
    break;	
		
		case HFI_SPEED_CURRENT_CLOSE:          //????HFI? 4006?2500RPM        
		{					
			MC.Speed.SpeedCalculateCnt++;  			
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //?SPEED_DIVISION_FACTOR ?????
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.HPLL.ETheta;                   //??? ????
				Calculate_Speed(&MC.Speed);                                  	 //?????ε???
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//??	
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
				
				if(MC.Speed.MechanicalSpeedSet >=  2500 )MC.Speed.MechanicalSpeedSet =  2500; //?HFI?У
				if(MC.Speed.MechanicalSpeedSet <= -2500 )MC.Speed.MechanicalSpeedSet = -2500; //?HFI?У
				
				if(MC.Speed.MechanicalSpeedSet != MC.Speed.MechanicalSpeedSetLast)               //μ??
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
				MC.SpdPid.Kp = MC.SpdPid.KpMin;	
				PID_Control(&MC.SpdPid);                            					 //???
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}			
		
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;			
			MC.Foc.Ialpha = MC.Foc.Iu;
			MC.Foc.Ibeta = MC.Foc.Iw;	

      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);		
			Pack_Transform(&MC.Foc);                                         //??任
			
			MC.HFI.Id = MC.Foc.Id;
			MC.HFI.Iq = MC.Foc.Iq;							
			MC.HFI.Ialpha = MC.Foc.Ialpha;
			MC.HFI.Ibeta  = MC.Foc.Ibeta;  	
			HFI_Calculate(&MC.HFI);
      if(MC.HFI.NSDFlag == 0)
			{
				MC.IdPid.Ref = MC.HFI.IdRef;
			}	
      if(MC.HFI.NSDOut == 1)
			{
				MC.HFI.NSDOut = 0;
				MC.HPLL.ThetaFore += ONE_PI;
				if(MC.HPLL.ThetaFore > TWO_PI) 
				{
					MC.HPLL.ThetaFore -= TWO_PI;  	                             //
				}				
			}				
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.HPLL.SinVal,&MC.HPLL.CosVal);				
			MC.HPLL.Ain = MC.HFI.IbetaOut;
			MC.HPLL.Bin = -MC.HFI.IalphaOut;
			PLL_Calculate(&MC.HPLL);							
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);                                          //Iq
			PID_Control(&MC.IdPid);                                          //Id		
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);                                        //PACK任

//			MC.SMO.Ialpha = MC.Foc.Ialpha;
//			MC.SMO.Ibeta  = MC.Foc.Ibeta;
//			MC.SMO.Ualpha = MC.Foc.Ualpha;
//			MC.SMO.Ubeta  = MC.Foc.Ubeta;
//			SMO_Calculate(&MC.SMO);	
//			MC.SPLL.Ain = MC.SMO.EalphaForeLPF;
//			MC.SPLL.Bin = MC.SMO.EbetaForeLPF;			
//			PLL_Calculate(&MC.SPLL);			  
//			Calculate_Sin_Cos(MC.SPLL.ETheta,&MC.SPLL.SinVal,&MC.SPLL.CosVal);			
		}break;	

		case HFI_POS_SPEED_CURRENT_CLOSE:                                  //HFIλ??        
		{
			MC.Speed.SpeedCalculateCnt++;  
			MC.Position.PosCalculateCnt++;	
			
			if(MC.Position.PosCalculateCnt >= POS_DIVISION_FACTOR)           //POS_DIVISION_FACTOR ??λ??
			{											
				MC.Position.PosCalculateCnt = 0;			
				MC.Position.ElectricalPosThis = MC.HPLL.ETheta;			           //??λ
				Calculate_Position(&MC.Position);                              //λ
				MC.PosPid.Fbk = MC.Position.ElectricalPosSum;								   //?λ
				MC.PosPid.Ref = MC.Position.MechanicalPosSet * POLEPAIRS;			 //?λ 
				MC.Position.MechanicalPosRaw = MC.Position.ElectricalPosSum / POLEPAIRS;
				PID_Control(&MC.PosPid);                                       //λ??
			}
		
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //?SPEED_DIVISION_FACTOR ?????
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.HPLL.ETheta;                   //??? ????
				Calculate_Speed(&MC.Speed);                                  	 //?????ε???
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//??	
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
				MC.SpdPid.Ref = MC.PosPid.Out;			
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //??	
				MC.SpdPid.Kp  = MC.SpdPid.KpMin * 2;	
				PID_Control(&MC.SpdPid);                            					 //???
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}			
		
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;			
			MC.Foc.Ialpha = MC.Foc.Iu;
			MC.Foc.Ibeta = MC.Foc.Iw;

      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);		
			Pack_Transform(&MC.Foc);                                         //??任
			
			MC.HFI.Id = MC.Foc.Id;
			MC.HFI.Iq = MC.Foc.Iq;							
			MC.HFI.Ialpha = MC.Foc.Ialpha;
			MC.HFI.Ibeta  = MC.Foc.Ibeta;  	
			HFI_Calculate(&MC.HFI);
      if(MC.HFI.NSDFlag == 0)
			{
				MC.IdPid.Ref = MC.HFI.IdRef;
			}	
      if(MC.HFI.NSDOut == 1)
			{
				MC.HFI.NSDOut = 0;
				MC.HPLL.ThetaFore += ONE_PI;
				if(MC.HPLL.ThetaFore > TWO_PI) 
				{
					MC.HPLL.ThetaFore -= TWO_PI;  	                             //???  0-2
				}				
			}				
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.HPLL.SinVal,&MC.HPLL.CosVal);				
			MC.HPLL.Ain = MC.HFI.IbetaOut;
			MC.HPLL.Bin = -MC.HFI.IalphaOut;
			PLL_Calculate(&MC.HPLL);							
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);                                          //Iq?
			PID_Control(&MC.IdPid);                                          //Id?			
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);                                        //PACK任						
		}break;	
		
	}				
	MC.Foc.Ubus = MC.Sample.BusReal;		
	Calculate_Stepper_PWM(&MC.Foc);	    
}

