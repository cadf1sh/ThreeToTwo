/**
  ******************************************************************************
void Sensorless_Control(MOTORCONTROL_STRUCT *mc)
	switch(mc->Motor.RunMode)
			mc->Foc.Ud = 0.6f;                        //ÿǿʱĵѹ
			mc->Encoder.ElectricalSpdSet = 1000;      //ǿʱĵٶȣRPM
			Electrical_Angle_Generator(&mc->Encoder); //趨ĵٶʵʱǶ
			Calculate_Sin_Cos(mc->Encoder.ElectricalValSet,&mc->Foc.SinVal,&mc->Foc.CosVal);	//ֵ		
			IPack_Transform(&mc->Foc);                //PACK任	
			mc->IdPid.Ref = 1;                        //ñջǿʱĵ
			mc->Encoder.ElectricalSpdSet = 1000;      //ǿʱĵٶȣRPM
			Electrical_Angle_Generator(&mc->Encoder); //趨ĵٶʵʱǶ
			Calculate_Sin_Cos(mc->Encoder.ElectricalValSet,&mc->Foc.SinVal,&mc->Foc.CosVal); //ֵ		
			mc->Foc.Iu = mc->Sample.IuReal;
			mc->Foc.Iw = mc->Sample.IwReal;			
			mc->Foc.Ialpha = mc->Foc.Iu;
			mc->Foc.Ibeta = mc->Foc.Iw;
			Pack_Transform(&mc->Foc);                 //ɿ˱任
			mc->Foc.IdLPF = mc->Foc.Id * mc->Foc.IdLPFFactor + mc->Foc.IdLPF * (1 - mc->Foc.IdLPFFactor); //Idͨ˲
			mc->Foc.IqLPF = mc->Foc.Iq * mc->Foc.IqLPFFactor + mc->Foc.IqLPF * (1 - mc->Foc.IqLPFFactor); //Iqͨ˲ 
			mc->IqPid.Fbk = mc->Foc.IqLPF;
			mc->IdPid.Fbk = mc->Foc.IdLPF;			
			PID_Control(&mc->IqPid);                  //Iqջ
			PID_Control(&mc->IdPid);                  //Idջ			
			mc->Foc.Uq = mc->IqPid.Out;			
			mc->Foc.Ud = mc->IdPid.Out;	 
      IPack_Transform(&mc->Foc);                //PACK任		
				mc->IqPid.Ref = 0;                       
				mc->IdPid.Ref = 0;   
				mc->SpdPid.Integrate = 0;
				mc->Encoder.ElectricalValSet = 0;
				mc->Speed.MechanicalSpeedSetLast = 0;
				mc->TAccDec.FinishFlag = 0;
				mc->TAccDec.SpeedOut = 0;
				mc->TAccDec.StartSpeed = 0;
				mc->TAccDec.EndSpeed = 0;
        mc->SPLL.WeForeLPF = 0;
        mc->SPLL.IPart = 0;
				if(mc->Speed.MechanicalSpeedSet >= 100 / POLEPAIRS || mc->Speed.MechanicalSpeedSet <= -100 / POLEPAIRS)
				mc->IqPid.Ref = 0;
				mc->IdPid.Ref = 5;                   //dǿ
				mc->Encoder.ElectricalSpdSet = mc->Speed.MechanicalSpeedSet * POLEPAIRS;
			  Electrical_Angle_Generator(&mc->Encoder);
			  Calculate_Sin_Cos(mc->Encoder.ElectricalValSet,&mc->Foc.SinVal,&mc->Foc.CosVal);					
				if(mc->Encoder.ElectricalSpdSet >= 0) mc->SPLL.Dir = -1;
        if(mc->Encoder.ElectricalSpdSet < 0)  mc->SPLL.Dir =  1;
				if(mc->Encoder.ElectricalValSet > 500 && mc->Encoder.ElectricalValSet < 3500)	
					if(mc->SPLL.ETheta > 500 && mc->SPLL.ETheta < 3500)          //ڽбȽ
						if(mc->Encoder.ElectricalValSet - mc->SPLL.ETheta <= mc->Encoder.EncoderValMax * 0.1f) //۲ֵǿϸֵСڡ%10
							if(mc->Encoder.ElectricalValSet -mc->SPLL.ETheta >= -mc->Encoder.EncoderValMax * 0.1f)
            mc->IdPid.Ref = 0;						
						mc->Encoder.ElectricalValSet = 0;
				mc->Speed.SpeedCalculateCnt++;				
				if(mc->Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //ÿSPEED_DIVISION_FACTOR ִһٶȱջ
					mc->Speed.SpeedCalculateCnt = 0;
					mc->Speed.ElectricalPosThis = mc->SPLL.ETheta;                   //ȡǰǶ ɸƵעõ
					Calculate_Speed(&mc->Speed);                                  	 //ݵǰǶȺϴεǶȼٶ
					mc->Speed.ElectricalSpeedLPF = mc->Speed.ElectricalSpeedRaw * mc->Speed.ElectricalSpeedLPFFactor
																			+ mc->Speed.ElectricalSpeedLPF * (1 - mc->Speed.ElectricalSpeedLPFFactor);//ͨ˲	
					mc->Speed.MechanicalSpeed = mc->Speed.ElectricalSpeedLPF / mc->Encoder.PolePairs;				
					if(mc->Speed.MechanicalSpeedSet >= 0 && mc->Speed.MechanicalSpeedSet <= 800) mc->Speed.MechanicalSpeedSet =  800; //ƻĤٶ
					if(mc->Speed.MechanicalSpeedSet <= 0 && mc->Speed.MechanicalSpeedSet >= -800)mc->Speed.MechanicalSpeedSet = -800; //ƻĤٶ
					if(mc->Speed.MechanicalSpeedSet != mc->Speed.MechanicalSpeedSetLast)               //µĿٶ
						mc->TAccDec.StartSpeed = mc->Speed.MechanicalSpeedSetLast * mc->Encoder.PolePairs;//óٶ
						mc->TAccDec.EndSpeed   = mc->Speed.MechanicalSpeedSet     * mc->Encoder.PolePairs;//ĩٶ
						T_Shaped_Acc_Dec(&mc->TAccDec);                                                 //TμӼټ
						if(mc->TAccDec.FinishFlag == 1)                                                 //ִӼ
							mc->Speed.MechanicalSpeedSetLast = mc->Speed.MechanicalSpeedSet;               //ϴĿٶ
							mc->TAccDec.FinishFlag = 0;
					mc->SpdPid.Ref = mc->TAccDec.SpeedOut;					                 //Ŀֵ  
					mc->SpdPid.Fbk = mc->Speed.ElectricalSpeedLPF;	 					       //ٶֵ	
					mc->SpdPid.Kp = mc->SpdPid.KpMin;	
					PID_Control(&mc->SpdPid);                            					 //ٶȱջ
					mc->IqPid.Ref = mc->SpdPid.Out;	
        Calculate_Sin_Cos(mc->SPLL.ETheta,&mc->Foc.SinVal,&mc->Foc.CosVal);
					if(mc->Speed.ElectricalSpeedLPF <= 1000 && mc->Speed.ElectricalSpeedLPF >= -1000)   //ת̫˲ʺϹ۲
			mc->Foc.Iu = mc->Sample.IuReal;
			mc->Foc.Iw = mc->Sample.IwReal;			
			mc->Foc.Ialpha = mc->Foc.Iu;
			mc->Foc.Ibeta = mc->Foc.Iw;
			Pack_Transform(&mc->Foc);               //ɿ˱任
			mc->Foc.IdLPF = mc->Foc.Id * mc->Foc.IdLPFFactor + mc->Foc.IdLPF * (1 - mc->Foc.IdLPFFactor); //Idͨ˲
			mc->Foc.IqLPF = mc->Foc.Iq * mc->Foc.IqLPFFactor + mc->Foc.IqLPF * (1 - mc->Foc.IqLPFFactor); //Iqͨ˲ 
			mc->IqPid.Fbk = mc->Foc.IqLPF;
			mc->IdPid.Fbk = mc->Foc.IdLPF;			
			PID_Control(&mc->IqPid);               //Iqջ
			PID_Control(&mc->IdPid);               //Idջ
			mc->Foc.Uq = mc->IqPid.Out;			
			mc->Foc.Ud = mc->IdPid.Out;
	    	IPack_Transform(&mc->Foc);             //PACK任				
			mc->SMO.Rs = mc->Identify.Rs;
			mc->SMO.Ld = mc->Identify.Ld;
			mc->SMO.Ialpha = mc->Foc.Ialpha;
			mc->SMO.Ibeta  = mc->Foc.Ibeta;
			mc->SMO.Ualpha = mc->Foc.Ualpha;
			mc->SMO.Ubeta  = mc->Foc.Ubeta;
			SMO_Calculate(&mc->SMO);	
			mc->SPLL.Ain = mc->SMO.EalphaForeLPF;
			mc->SPLL.Bin = mc->SMO.EbetaForeLPF;			
			PLL_Calculate(&mc->SPLL);			  
			Calculate_Sin_Cos(mc->SPLL.ETheta,&mc->SPLL.SinVal,&mc->SPLL.CosVal);	
			mc->Foc.Iu = mc->Sample.IuReal;
			mc->Foc.Iw = mc->Sample.IwReal;			
			mc->Foc.Ialpha = mc->Foc.Iu;
			mc->Foc.Ibeta = mc->Foc.Iw;
      Calculate_Sin_Cos(mc->HPLL.ETheta,&mc->Foc.SinVal,&mc->Foc.CosVal);		
			Pack_Transform(&mc->Foc);                 //ɿ˱任
			mc->HFI.Id = mc->Foc.Id;
			mc->HFI.Iq = mc->Foc.Iq;							
			mc->HFI.Ialpha = mc->Foc.Ialpha;
			mc->HFI.Ibeta  = mc->Foc.Ibeta;  	
			HFI_Calculate(&mc->HFI);
      if(mc->HFI.NSDFlag == 0)
				mc->IdPid.Ref = mc->HFI.IdRef;
      if(mc->HFI.NSDOut == 1)
				mc->HFI.NSDOut = 0;
				mc->HPLL.ThetaFore += ONE_PI;
				if(mc->HPLL.ThetaFore > TWO_PI) 
					mc->HPLL.ThetaFore -= TWO_PI;  	      //Ƕȹһ  0-2
      Calculate_Sin_Cos(mc->HPLL.ETheta,&mc->HPLL.SinVal,&mc->HPLL.CosVal);				
			mc->HPLL.Ain = mc->HFI.IbetaOut;
			mc->HPLL.Bin = -mc->HFI.IalphaOut;
			PLL_Calculate(&mc->HPLL);							
			mc->IqPid.Fbk = mc->HFI.IqBase;
			mc->IdPid.Fbk = mc->HFI.IdBase;			
			PID_Control(&mc->IqPid);              //Iqջ
			PID_Control(&mc->IdPid);              //Idջ			
			mc->Foc.Uq = mc->IqPid.Out;			
			mc->Foc.Ud = mc->IdPid.Out + mc->HFI.Uin;			
      IPack_Transform(&mc->Foc);            //PACK任		
			mc->Speed.SpeedCalculateCnt++;  			
			if(mc->Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //ÿSPEED_DIVISION_FACTOR ִһٶȱջ
				mc->Speed.SpeedCalculateCnt = 0;
				mc->Speed.ElectricalPosThis = mc->HPLL.ETheta;                   //ȡǰǶ ɸƵעõ
				Calculate_Speed(&mc->Speed);                                  	 //ݵǰǶȺϴεǶȼٶ
				mc->Speed.ElectricalSpeedLPF = mc->Speed.ElectricalSpeedRaw * mc->Speed.ElectricalSpeedLPFFactor
				                            + mc->Speed.ElectricalSpeedLPF * (1 - mc->Speed.ElectricalSpeedLPFFactor);//ͨ˲	
				mc->Speed.MechanicalSpeed = mc->Speed.ElectricalSpeedLPF / mc->Encoder.PolePairs;				
				if(mc->Speed.MechanicalSpeedSet >=  2500 )mc->Speed.MechanicalSpeedSet =  2500; //٣HFIɸУ
				if(mc->Speed.MechanicalSpeedSet <= -2500 )mc->Speed.MechanicalSpeedSet = -2500; //٣HFIɸУ
				if(mc->Speed.MechanicalSpeedSet != mc->Speed.MechanicalSpeedSetLast)               //µĿٶ
					mc->TAccDec.StartSpeed = mc->Speed.MechanicalSpeedSetLast * mc->Encoder.PolePairs;//óٶ
					mc->TAccDec.EndSpeed   = mc->Speed.MechanicalSpeedSet     * mc->Encoder.PolePairs;//ĩٶ
					T_Shaped_Acc_Dec(&mc->TAccDec);                                                 //TμӼټ
					if(mc->TAccDec.FinishFlag == 1)                                                 //ִӼ
						mc->Speed.MechanicalSpeedSetLast = mc->Speed.MechanicalSpeedSet;               //ϴĿٶ
						mc->TAccDec.FinishFlag = 0;
				mc->SpdPid.Ref = mc->TAccDec.SpeedOut;					                 //Ŀֵ  
				mc->SpdPid.Fbk = mc->Speed.ElectricalSpeedLPF;	 					       //ٶֵ	
				mc->SpdPid.Kp = mc->SpdPid.KpMin;	
				PID_Control(&mc->SpdPid);                            					 //ٶȱջ
				mc->IqPid.Ref = mc->SpdPid.Out;	
			mc->Foc.Iu = mc->Sample.IuReal;
			mc->Foc.Iw = mc->Sample.IwReal;			
			mc->Foc.Ialpha = mc->Foc.Iu;
			mc->Foc.Ibeta = mc->Foc.Iw;	
      Calculate_Sin_Cos(mc->HPLL.ETheta,&mc->Foc.SinVal,&mc->Foc.CosVal);		
			Pack_Transform(&mc->Foc);                                         //ɿ˱任
			mc->HFI.Id = mc->Foc.Id;
			mc->HFI.Iq = mc->Foc.Iq;							
			mc->HFI.Ialpha = mc->Foc.Ialpha;
			mc->HFI.Ibeta  = mc->Foc.Ibeta;  	
			HFI_Calculate(&mc->HFI);
      if(mc->HFI.NSDFlag == 0)
				mc->IdPid.Ref = mc->HFI.IdRef;
      if(mc->HFI.NSDOut == 1)
				mc->HFI.NSDOut = 0;
				mc->HPLL.ThetaFore += ONE_PI;
				if(mc->HPLL.ThetaFore > TWO_PI) 
					mc->HPLL.ThetaFore -= TWO_PI;  	                             //Ƕȹһ  0-2
      Calculate_Sin_Cos(mc->HPLL.ETheta,&mc->HPLL.SinVal,&mc->HPLL.CosVal);				
			mc->HPLL.Ain = mc->HFI.IbetaOut;
			mc->HPLL.Bin = -mc->HFI.IalphaOut;
			PLL_Calculate(&mc->HPLL);							
			mc->IqPid.Fbk = mc->HFI.IqBase;
			mc->IdPid.Fbk = mc->HFI.IdBase;			
			PID_Control(&mc->IqPid);                                          //Iqջ
			PID_Control(&mc->IdPid);                                          //Idջ			
			mc->Foc.Uq = mc->IqPid.Out;			
			mc->Foc.Ud = mc->IdPid.Out + mc->HFI.Uin;			
      IPack_Transform(&mc->Foc);                                        //PACK任
//			mc->SMO.Ialpha = mc->Foc.Ialpha;
//			mc->SMO.Ibeta  = mc->Foc.Ibeta;
//			mc->SMO.Ualpha = mc->Foc.Ualpha;
//			mc->SMO.Ubeta  = mc->Foc.Ubeta;
//			SMO_Calculate(&mc->SMO);	
//			mc->SPLL.Ain = mc->SMO.EalphaForeLPF;
//			mc->SPLL.Bin = mc->SMO.EbetaForeLPF;			
//			PLL_Calculate(&mc->SPLL);			  
//			Calculate_Sin_Cos(mc->SPLL.ETheta,&mc->SPLL.SinVal,&mc->SPLL.CosVal);			
			mc->Speed.SpeedCalculateCnt++;  
			mc->Position.PosCalculateCnt++;	
			if(mc->Position.PosCalculateCnt >= POS_DIVISION_FACTOR)           //POS_DIVISION_FACTOR ִһλñջ
				mc->Position.PosCalculateCnt = 0;			
				mc->Position.ElectricalPosThis = mc->HPLL.ETheta;			           //ȡǰλ
				Calculate_Position(&mc->Position);                              //λ
				mc->PosPid.Fbk = mc->Position.ElectricalPosSum;								   //ʵλ
				mc->PosPid.Ref = mc->Position.MechanicalPosSet * POLEPAIRS;			 //Ŀλ 
				mc->Position.MechanicalPosRaw = mc->Position.ElectricalPosSum / POLEPAIRS;
				PID_Control(&mc->PosPid);                                       //λñջ
			if(mc->Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //ÿSPEED_DIVISION_FACTOR ִһٶȱջ
				mc->Speed.SpeedCalculateCnt = 0;
				mc->Speed.ElectricalPosThis = mc->HPLL.ETheta;                   //ȡǰǶ ɸƵעõ
				Calculate_Speed(&mc->Speed);                                  	 //ݵǰǶȺϴεǶȼٶ
				mc->Speed.ElectricalSpeedLPF = mc->Speed.ElectricalSpeedRaw * mc->Speed.ElectricalSpeedLPFFactor
				                            + mc->Speed.ElectricalSpeedLPF * (1 - mc->Speed.ElectricalSpeedLPFFactor);//ͨ˲	
				mc->Speed.MechanicalSpeed = mc->Speed.ElectricalSpeedLPF / mc->Encoder.PolePairs;				
				mc->SpdPid.Ref = mc->PosPid.Out;			
				mc->SpdPid.Fbk = mc->Speed.ElectricalSpeedLPF;	 					       //ٶֵ	
				mc->SpdPid.Kp  = mc->SpdPid.KpMin * 2;	
				PID_Control(&mc->SpdPid);                            					 //ٶȱջ
				mc->IqPid.Ref = mc->SpdPid.Out;	
			mc->Foc.Iu = mc->Sample.IuReal;
			mc->Foc.Iw = mc->Sample.IwReal;			
			mc->Foc.Ialpha = mc->Foc.Iu;
			mc->Foc.Ibeta = mc->Foc.Iw;
      Calculate_Sin_Cos(mc->HPLL.ETheta,&mc->Foc.SinVal,&mc->Foc.CosVal);		
			Pack_Transform(&mc->Foc);                                         //ɿ˱任
			mc->HFI.Id = mc->Foc.Id;
			mc->HFI.Iq = mc->Foc.Iq;							
			mc->HFI.Ialpha = mc->Foc.Ialpha;
			mc->HFI.Ibeta  = mc->Foc.Ibeta;  	
			HFI_Calculate(&mc->HFI);
      if(mc->HFI.NSDFlag == 0)
				mc->IdPid.Ref = mc->HFI.IdRef;
      if(mc->HFI.NSDOut == 1)
				mc->HFI.NSDOut = 0;
				mc->HPLL.ThetaFore += ONE_PI;
				if(mc->HPLL.ThetaFore > TWO_PI) 
					mc->HPLL.ThetaFore -= TWO_PI;  	                             //Ƕȹһ  0-2
      Calculate_Sin_Cos(mc->HPLL.ETheta,&mc->HPLL.SinVal,&mc->HPLL.CosVal);				
			mc->HPLL.Ain = mc->HFI.IbetaOut;
			mc->HPLL.Bin = -mc->HFI.IalphaOut;
			PLL_Calculate(&mc->HPLL);							
			mc->IqPid.Fbk = mc->HFI.IqBase;
			mc->IdPid.Fbk = mc->HFI.IdBase;			
			PID_Control(&mc->IqPid);                                          //Iqջ
			PID_Control(&mc->IdPid);                                          //Idջ			
			mc->Foc.Uq = mc->IqPid.Out;			
			mc->Foc.Ud = mc->IdPid.Out + mc->HFI.Uin;			
      IPack_Transform(&mc->Foc);                                        //PACK任						
	mc->Foc.Ubus = mc->Sample.BusReal;		
	Calculate_Stepper_PWM(&mc->Foc);	    
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);              //Iq闭环
			PID_Control(&MC.IdPid);              //Id闭环			
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);            //反PACK变换		
		}	
    break;	
		
		case HFI_SPEED_CURRENT_CLOSE:          //零低速区域无感速度闭环（纯HFI不可高速运行 针对4006无刷电机限速2500RPM）        
		{					
			MC.Speed.SpeedCalculateCnt++;  			
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //每SPEED_DIVISION_FACTOR次 执行一次速度闭环
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.HPLL.ETheta;                   //获取当前电角度 由高频注入得到
				Calculate_Speed(&MC.Speed);                                  	 //根据当前电角度和上次电角度计算电角速度
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波	
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
				
				if(MC.Speed.MechanicalSpeedSet >=  2500 )MC.Speed.MechanicalSpeedSet =  2500; //限速（纯HFI不可高速运行）
				if(MC.Speed.MechanicalSpeedSet <= -2500 )MC.Speed.MechanicalSpeedSet = -2500; //限速（纯HFI不可高速运行）
				
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
				MC.SpdPid.Kp = MC.SpdPid.KpMin;	
				PID_Control(&MC.SpdPid);                            					 //速度闭环
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}			
		
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;			
			MC.Foc.Ialpha = MC.Foc.Iu;
			MC.Foc.Ibeta = MC.Foc.Iw;	

      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);		
			Pack_Transform(&MC.Foc);                                         //派克变换
			
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
					MC.HPLL.ThetaFore -= TWO_PI;  	                             //角度归一化  0-2Π
				}				
			}				
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.HPLL.SinVal,&MC.HPLL.CosVal);				
			MC.HPLL.Ain = MC.HFI.IbetaOut;
			MC.HPLL.Bin = -MC.HFI.IalphaOut;
			PLL_Calculate(&MC.HPLL);							
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);                                          //Iq闭环
			PID_Control(&MC.IdPid);                                          //Id闭环			
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);                                        //反PACK变换

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

		case HFI_POS_SPEED_CURRENT_CLOSE:                                  //零低速HFI位置闭环        
		{
			MC.Speed.SpeedCalculateCnt++;  
			MC.Position.PosCalculateCnt++;	
			
			if(MC.Position.PosCalculateCnt >= POS_DIVISION_FACTOR)           //POS_DIVISION_FACTOR 执行一次位置闭环
			{											
				MC.Position.PosCalculateCnt = 0;			
				MC.Position.ElectricalPosThis = MC.HPLL.ETheta;			           //获取当前位置
				Calculate_Position(&MC.Position);                              //计算总位置
				MC.PosPid.Fbk = MC.Position.ElectricalPosSum;								   //反馈实际位置
				MC.PosPid.Ref = MC.Position.MechanicalPosSet * POLEPAIRS;			 //给定目标位置 
				MC.Position.MechanicalPosRaw = MC.Position.ElectricalPosSum / POLEPAIRS;
				PID_Control(&MC.PosPid);                                       //位置闭环
			}
		
			if(MC.Speed.SpeedCalculateCnt >= SPEED_DIVISION_FACTOR)          //每SPEED_DIVISION_FACTOR次 执行一次速度闭环
			{
				MC.Speed.SpeedCalculateCnt = 0;
				MC.Speed.ElectricalPosThis = MC.HPLL.ETheta;                   //获取当前电角度 由高频注入得到
				Calculate_Speed(&MC.Speed);                                  	 //根据当前电角度和上次电角度计算电角速度
				MC.Speed.ElectricalSpeedLPF = MC.Speed.ElectricalSpeedRaw * MC.Speed.ElectricalSpeedLPFFactor
				                            + MC.Speed.ElectricalSpeedLPF * (1 - MC.Speed.ElectricalSpeedLPFFactor);//低通滤波	
				MC.Speed.MechanicalSpeed = MC.Speed.ElectricalSpeedLPF / MC.Encoder.PolePairs;				
				MC.SpdPid.Ref = MC.PosPid.Out;			
				MC.SpdPid.Fbk = MC.Speed.ElectricalSpeedLPF;	 					       //反馈速度值	
				MC.SpdPid.Kp  = MC.SpdPid.KpMin * 2;	
				PID_Control(&MC.SpdPid);                            					 //速度闭环
				MC.IqPid.Ref = MC.SpdPid.Out;	
			}			
		
			MC.Foc.Iu = MC.Sample.IuReal;
			MC.Foc.Iw = MC.Sample.IwReal;			
			MC.Foc.Ialpha = MC.Foc.Iu;
			MC.Foc.Ibeta = MC.Foc.Iw;

      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.Foc.SinVal,&MC.Foc.CosVal);		
			Pack_Transform(&MC.Foc);                                         //派克变换
			
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
					MC.HPLL.ThetaFore -= TWO_PI;  	                             //角度归一化  0-2Π
				}				
			}				
      Calculate_Sin_Cos(MC.HPLL.ETheta,&MC.HPLL.SinVal,&MC.HPLL.CosVal);				
			MC.HPLL.Ain = MC.HFI.IbetaOut;
			MC.HPLL.Bin = -MC.HFI.IalphaOut;
			PLL_Calculate(&MC.HPLL);							
			
			MC.IqPid.Fbk = MC.HFI.IqBase;
			MC.IdPid.Fbk = MC.HFI.IdBase;			
			PID_Control(&MC.IqPid);                                          //Iq闭环
			PID_Control(&MC.IdPid);                                          //Id闭环			
			
			MC.Foc.Uq = MC.IqPid.Out;			
			MC.Foc.Ud = MC.IdPid.Out + MC.HFI.Uin;			
      IPack_Transform(&MC.Foc);                                        //反PACK变换						
		}break;	
		
	}				
	MC.Foc.Ubus = MC.Sample.BusReal;		
	Calculate_Stepper_PWM(&MC.Foc);	    
}	


