/**
  ******************************************************************************
void Motor_Identify(MOTORCONTROL_STRUCT *mc)
	switch (mc->Identify.State)
			if(mc->Identify.Flag == 0)           //ղ
				mc->Foc.Uq = 0;
				mc->Foc.Ud = 0;	
        mc->Identify.Count = 0;					
				mc->Identify.WaitTim = 0;			
				mc->Identify.Flag = 1;
			if(mc->Identify.Flag == 1)           
				if(( mc->Sample.IuReal * mc->Foc.Ud * 1.5f) / mc->Sample.BusReal >= 0.6f * mc->Identify.CurMax) 
					mc->Identify.Flag = 2;
					mc->Foc.Ud += 0.0001f;				   	     //ӵȣС׼
					mc->Identify.VoltageSet[0] = mc->Foc.Ud;							
			if(mc->Identify.Flag == 2)
					mc->Identify.WaitTim++;
					if(mc->Identify.WaitTim > 4000)       // 0.2S ȴȶ
						mc->Identify.CurSum += mc->Sample.IuReal;
					if(mc->Identify.WaitTim >= 4100)      // ¼100εֵ
						mc->Identify.CurAverage[0] = mc->Identify.CurSum * 0.01f; //ƽ								
						mc->Identify.WaitTim = 0;
						mc->Identify.CurSum = 0;
						mc->Identify.Flag = 3;
			if(mc->Identify.Flag == 3)
				if((mc->Sample.IuReal * mc->Foc.Ud * 1.5f) / mc->Sample.BusReal >= mc->Identify.CurMax) 
					mc->Identify.Flag = 4;
					mc->Foc.Ud += 0.0001f;					    // ӵȣС׼
					mc->Identify.VoltageSet[1] = mc->Foc.Ud;							
			if(mc->Identify.Flag == 4)
					mc->Identify.WaitTim++;
					if(mc->Identify.WaitTim > 4000)       // 0.2S ȴȶ
						mc->Identify.CurSum += mc->Sample.IuReal;
					if(mc->Identify.WaitTim >= 4100)      // ¼100εֵ
						mc->Identify.CurAverage[1] = mc->Identify.CurSum * 0.01f; //ƽ
						mc->Identify.WaitTim = 0;
						mc->Identify.CurSum = 0;
						mc->Identify.Flag = 5;
			if(mc->Identify.Flag == 5)
				mc->Identify.Rs = (mc->Identify.VoltageSet[1] - mc->Identify.VoltageSet[0]) / (mc->Identify.CurAverage[1] - mc->Identify.CurAverage[0]);
				mc->Foc.Ud = 0;
				mc->Identify.Flag = 0;
				mc->Identify.State = INDUCTANCE_IDENTIFICATION;			
		  mc->Foc.SinVal = 0;                  //ǶΪ0ֵΪ0
		  mc->Foc.CosVal = 1;           		 		//ǶΪ0ֵΪ1	
		  IPack_Transform(&mc->Foc);           //PACK任			
			if(mc->Identify.Flag == 0)
				mc->Foc.Uq = 0;
				mc->Foc.Ud = 0;						
				if(mc->Sample.IuReal >= -0.05f && mc->Sample.IuReal <= 0.05f)       
					mc->Identify.Flag = 1;
			if(mc->Identify.Flag == 1)
				mc->Foc.Ud = mc->Identify.VoltageSet[1];
				mc->Identify.WaitTim++;            						
				if(mc->Sample.IuReal >= mc->Identify.CurAverage[1] * 0.95f)
					mc->Identify.LsSum += mc->Identify.Rs * 0.334f * 0.00005f *  mc->Identify.WaitTim;
					mc->Identify.WaitTim = 0;
					mc->Identify.Count++;
					mc->Identify.Flag = 0;
					mc->Foc.Ud = 0;
					if(mc->Identify.Count >= 100)
						mc->Identify.Flag = 2;
			if(mc->Identify.Flag == 2)
				mc->Identify.Ls = mc->Identify.LsSum * 0.01f;
				mc->Identify.Ld = mc->Identify.Ls;
				mc->Identify.Lq = mc->Identify.Ls;		
				mc->Identify.Flag = 0;
				mc->Identify.LsSum = 0;
				mc->Identify.WaitTim = 0;
				mc->Identify.State = RESISTANCE_IDENTIFICATION;
				mc->Identify.EndFlag = 1;
			mc->Foc.SinVal = 0;                  //ǶΪ0ֵΪ0
			mc->Foc.CosVal = 1;           		 		//ǶΪ0ֵΪ1	
			IPack_Transform(&mc->Foc);           //PACK任
	mc->Foc.Ubus = mc->Sample.BusReal;		
	Calculate_Stepper_PWM(&mc->Foc);	          //stepper PWM				
				}					
			}
			
			if(MC.Identify.Flag == 4)
			{
					MC.Identify.WaitTim++;
					if(MC.Identify.WaitTim > 4000)       // 0.2S 等待电流稳定
					{
						MC.Identify.CurSum += MC.Sample.IuReal;
					}	
					
					if(MC.Identify.WaitTim >= 4100)      // 记录100次电流值
					{
						MC.Identify.CurAverage[1] = MC.Identify.CurSum * 0.01f; //计算平均电流
						MC.Identify.WaitTim = 0;
						MC.Identify.CurSum = 0;
						MC.Identify.Flag = 5;
					}						
			}

			if(MC.Identify.Flag == 5)
			{
				MC.Identify.Rs = (MC.Identify.VoltageSet[1] - MC.Identify.VoltageSet[0]) / (MC.Identify.CurAverage[1] - MC.Identify.CurAverage[0]);
				MC.Foc.Ud = 0;
				MC.Identify.Flag = 0;
				MC.Identify.State = INDUCTANCE_IDENTIFICATION;			
			}	

		  MC.Foc.SinVal = 0;                  //电角度为0，正弦值为0
		  MC.Foc.CosVal = 1;           		 		//电角度为0，余弦值为1	
		  IPack_Transform(&MC.Foc);           //反PACK变换			
		}break;	

		case INDUCTANCE_IDENTIFICATION:          //电感识别             
		{
			if(MC.Identify.Flag == 0)
			{
				MC.Foc.Uq = 0;
				MC.Foc.Ud = 0;						
				if(MC.Sample.IuReal >= -0.05f && MC.Sample.IuReal <= 0.05f)       
				{
					MC.Identify.Flag = 1;
				}						
			}
			
			if(MC.Identify.Flag == 1)
			{
				MC.Foc.Ud = MC.Identify.VoltageSet[1];
				MC.Identify.WaitTim++;            						
				if(MC.Sample.IuReal >= MC.Identify.CurAverage[1] * 0.95f)
				{
					MC.Identify.LsSum += MC.Identify.Rs * 0.334f * 0.00005f *  MC.Identify.WaitTim;
					MC.Identify.WaitTim = 0;
					MC.Identify.Count++;
					MC.Identify.Flag = 0;
					MC.Foc.Ud = 0;
					if(MC.Identify.Count >= 100)
					{
						MC.Identify.Flag = 2;
					}
				}						
			}	
							
			if(MC.Identify.Flag == 2)
			{
				MC.Identify.Ls = MC.Identify.LsSum * 0.01f;
				MC.Identify.Ld = MC.Identify.Ls;
				MC.Identify.Lq = MC.Identify.Ls;		
				
				MC.Identify.Flag = 0;
				MC.Identify.LsSum = 0;
				MC.Identify.WaitTim = 0;
				MC.Identify.State = RESISTANCE_IDENTIFICATION;
				MC.Identify.EndFlag = 1;
			}
			
			MC.Foc.SinVal = 0;                  //电角度为0，正弦值为0
			MC.Foc.CosVal = 1;           		 		//电角度为0，余弦值为1	
			IPack_Transform(&MC.Foc);           //反PACK变换
		}break;	
	}	
	
	MC.Foc.Ubus = MC.Sample.BusReal;		
	Calculate_Stepper_PWM(&MC.Foc);	          //stepper PWM				
}





