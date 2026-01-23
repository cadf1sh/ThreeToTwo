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

#include "motor_identify.h"

/**
  * 函数功能:电机参数辨识 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */

void Motor_Identify()
{
	switch (MC.Identify.State)
	{		
		case RESISTANCE_IDENTIFICATION:       //?                 
		{
			if(MC.Identify.Flag == 0)           //?
			{
				MC.Foc.Uq = 0;
				MC.Foc.Ud = 0;	
        MC.Identify.Count = 0;					
				MC.Identify.WaitTim = 0;			
				MC.Identify.Flag = 1;
			}	
			
			if(MC.Identify.Flag == 1)           
			{							
				if(( MC.Sample.IuReal * MC.Foc.Ud * 1.5f) / MC.Sample.BusReal >= 0.6f * MC.Identify.CurMax) 
				{						
					MC.Identify.Flag = 2;
				}
				else
				{
					MC.Foc.Ud += 0.0001f;				   	     //??С?
					MC.Identify.VoltageSet[0] = MC.Foc.Ud;							
				}
				
			}
			
			if(MC.Identify.Flag == 2)
			{
					MC.Identify.WaitTim++;
					if(MC.Identify.WaitTim > 4000)       // 0.2S ??
					{
						MC.Identify.CurSum += MC.Sample.IuReal;
					}	
					
					if(MC.Identify.WaitTim >= 4100)      // ?100ε?
					{
						MC.Identify.CurAverage[0] = MC.Identify.CurSum * 0.01f; //?								
						MC.Identify.WaitTim = 0;
						MC.Identify.CurSum = 0;
						MC.Identify.Flag = 3;
					}						
			}	
		
			if(MC.Identify.Flag == 3)
			{				
				if((MC.Sample.IuReal * MC.Foc.Ud * 1.5f) / MC.Sample.BusReal >= MC.Identify.CurMax) 
				{						
					MC.Identify.Flag = 4;
				}
				else
				{
					MC.Foc.Ud += 0.0001f;					    // ??С?
					MC.Identify.VoltageSet[1] = MC.Foc.Ud;							
				}					
			}
			
			if(MC.Identify.Flag == 4)
			{
					MC.Identify.WaitTim++;
					if(MC.Identify.WaitTim > 4000)       // 0.2S ??
					{
						MC.Identify.CurSum += MC.Sample.IuReal;
					}	
					
					if(MC.Identify.WaitTim >= 4100)      // ?100ε?
					{
						MC.Identify.CurAverage[1] = MC.Identify.CurSum * 0.01f; //?
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

		  MC.Foc.SinVal = 0;                  //??0??0
		  MC.Foc.CosVal = 1;           		 		//??0??1	
		  IPack_Transform(&MC.Foc);           //PACK任			
		}break;	

		case INDUCTANCE_IDENTIFICATION:          //?             
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
			
			MC.Foc.SinVal = 0;                  //??0??0
			MC.Foc.CosVal = 1;           		 		//??0??1	
			IPack_Transform(&MC.Foc);           //PACK任
		}break;	
	}	
	
	MC.Foc.Ubus = MC.Sample.BusReal;		
	Calculate_Stepper_PWM(&MC.Foc);	          //stepper PWM				
}




