void Motor_System_Run()
{
	if(MC.Sample.EndFlag == 1)                 //?§µ?ADC
	{
		Calculate_Bus_Voltage(&MC.Sample);       //????
		Calculate_Phase_Current(&MC.Sample);     //?	
		MC.SMO.Gain = MC.Sample.BusReal * 0.57735f;
    MC.IdPid.OutMax =  MC.Sample.BusReal * 0.57735f; 
		MC.IdPid.OutMin = -MC.Sample.BusReal * 0.57735f;
    MC.IqPid.OutMax =  MC.Sample.BusReal * 0.57735f; 
		MC.IqPid.OutMin = -MC.Sample.BusReal * 0.57735f;
		if(MC.Sample.BusReal <= 12 || MC.Sample.BusReal >= 40)
		{
			MC.Motor.RunState = MOTOR_ERROR;       //» 
		}
	}	
	switch (MC.Motor.RunState)
	{		
		case ADC_CALIB:                 //ADC§µ?
		{
			Calculate_Adc_Offset(&MC.Sample);
			if(MC.Sample.EndFlag == 1)
			{
				MC.Motor.RunState = MOTOR_SENSORUSE;
			}		
		}break;
		
		case MOTOR_SENSORUSE:                     //§Ú§á
		{
		  Calculate_Encoder_Data(&MC.Encoder);    //			
      Sensoruse_Control();
		}break;	

		case MOTOR_ERROR:                         //?
		{
			MC.Foc.DutyCycleA = 0;
			MC.Foc.DutyCycleB = 0;
			MC.Foc.DutyCycleC = 0;
			MC.Foc.DutyCycleD = 0;
		}break;		
		
		case MOTOR_STOP:                          //?
		{	  			
			MC.Foc.DutyCycleA = 0;
			MC.Foc.DutyCycleB = 0;
			MC.Foc.DutyCycleC = 0;
			MC.Foc.DutyCycleD = 0;
		}break;
		
		default :
		 break;			
	}		
}



