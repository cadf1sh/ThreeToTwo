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

#include "motor_system.h"


/**
  * 函数功能:电机系统初始化 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
 
void Motor_System_Run()
{
	if(MC.Sample.EndFlag == 1)                 //?У?ADC
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
			MC.Motor.RunState = MOTOR_ERROR;       //粻
		}
	}	
	switch (MC.Motor.RunState)
	{		
		case ADC_CALIB:                 //ADCУ?
		{
			Calculate_Adc_Offset(&MC.Sample);
			if(MC.Sample.EndFlag == 1)
			{
				MC.Motor.RunState = MOTOR_SENSORUSE;
			}		
		}break;
		
		case MOTOR_SENSORUSE:                     //ип
		{
		  Calculate_Encoder_Data(&MC.Encoder);    //			
      Sensoruse_Control();
		}break;	

		case 2:                         //error
		{
			MC.Foc.DutyCycleA = 0;
			MC.Foc.DutyCycleB = 0;
			MC.Foc.DutyCycleC = 0;
			MC.Foc.DutyCycleD = 0;
		}break;		
		
		case 1:                          //stop
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



