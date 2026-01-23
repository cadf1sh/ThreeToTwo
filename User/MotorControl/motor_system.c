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
void Motor_System_Init(void)
{
	Motor_Struct_Init();                       //结构体参数初始化
}

/**
  * 函数功能:系统运行 
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
		case ADC_CALIB:                          //ADCУ?
		{
			Calculate_Adc_Offset(&MC.Sample);
			if(MC.Sample.EndFlag == 1)
			{
#if MOTOR_TWO_PHASE_STEPPER
				MC.Motor.RunState = MOTOR_SENSORUSE;  // Stepper: skip identify
#else
				MC.Motor.RunState = MOTOR_IDENTIFY;
#endif
			}		
		}break;
		
		case MOTOR_IDENTIFY:                     //?
		{
      Motor_Identify();
			if(MC.Identify.EndFlag == 1)           //?
			{
			  MC.SMO.Rs = MC.Identify.Rs;          //?????
			  MC.SMO.Ld = MC.Identify.Ld;          //?????
				MC.Motor.RunState = MOTOR_SENSORUSE;   
			}				
		}break;		

		case MOTOR_SENSORUSE:                     //ип
		{
		  Calculate_Encoder_Data(&MC.Encoder);    //			
      Sensoruse_Control();
		}break;	

		case MOTOR_SENSORLESS:                    //?п
		{
      Sensorless_Control();
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
