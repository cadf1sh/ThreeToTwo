/**
  ******************************************************************************
  * 文件名程: 
  * 作    者: 
  * 版    本: V1.0
  * 编写日期: 
  * 功    能: 
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/

#include "motor_system.h"

static void Motor_System_StopOutput(void)
{
	MC.Foc.DutyCycleA = 0;
	MC.Foc.DutyCycleB = 0;
	MC.Foc.DutyCycleC = 0;
	MC.Foc.DutyCycleD = 0;
}

void Motor_System_Init(void)
{
	Motor_Struct_Init();                       //??
	Stepper_Foc_Init();
	Stepper_Foc_SetCurrentLimit();
	Stepper_Foc_SetCurrentRef();
}


void Motor_System_Run()
{
	Calculate_Bus_Voltage(&MC.Sample);
	Calculate_Phase_Current(&MC.Sample);
	if (MC.Motor.RunState == ADC_CALIB)
	{
		Calculate_Adc_Offset(&MC.Sample);
		Motor_System_StopOutput();
		if (MC.Sample.EndFlag == 1)
		{
			MC.Motor.RunState = MOTOR_SENSORUSE;
		}
		return;
	}

	if (MC.Sample.EndFlag == 1)
	{
		if ((MC.Sample.BusReal >= 3 && MC.Sample.BusReal <= 11.8) || MC.Sample.BusReal >= 24)
		{
			MC.Motor.RunState = MOTOR_ERROR;
		}
	}

	switch (MC.Motor.RunState)
	{
		case MOTOR_SENSORUSE:
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);	
			Calculate_Encoder_Data(&MC.Encoder);
			Stepper_Foc_SetCurrentRef();
			Stepper_Foc_Run();
		}break;

		case MOTOR_STOP:
		case MOTOR_ERROR:
		default:
			Motor_System_StopOutput();
			break;
	}
}
