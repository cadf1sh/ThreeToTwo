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
	MC.IdPid.Ref = 0.0f;
	MC.IqPid.Ref = 1.0f;
	Stepper_Foc_SetCurrentRef();
}

static u8 Motor_System_CalibAlign(void)
{
	if (MC.Encoder.CalibFlag >= 2)
	{
		return 1;
	}

	MC.Foc.Uq = 0.0f;
	if (MC.Encoder.CalibFlag == 0)
	{
		MC.Foc.Ud += 0.0001f;
		MC.Foc.SinVal = 1.0f;
		MC.Foc.CosVal = 0.0f;
		if (MC.Foc.Ud >= MC.Identify.VoltageSet[1])
		{
			MC.Foc.Ud = 0.0f;
			MC.Encoder.CalibFlag = 1;
		}
	}

	if (MC.Encoder.CalibFlag == 1)
	{
		MC.Foc.Ud += 0.0001f;
		MC.Foc.SinVal = 0.0f;
		MC.Foc.CosVal = 1.0f;
		if (MC.Foc.Ud >= MC.Identify.VoltageSet[1])
		{
			MC.Encoder.CalibOffset = MC.Encoder.EncoderVal;
			MC.Encoder.CalibFlag = 2;
			MC.Foc.Ud = 0.0f;
		}
	}
	IPack_Transform(&MC.Foc);
	Calculate_Stepper_PWM(&MC.Foc);
	return (MC.Encoder.CalibFlag >= 2);
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
		if ((MC.Sample.BusReal >= 3 && MC.Sample.BusReal <= 12) || MC.Sample.BusReal >= 24)
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
			if (Motor_System_CalibAlign() == 0)
			{
				break;
			}
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
