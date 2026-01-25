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

static STEPPER_FOC_STRUCT stepper_foc_ctrl;

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
	Stepper_Foc_Init(&stepper_foc_ctrl, MC.Foc.PwmCycle, MC.Foc.PwmLimit);
	Stepper_Foc_SetCurrentLimit(&stepper_foc_ctrl, 3.0f, 3.0f);
	Stepper_Foc_SetCurrentRef(&stepper_foc_ctrl, 0.0f, 0.0f);
	MC.IdPid.Ref = 0.0f;
	MC.IqPid.Ref = 1.0f;
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
		if (MC.Sample.BusReal <= 12 || MC.Sample.BusReal >= 40)
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
			Stepper_Foc_SetCurrentRef(&stepper_foc_ctrl, MC.IdPid.Ref, MC.IqPid.Ref);
			Stepper_Foc_Run(&stepper_foc_ctrl, MC.Sample.IuReal, MC.Sample.IwReal, (float)MC.Encoder.ElectricalVal, MC.Sample.BusReal);
			MC.Foc = stepper_foc_ctrl.foc;
		}break;

		case MOTOR_STOP:
		case MOTOR_ERROR:
		default:
			Motor_System_StopOutput();
			break;
	}
}
