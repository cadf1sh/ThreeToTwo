/**
  ******************************************************************************
  * 文件名程: lcd_task.c
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 2024-03-29
  * 功    能: 定时进行lcd的显示任务
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "lcd_task.h"
#include "lcd_drv.h"
#include "motor_publicdata.h"

volatile u16 LcdTaskId = 10;
volatile u16 LcdTaskTim = 0;

float machinebutton = 0;
u16 IIA = 0;
u16 IIB = 0;
s32 Encod = 0;
s32  Volt = 0;
/**
  * 函数功能: 定时进行lcd的显示任务
  * 输入参数:
  * 返 回 值: 
  * 说    明:
  */
void Lcd_Task(void)
{
	switch(LcdTaskId)
	{
		case 10:
		{
			if(LcdTaskTim>=20)        //1ms
			{
				LcdTaskTim = 0;
				LcdTaskId = 20;
			}
		}
		break;
		
		case 20:
		{ 
		LCD_ShowIntNum(0,16*0,MC.Speed.MechanicalSpeedSet,5,BLACK,WHITE,16);
		LCD_ShowFloatNum1(0,16*1,MC.Foc.Ubus,5,BLACK,WHITE,16);
		LCD_ShowFloatNum1(80,16*1,MC.Foc.Uq,5,BLACK,WHITE,16);
		LCD_ShowFloatNum1(0,16*2,MC.Sample.IuReal,5,BLACK,WHITE,16);
		LCD_ShowFloatNum1(0,16*3,MC.Sample.IwReal,5,BLACK,WHITE,16);
		LCD_ShowIntNum(0,16*4,MC.Encoder.EncoderVal,5,BLACK,WHITE,16);
		LCD_ShowIntNum(80,16*4,MC.Encoder.ElectricalVal,5,BLACK,WHITE,16);
			LcdTaskId = 10; 
		}
		break;
		
    default:
      break;			
	}
}



