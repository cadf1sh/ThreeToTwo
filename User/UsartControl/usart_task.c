/**
  ******************************************************************************
  * 文件名程: usart_task.c
  * 作    者: 浩然
  * 版    本: V1.0
  * 编写日期: 2024-03-29
  * 功    能: 定时执行串口任务
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "usart_task.h"
#include "usart_drv.h"
#include "motor_system.h"
#include "motor_publicdata.h"

volatile u16 UsartTaskId = 10;
volatile u16 UsartTaskTim = 0;

/**
  * 函数功能: 定时执行串口任务
  * 输入参数:
  * 返 回 值: 
  * 说    明:
  */
void Usart_Task(void)
{
	switch(UsartTaskId)
	{
		case 10:
		{
			if(UsartTaskTim>=20)        //1ms
			{
				UsartTaskTim = 0;
				UsartTaskId = 20;

			}
		}
		break;
		
		case 20:
		{ 
		//printf("%0.3f,%0.3f\n",(float)MC.Sample.IaRaw,(float)MC.Sample.IbRaw);                               //采样原始值			
	//printf("%0.3f,%0.3f\n",MC.Sample.IaReal,MC.Sample.IbReal);                    //三相电流值（正弦波）
//			printf("%0.3f,%0.3f\n",MC.Foc.Ialpha,MC.Foc.Ibeta);                                                  //α轴和β轴电流值（正弦波）			
//		  printf("%0.3f,%0.3f,%0.3f\n",(float)MC.Foc.Channel1,(float)MC.Foc.Channel2,(float)MC.Foc.Channel3);  //三相占空比（马鞍波）		
//			printf("%0.3f,%0.3f\n",MC.IdPid.Ref,MC.IdPid.Fbk);                                                   //D轴电流目标值和反馈值	
//			printf("%0.3f,%0.3f\n",MC.IqPid.Ref,MC.IqPid.Fbk);                                                   //Q轴电流目标值和反馈值				
//	    printf("%0.3f\n",(float)MC.Encoder.EncoderVal);                                                   //电角度值
//			printf("%0.3f,%0.3f\n",MC.TShapedAccDec.SpeedOut/7,MC.Speed.MechanicalSpeed);                        //目标速度与实际速度（机械速度，单位RPM）	
//      printf("%0.3f\n",MC.SPLL.ETheta);                                                                    //滑膜观测器计算得到的电角度			
      printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%0.3f\n",MC.IqPid.Ref,MC.Foc.IqLPF,MC.Foc.Uq,MC.Foc.Ud,MC.Foc.Ialpha, MC.Foc.Ibeta,MC.Encoder.ElectricalVal,MC.Foc.CosVal); 	//    	printf("%0.3d,%0.3d,%0.3d,%0.3d\n",MC.Foc.DutyCycleA,MC.Foc.DutyCycleB,MC.Foc.DutyCycleC,MC.Foc.DutyCycleD); 
			UsartTaskId = 10; 
		}
		break;
		
    default:
      break;			
	}
}


