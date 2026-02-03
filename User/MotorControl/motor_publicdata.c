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

#include "motor_publicdata.h"

MOTORCONTROL_STRUCT MC;                           //实例化总结构体

/**
  * 函数功能:电机结构体初始化 
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Motor_Struct_Init()
{	
	/*参数初始化*/
	MC.Motor.RunState = ADC_CALIB;      			 	  	 //设置电机最初的运行状态
	MC.Motor.RunMode = ENCODER_CALIB;                //设置运行后最初的运行模式
		
	MC.Sample.CurrentDir = -1;       					  	   //设置电机电流采样的方向(由硬件决定)
	MC.Sample.CurrentFactor = PHASE_CURRENT_FACTOR;  //相电流计算系数(由采样电阻值和放大倍数以及ADC分辨率计算得出)
	MC.Sample.BusFactor = VBUS_FACTOR;               //母线电压计算系数（由分压电阻计算得出）

	MC.Encoder.Dir = CCW;              							 //设置编码器的方向（逆时针转动 角度从0向360度增加）
	MC.Encoder.PolePairs = POLEPAIRS;								 //设置电机的极对数（磁铁数除以2）
	MC.Encoder.EncoderValMax = PUL_MAX;  					   //设置编码器单圈脉冲的最大值
		MC.Encoder.CalibOffset = 0;
	MC.Encoder.CalibFlag = 0;

	MC.Foc.PwmCycle = PWM_CYCLE;									   //设置PWM周期
	MC.Foc.PwmLimit = PWM_LIMLT;									   //设置PWM限幅值
	
	MC.Position.ElectricalValMax = PUL_MAX; 			   //设置电度角的最大值
	
	MC.Speed.ElectricalValMax = PUL_MAX; 					   //设置编码器单圈脉冲的最大值	
	MC.Speed.ElectricalSpeedLPFFactor = 0.05f;       //设置速度低通滤波系数
	MC.Speed.ElectricalSpeedFactor = 146.5f;         //设置速度计算系数


	MC.Foc.IdLPFFactor = 0.1f;
  MC.Foc.IqLPFFactor = 0.1f;
  MC.Foc.IdLPF = 0.0f;
  MC.Foc.IqLPF = 0.0f;

  MC.IdPid.Ref = 0.0f;
  MC.IqPid.Ref = 0.0f;
  MC.IdPid.Ref_lim = 2.0f;
  MC.IqPid.Ref_lim = 2.0f;
	MC.IdPid.ErrLim = 5.0f;
	MC.IqPid.ErrLim = 5.0f;
	
  MC.IdPid.Kp = 0.0005f;
  MC.IdPid.Ki = 0.0005f;
  MC.IdPid.Kd = 0.0f;
  MC.IdPid.OutMax = 6.0f;
  MC.IdPid.OutMin = -6.0f;

  MC.IqPid.Kp = 0.0005f;
  MC.IqPid.Ki = 0.0005f;
  MC.IqPid.Kd = 0.0f;
  MC.IqPid.OutMax = 6.0f;
  MC.IqPid.OutMin = -6.0f;

	MC.SpdPid.Kp = 0.005;                           //设置速度PID比例系数
	MC.SpdPid.KpMax = 0.005;                       //设置速度PID比例系数最大值（用于分段或模糊PID）
	MC.SpdPid.KpMin = 0.005;	                       //设置速度PID比例系数最小值（用于分段或模糊PID）
	MC.SpdPid.Ki = 0.0005f;                        //设置速度PID积分系数
	MC.SpdPid.OutMax = 2;                            //设置速度PID输出上限  
	MC.SpdPid.OutMin = -2;	                         //设置速度PID输出下限
	MC.SpdPid.ErrLim = 200;
	
  MC.PosPid.Kp = 0.5f;                             //设置位置PID比例系数
	MC.PosPid.Ki = 0;                                //设置位置PID积分系数
	MC.PosPid.Kd = 0;                                //设置位置PID微分系数
	MC.PosPid.OutMax = 14000;                        //设置位置PID输出上限
	MC.PosPid.OutMin = -14000;                       //设置位置PID输出下限
}  