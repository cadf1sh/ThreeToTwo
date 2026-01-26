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

#include "speed_drv.h"                              
#include "motor_publicdata.h"



void Calculate_Speed(SPEED_STRUCT *p)
{
  p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast;      //计算单位时间内位移
	p->ElectricalPosLast = p->ElectricalPosThis;
	if(p->ElectricalPosChange >= (p->ElectricalValMax * 0.5f))                 //越过编码器零点
	{
		p->ElectricalPosChange = p->ElectricalPosChange - p->ElectricalValMax;
	}
	if(p->ElectricalPosChange <= (-p->ElectricalValMax * 0.5f))
	{
		p->ElectricalPosChange = p->ElectricalPosChange + p->ElectricalValMax;   //越过编码器零点
	}
	p->ElectricalSpeedRaw = p->ElectricalPosChange * p->ElectricalSpeedFactor; //计算原始电角速度
}


