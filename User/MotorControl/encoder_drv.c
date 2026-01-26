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

#include "encoder_drv.h"                           

/**
  * 函数功能:计算外部编码器数据
  * 输入参数:
  * 返回参数:
  * 说    明: 
  */
void Calculate_Encoder_Data(ENCODER_STRUCT *p)
{
	if(p->Dir == 1)                                                         //判断编码器方向  
	{
		p->EncoderVal = p->EncoderValMax - p->EncoderVal;			                //方向取反
	}
	
/**********************************计算电角度********************************/	
	p->ElectricalVal = ((p->EncoderVal - p->CalibOffset) * p->PolePairs) % p->EncoderValMax; 
	p->ElectricalVal = p->EncoderValMax - p->ElectricalVal;
	if(p->ElectricalVal < 0)                                                //处理校准可能带来的负值
	{
		p->ElectricalVal = p->ElectricalVal + p->EncoderValMax;			          //计算电角度
	}
}