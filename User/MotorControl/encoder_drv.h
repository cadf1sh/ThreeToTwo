#ifndef __ENCODER_DRV_H
#define __ENCODER_DRV_H

#include "main.h"

typedef struct
{
	u8    Dir;                        // 编码器方向	
	u8    PolePairs;                  // 转子极对数
	s32   EncoderVal;                 // 编码器原始数据
	s32   EncoderValMax;              // 编码器最大原始值
	s32   ElectricalVal;              // 电气角度
	u16   CalibOffset;                // 转子零位偏差	
}ENCODER_STRUCT; 

void Calculate_Encoder_Data(ENCODER_STRUCT *p);

#endif 