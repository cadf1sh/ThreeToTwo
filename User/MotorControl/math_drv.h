#ifndef __MATH_DRV_H
#define __MATH_DRV_H

#include "main.h"

#define SIN_RAD     0X0300
#define U0_90       0x0000 
#define U90_180     0x0100
#define U180_270    0x0200
#define U270_360    0x0300

void Calculate_Sin_Cos(s32 angle, float *sinval, float *cosval);
void Amplitude_Limit(s32 *input, s32 min, s32 max);

#endif 
