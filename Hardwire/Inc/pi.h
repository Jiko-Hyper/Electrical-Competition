#ifndef _PI_H
#define _PI_H

#include <math.h>

typedef struct{
	float Kp;
	float Ki;
	float Ts;
	
	float Set;
	float Actrual;
	float Error;
	float Integral;
	float Out;
	
	float OutUpLimit;
	float OutLowLimit;
	
} PI_Struct;
/* 新增：acac.c 通过这两个接口初始化并执行母线 PI。 */
void PI_Struct_Init(PI_Struct *pi, float kp, float ki, float ts,
                    float outUp, float outLow);
void PI_Cal(PI_Struct *pi);
#endif
