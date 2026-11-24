#ifndef _QPR_H
#define _QPR_H

#include <math.h>

typedef struct{
	float Kp;
	float Kr;
	float Wc;
	float W0;
	float Ts;				//Control period
	
	/* 新增：用 double 保存归一化系数，降低低频递推误差。 */
	// float B0;
	double B0;
	// float B1;
	double B1;
	// float B2;
	double B2;
	// float A0;
	double A0;
	// float A1;
	double A1;
	// float A2;
	double A2;
	
	float Set;
	float Actural;
	
	float Error;
	float preError;
	float prepreError;
	
	float Out;
	/* 新增：用 double 保存限幅前的历史输出。 */
	// float preOut;
	double preOut;
	// float prepreOut;
	double prepreOut;
	
	float OutUpperLimit;
	float OutLowerLimit;
}QPR_STRUCT;

void QPR_Struct_Init(QPR_STRUCT* qpr_struct, float kp, float kr, float wc, float w0, float ts, float outUp, float outLow);
void QPR_Tustin(QPR_STRUCT* qpr_struct);
void QPR_Cal(QPR_STRUCT* qpr_struct);

#endif
