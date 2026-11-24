#include "pi.h"

void PI_Struct_Init(PI_Struct* pi_struct, float kp, float ki, float ts, float outUp, float outLow)
{
	pi_struct->Kp = kp;
	pi_struct->Ki = ki;
	pi_struct->Ts = ts;
	
	pi_struct->Set = 			0.0f;
	pi_struct->Actrual = 	0.0f;
	pi_struct->Error = 		0.0f;
	pi_struct->Integral = 0.0f;
	pi_struct->Out = 			0.0f;
	
	pi_struct->OutUpLimit =  outUp;
	pi_struct->OutLowLimit = outLow;
}

void PI_Cal(PI_Struct* pi_struct)
{
	/* 新增：母线电压偏低时，提高输入电流指令。 */
	// pi_struct->Error = pi_struct->Actrual - pi_struct->Set; 
	pi_struct->Error = pi_struct->Set - pi_struct->Actrual;
	float p = pi_struct->Kp * pi_struct->Error;
	float detaI = pi_struct->Ki * pi_struct->Ts * pi_struct->Error;
	float nextI = pi_struct->Integral + detaI;
	
	if(p + nextI > pi_struct->OutUpLimit)
	{
		nextI = pi_struct->OutUpLimit - p;
	}
	else if(p + nextI < pi_struct->OutLowLimit)
	{
		nextI = pi_struct->OutLowLimit - p;
	}
	
	pi_struct->Integral = nextI;
	pi_struct->Out = fminf(pi_struct->OutUpLimit, fmaxf(pi_struct->OutLowLimit, p + nextI));
}