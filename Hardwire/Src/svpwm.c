#include "svpwm.h"

int SVPWM_Cal(SVPWM_Struct* svpwm_struct)
{
	float vdc = svpwm_struct->Vdc;
	
	if(!isfinite(vdc) || vdc <= 0.0f || 
		!isfinite(svpwm_struct->Va) || !isfinite(svpwm_struct->Vb) || !isfinite(svpwm_struct->Vc)) return 0;
	
	float minVoltage = fminf(svpwm_struct->Va, fminf(svpwm_struct->Vb, svpwm_struct->Vc));
	float maxVoltage = fmaxf(svpwm_struct->Va, fmaxf(svpwm_struct->Vb, svpwm_struct->Vc));
	float offset = (minVoltage + maxVoltage) / 2.0f;
	
	float span = maxVoltage - minVoltage;
	float maxSpan = (DUTY_MAX - DUTY_MIN) * vdc;
	
	float scale = 1.0f;
	if(span > maxSpan) scale = maxSpan / span;
	
	svpwm_struct->DutyA = 0.5f + (svpwm_struct->Va - offset) * scale / vdc;
	svpwm_struct->DutyB = 0.5f + (svpwm_struct->Vb - offset) * scale / vdc;
	svpwm_struct->DutyC = 0.5f + (svpwm_struct->Vc - offset) * scale / vdc;
	
	// Amplitude limiting
	svpwm_struct->DutyA = fminf(DUTY_MAX, fmaxf(DUTY_MIN, svpwm_struct->DutyA));
	svpwm_struct->DutyB = fminf(DUTY_MAX, fmaxf(DUTY_MIN, svpwm_struct->DutyB));
	svpwm_struct->DutyC = fminf(DUTY_MAX, fmaxf(DUTY_MIN, svpwm_struct->DutyC));
	svpwm_struct->scale = scale;
	
	return 1;
}
	