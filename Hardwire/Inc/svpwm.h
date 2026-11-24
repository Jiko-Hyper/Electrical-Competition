#ifndef _SVPWM_H
#define _SVPWM_H

#include "acac_config.h"
#include <math.h>

typedef struct{
	float Va;
	float Vb;
	float Vc;
	float Vdc;
	
	float DutyA;
	float DutyB;
	float DutyC;
	
	float scale;
	
}SVPWM_Struct;
int SVPWM_Cal(SVPWM_Struct* svpwm_struct);
#endif
