#include "qpr.h"

void QPR_Struct_Init(QPR_STRUCT* qpr_struct, float kp, float kr, float wc, float w0, float ts, float outUp, float outLow)
{
	qpr_struct->Kp = kp;
	qpr_struct->Kr = kr;
	qpr_struct->Wc = wc;
	qpr_struct->W0 = w0;
	qpr_struct->Ts = ts;
	
	qpr_struct->B0 = 0.0f;
	qpr_struct->B1 = 0.0f;
	qpr_struct->B2 = 0.0f;
	qpr_struct->A0 = 0.0f;
	qpr_struct->A1 = 0.0f;
	qpr_struct->A2 = 0.0f;

	qpr_struct->Set = 		0.0f;		// Really ??
	qpr_struct->Actural = 0.0f;

  qpr_struct->Error = 			0.0f;
	qpr_struct->preError = 		0.0f;
	qpr_struct->prepreError = 0.0f;
	
	qpr_struct->Out = 				0.0f;
	qpr_struct->preOut = 			0.0f;
	qpr_struct->prepreOut = 	0.0f;
	
	qpr_struct->OutUpperLimit = outUp;		
	qpr_struct->OutLowerLimit = outLow;		
}


/* 新增：以下两段原实现保留供对照，启用后面的归一化版本。 */
// void QPR_Tustin(QPR_STRUCT* qpr_struct)
// {
// 	float k = qpr_struct->W0 / tanf(qpr_struct->W0 * qpr_struct->Ts / 2);
	
// 	qpr_struct->A0 = k*k + 2*qpr_struct->Wc*k + qpr_struct->W0*qpr_struct->W0;
// 	qpr_struct->A1 = 2*(qpr_struct->W0*qpr_struct->W0 - k*k);
// 	qpr_struct->A2 = k*k - 2*qpr_struct->W0*k + qpr_struct->W0*qpr_struct->W0;
// 	qpr_struct->B0 = qpr_struct->Kp*(k*k + qpr_struct->W0*qpr_struct->W0) + 2*(qpr_struct->Kp + qpr_struct->Kr)*qpr_struct->Wc*k;
// 	qpr_struct->B1 = 2*qpr_struct->Kp*(qpr_struct->W0*qpr_struct->W0 - k*k);
// 	qpr_struct->B2 = qpr_struct->Kp*(k*k + qpr_struct->W0*qpr_struct->W0) - 2*(qpr_struct->Kp + qpr_struct->Kr)*qpr_struct->Wc*k;
// }

// void QPR_Cal(QPR_STRUCT* qpr_struct)
// {
// 	qpr_struct->Error = qpr_struct->Actural - qpr_struct->Set;
// 	qpr_struct->Out = qpr_struct->B0/qpr_struct->A0*qpr_struct->Error
// 									+ qpr_struct->B1/qpr_struct->A0*qpr_struct->preError
// 									+ qpr_struct->B2/qpr_struct->A0*qpr_struct->prepreError
// 									- qpr_struct->A1/qpr_struct->A0*qpr_struct->preOut
// 									- qpr_struct->A2/qpr_struct->A0*qpr_struct->prepreOut;
	
// 	// Refresh errors and outs
// 	qpr_struct->prepreError = qpr_struct->preError;
// 	qpr_struct->preError = qpr_struct->Error;
// 	qpr_struct->prepreOut = qpr_struct->preOut;
// 	qpr_struct->preOut = qpr_struct->Out;
	
// 	// Amplitude limiting in the end
// 	qpr_struct->Out = fminf(qpr_struct->OutUpperLimit, fmaxf(qpr_struct->OutLowerLimit, qpr_struct->Out));
// }


/* 新增：初始化或切频时计算归一化 QPR 系数，调用点见 acac.c。 */
void QPR_Tustin(QPR_STRUCT *q)
{
    double w = q->W0;
    double k = w / tan(w * q->Ts * 0.5);
    double sum = k * k + w * w;
    double diff = w * w - k * k;
    double damping = 2.0 * q->Wc * k;
    double a0 = sum + damping;
    double resonant = (q->Kp + q->Kr) * damping;

    q->B0 = (q->Kp * sum + resonant) / a0;
    q->B1 = 2.0 * q->Kp * diff / a0;
    q->B2 = (q->Kp * sum - resonant) / a0;
    q->A0 = 1.0;
    q->A1 = 2.0 * diff / a0;
    q->A2 = (sum - damping) / a0;
}

/* 新增：每帧用目标减测量，算电压修正量，由前后级闭环调用。 */
void QPR_Cal(QPR_STRUCT *q)
{
    q->Error = q->Set - q->Actural;

    double raw = q->B0 * q->Error
               + q->B1 * q->preError
               + q->B2 * q->prepreError
               - q->A1 * q->preOut
               - q->A2 * q->prepreOut;

    q->prepreError = q->preError;
    q->preError = q->Error;
    q->prepreOut = q->preOut;
    q->preOut = raw;  /* QPR 的历史记录保留限幅前的值。 */

    if (isfinite(raw))
        q->Out = fminf(q->OutUpperLimit, fmaxf(q->OutLowerLimit, (float)raw));
    else
        q->Out = (float)raw;  /* 交给控制层记录故障。 */
}
