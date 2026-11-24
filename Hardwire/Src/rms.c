#include "rms.h"

#include <math.h>

/* 作用：清空有效值累计状态并设置窗口点数。
   调用位置：acac_init 设置输入窗口，set_output_frequency 设置输出窗口。 */
void rms_init(RMS *rms, uint32_t period)
{
    *rms = (RMS){0};
    rms->period = period;
}

/* 作用：累计一个采样点，满一个窗口后更新有效值。
   调用位置：acac_step 每帧对输入电压、两路线电压和输出电流调用。 */
void rms_step(RMS *rms, float sample)
{
    rms->sum += (double)sample * sample;
    ++rms->count;

    /* 收满一整周期，再发布一个有效值。 */
    if (rms->count >= rms->period)
    {
        rms->value = (float)sqrt(rms->sum / rms->count);
        rms->valid = 1;
        rms->sum = 0.0;
        rms->count = 0;
    }
}
