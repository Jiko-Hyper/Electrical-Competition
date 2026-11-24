#ifndef ACAC_RMS_H
#define ACAC_RMS_H

#include <stdint.h>

typedef struct
{
    double sum;
    uint32_t count;
    uint32_t period;  /* 每个有效值窗口要累计的采样点数 */
    float value;
    uint8_t valid;
} RMS;

/* acac_init、set_output_frequency 调用，复位有效值窗口。 */
void rms_init(RMS *rms, uint32_t period);
/* acac_step 每个采样帧调用，累计并更新有效值。 */
void rms_step(RMS *rms, float sample);

#endif
