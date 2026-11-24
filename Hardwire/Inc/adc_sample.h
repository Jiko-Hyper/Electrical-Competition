#ifndef ACAC_ADC_SAMPLE_H
#define ACAC_ADC_SAMPLE_H

#include <stdint.h>

/* ACAC_Board_Init 在 CubeMX 的 MX_ADC1_Init、MX_ADC3_Init 之后调用。
   只打开 DMA 等待触发，启动 TIM1 后才会真正开始采样。 */
int adc_sample_start(void);

extern volatile uint16_t g_adc1_raw[9];
extern volatile uint16_t g_adc3_raw[2];

#endif
