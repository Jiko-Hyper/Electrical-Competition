#include "adc_sample.h"
#include "acac_board.h"
#include "acac_config.h"
#include "adc.h"

volatile uint16_t g_adc1_raw[9];
volatile uint16_t g_adc3_raw[2];

/* 每路 DMA 都放两帧。Keil 用 acac_dma.sct 将这两个数组放进 AXI SRAM，
   当前工程只开 I-Cache，CPU 可以直接读取 DMA 写入的结果。 */
__attribute__((section("adc_dma"), aligned(32)))
static volatile uint16_t adc1_dma[2][9];
__attribute__((section("adc_dma"), aligned(32)))
static volatile uint16_t adc3_dma[2][2];

static uint8_t frame_mask;  /* bit0 表示 ADC1 到了，bit1 表示 ADC3 到了 */
static uint8_t frame_half;  /* 本次要凑齐的是第 0 帧还是第 1 帧 */

/* 作用：校准 ADC1、ADC3 并启动双帧循环 DMA，等待 TIM1 触发。
   调用位置：ACAC_Board_Init。 */
int adc_sample_start(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
                                    ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY,
                                    ADC_SINGLE_ENDED) != HAL_OK)
    {
        return 0;
    }

    /* ADC1 每帧 9 个码，ADC3 每帧 2 个码，长度都按 16 位采样点数填。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma, 18) != HAL_OK ||
        HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_dma, 4) != HAL_OK)
    {
        return 0;
    }

    return 1;
}

/* 作用：按 ADC Rank 把一帧原始码换算成六个电压、电流值。
   调用位置：adc_frame_ready，在 ADC1 和 ADC3 同一帧都到齐后。 */
static Measurement read_measurement(void)
{
    Measurement m;

    /* 这些下标对应你在 CubeMX 配的 ADC Rank，数组从 0 开始。 */
    m.uin = ((float)g_adc1_raw[0] - g_adc1_raw[1] - CAL_UIN_ZERO) * CAL_UIN_GAIN;
    m.iin = ((float)g_adc1_raw[2] - g_adc1_raw[3] - CAL_IIN_ZERO) * CAL_IIN_GAIN;
    m.uab = ((float)g_adc1_raw[4] - g_adc1_raw[5] - CAL_UAB_ZERO) * CAL_UAB_GAIN;
    m.io  = ((float)g_adc1_raw[6] - g_adc1_raw[7] - CAL_IO_ZERO) * CAL_IO_GAIN;
    m.udc = ((float)g_adc1_raw[8] - CAL_UDC_ZERO) * CAL_UDC_GAIN;
    m.ubc = ((float)g_adc3_raw[0] - g_adc3_raw[1] - CAL_UBC_ZERO) * CAL_UBC_GAIN;

    /* 交流通道靠近 ADC 上下限时可能削顶。母线通道可以读到零。 */
    m.valid = 1;
    for (unsigned i = 0; i < 9; ++i)
    {
        if (g_adc1_raw[i] > 65503u || (i < 8 && g_adc1_raw[i] < 32u))
            m.valid = 0;
    }
    for (unsigned i = 0; i < 2; ++i)
    {
        if (g_adc3_raw[i] < 32u || g_adc3_raw[i] > 65503u)
            m.valid = 0;
    }
    return m;
}

/* 作用：复制刚写完的 DMA 半区、配对两路帧并交给闭环。
   调用位置：HAL_ADC_ConvHalfCpltCallback 和 HAL_ADC_ConvCpltCallback。 */
static void adc_frame_ready(ADC_HandleTypeDef *adc, uint8_t half)
{
    uint32_t cycle_start = DWT->CYCCNT;
    uint8_t adc_bit = (adc->Instance == ADC1) ? 1u : 2u;

    if (g_board_fault)
        return;

    /* 两路 ADC 应该各来一次，而且必须是同一个 DMA 半区。 */
    if ((frame_mask & adc_bit) || (frame_mask && frame_half != half))
    {
        ACAC_Board_SamplingFault(FAULT_ADC);
        return;
    }

    frame_half = half;
    if (adc_bit == 1u)
    {
        for (unsigned i = 0; i < 9; ++i)
            g_adc1_raw[i] = adc1_dma[half][i];
    }
    else
    {
        for (unsigned i = 0; i < 2; ++i)
            g_adc3_raw[i] = adc3_dma[half][i];
    }

    frame_mask |= adc_bit;
    if (frame_mask == 3u)
    {
        frame_mask = 0;
        Measurement sample = read_measurement();
        ACAC_Board_OnSample(&sample, cycle_start);
    }
}

/* 作用：处理 DMA 缓冲区前半的采样帧。
   调用位置：DMA1_Stream0/1_IRQHandler 经 HAL 的 ADC DMA 半满回调。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc)
{
    adc_frame_ready(adc, 0);
}

/* 作用：处理 DMA 缓冲区后半的采样帧。
   调用位置：DMA1_Stream0/1_IRQHandler 经 HAL 的 ADC DMA 全满回调。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    adc_frame_ready(adc, 1);
}

/* 作用：记录 ADC 或 DMA 报错，并关闭功率输出。
   调用位置：ADC 或 DMA 中断处理过程中由 HAL 调用。 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{
    (void)adc;
    ACAC_Board_SamplingFault(FAULT_ADC);
}
