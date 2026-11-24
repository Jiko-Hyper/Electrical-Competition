#include "acac_board.h"
#include "acac_config.h"
#include "adc_sample.h"
#include "main.h"
#include "tim.h"

Controller g_control;
volatile uint32_t g_irq_cycles;
volatile uint32_t g_board_fault;

static volatile uint32_t last_frame_ms;
static volatile uint32_t pending_command;
static volatile uint32_t external_trip;

static uint8_t rect_previous;
static uint8_t inv_previous;

/* 作用：拉高 PE14 禁止驱动，清除 TIM1 和 TIM8 的主输出使能。
   调用位置：初始化、PWM 禁止路径、ADC 故障和外部故障处理。 */
static void DriverOff(void)
{
    GPIOE->BSRR = DRIVER_DIS_HIGH_Pin;
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    CLEAR_BIT(TIM8->BDTR, TIM_BDTR_MOE);
    rect_previous = 0;
    inv_previous = 0;
}

/* 作用：立即关闭驱动，禁止中断并停在故障处。
   调用位置：Error_Handler、CPU 异常入口和 ACAC_Board_Init 初始化失败时。 */
void ACAC_Board_Fatal(void)
{
    __disable_irq();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    DriverOff();
    MODIFY_REG(GPIOE->MODER, GPIO_MODER_MODE14, GPIO_MODER_MODE14_0);

    for (;;)
    {
        __NOP();
    }
}

/* 作用：锁存采样或实时性故障，并关闭 PWM，复位后才可恢复。
   调用位置：ADC 错误或丢帧回调、ACAC_Board_OnSample 和 ACAC_Board_Poll。 */
void ACAC_Board_SamplingFault(uint32_t reason)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    g_board_fault |= reason;   /* 采样和超时故障锁定到复位 */
    DriverOff();
    __set_PRIMASK(irq);
}

/* 作用：读取 PG2 外部过流或保护信号。
   调用位置：ACAC_Board_OnSample 和 WritePwm。 */
static int ExternalFault(void)
{
    return HAL_GPIO_ReadPin(EXTERNAL_FAULT_HIGH_GPIO_Port,
                            EXTERNAL_FAULT_HIGH_Pin) == GPIO_PIN_SET;
}

/* 作用：确认采样增益和上板确认项齐全，决定是否允许启动。
   调用位置：ACAC_Board_Init。 */
static int BoardReady(void)
{
    return ACAC_WIRING_VERIFIED && ACAC_CALIB_VERIFIED &&
           ACAC_POWER_VERIFIED && ACAC_HARDTRIP_VERIFIED &&
           CAL_UIN_GAIN != 0.0f && CAL_IIN_GAIN != 0.0f &&
           CAL_UAB_GAIN != 0.0f && CAL_UBC_GAIN != 0.0f &&
           CAL_IO_GAIN != 0.0f && CAL_UDC_GAIN != 0.0f;
}

/* 作用：把闭环算出的占空比写到 TIM1 和 TIM8，并检查使能条件。
   调用位置：ACAC_Board_OnSample 在每次控制计算完成后。 */
static void WritePwm(void)
{
    const PWMCommand *p = &g_control.pwm;
    uint32_t irq = __get_PRIMASK();

    __disable_irq();
    if (!g_control.ready || g_control.fault || g_board_fault ||
        external_trip || ExternalFault() || !p->rect_enable)
    {
        DriverOff();
        __set_PRIMASK(irq);
        return;
    }

    /* TIM8 是整流桥，TIM1 是逆变桥。CCR = 占空比 * ARR。 */
    TIM8->CCR1 = (uint32_t)(p->rect[0] * ACAC_PWM_ARR);
    TIM8->CCR2 = (uint32_t)(p->rect[1] * ACAC_PWM_ARR);
    TIM1->CCR1 = (uint32_t)(p->inv[0] * ACAC_PWM_ARR);
    TIM1->CCR2 = (uint32_t)(p->inv[1] * ACAC_PWM_ARR);
    TIM1->CCR3 = (uint32_t)(p->inv[2] * ACAC_PWM_ARR);

    /* 每个桥先写入占空比，等一次定时器更新再开输出。 */
    if (rect_previous)
    {
        SET_BIT(TIM8->BDTR, TIM_BDTR_MOE);
        GPIOE->BSRR = (uint32_t)DRIVER_DIS_HIGH_Pin << 16;
    }

    if (p->inv_enable && inv_previous)
        SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    else
        CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);

    rect_previous = p->rect_enable;
    inv_previous = p->inv_enable;
    __set_PRIMASK(irq);
}

/* 作用：处理一帧采样，交给控制状态机，再检查耗时并更新 PWM。
   调用位置：adc_sample.c 的 adc_frame_ready。 */
void ACAC_Board_OnSample(const Measurement *sample, uint32_t cycle_start)
{
    last_frame_ms = HAL_GetTick();

    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    uint32_t command = pending_command;
    uint32_t trip = external_trip;
    pending_command = 0;
    external_trip = 0;
    __set_PRIMASK(irq);

    if (trip || ExternalFault())
    {
        acac_trip(&g_control, FAULT_EXTERNAL);
        command &= ~CMD_CLEAR;
    }

    acac_step(&g_control, sample, command);

    /* 180 MHz 下 3500 个周期约为 19.4 us。 */
    g_irq_cycles = DWT->CYCCNT - cycle_start;
    if (g_irq_cycles > 3500u)
    {
        ACAC_Board_SamplingFault(FAULT_TIMING);
        return;
    }

    WritePwm();
}

/* 作用：响应 PG2 硬件故障上升沿，立即关闭功率输出。
   调用位置：EXTI2_IRQHandler 调用 HAL_GPIO_EXTI_IRQHandler 后由 HAL 调用。 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == EXTERNAL_FAULT_HIGH_Pin)
    {
        external_trip = 1;
        DriverOff();
    }
}

/* 作用：检查时钟、启动 ADC DMA 和定时器，保持功率输出关闭。
   调用位置：main 的 USER CODE 2，在 CubeMX 外设初始化之后。 */
void ACAC_Board_Init(void)
{
    DriverOff();
    acac_init(&g_control, BoardReady());

    if (HAL_RCC_GetSysClockFreq() != 180000000u ||
        HAL_RCC_GetPCLK2Freq() != ACAC_TIM_HZ ||
        TIM1->ARR != ACAC_PWM_ARR || TIM8->ARR != ACAC_PWM_ARR)
        ACAC_Board_Fatal();

    SET_BIT(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk);
    DWT->CYCCNT = 0;
    SET_BIT(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);

    SET_BIT(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE |
             TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
    SET_BIT(TIM8->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE |
             TIM_CCER_CC2E | TIM_CCER_CC2NE);

    /* 先装载比较寄存器和重复计数器，再让 ADC 等待触发。 */
    TIM8->EGR = TIM_EGR_UG;
    TIM1->EGR = TIM_EGR_UG;
    TIM8->SR = 0;
    TIM1->SR = 0;

    if (!adc_sample_start())
        ACAC_Board_Fatal();

    last_frame_ms = HAL_GetTick();
    SET_BIT(TIM8->CR1, TIM_CR1_CEN);
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
}

/* 作用：轮询和消抖四个按键，并监视采样超时。
   调用位置：main 循环的 USER CODE 3。 */
void ACAC_Board_Poll(void)
{
    static uint32_t last_keys_ms;
    static uint8_t history[4] = {255, 255, 255, 255};
    static uint8_t pressed[4];

    uint32_t now = HAL_GetTick();
    uint32_t last = last_frame_ms;
    if (!g_board_fault && (int32_t)(now - last) > 5)
        ACAC_Board_SamplingFault(FAULT_ADC);

    if (now - last_keys_ms < 5u)
        return;
    last_keys_ms = now;

    /* PD0～PD3 低电平按下，连续 8 次相同才确认。 */
    const uint32_t commands[4] = {
        CMD_START, CMD_STOP,
        g_control.out_hz == 60u ? CMD_30HZ : CMD_60HZ,
        CMD_CLEAR
    };
    uint32_t command = 0;

    for (unsigned i = 0; i < 4; ++i)
    {
        history[i] = (uint8_t)((history[i] << 1) | ((GPIOD->IDR >> i) & 1u));
        if (history[i] == 0 && !pressed[i])
        {
            pressed[i] = 1;
            command |= commands[i];
        }
        if (history[i] == 255)
            pressed[i] = 0;
    }

    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    pending_command |= command;
    __set_PRIMASK(irq);
}
