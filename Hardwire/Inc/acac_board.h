#ifndef ACAC_BOARD_H
#define ACAC_BOARD_H

#include "acac.h"

/* main 初始化外设后调用，启动采样计时。 */
void ACAC_Board_Init(void);
/* main 的 while 循环调用，处理按键和采样超时。 */
void ACAC_Board_Poll(void);
/* Error_Handler 和 CPU 异常入口调用，关断驱动并停机。 */
void ACAC_Board_Fatal(void);
/* ADC 两路数据凑齐时调用，运行一次闭环。 */
void ACAC_Board_OnSample(const Measurement *sample, uint32_t cycle_start);
/* DMA 或板级检查报错时调用，锁存故障并关闭驱动。 */
void ACAC_Board_SamplingFault(uint32_t reason);

/* Keil 调试窗口里查看运行状态、控制用时、ADC/DMA 故障。 */
extern Controller g_control;
extern volatile uint32_t g_irq_cycles;
extern volatile uint32_t g_board_fault;

#endif
