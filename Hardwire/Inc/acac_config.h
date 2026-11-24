#ifndef ACAC_CONFIG_H
#define ACAC_CONFIG_H

// svpwm.c
#define DUTY_MAX		0.98f
#define DUTY_MIN		0.02f

/* 新增：闭环时序、采样标定、保护和控制参数。 */
/* 现有工程的 TIM 时钟为 90 MHz，中心对齐计数走 0 -> ARR -> 0。
   因此 PWM 和每次闭环采样的频率都是 90 MHz / (2 * 2500)。 */
#define ACAC_TIM_HZ       90000000u
#define ACAC_PWM_ARR      2500u
#define ACAC_FS_HZ        18000u
#define ACAC_DT           (1.0f / ACAC_FS_HZ)

/* 保留示例的上板锁定。核对对应项目后再将其设为 1。 */
#define ACAC_WIRING_VERIFIED    0
#define ACAC_CALIB_VERIFIED     0
#define ACAC_POWER_VERIFIED     0
#define ACAC_HARDTRIP_VERIFIED  0  /* 外部保护应能直接关断驱动 */

/* 量值 = (原始码差 - 零点码差) * 增益。
   增益单位是 V/码或 A/码，符号由传感器和接线方向决定。
   Udc 是单通道，其增益下面给的是旧示例的理论初值。 */
#define CAL_UIN_GAIN   0.0f
#define CAL_IIN_GAIN   0.0f
#define CAL_UAB_GAIN   0.0f
#define CAL_UBC_GAIN   0.0f
#define CAL_IO_GAIN    0.0f
#define CAL_UDC_GAIN   (30.0f * 3.0f / 65535.0f)

#define CAL_UIN_ZERO   0.0f
#define CAL_IIN_ZERO   0.0f
#define CAL_UAB_ZERO   0.0f
#define CAL_UBC_ZERO   0.0f
#define CAL_IO_ZERO    0.0f
#define CAL_UDC_ZERO   0.0f

/* 母线 65 V 和保护阈值沿用旧示例，电流保护是瞬时峰值。 */
#define ACAC_BUS_TARGET       65.0f
#define ACAC_BUS_OV           75.0f
#define ACAC_BUS_UV           50.0f  /* 后级维持 32 V 时给母线留一点余量 */
#define ACAC_IIN_PEAK         7.0f
#define ACAC_IIN_COMMAND_PEAK 6.0f  /* 母线 PI 输出上限，输入电流峰值 A */
#define ACAC_IO_PEAK          4.0f
#define ACAC_UIN_PEAK         65.0f
#define ACAC_ULINE_PEAK       55.0f

#define ACAC_LINE_TARGET_RMS 32.0f
#define ACAC_INPUT_HZ        50u

/* 前级电流 QPR 和母线电压 PI */
#define ACAC_RECT_QPR_KP  4.5f
#define ACAC_RECT_QPR_KR  35.0f
#define ACAC_RECT_QPR_WC  5.0f        /* rad/s */
#define ACAC_BUS_PI_KP    0.12f
#define ACAC_BUS_PI_KI    9.0f

/* 后级两路线电压 QPR，中心频率随 30/60 Hz 档位变化。 */
#define ACAC_INV_QPR_KP  0.005f
#define ACAC_INV_QPR_KR  10.0f
#define ACAC_INV_QPR_WC  18.84955592f  /* 2*pi*3 rad/s */
/* 65 V 母线下，每路修正的保守余量是
   ((DUTY_MAX - DUTY_MIN) * 65 - 32 * sqrt(2)) / 2，约 8.57 V。
   两路先各限到 ±8 V，SVPWM 再按实测母线限制三相总指令。 */
#define ACAC_INV_QPR_LIMIT 8.0f

#endif
