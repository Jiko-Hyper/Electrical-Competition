#ifndef ACAC_H
#define ACAC_H

#include <stdint.h>

#include "pi.h"
#include "qpr.h"
#include "rms.h"
#include "svpwm.h"

/* 母线 100 Hz 陷波器的系数与历史状态。 */
typedef struct
{
    float b0, b1, b2, a1, a2;
    double s1, s2;
} Biquad;

typedef enum
{
    ST_LOCKED,     /* 硬件或采样还未确认，只允许读 ADC */
    ST_IDLE,       /* 待机，等待按键 */
    ST_RECT_RAMP,  /* 整流桥工作，缓慢升母线电压 */
    ST_BUS_WAIT,   /* 等母线电压稳定 */
    ST_INV_RAMP,   /* 逆变桥工作，缓慢升交流输出 */
    ST_RUN,
    ST_FAULT
} State;

/* 用位表示故障，多个故障可以同时记录。 */
enum
{
    FAULT_NONE          = 0,
    FAULT_SENSOR        = 1u << 0,
    FAULT_OVERVOLT      = 1u << 1,
    FAULT_OVERCURRENT   = 1u << 2,
    FAULT_UNDERVOLT     = 1u << 3,
    FAULT_START_TIMEOUT = 1u << 4,
    FAULT_EXTERNAL      = 1u << 5,
    FAULT_ADC           = 1u << 6,
    FAULT_TIMING        = 1u << 7,
    FAULT_MODULATION    = 1u << 8,
    FAULT_LOAD          = 1u << 9
};

enum
{
    CMD_START = 1u << 0,
    CMD_STOP  = 1u << 1,
    CMD_CLEAR = 1u << 2,
    CMD_30HZ  = 1u << 3,
    CMD_60HZ  = 1u << 4
};

typedef struct
{
    float uin;  /* 输入电压瞬时值 */
    float iin;
    float udc;
    float uab;  /* 后级 A、B 两相之间的瞬时线电压 */
    float ubc;
    float io;
    uint8_t valid;
} Measurement;

typedef struct
{
    float rect[2];  /* TIM8 CH1、CH2 */
    float inv[3];   /* TIM1 CH1、CH2、CH3 */
    uint8_t rect_enable;
    uint8_t inv_enable;
} PWMCommand;

typedef struct
{
    State state;
    uint32_t fault;
    uint32_t ticks;
    uint32_t stable;
    uint32_t clip_ticks;
    uint32_t lowload_ticks;
    uint32_t samples;
    uint32_t out_hz;

    float bus_ref;
    float line_ref;  /* 线电压有效值指令，逐步升至 32 V */
    float phase;
    float rect_qpr_v;
    float inv_ab_cmd;
    float inv_bc_cmd;
    float mod_scale;

    PI_Struct bus_pi;
    QPR_STRUCT rect_qpr;
    QPR_STRUCT inv_ab;
    QPR_STRUCT inv_bc;
    Biquad bus_notch;
    RMS input_rms;
    RMS ab_rms;
    RMS bc_rms;
    RMS io_rms;

    PWMCommand pwm;
    Measurement m;
    uint8_t ready;
} Controller;

/* ACAC_Board_Init 调用，初始化整个控制器。 */
void acac_init(Controller *ctrl, uint8_t commissioned);
/* 采样处理和闭环保护调用，记录故障并撤销 PWM。 */
void acac_trip(Controller *ctrl, uint32_t reason);
/* ACAC_Board_OnSample 每帧调用，执行状态机与闭环。 */
void acac_step(Controller *ctrl, const Measurement *m, uint32_t command);

#endif
