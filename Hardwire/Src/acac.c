#include "acac.h"
#include "acac_config.h"

#include <math.h>

#define PI_F   3.14159265358979323846f
#define SQRT_2 1.41421356237f

/* 作用：把占空比设成 50%，同时关闭前后级输出标志。
   调用位置：acac_init、acac_trip 和 acac_step 的停机分支。 */
static void neutral_pwm(Controller *ctrl)
{
    ctrl->pwm.rect[0] = 0.5f;
    ctrl->pwm.rect[1] = 0.5f;
    ctrl->pwm.inv[0] = 0.5f;
    ctrl->pwm.inv[1] = 0.5f;
    ctrl->pwm.inv[2] = 0.5f;
    ctrl->pwm.rect_enable = 0;
    ctrl->pwm.inv_enable = 0;
}

/* 作用：计算母线 100 Hz 陷波器的系数。
   调用位置：acac_init 初始化控制器时。 */
static void init_bus_notch(Biquad *filter)
{
    double w = 2.0 * PI_F * 100.0;
    double k = w / tan(w * ACAC_DT * 0.5);
    double sum = k * k + w * w;
    double damping = w * k / 12.5;
    double a0 = sum + damping;

    filter->b0 = (float)(sum / a0);
    filter->b1 = (float)(2.0 * (w * w - k * k) / a0);
    filter->b2 = filter->b0;
    filter->a1 = filter->b1;
    filter->a2 = (float)((sum - damping) / a0);
}

/* 作用：用本次母线采样更新陷波器，返回滤波后的电压。
   调用位置：rectifier_step 计算电压 PI 之前。 */
static float filter_bus(Biquad *filter, float input)
{
    double output = (double)filter->b0 * input + filter->s1;
    filter->s1 = (double)filter->b1 * input - (double)filter->a1 * output + filter->s2;
    filter->s2 = (double)filter->b2 * input - (double)filter->a2 * output;
    return (float)output;
}

/* 作用：清除某一路 QPR 的历史误差和输出。
   调用位置：reset_loops 和 wait_for_bus。 */
static void reset_qpr(QPR_STRUCT *q)
{
    q->Error = 0.0f;
    q->preError = 0.0f;
    q->prepreError = 0.0f;
    q->Out = 0.0f;
    q->preOut = 0.0;
    q->prepreOut = 0.0;
}

/* 作用：清除前后级控制器和软启动的历史状态。
   调用位置：idle_step 启动、acac_trip 故障、acac_step 停机或清故障时。 */
static void reset_loops(Controller *ctrl)
{
    reset_qpr(&ctrl->rect_qpr);
    reset_qpr(&ctrl->inv_ab);
    reset_qpr(&ctrl->inv_bc);
    ctrl->bus_notch.s1 = 0.0;
    ctrl->bus_notch.s2 = 0.0;
    ctrl->bus_pi.Integral = 0.0f;

    ctrl->phase = 0.0f;
    ctrl->line_ref = 0.0f;
    ctrl->ticks = 0;
    ctrl->stable = 0;
    ctrl->clip_ticks = 0;
    ctrl->lowload_ticks = 0;
}

/* 作用：按 30 Hz 或 60 Hz 重算后级 QPR 系数和有效值窗口。
   调用位置：acac_init 和 idle_step。 */
static void set_output_frequency(Controller *ctrl, uint32_t hz)
{
    /* 换频时，两路电压 QPR 和有效值窗口都跟着换。 */
    ctrl->out_hz = hz;
    float w0 = 2.0f * PI_F * (float)hz;
    QPR_Struct_Init(&ctrl->inv_ab, ACAC_INV_QPR_KP, ACAC_INV_QPR_KR,
                    ACAC_INV_QPR_WC, w0, ACAC_DT,
                    ACAC_INV_QPR_LIMIT, -ACAC_INV_QPR_LIMIT);
    QPR_Struct_Init(&ctrl->inv_bc, ACAC_INV_QPR_KP, ACAC_INV_QPR_KR,
                    ACAC_INV_QPR_WC, w0, ACAC_DT,
                    ACAC_INV_QPR_LIMIT, -ACAC_INV_QPR_LIMIT);
    QPR_Tustin(&ctrl->inv_ab);
    QPR_Tustin(&ctrl->inv_bc);

    rms_init(&ctrl->ab_rms, ACAC_FS_HZ / hz);
    rms_init(&ctrl->bc_rms, ACAC_FS_HZ / hz);
    rms_init(&ctrl->io_rms, ACAC_FS_HZ / hz);
    ctrl->phase = 0.0f;
}

/* 作用：初始化状态机、控制器和有效值统计。
   调用位置：ACAC_Board_Init，在启动 ADC 触发之前。 */
void acac_init(Controller *ctrl, uint8_t commissioned)
{
    *ctrl = (Controller){0};
    ctrl->ready = commissioned;
    ctrl->state = commissioned ? ST_IDLE : ST_LOCKED;

    QPR_Struct_Init(&ctrl->rect_qpr, ACAC_RECT_QPR_KP, ACAC_RECT_QPR_KR,
                    ACAC_RECT_QPR_WC, 2.0f * PI_F * ACAC_INPUT_HZ,
                    ACAC_DT, 0.0f, 0.0f);
    QPR_Tustin(&ctrl->rect_qpr);
    init_bus_notch(&ctrl->bus_notch);

    PI_Struct_Init(&ctrl->bus_pi, ACAC_BUS_PI_KP, ACAC_BUS_PI_KI, ACAC_DT,
                   ACAC_IIN_COMMAND_PEAK, 0.0f);

    rms_init(&ctrl->input_rms, ACAC_FS_HZ / ACAC_INPUT_HZ);
    set_output_frequency(ctrl, 60u);
    neutral_pwm(ctrl);
}

/* 作用：记录故障，把状态转入 ST_FAULT，清除控制量。
   调用位置：ACAC_Board_OnSample 和 acac_step 内的保护分支。 */
void acac_trip(Controller *ctrl, uint32_t reason)
{
    ctrl->fault |= reason;
    ctrl->state = ST_FAULT;
    neutral_pwm(ctrl);
    reset_loops(ctrl);
}

/* 作用：检查采样有效性、过压和过流，返回对应故障位。
   调用位置：acac_step 在执行闭环之前。 */
static uint32_t check_measurement(const Measurement *m)
{
    if (!m->valid || !isfinite(m->uin) || !isfinite(m->iin) ||
        !isfinite(m->udc) || !isfinite(m->uab) || !isfinite(m->ubc) ||
        !isfinite(m->io) || m->udc < 0.0f)
    {
        return FAULT_SENSOR;
    }

    if (m->udc > ACAC_BUS_OV || fabsf(m->uin) > ACAC_UIN_PEAK ||
        fabsf(m->uab) > ACAC_ULINE_PEAK || fabsf(m->ubc) > ACAC_ULINE_PEAK)
    {
        return FAULT_OVERVOLT;
    }

    if (fabsf(m->iin) > ACAC_IIN_PEAK || fabsf(m->io) > ACAC_IO_PEAK)
    {
        return FAULT_OVERCURRENT;
    }

    return FAULT_NONE;
}

/* 作用：在待机时处理频率切换和启动命令。
   调用位置：acac_step 的 ST_IDLE 分支。 */
static void idle_step(Controller *ctrl, const Measurement *m, uint32_t command)
{
    if (command & CMD_30HZ)
        set_output_frequency(ctrl, 30u);
    if (command & CMD_60HZ)
        set_output_frequency(ctrl, 60u);

    /* 输入要有有效值，母线要先通过硬件预充。 */
    if ((command & CMD_START) && ctrl->input_rms.valid &&
        ctrl->input_rms.value > 25.0f && ctrl->input_rms.value < 43.0f &&
        m->udc > 20.0f && m->udc < ACAC_BUS_TARGET + 3.0f)
    {
        reset_loops(ctrl);
        ctrl->bus_ref = m->udc;
        ctrl->state = ST_RECT_RAMP;
    }
}

/* 作用：运行母线 PI 和输入电流 QPR，计算 TIM8 两桥臂的占空比。
   调用位置：acac_step 在母线爬升、等待和逆变运行时。 */
static void rectifier_step(Controller *ctrl, const Measurement *m)
{
    ctrl->bus_ref = fminf(ACAC_BUS_TARGET, ctrl->bus_ref + 20.0f * ACAC_DT);
    if (ctrl->state == ST_RECT_RAMP && ctrl->bus_ref >= ACAC_BUS_TARGET)
        ctrl->state = ST_BUS_WAIT;

    ctrl->bus_pi.Set = ctrl->bus_ref;
    ctrl->bus_pi.Actrual = filter_bus(&ctrl->bus_notch, m->udc);
    PI_Cal(&ctrl->bus_pi);

    /* PI 输出是电流峰值，与归一化输入电压相乘得到瞬时电流指令。 */
    float input_peak = fmaxf(SQRT_2 * ctrl->input_rms.value, 1.0f);
    ctrl->rect_qpr.Set = ctrl->bus_pi.Out * m->uin / input_peak;
    ctrl->rect_qpr.Actural = m->iin;

    /* 桥电压 = Uin - QPR.Out，桥电压允许范围是 ±duty_span * Udc。
       由此反推 QPR 的上下限。Uin 是本帧带符号的瞬时电压。 */
    float duty_span = DUTY_MAX - DUTY_MIN;
    float voltage_max = duty_span * m->udc;
    ctrl->rect_qpr.OutUpperLimit = m->uin + voltage_max;
    ctrl->rect_qpr.OutLowerLimit = m->uin - voltage_max;
    QPR_Cal(&ctrl->rect_qpr);
    ctrl->rect_qpr_v = ctrl->rect_qpr.Out;

    float bridge_voltage = m->uin - ctrl->rect_qpr_v;
    float modulation = bridge_voltage / m->udc;
    modulation = fminf(duty_span, fmaxf(-duty_span, modulation));

    ctrl->pwm.rect[0] = 0.5f * (1.0f + modulation);
    ctrl->pwm.rect[1] = 0.5f * (1.0f - modulation);
    ctrl->pwm.rect_enable = 1;
}

/* 作用：确认母线稳定满 0.2 s 后，再启动逆变级。
   调用位置：acac_step 的 ST_BUS_WAIT 分支。 */
static void wait_for_bus(Controller *ctrl, float bus_voltage)
{
    /* 母线连续 0.2 s 在目标值 ±2 V 内，才启动后级。 */
    if (fabsf(bus_voltage - ACAC_BUS_TARGET) < 2.0f)
        ++ctrl->stable;
    else
        ctrl->stable = 0;

    if (ctrl->stable >= ACAC_FS_HZ / 5u)
    {
        ctrl->state = ST_INV_RAMP;
        ctrl->ticks = 0;
        ctrl->phase = 0.0f;
        reset_qpr(&ctrl->inv_ab);
        reset_qpr(&ctrl->inv_bc);
    }
}

/* 作用：运行两路线电压 QPR 和 SVPWM，计算 TIM1 的三相占空比。
   调用位置：acac_step 的 ST_INV_RAMP 和 ST_RUN 分支。 */
static void inverter_step(Controller *ctrl, const Measurement *m)
{
    ctrl->line_ref = fminf(ACAC_LINE_TARGET_RMS, ctrl->line_ref + 16.0f * ACAC_DT);
    if (ctrl->line_ref >= ACAC_LINE_TARGET_RMS)
        ctrl->state = ST_RUN;

    /* 32 V 是有效值。乘根号 2 得到正弦指令的峰值。 */
    float line_peak = SQRT_2 * ctrl->line_ref;
    float ab_ref = line_peak * sinf(ctrl->phase);
    float bc_ref = line_peak * sinf(ctrl->phase - 2.0f * PI_F / 3.0f);

    ctrl->inv_ab.Set = ab_ref;
    ctrl->inv_ab.Actural = m->uab;
    QPR_Cal(&ctrl->inv_ab);

    ctrl->inv_bc.Set = bc_ref;
    ctrl->inv_bc.Actural = m->ubc;
    QPR_Cal(&ctrl->inv_bc);

    ctrl->inv_ab_cmd = ab_ref + ctrl->inv_ab.Out;
    ctrl->inv_bc_cmd = bc_ref + ctrl->inv_bc.Out;

    /* 从 Va - Vb、Vb - Vc 和 Va + Vb + Vc = 0 还原三相。 */
    SVPWM_Struct pwm = {0};
    pwm.Va = (2.0f * ctrl->inv_ab_cmd + ctrl->inv_bc_cmd) / 3.0f;
    pwm.Vb = (ctrl->inv_bc_cmd - ctrl->inv_ab_cmd) / 3.0f;
    pwm.Vc = -pwm.Va - pwm.Vb;
    pwm.Vdc = m->udc;

    if (!SVPWM_Cal(&pwm))
    {
        acac_trip(ctrl, FAULT_SENSOR);
        return;
    }

    ctrl->pwm.inv[0] = pwm.DutyA;
    ctrl->pwm.inv[1] = pwm.DutyB;
    ctrl->pwm.inv[2] = pwm.DutyC;
    ctrl->mod_scale = pwm.scale;
    ctrl->pwm.inv_enable = 1;

    if (ctrl->line_ref > 10.0f && ctrl->io_rms.valid && ctrl->io_rms.value < 0.05f)
        ++ctrl->lowload_ticks;
    else
        ctrl->lowload_ticks = 0;

    if (ctrl->lowload_ticks > ACAC_FS_HZ / 10u)
    {
        acac_trip(ctrl, FAULT_LOAD);
        return;
    }

    ctrl->phase += 2.0f * PI_F * (float)ctrl->out_hz * ACAC_DT;
    if (ctrl->phase >= 2.0f * PI_F)
        ctrl->phase -= 2.0f * PI_F;
}

/* 作用：检查前级 QPR 异常、后级持续过调制和非有限输出。
   调用位置：acac_step 算完前后级以后。 */
static void check_modulation(Controller *ctrl, float bus_voltage)
{
    if (fabs(ctrl->rect_qpr.preOut) > bus_voltage * 1.5 ||
        (ctrl->pwm.inv_enable && ctrl->mod_scale < 0.999f))
        ++ctrl->clip_ticks;
    else
        ctrl->clip_ticks = 0;

    if (!isfinite(ctrl->rect_qpr_v) ||
        !isfinite(ctrl->inv_ab_cmd) || !isfinite(ctrl->inv_bc_cmd))
        acac_trip(ctrl, FAULT_SENSOR);
    else if (ctrl->clip_ticks > ACAC_FS_HZ / 20u)
        acac_trip(ctrl, FAULT_MODULATION);
}

/* 作用：每次按最新采样和按键命令推进状态机，运行并保护闭环。
   调用位置：ACAC_Board_OnSample，每凑齐一帧 ADC1 和 ADC3 时。 */
void acac_step(Controller *ctrl, const Measurement *m, uint32_t command)
{
    ctrl->m = *m;
    ++ctrl->samples;

    if (command & CMD_STOP)
    {
        neutral_pwm(ctrl);
        reset_loops(ctrl);
        if (ctrl->state != ST_FAULT)
            ctrl->state = ctrl->ready ? ST_IDLE : ST_LOCKED;
        return;
    }

    if (ctrl->state == ST_LOCKED)
        return;

    uint32_t fault = check_measurement(m);
    if (fault != FAULT_NONE)
    {
        acac_trip(ctrl, fault);
        return;
    }

    rms_step(&ctrl->input_rms, m->uin);
    rms_step(&ctrl->ab_rms, m->uab);
    rms_step(&ctrl->bc_rms, m->ubc);
    rms_step(&ctrl->io_rms, m->io);

    if (ctrl->state == ST_FAULT)
    {
        if ((command & CMD_CLEAR) && m->udc < 10.0f &&
            fabsf(m->iin) < 0.1f && fabsf(m->io) < 0.1f)
        {
            ctrl->fault = FAULT_NONE;
            ctrl->state = ctrl->ready ? ST_IDLE : ST_LOCKED;
            reset_loops(ctrl);
        }
        return;
    }

    if (ctrl->state == ST_IDLE)
    {
        idle_step(ctrl, m, command);
        return;
    }

    ++ctrl->ticks;
    int inverter_running = ctrl->state == ST_INV_RAMP || ctrl->state == ST_RUN;
    if (m->udc < 15.0f || (inverter_running && m->udc < ACAC_BUS_UV))
    {
        acac_trip(ctrl, FAULT_UNDERVOLT);
        return;
    }

    if (!inverter_running && ctrl->ticks > 5u * ACAC_FS_HZ)
    {
        acac_trip(ctrl, FAULT_START_TIMEOUT);
        return;
    }

    rectifier_step(ctrl, m);
    if (ctrl->state == ST_BUS_WAIT)
        wait_for_bus(ctrl, m->udc);
    if (ctrl->state == ST_INV_RAMP || ctrl->state == ST_RUN)
        inverter_step(ctrl, m);

    if (ctrl->state != ST_FAULT)
        check_modulation(ctrl, m->udc);
}
