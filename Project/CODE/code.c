#include "code.h"
#include "zf_function.h"

/*===================================================================================================================
 *  全局变量定义
 *==================================================================================================================*/
adc_channel_t ch_lx = { ADC_LEFTX_CHANNEL,  LX_MAX, LX_MIN };
adc_channel_t ch_rx = { ADC_RIGHTX_CHANNEL, RX_MAX, RX_MIN };
adc_channel_t ch_ly = { ADC_LEFTY_CHANNEL,  LY_MAX, LY_MIN };
adc_channel_t ch_ry = { ADC_RIGHTY_CHANNEL, RY_MAX, RY_MIN };

PID servo_pid;                  /* 方向环（位置式 PID） */
PID left_pid, right_pid;        /* 速度环（增量式 PI） */

int16 target_speed = TARGET_SPEED_DEFAULT;
int16 target_duty  = TARGET_DUTY_DEFAULT;

uint16 lh, rh, lv, rv;          /* 归一化结果 */
int16  error;                   /* 偏差 */
int16  speed_l, speed_r;        /* 左右速度 */

uint16 raw_lx, raw_rx, raw_ly, raw_ry;  /* 滤波后的原始值（屏幕显示用） */

volatile elem_state_enum elem_state = ELEM_NONE;
uint16 elem_timer = 0;          /* 当前状态的计时（单位 10ms） */
uint8  elem_cnt   = 0;          /* 连续满足条件的周期数 */
uint8  ring_done = 0;           /* 圆环是否已经跑完（只触发一次） */
uint16 run_tick   = 0;                /* 上电后经过的 10ms 数 */

/*===================================================================================================================
 *  L4 4.4.3  去极值平均滤波
 *  采 ADC_FILTER_NUM 次，去掉最大最小各一个，剩下的求平均
 *==================================================================================================================*/
uint16 adc_filter(ADCN_enum ch)
{
    uint16 i, v, vmax = 0, vmin = 0xFFFF;
    uint32 sum = 0;

    for(i = 0; i < ADC_FILTER_NUM; i++)
    {
        v = adc_once(ch, ADC_12BIT);        /* 12 位：0~4095 */
        sum += v;
        if(v > vmax) vmax = v;
        if(v < vmin) vmin = v;
    }
    return (uint16)((sum - vmax - vmin) / (ADC_FILTER_NUM - 2));
}

/*===================================================================================================================
 *  L4 4.5.2  归一化：把 [min, max] 线性映射到 [0, 100]
 *==================================================================================================================*/
uint16 normalize(adc_channel_t *c)
{
    uint16 raw = adc_filter(c->ch);

    /* 顺便把原始值存下来，给屏幕显示用（不要在中断和主循环里同时采 ADC） */
    if(c->ch == ADC_LEFTX_CHANNEL)       raw_lx = raw;
    else if(c->ch == ADC_RIGHTX_CHANNEL) raw_rx = raw;
    else if(c->ch == ADC_LEFTY_CHANNEL)  raw_ly = raw;
    else if(c->ch == ADC_RIGHTY_CHANNEL) raw_ry = raw;

    if(raw <= c->min) return 0;
    if(raw >= c->max) return 100;
    return (uint16)(((uint32)(raw - c->min) * 100) / (c->max - c->min));
}

/*===================================================================================================================
 *  L4 4.6.2  差比和：偏差 = (左 - 右) / (左 + 右) * 100，范围 -100~+100
 *==================================================================================================================*/
int16 calc_error(uint16 left, uint16 right)
{
    int32 diff = (int32)left - (int32)right;
    int32 sum  = (int32)left + (int32)right;

    return (int16)((diff * 100) / (sum + 1));       /* +1 防止除零 */
}

/*===================================================================================================================
 *  L6 6.4.2  元素状态机
 *  ---------------------------------------------------------------------------------------------------------------
 *  只有 ELEM_ENABLE == 1 时才真正切换状态；否则一直保持 ELEM_NONE（普通路段）
 *  规则（L6 6.4.3）：
 *      ① 连续满足 N 个周期才切换状态，避免磁场噪声误判
 *      ② 每个状态都有退出条件 + 超时保护
 *      ③ 元素期间只改"修正量"，不另写一套控制
 *==================================================================================================================*/
void elem_check(uint16 lh_now, uint16 rh_now, uint16 lv_now, uint16 rv_now)
{
    uint16 sum_h;                      /* 横向电感和：LX + RX */
    uint16 sum_v;                      /* 竖向电感和：LY + RY */
    uint8  h_flag;                     /* 横向达标 */
    uint8  v_flag;                     /* 竖向达标 */
    uint8  both_flag;                  /* 横向达标 且 竖向小于阈值 = 检测到 */

    if(run_tick < 60000)  run_tick++;

    sum_h = (uint16)(lh_now + rh_now);
    sum_v = (uint16)(lv_now + rv_now);

    h_flag = (sum_h > RING_H_TH) ? 1 : 0;
    v_flag = (sum_v < RING_V_MAX) ? 1 : 0;    /* 竖向要"干净"（小于 2） */

    both_flag = (uint8)(h_flag & v_flag);      /* 必须同时达标才算数 */

#if (ELEM_ENABLE == 1)
    switch(elem_state)
    {
        /*------------------ ① 进环：强制左打，直到横竖不再同时达标 ------------------*/
        case ELEM_ROUND_IN:
            elem_timer++;

            /* 兜底：打角最多持续 RING_IN_MAX */
            if(elem_timer > RING_IN_MAX)
            {
                elem_state = ELEM_ROUND_ON;
                elem_timer = 0;
                elem_cnt   = 0;
                break;
            }

            /* 至少打满 RING_IN_MIN，之后 both 消失才退出 */
            if((elem_timer > RING_IN_MIN) && (0 == both_flag))
            {
                elem_state = ELEM_ROUND_ON;
                elem_timer = 0;
                elem_cnt   = 0;
            }
            break;

        /*------------------ ② 环内：差比和，等【第二次】检测到阈值（出环口） ------------------*/
        case ELEM_ROUND_ON:
            elem_timer++;
            if((elem_timer > RING_ON_MIN) && both_flag)
            {
                if(++elem_cnt >= RING_COUNT)
                {
                    elem_cnt   = 0;
                    elem_state = ELEM_ROUND_OUT;
                    elem_timer = 0;
                }
            }
            else
            {
                elem_cnt = 0;
            }
            break;

        /*------------------ ③ 出环：强制右打一小段 ------------------*/
        case ELEM_ROUND_OUT:
            elem_timer++;
            if(elem_timer > RING_OUT_TIME)
            {
                ring_done  = 1;                /* 之后再也不响应阈值 */
                elem_state = ELEM_NONE;
                elem_timer = 0;
            }
            break;

        /*------------------ 直道：确认 → 延时 → 进环 ------------------*/
        default:
            if(0 == ring_done)
            {
                if(both_flag)
                {
                    if(elem_cnt < RING_COUNT)          /* 阶段①：连续确认 */
                    {
                        elem_cnt++;
                    }
                    else                               /* 阶段②：延时 RING_DELAY */
                    {
                        elem_timer++;
                        if(elem_timer >= RING_DELAY)
                        {
                            elem_cnt   = 0;
                            elem_timer = 0;
                            elem_state = ELEM_ROUND_IN;
                        }
                    }
                }
                else                                   /* 阈值消失 → 重新确认 */
                {
                    elem_cnt   = 0;
                    elem_timer = 0;
                }
            }
            break;
    }
#else
    elem_state = ELEM_NONE;
    elem_timer = 0;
#endif
}

/*===================================================================================================================
 *  L6 6.4.2  元素期间只改偏差
 *      十字：偏差清零，直着冲过去
 *      环内：额外往一侧偏，贴着环走
 *      出环：往反方向补一点
 *==================================================================================================================*/
int16 elem_correct(int16 error_in)
{
    int16 out;

    out = error_in;

#if (ELEM_ENABLE == 1)
    /* 进环阶段：检测到四路和超过阈值 → 强制打角（往左） */
    if(ELEM_ROUND_IN == elem_state)
    {
        out = error_in + RING_BIAS;
    }
    /* 出环阶段：强制往反方向打角，把车推出环 */
    else if(ELEM_ROUND_OUT == elem_state)
    {
        out = error_in + RING_OUT_BIAS;
    }
#endif

    out = (int16)func_limit_ab(out, -100, 100);

    return out;
}

/*===================================================================================================================
 *  速度决策：直道满占空比，检测到弯道立刻减速（解决急弯反应不过来）
 *  ---------------------------------------------------------------------------------------------------------------
 *  判定分两路，
 *      |error|  > SPEED_E_ON   → 已经进弯
 *      |Δerror| > SPEED_DE_ON  → 偏差正在快速变大 = 急弯入口（比 |error| 更早发现）
 *  减速后要等 |error| < SPEED_E_OFF 且连续 SPEED_HOLD 个周期，才恢复满速（回差防抖）
 *==================================================================================================================*/
int16 speed_now      = 0;           /* 当前下发的占空比 */
int16 error_last     = 0;           /* 上一周期偏差（算 Δerror 用） */
int16 target_l       = 0;           /* 左轮速度环目标（基础速度 + 差速） */
int16 target_r       = 0;           /* 右轮速度环目标 */
int16 duty_diff      = 0;           /* 开环差速量（占空比，正 = 左轮慢） */
uint8 speed_slowed   = 0;           /* 0 = 满速  1 = 已减速 */
uint8 speed_slow_cnt = 0;           /* 出弯计数 */
uint8 pid_state_prev = 0;           /* 上一周期用的是哪套 PID（0 直道 / 1 弯道） */
uint8 speed_back_cnt = 0;           /* 回正退出计数（出弯提前切 KP） */

int16 speed_decide(int16 e)
{
    int16 de_s;
    int16 de;
    int16 ae;
    int16 diff;
    int16 diff2;                       /* 速度环用的差速（脉冲单位） */
    int16 base_spd;                    /* 当前基础速度（直道/弯道） */

    de_s = (int16)(e - error_last);
    error_last = e;

    ae = (e    >= 0) ? e    : -e;
    de = (de_s >= 0) ? de_s : -de_s;

    if(0 == speed_slowed)
    {
        if((ae > SPEED_E_ON) || (de > SPEED_DE_ON))
        {
            speed_slowed   = 1;
            speed_slow_cnt = 0;
            speed_back_cnt = 0;
        }
    }
    else
    {
        if(ae < SPEED_E_OFF)  speed_slow_cnt++;
        else                  speed_slow_cnt = 0;

        if(speed_slow_cnt >= SPEED_HOLD)
        {
            speed_slowed   = 0;
            speed_slow_cnt = 0;
            speed_back_cnt = 0;
        }
        /* 只要"正在回正"就立刻切回直道 PID（不再要求 |E| 很小） */
        else if(((e > 0) && (de_s < -(int16)SPEED_DE_BACK)) ||
                ((e < 0) && (de_s >  (int16)SPEED_DE_BACK)))
        {
            if(++speed_back_cnt >= SPEED_BACK_CNT)
            {
                speed_slowed   = 0;
                speed_slow_cnt = 0;
                speed_back_cnt = 0;
            }
        }
        else
        {
            speed_back_cnt = 0;
        }
    }

    /* ===== 两轮差速（不依赖弯道状态，直接按偏差给） =====
       E > 0 → 左打 → 内侧左轮慢、外侧右轮快
       E < 0 → 右打 → 内侧右轮慢、外侧左轮快
       E ≈ 0（直道）→ 差速自动为 0 ============================ */
    diff  = (int16)(DIFF_DUTY_K * (float)e);            /* 开环用：占空比单位 */
    if(diff >  DIFF_DUTY_LIMIT)  diff =  DIFF_DUTY_LIMIT;
    if(diff < -DIFF_DUTY_LIMIT)  diff = -DIFF_DUTY_LIMIT;
    duty_diff = diff;

    /* ---- 弯道减速：直道用 TARGET_SPEED_BASE，弯道用 TARGET_SPEED_CURVE ---- */
    if(1 == speed_slowed)  base_spd = TARGET_SPEED_CURVE;
    else                   base_spd = TARGET_SPEED_BASE;

    diff2 = (int16)(DIFF_K * (float)e);                 /* 速度环用：脉冲单位 */
    if(diff2 >  DIFF_LIMIT)  diff2 =  DIFF_LIMIT;
    if(diff2 < -DIFF_LIMIT)  diff2 = -DIFF_LIMIT;
    target_l = (int16)(base_spd + diff2);              /* 内侧：数值变大 = 慢 */
    target_r = (int16)(base_spd - diff2);              /* 外侧：数值变小 = 快 */

    /* ===== 圆环期间强制使用弯道 PID（环本身就是弯道，不受 |error| 阈值影响） ===== */
#if (ELEM_ENABLE == 1)
    if((ELEM_ROUND_IN  == elem_state) ||
       (ELEM_ROUND_ON  == elem_state) ||
       (ELEM_ROUND_OUT == elem_state))
    {
        speed_slowed = 1;
    }
#endif

    /* ===== 弯道 PID 已停用（暂时）：一律使用直道 PID =====
       speed_slowed 仍然计算，但只用于"过弯减速"和"差速"，不再切换 PID
    #if 0
    if(speed_slowed != pid_state_prev)
    {
        servo_pid.integral = 0;
        pid_state_prev     = speed_slowed;
    }

    if(1 == speed_slowed)
    {
        servo_pid.kp = KP_CURVE;
        servo_pid.ki = KI_CURVE;
        servo_pid.kd = KD_CURVE;
    }
    else
    {
        servo_pid.kp = SERVO_KP;
        servo_pid.ki = SERVO_KI;
        servo_pid.kd = SERVO_KD;
    }
    #endif
    ===== 弯道 PID 停用结束 ===== */

    servo_pid.kp = SERVO_KP;
    servo_pid.ki = SERVO_KI;
    servo_pid.kd = SERVO_KD;
    if(1 == speed_slowed)  return SPEED_CURVE_DUTY;    /* 弯道减速 */
    return SPEED_STRAIGHT;             /* 直道满占空比 */
}
