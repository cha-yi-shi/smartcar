#include "code.h"
#include "motor.h"

/*===================================================================================================================
 *  全局变量
 *==================================================================================================================*/
adc_proc_t ch_lx, ch_rx, ch_ly, ch_ry;

PID servo_pid;
PID left_pid, right_pid;

uint16 run_tick = 0;

uint16 lh = 0, rh = 0, lv = 0, rv = 0;
uint16 raw_lx = 0, raw_rx = 0, raw_ly = 0, raw_ry = 0;
uint16 sv_now = 0, ss_now = 0;
int16  error = 0;
int16  error_last = 0;
int16  de_now = 0;
float  curv = 0.0f;
float  k_pred = 1.0f;
int16  err_in = 0;
int16  servo_out = 0;
int16  servo_duty = 0;
int16  speed_l = 0, speed_r = 0;
float  enc_f_l = 0.0f, enc_f_r = 0.0f;
int16  enc_acc_l = 0, enc_acc_r = 0;
uint8  spd_div = 0;
int16  motor_l = 0, motor_r = 0;
uint8  curve_flag = 0;

/*===================================================================================================================
 *  取 n 个样本去极值平均
 *  n = 5：去掉最大最小，剩 3 个平均
 *  n = 3：去掉最大最小，剩 1 个（中值）
 *==================================================================================================================*/
static uint16 win_average(uint16 *buf, uint8 n)
{
    uint16 i;
    uint16 vmax, vmin;
    uint32 sum;

    if(0 == n) return 0;
    if(1 == n) return buf[0];

    vmax = buf[0];
    vmin = buf[0];
    sum  = 0;

    for(i = 0; i < n; i++)
    {
        sum += buf[i];
        if(buf[i] > vmax) vmax = buf[i];
        if(buf[i] < vmin) vmin = buf[i];
    }

    /* 去掉最大最小 */
    sum = sum - vmax - vmin;

    /* 剩 n-2 个 */
    return (uint16)(sum / (uint16)(n - 2));
}

/*===================================================================================================================
 *  ADC 通道处理（每 5ms 调一次）
 *    ① 采一次 12 位 ADC
 *    ② 存入环形缓冲
 *    ③ 按当前窗口（5 或 3）做去极值平均
 *    ④ 一阶低通：y += alpha * (x - y)，alpha 随 curv 变化
 *    ⑤ 归一化到 0~100
 *==================================================================================================================*/
uint16 adc_proc(adc_proc_t *c)
{
    uint16 raw;
    uint16 m;
    uint8  win;
    float  alpha;
    float  y;

    /* ① 采一次 */
    raw = adc_once(c->ch, ADC_12BIT);

    /* ② 环形缓冲 */
    c->buf[c->idx] = raw;
    c->idx = (uint8)((c->idx + 1) % WIN_STRAIGHT);
    if(c->cnt < WIN_STRAIGHT) c->cnt++;

    /* ③ 窗口大小：由上一拍的 curv 决定（滞回） */
    if((curv > WIN_UP) || ((curve_flag) && (curv > WIN_DOWN)))
    {
        win = WIN_CURVE;
        curve_flag = 1;
    }
    else
    {
        win = WIN_STRAIGHT;
        curve_flag = 0;
    }
    if(win > c->cnt) win = c->cnt;
    if(win < 1)      win = 1;

    /* 把最近 win 个样本拷出来（从 idx 往前数）*/
    {
        uint16 tmp[WIN_STRAIGHT];
        uint8  k;
        uint8  pos;

        for(k = 0; k < win; k++)
        {
            pos = (uint8)((c->idx + WIN_STRAIGHT - 1 - k) % WIN_STRAIGHT);
            tmp[k] = c->buf[pos];
        }
        m = win_average(tmp, win);
    }

    /* ④ 一阶低通 */
    alpha = ALPHA_STRAIGHT + curv * (ALPHA_CURVE - ALPHA_STRAIGHT);
    y = c->lpf;
    y = y + alpha * ((float)m - y);
    c->lpf = y;
    c->val = (uint16)y;

    /* ⑤ 归一化 0~100 */
    if(c->val <= c->min) return 0;
    if(c->val >= c->max) return 100;
    return (uint16)(((uint32)(c->val - c->min) * 100) / (c->max - c->min));
}

/*===================================================================================================================
 *  差比和：偏差 = (左 - 右) / (左 + 右) * 100，范围 -100~+100
 *==================================================================================================================*/
int16 calc_error(uint16 left, uint16 right)
{
    int32 diff;
    int32 sum;

    diff = (int32)left - (int32)right;
    sum  = (int32)left + (int32)right;

    return (int16)((diff * 100) / (sum + 1));       /* +1 防除零 */
}

/*===================================================================================================================
 *  曲率系数 + 预判增益
 *    fe = |error| / E_FULL      （车已经偏了多少）
 *    fd = |Δerror| / DE_FULL    （偏得多快 = 曲率）
 *    curv = W_E*fe + W_D*fd
 *    十字（Sv ≥ CROSS_V_TH）→ curv = 0
 *    K = 1 + (K_MAX-1)*curv
 *==================================================================================================================*/
void curv_update(void)
{
    float fe;
    float fd;
    int16 dea;

    /* Δerror（并加死区）*/
    de_now   = (int16)(error - error_last);
    error_last = error;

    dea = (de_now >= 0) ? de_now : -de_now;
    if(dea < DE_DEAD) dea = 0;

    /* fe：当前偏差 */
    fe = ((float)((error >= 0) ? error : -error)) / E_FULL;
    if(fe > 1.0f) fe = 1.0f;

    /* fd：偏差变化 */
    fd = ((float)dea) / DE_FULL;
    if(fd > 1.0f) fd = 1.0f;

    /* 叠加 */
    curv = W_E * fe + W_D * fd;
    if(curv > 1.0f) curv = 1.0f;

    /* 十字保护：竖向和特别大 → 曲率归零，绝不打角 */
    if(sv_now >= CROSS_V_TH) curv = 0.0f;

    /* 预判增益 */
    k_pred = 1.0f + (K_MAX - 1.0f) * curv;
}

/*===================================================================================================================
 *  圆环开环强制打角：到点就固定 duty，覆盖方向环
 *==================================================================================================================*/
void ring_force(void)
{
#if (RING_FORCE_ENABLE == 1)
    if((run_tick >= FORCE_START_TICK) &&
       (run_tick <  (FORCE_START_TICK + FORCE_TIME_TICK)))
    {
        Set_Servo_PWM((int32)FORCE_DUTY);
        servo_duty = FORCE_DUTY;
    }
#endif
}
