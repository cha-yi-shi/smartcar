#ifndef _CODE_H_
#define _CODE_H_

#include "headfile.h"
#include "pid.h"

/*===================================================================================================================
 *  电感 + 元素状态机（L4 / L6）
 *  ==================================================================================================================
 *  电感通道照 CarDemo1：ADC_P00 / ADC_P01 / ADC_P02 / ADC_P03（P0.0 ~ P0.3）
 *  下面的 "??" 都是要你自己实测/自己调的参数
 *==================================================================================================================*/

/*----------------------------- 电感通道 -----------------------------*/
/*----------------------------- 电感通道（按实际接线修正 2026-11-03） -----------------------------*/
/*  adc1 = P0.0 = LX 左横
    adc2 = P0.1 = LY 左竖
    adc3 = P0.2 = RY 右竖
    adc4 = P0.3 = RX 右横   */
#define ADC_LEFTX_CHANNEL       ADC_P00     /* LX 左横 */
#define ADC_RIGHTX_CHANNEL      ADC_P03     /* RX 右横 */
#define ADC_LEFTY_CHANNEL       ADC_P01     /* LY 左竖 */
#define ADC_RIGHTY_CHANNEL      ADC_P02     /* RY 右竖 */

/*----------------------------- 标定值（按通道对应，实测值） -----------------------------*/
#define LX_MAX      3300        /* P0.0 实测 */
#define LX_MIN      9         /* P0.0 实测 */
#define RX_MAX      3000        /* P0.3 实测 */
#define RX_MIN      0
#define LY_MAX      3600        /* P0.1 实测 */
#define LY_MIN      0
#define RY_MAX      3580        /* P0.2 实测 */
#define RY_MIN      0

/*----------------------------- 滤波（L4 4.4.3） -----------------------------*/
#define ADC_FILTER_NUM      10   /* 去极值平均：采 5 次去掉最大最小，剩 3 次平均 */

/*----------------------------- 手动调参区 -----------------------------*/
#define TARGET_DUTY_DEFAULT     -8000       /* ?? 开环占空比 0~10000（想跑就改成 3000 左右） */
#define TARGET_SPEED_DEFAULT    0       /* ?? 闭环目标速度（脉冲/10ms） */

#define SPEED_LOOP_ENABLE       0       /* 1 = 速度环闭环（已加"禁止反转"） */
#define ELEM_ENABLE             1       /* 0 = 元素功能先关掉（调试阶段）  1 = 打开十字/环岛（L6 6.4） */

/*----------------------------- 方向环 PID（L4 4.8） -----------------------------*/
/* 调舵机中位时：把 SERVO_KP 改成 0，舵机就会固定停在 SERVO_MID */
#define SERVO_KP        0.8f
#define SERVO_KI        0.0f
#define SERVO_KD        2.0f
#define SERVO_MAX_I     100.0f
#define SERVO_MAX_OUT   400.0f

/* 弯道专用 PID（直道用上面的 SERVO_KP/KI/KD） */
#define KP_CURVE        4.5f    /* ?? 弯道 Kp（直道 2.5 的 1.4 倍，弯道打角更狠） */
#define KI_CURVE        0.0f
#define KD_CURVE        4.0f

/*----------------------------- 入弯减速（竞速：直道满占空比） -----------------------------*/
#define SPEED_STRAIGHT -10000   /* ?? 直道：满占空比（保持竞速速度） */
#define SPEED_CURVE_DUTY -6000    /* ?? 开环时弯道占空比（?10000 → ?7000，降 30%） */

/*----------------------------- 两轮差速（给在速度环 PID 的目标值上） -----------------------------*/
#define TARGET_SPEED_BASE  -15     /* ?? 基础目标速度：满速 15 的 80% */
#define TARGET_SPEED_CURVE  -11     /* ?? 弯道基础目标速度（直道 ?15 → 弯道 ?11，降 27%） */
/* 开环差速：直接改电机占空比（不用速度环、不用编码器） */
#define SPEED_NO_REVERSE    1       /* ?? 1 = 禁止电机反转：超过目标速度就断油到 0，不准反向 */
#define DEAD_SPEED          1       /* ?? 速度死区（脉冲/10ms）：±1 内不调节 */

#define DIFF_DUTY_K      20      /* ?? 开环差速系数：error=40 → 左右占空比差 ±800（8%） */
#define DIFF_DUTY_LIMIT  2500    /* ?? 最大差速（占空比）≈ 25% */

/* 速度环用的差速（脉冲单位，加在速度环目标值上） */
#define DIFF_K              0.3f   /* ?? 差速系数：error 33 → diff 1；error 66 → diff 2 */
#define DIFF_LIMIT          3.0       /* ?? 差速上限（脉冲/10ms）≈ 25% */

#define SPEED_E_ON         50   /* ?? |error| > 18 → 判定进弯，减速 */
#define SPEED_DE_ON        10   /* ?? |Δerror| > 10 → 急弯特征，更早减速 */
#define SPEED_E_OFF        40   /* ?? |error| < 10 → 认为出弯（回差，防抖） */
#define SPEED_HOLD         20   /* ?? 出弯后要连续 8 个周期(80ms) 才恢复满速 */

/* 出弯提前退出：偏差正在快速回零时立刻切回直道 KP（消除出弯打角过大） */
#define SPEED_E_BACK      30   /* ?? |error| 已回落到 30 以下 */
#define SPEED_DE_BACK      8   /* ?? 且每 10ms 回正超过 8 */
#define SPEED_BACK_CNT     2   /* ?? 连续 2 次确认才退出 */



/*----------------------------- 速度环 PI（L5 5.3） -----------------------------*/
#define SPEED_KP           40.0f    /* ?? 速度环阻尼 */
#define SPEED_KI           15.0f    /* ?? 速度环主响应 */
#define SPEED_KD            0.0f
#define SPEED_MAX_OUT      10000.0f  /* ?? 输出限幅（占空比）：12 脉冲约需 8000 */

/*===================================================================================================================
 *  元素状态机参数（L6 6.4）—— 全部待填，"时间为 0" 表示还没填
 *  ==================================================================================================================
 *  判定值都是归一化后的 0~100
 *==================================================================================================================*/
/*----------------------------- 圆环（环在左侧，必须进环） -----------------------------*/
/* 进环 / 出环判定：横向电感和 与 竖向电感和 必须【同时】超过各自阈值 */
#define RING_H_TH       175     /* ?? 横向：(LX + RX) > 此值（直道约 70，切点约 178） */
#define RING_V_TH        1
/* ?? 竖向：(LY + RY) > 此值（直道约 0，需要你实测） */
#define RING_COUNT        5     /* 连续 5 个 10ms（50ms）确认 */
#define RING_DELAY        2      /* ?? 确认后再延时多久才打角（20×10ms = 0.2 秒）——治"打角太早" */
#define RING_IN_MIN      160     /* ?? 进环强制打角【至少】持续 40×10ms = 0.4 秒（最重要的参数） */
#define RING_IN_MAX     640     /* ?? 进环强制打角【最多】持续 1.2 秒（防止一直打角） */
#define RING_ON_MIN     500     /* ?? 环内至少 0.5 秒，才允许判"到出环口" */
#define RING_TIMEOUT    640     /* ?? 兜底：环内最多 6.4 秒，超时强制出环 */
#define RING_OUT_TIME    100     /* ?? 出环：强制右打 0.4 秒 */
#define RING_OUT_BIAS     -20     /* 出环打角（负数 = 往右） */
#define RING_BIAS        100     /* 进环打角（正数 = 往左） */

extern uint16 run_tick;         /* 上电后经过的 10ms 数（定义在 code.c） */

/*===================================================================================================================
 *  类型与变量
 *==================================================================================================================*/
typedef struct
{
    ADCN_enum ch;               /* AD 通道 */
    uint16    max;              /* 标定上限 */
    uint16    min;              /* 标定下限 */
} adc_channel_t;

typedef enum
{
    ELEM_NONE = 0,              /* 普通路段：正常方向环 */
    ELEM_CROSS,                 /* 十字：拉直过去 */
    ELEM_ROUND_IN,              /* 进环 */
    ELEM_ROUND_ON,              /* 环内 */
    ELEM_ROUND_OUT              /* 出环 */
} elem_state_enum;

extern adc_channel_t ch_lx, ch_rx, ch_ly, ch_ry;

extern PID servo_pid;           /* 方向环 */
extern PID left_pid, right_pid; /* 速度环 */

extern int16 target_speed;      /* 速度环目标 */
extern int16  target_l, target_r;   /* 左/右轮速度环目标（含差速） */
extern int16  duty_diff;            /* 开环差速量（占空比） */
extern int16 target_duty;       /* 开环占空比 */

extern uint16 lh, rh, lv, rv;   /* 四路归一化结果（10ms 更新一次） */
extern int16  error;            /* 差比和偏差 */
extern int16  speed_l, speed_r; /* 左右轮速度（脉冲/10ms） */

extern uint16 raw_lx, raw_rx, raw_ly, raw_ry;   /* 四路滤波后的原始值（给屏幕显示用） */
extern volatile elem_state_enum elem_state;     /* 当前赛道元素状态 */
extern int16  speed_now;        /* 当前实际下发的电机占空比（直道/弯道自动切换） */
extern int16  error_last;       /* 上一周期的偏差，用来算 Δerror */

/*===================================================================================================================
 *  函数
 *==================================================================================================================*/
uint16 adc_filter(ADCN_enum ch);                    /* 去极值平均（L4 4.4.3） */
uint16 normalize(adc_channel_t *c);                 /* 归一化 0~100（L4 4.5.2） */
int16  calc_error(uint16 left, uint16 right);       /* 差比和 -100~+100（L4 4.6.2） */
void   elem_check(uint16 lh_now, uint16 rh_now, uint16 lv_now, uint16 rv_now);
int16  elem_correct(int16 error_in);                /* 元素修正（L6 6.4.2） */
int16  speed_decide(int16 e);                      /* 直道满速 / 入弯减速 */

#endif
