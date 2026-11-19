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
#define TARGET_DUTY_DEFAULT     -10000       /* ?? 开环占空比 0~10000（想跑就改成 3000 左右） */
#define TARGET_SPEED_DEFAULT    0       /* ?? 闭环目标速度（脉冲/10ms） */

#define SPEED_LOOP_ENABLE       1       /* 0 = 开环，占空比固定 -10000（用于标定测量） */
#define LCD_DEBUG               0       /* ?? 0 = 关闭屏幕打印（提速，不占用主循环） 1 = 打开 */
#define ELEM_ENABLE             0       /* 0 = 圆环闭环关闭，改按时间开环进环 */

/*----------------------------- 方向环 PID（L4 4.8） -----------------------------*/
/* 调舵机中位时：把 SERVO_KP 改成 0，舵机就会固定停在 SERVO_MID */
#define SERVO_KP        2.6f
#define SERVO_KI        0.0f
#define SERVO_KD        7.0f
#define SERVO_MAX_I     100.0f
#define SERVO_MAX_OUT   400.0f

/* 弯道专用 PID（直道用上面的 SERVO_KP/KI/KD） */
#define KP_CURVE        3.5f    /* ?? 弯道 Kp（直道 2.5 的 1.4 倍，弯道打角更狠） */
#define KI_CURVE        0.0f
#define KD_CURVE        4.0f

/*----------------------------- 入弯减速（竞速：直道满占空比） -----------------------------*/
#define SPEED_STRAIGHT -10000   /* ?? 直道：满占空比（保持竞速速度） */

/*----------------------------- 两轮差速（给在速度环 PID 的目标值上） -----------------------------*/
#define TARGET_SPEED_BASE   -7     /* ?? 每 5ms 目标路程（脉冲）：实测极限 -6，设 -7 让速度环顶在 100% */
/* #define CTRL_PERIOD_MS 5 --- 已改为直接用每周期路程，不再换算速度 */
/* 开环差速：直接改电机占空比（不用速度环、不用编码器） */
#define SPEED_NO_REVERSE    1       /* ?? 1 = 禁止电机反转：超过目标速度就断油到 0，不准反向 */
#define DEAD_SPEED          1       /* ?? 速度死区（20ms 累加单位）：4 = 1 脉冲/5ms */
#define SPEED_SAMPLE_DIV    2       /* ?? 速度环采样分频：4×5ms = 20ms 累加一次（分辨率×4） */

#define DIFF_DUTY_K     100      /* ?? 开环差速系数：error=40 → 左右占空比差 ±4000（40%） */
#define DIFF_DUTY_LIMIT  8000    /* ?? 最大差速（占空比）= 60%，内侧最慢可到 40% */
#define DIFF_DEAD          20     /* ?? 差速死区：|error| < 10 不做差速，保证直道满速 */

/* 速度环用的差速（脉冲单位，加在速度环目标值上） */
#define DIFF_K              0.03f  /* ?? 速度环差速系数（脉冲单位）：error 50 → 1 个脉冲 */
#define DIFF_LIMIT          2       /* ?? 速度环差速上限：最多差 2 个脉冲（约 27%） */
/* #define SPEED_CURVE -8000 --- 入弯减速已取消，改为两轮差速（差速给在速度环 PID 里） */

#define SPEED_E_ON         35   /* ?? |error| > 18 → 判定进弯，减速 */
#define SPEED_DE_ON        10   /* ?? |Δerror| > 10 → 急弯特征，更早减速 */
#define SPEED_E_OFF        30   /* ?? |error| < 10 → 认为出弯（回差，防抖） */
#define SPEED_HOLD         16   /* 8→16：连续 16 个周期(80ms) —— 周期改成 5ms 后翻倍 */

/* 出弯提前退出：偏差正在快速回零时立刻切回直道 KP（消除出弯打角过大） */
#define SPEED_E_BACK      30   /* ?? |error| 已回落到 30 以下 */
#define SPEED_DE_BACK      8   /* ?? 且每 10ms 回正超过 8 */
#define SPEED_BACK_CNT     2   /* ?? 连续 2 次确认才退出 */



/*----------------------------- 速度环 PI（L5 5.3） -----------------------------*/
#define SPEED_KP           40.0f    /* ?? 速度环阻尼 */
#define SPEED_KI           60.0f    /* ?? 速度环主响应 */
#define SPEED_KD            0.0f
#define SPEED_MAX_OUT      10000.0f  /* ?? 输出限幅（占空比）：12 脉冲约需 8000 */

/*===================================================================================================================
 *  元素状态机参数（L6 6.4）—— 全部待填，"时间为 0" 表示还没填
 *  ==================================================================================================================
 *  判定值都是归一化后的 0~100
 *==================================================================================================================*/
/*----------------------------- 圆环（环在左侧，必须进环） -----------------------------*/
/* 进环 / 出环判定：横向电感和 与 竖向电感和 必须【同时】超过各自阈值 */
#define RING_H_TH       163     /* ?? 横向和 (LX + RX) > 175 才算（直道约 70，切点约 178） */
#define RING_V_MAX       3     /* ?? 竖向和 (LY + RY) < 10 才算（竖向必须"干净"） */
/* ?? 竖向：(LY + RY) > 此值（直道约 0，需要你实测） */
#define RING_COUNT        5     /* 5→10：连续 50ms —— 周期改成 5ms 后翻倍 */
#define RING_DELAY        15     /* 2→4：延时 20ms */
#define RING_IN_MIN       320     /* 160→320：进环至少 1.6 秒 */
#define RING_IN_MAX      1280     /* 640→1280：进环最多 6.4 秒 */
#define RING_ON_MIN      1000     /* 500→1000：环内至少 5 秒 */
#define RING_TIMEOUT     1280     /* 640→1280：兜底 6.4 秒 */
#define RING_OUT_TIME     200     /* 100→200：出环强制右打 1 秒 */
#define RING_OUT_BIAS     0     /* 出环打角（负数 = 往右） */
#define RING_BIAS        100     /* 进环打角（正数 = 往左） */

extern uint16 run_tick;         /* 上电后经过的 10ms 数（定义在 code.c） */

/*------------------- 圆环开环：按时间强制打角 -------------------*/
#define RING_FORCE_ENABLE    1     /* ?? 1 = 启用进环强制打角（出环打角已停用） */


#define FORCE_START_TICK   910     /* ?? 5.3 秒 开始进环强制左打 */
#define FORCE_TIME_TICK    300     /* ?? 240×5ms = 1200 毫秒 强制左打时长 */
#define FORCE_DUTY         350     /* ?? 打角 duty（650=中位，越小越左） */
#define FORCE_OUT_START    590     /* 开始出环 (环内时间)（0543）*/
#define FORCE_OUT_TIME       0     /* ?? 160×5ms = 0.8 秒 出环时长 */
#define FORCE_OUT_DUTY     550     /* ?? 出环duty（650=中位，越大越右） */

/*------------ 圆环专用：独立 PID（KP=4）+ 电机固定占空比 ------------*/
#define RING_PID_ENABLE      1     /* ?? 1 = 圆环段用独立 PID(KP=4) + 固定占空比 */
#define RING_ACTIVE_START   910    /* ?? 5.3 秒进入圆环段（独立PID+固定占空比） */
#define RING_ACTIVE_END    1700    /* ?? 9.3 秒离开圆环段（恢复普通 PID） */
#define RING_KP             6.0f   /* ?? 圆环专用 KP */
#define RING_KI             0.0f
#define RING_KD             1.0f
#define RING_MOTOR_DUTY   -10000    /* ?? 圆环内电机固定占空比（负数=前进） */

/*------------ 时间段专用方向环 PID（KP=6.0，其他同 SERVO_*）------------*/
#define PID2_KP             6.0f   /* ?? 专用方向环 KP */
#define PID2_T1_START      4000    /* ?? 20 秒 */
#define PID2_T1_END        4200    /* ?? 21 秒 */
#define PID2_T2_START      5400    /* ?? 27 秒 */
#define PID2_T2_END        5800    /* ?? 29 秒 */

/*------------ 带差速的速度环：仅在这个时间段生效 ------------*/
#define DIFF_T_START       4000    /* ?? 20 秒（4000×5ms）开始启用差速 */
#define DIFF_T_END         4200    /* ?? 21 秒（4200×5ms）停止差速 */
#define DIFF_T2_START      5400    /* ?? 27 秒（5400×5ms）第二个差速窗口开始 */
#define DIFF_T2_END        5700    /* ?? 28 秒（5600×5ms）第二个差速窗口结束 */

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
extern int16  enc_acc_l, enc_acc_r;  /* 速度环 20ms 累加 */
extern int32  total_l, total_r;      /* 累计脉冲（路程，用于标定每圈脉冲数） */
extern int16  avg_l, avg_r;          /* 平滑速度（标定用） */
extern int16  peak_l, peak_r;        /* 峰值速度（标定用） */
extern int32  avg_sum_l, avg_sum_r;  /* 平均值累加器 */
extern uint8  avg_cnt;              /* 平均采样计数 */
extern uint8  spd_div;              /* 速度环分频计数 */
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
