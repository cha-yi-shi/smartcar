/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 ********************************************************************************************************************/
#include "headfile.h"
#include "encoder.h"
#include "key.h"
#include "motor.h"
#include "code.h"

int L_encoder=0,R_encoder=0;

//UART1中断
void UART1_Isr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        //程序自动下载
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

//UART2中断
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//接收数据寄存器为：S2BUF
		gps_uart_callback(S2BUF);
	}
}


//UART3中断
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//接收数据寄存器为：S3BUF

	}
}


//UART4中断
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;

		//接收数据寄存器为：S4BUF;
		if(wireless_module_uart_handler != NULL)
		{
			// 该函数为函数指针
			// 再初始化无线模块的时候，设置该函数地址
			wireless_module_uart_handler(S4BUF);
		}
	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED=~LED;
}
void INT1_Isr() interrupt 2
{
	
}


//INT2和INT3作为编码器输入，不允许再使用
//void INT2_Isr() interrupt 10
//{
//	INT2_CLEAR_FLAG;  //清除中断标志
//}
//void INT3_Isr() interrupt 11
//{
//	INT3_CLEAR_FLAG;  //清除中断标志
//}


void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //清除中断标志
}

/*-------------------------------------------------------------------------------------------------------------------
 *  TM0：CarDemo1 里用来读编码器（20ms），本工程把编码器读取放到了 TM4 里，
 *       所以这里保持空函数即可（也没启动 TIM_0）
 *------------------------------------------------------------------------------------------------------------------*/
void TM0_Isr() interrupt 1
{

}
void TM1_Isr() interrupt 3
{

}
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //清除中断标志
	
}
void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //清除中断标志
	
}

/*-------------------------------------------------------------------------------------------------------------------
 *  TM4：5ms 控制周期（电感每 5ms 读一次）
 *      ① 采电感 → 滤波 → 归一化      （L4 4.4 ~ 4.5）
 *      ② 差比和 → 偏差 error          （L4 4.6）
 *      ③ 元素状态机修正 error         （L6 6.4）
 *      ④ 方向环 PID → 舵机打角        （L4 4.8）
 *      ⑤ 读编码器 → 速度环 → 电机     （L5 5.2 ~ 5.3）
 *  中断里不延时、不等待、不打印
 *------------------------------------------------------------------------------------------------------------------*/
void TM4_Isr() interrupt 20
{
#if (RING_PID_ENABLE == 1)
    uint8 ring_now;                              /* 1 = 当前处于圆环段 */
#endif
	TIM4_CLEAR_FLAG; //清除中断标志
	key_handler();   //按键扫描（L3 3.3.2）

    /* ① 采样 → 滤波 → 归一 */
    lh = normalize(&ch_lx);
    rh = normalize(&ch_rx);
    lv = normalize(&ch_ly);
    rv = normalize(&ch_ry);

    /* ② 差比和 */
    error = calc_error(lh, rh);

    /* ③ 元素状态机修正（ELEM_ENABLE = 0 时什么都不做） */
    elem_check(lh, rh, lv, rv);                   /* 圆环：四路和触发 */
    error = elem_correct(error);


    /* ③-c 直道 / 弯道 判定：同时决定 速度 和 用哪套 PID */
    speed_now = speed_decide(error);

    /* ④ 方向环 → 舵机（用上面选好的 PID） */
#if (RING_PID_ENABLE == 1)
    /* ===== 圆环段：换用独立 PID（RING_KP=4），其他参数不变 ===== */
    ring_now = (uint8)((run_tick >= RING_ACTIVE_START) && (run_tick < RING_ACTIVE_END));
    if(ring_now)
    {
        servo_pid.kp = RING_KP;
        servo_pid.ki = RING_KI;
        servo_pid.kd = RING_KD;
    }
    else
    {
        servo_pid.kp = SERVO_KP;
        servo_pid.ki = SERVO_KI;
        servo_pid.kd = SERVO_KD;
    }
#else
    servo_pid.kp = SERVO_KP;
    servo_pid.ki = SERVO_KI;
    servo_pid.kd = SERVO_KD;
#endif

    /* ===== 时间段专用方向环 PID：20~21 秒 / 27~29 秒 用 KP=6.0 ===== */
    if(((run_tick >= PID2_T1_START) && (run_tick < PID2_T1_END)) ||
       ((run_tick >= PID2_T2_START) && (run_tick < PID2_T2_END)))
    {
        servo_pid.kp = PID2_KP;
        servo_pid.ki = SERVO_KI;
        servo_pid.kd = SERVO_KD;
    }

    /* ===== 第三套方向环 PID：1.5~2.5 秒 / 32~33 秒 用 KP=2.8 ===== */
    if(((run_tick >= PID3_T1_START) && (run_tick < PID3_T1_END)) ||
       ((run_tick >= PID3_T2_START) && (run_tick < PID3_T2_END)))
    {
        servo_pid.kp = PID3_KP;
        servo_pid.ki = SERVO_KI;
        servo_pid.kd = SERVO_KD;
    }

    /* ===== 第四套方向环 PID：8~9 秒 用 KP=1.0 ===== */
    if(((run_tick >= PID4_T_START)   && (run_tick < PID4_T_END)) ||
       ((run_tick >= PID4_T2_START)  && (run_tick < PID4_T2_END)))
    {
        servo_pid.kp = PID4_KP;
        servo_pid.ki = SERVO_KI;
        servo_pid.kd = SERVO_KD;
    }

    Positional_PID_Calc(&servo_pid, 0, (float)error);
    Set_Servo_PWM((int32)(SERVO_MID + servo_pid.output));


    /* ===== 圆环开环：到点强制左打，之后交回差比和 ===== */
    #if (RING_FORCE_ENABLE == 1)
    if((run_tick >= FORCE_START_TICK) &&
       (run_tick <  (FORCE_START_TICK + FORCE_TIME_TICK)))
    {
        Set_Servo_PWM((int32)FORCE_DUTY);
    }
    /* 出环强制打角已停用（按要求只保留进环打角）
    else if((run_tick >= FORCE_OUT_START) &&
            (run_tick <  (FORCE_OUT_START + FORCE_OUT_TIME)))
    {
        Set_Servo_PWM((int32)FORCE_OUT_DUTY);
    }
    ===== 出环强制打角停用结束 ===== */
    #endif
        /* 临时：跳过 PID，把偏差放大 2 倍直接打舵 */
   // Set_Servo_PWM((int32)(SERVO_MID - (int32)(error * 2)));
    /* ⑤ 读编码器 → 速度环 → 电机 */
    speed_l = (int16)encoder_get_count(ENCODER1, 0);
    speed_r = (int16)encoder_get_count(ENCODER2, 1);
    L_encoder = speed_l;
    R_encoder = speed_r;

    /* 累计路程（脉冲）：用于标定"每圈多少个脉冲" */
    total_l += speed_l;
    total_r += speed_r;

    /* 平滑速度（时间常数约 8×5ms = 40ms）与峰值，用于标定 */
    /* 平均：先累加 16 个样本（80ms）再除，避免整数截断误差 */
    avg_sum_l += speed_l;
    avg_sum_r += speed_r;
    avg_cnt++;
    if(avg_cnt >= 16)
    {
        avg_l = (int16)(avg_sum_l / 16);
        avg_r = (int16)(avg_sum_r / 16);
        avg_sum_l = 0;
        avg_sum_r = 0;
        avg_cnt = 0;
    }
    if(speed_l < peak_l)  peak_l = speed_l;        /* 前进为负，取最负 = 最快 */
    if(speed_r < peak_r)  peak_r = speed_r;

#if (RING_PID_ENABLE == 1)
    /* ===== 圆环段：电机固定占空比（不走速度环）===== */
    if(ring_now)
    {
        Motor_Set((int)RING_MOTOR_DUTY, (int)RING_MOTOR_DUTY);
    }
    else
#endif
    {
#if (SPEED_LOOP_ENABLE == 1)
    /* ===== 速度环：20ms 判断一次（4 个 5ms 周期累加） =====
       每个 5ms 把脉冲累加，每 SPEED_SAMPLE_DIV 次才做一次 PID
       → 分辨率从 13% 提升到 3.3% */
    enc_acc_l += speed_l;
    enc_acc_r += speed_r;

    spd_div++;
    if(spd_div >= SPEED_SAMPLE_DIV)
    {
        int16 tl, tr, dl, dr, sl, sr;

        spd_div = 0;
        sl = enc_acc_l;  enc_acc_l = 0;
        sr = enc_acc_r;  enc_acc_r = 0;

        tl = (int16)(target_l * SPEED_SAMPLE_DIV);
        tr = (int16)(target_r * SPEED_SAMPLE_DIV);

        dl = (int16)(sl - tl);
        dr = (int16)(sr - tr);

        if((dl > DEAD_SPEED) || (dl < -DEAD_SPEED))
            Incremental_PID_Calc(&left_pid,  (float)tl, (float)sl);
        if((dr > DEAD_SPEED) || (dr < -DEAD_SPEED))
            Incremental_PID_Calc(&right_pid, (float)tr, (float)sr);

#if (SPEED_NO_REVERSE == 1)
        if(left_pid.output  > 0.0f)  left_pid.output  = 0.0f;
        if(right_pid.output > 0.0f)  right_pid.output = 0.0f;
#endif
        Motor_Set((int)left_pid.output, (int)right_pid.output);
    }
#else
    /* 开环：基础占空比 ± 差速 */
    {
        int16 d_l, d_r;
        d_l = (int16)(speed_now - duty_diff);      /* E>0 → 左轮快 */
        d_r = (int16)(speed_now + duty_diff);      /* E>0 → 右轮慢 */
        if(d_l >  10000) d_l =  10000;
        if(d_l < -10000) d_l = -10000;
        if(d_r >  10000) d_r =  10000;
        if(d_r < -10000) d_r = -10000;
        Motor_Set(d_l, d_r);
    }
#endif
}

    }

    /* ===== 丢线保护：四路电感和太小 → 减速但不停 ===== */
    if((lh + rh + lv + rv) < LOST_SUM)
    {
        Motor_Set((int)LOST_DUTY, (int)LOST_DUTY);
    }

//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;
