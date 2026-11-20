/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 ********************************************************************************************************************/

#include "headfile.h"
#include "encoder.h"
#include "motor.h"
#include "code.h"
#include <string.h>

/*
 * v3：电感（自适应窗口去极值 + 一阶低通）→ 差比和 → 曲率系数 → 预判增益
 *      → 位置式 PID（方向）+ 死区补偿 → 舵机
 *      → 编码器（一阶低通）→ 增量式 PID（速度）→ 两轮同速
 */

char txt[60];
extern int L_encoder,R_encoder;

void main()
{
	board_init();			// 初始化寄存器,勿删除此句代码。

	lcd_init();
    Motor_Init();

    encoder_init(ENCODER1);
	encoder_init(ENCODER2);

    /* 四路电感通道各初始化一次（P0.0 ~ P0.3） */
    adc_init(ADC_LEFTX_CHANNEL,  ADC_SYSclk_DIV_2);
    adc_init(ADC_RIGHTX_CHANNEL, ADC_SYSclk_DIV_2);
    adc_init(ADC_LEFTY_CHANNEL,  ADC_SYSclk_DIV_2);
    adc_init(ADC_RIGHTY_CHANNEL, ADC_SYSclk_DIV_2);

    /* 通道结构体初始化 */
    ch_lx.ch = ADC_LEFTX_CHANNEL;   ch_lx.max = LX_MAX;   ch_lx.min = LX_MIN;
    ch_rx.ch = ADC_RIGHTX_CHANNEL;  ch_rx.max = RX_MAX;   ch_rx.min = RX_MIN;
    ch_ly.ch = ADC_LEFTY_CHANNEL;   ch_ly.max = LY_MAX;   ch_ly.min = LY_MIN;
    ch_ry.ch = ADC_RIGHTY_CHANNEL;  ch_ry.max = RY_MAX;   ch_ry.min = RY_MIN;

    pwm_init(PWMB_CH1_P74,50,650);          /* 舵机 50Hz */

    /* 方向环：位置式 PID */
    Positional_PID_Init(&servo_pid, SERVO_KP, SERVO_KI, SERVO_KD, SERVO_MAX_I, SERVO_MAX_OUT);

    /* 速度环：增量式 PID */
    Incremental_PID_Init(&left_pid,  SPEED_KP, SPEED_KI, SPEED_KD, SPEED_MAX_OUT);
    Incremental_PID_Init(&right_pid, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_MAX_OUT);

    pit_timer_ms(TIM_4,5);                  /* 5ms 控制周期 */

    while(1)
    {
#if (LCD_DEBUG == 1)
        sprintf(txt,"Sv:%d Ss:%d      ",sv_now,ss_now);
        lcd_showstr(0,0,(int8 *)txt);

        sprintf(txt,"E:%d dE:%d      ",error,de_now);
        lcd_showstr(0,1,(int8 *)txt);

        sprintf(txt,"C:%d K:%d      ",(int16)(curv*100.0f),(int16)(k_pred*10.0f));
        lcd_showstr(0,2,(int8 *)txt);

        sprintf(txt,"O:%d D:%d      ",servo_out,servo_duty);
        lcd_showstr(0,3,(int8 *)txt);

        sprintf(txt,"a1:%d a2:%d      ",raw_lx,raw_rx);
        lcd_showstr(0,4,(int8 *)txt);

        sprintf(txt,"a3:%d a4:%d      ",raw_ly,raw_ry);
        lcd_showstr(0,5,(int8 *)txt);

        sprintf(txt,"L:%d R:%d      ",L_encoder,R_encoder);
        lcd_showstr(0,6,(int8 *)txt);

        sprintf(txt,"M:%d,%d        ",motor_l,motor_r);
        lcd_showstr(0,7,(int8 *)txt);

        delay_ms(1);
#endif
    }
}
