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
#include "key.h"
#include "code.h"
#include "seekfree_assistant.h"
#include <string.h>


/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 */

char txt[60];
extern int L_encoder,R_encoder;
int SW1,SW2,SW3,SW4;
uint16 adc1,adc2,adc3,adc4;

/*-------------------------------------------------------------------------------------------------------------------
 *  L6 6.2.2 第①步：注册"发送函数"，告诉组件数据从哪个串口发出去
 *------------------------------------------------------------------------------------------------------------------*/
uint32 assistant_transfer_cb(const uint8 *buff, uint32 length)
{
    uart_putbuff(DEBUG_UART, (uint8 *)buff, (uint16)length);
    return length;
}

void main()
{
	board_init();			// 初始化寄存器,勿删除此句代码。

	lcd_init();
    Motor_Init();

    encoder_init(ENCODER1);
	encoder_init(ENCODER2);

    key_list_init(5);                       /* 按键 5ms 扫描（跟控制周期一致） */

    /* L4 4.3：四路电感通道各初始化一次（照 CarDemo1：P0.0 ~ P0.3） */
    adc_init(ADC_LEFTX_CHANNEL,  ADC_SYSclk_DIV_2);
    adc_init(ADC_RIGHTX_CHANNEL, ADC_SYSclk_DIV_2);
    adc_init(ADC_LEFTY_CHANNEL,  ADC_SYSclk_DIV_2);
    adc_init(ADC_RIGHTY_CHANNEL, ADC_SYSclk_DIV_2);

    pwm_init(PWMB_CH1_P74,50,650);          /* 舵机 50Hz（照 CarDemo1） */

    /* L4 4.8.2：方向环（位置式 PID） */
    Positional_PID_Init(&servo_pid, SERVO_KP, SERVO_KI, SERVO_KD, SERVO_MAX_I, SERVO_MAX_OUT);

    /* L5 5.3：速度环（增量式 PI） */
    Incremental_PID_Init(&left_pid,  SPEED_KP, SPEED_KI, SPEED_KD, SPEED_MAX_OUT);
    Incremental_PID_Init(&right_pid, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_MAX_OUT);

    pit_timer_ms(TIM_4,5);                  /* 5ms 控制周期（电感每 5ms 读一次） */

    while(1)
    {
#if (LCD_DEBUG == 1)
        /* ===================== 以下 6 行完全照 CarDemo1 原样 ===================== */
        sprintf(txt,"L:%d,R:%d         ",L_encoder,R_encoder);
        lcd_showstr(0,0,(int8 *)txt);

        sprintf(txt,"Key4:%d  Key3:%d         ",key_cnt4,key_cnt3);
        lcd_showstr(0,1,(int8 *)txt);

        sprintf(txt,"Key2:%d  Key1:%d         ",key_cnt2,key_cnt1);
        lcd_showstr(0,2,(int8 *)txt);

        SW1 = P64;
        SW2 = P65;
        SW3 = P66;
        SW4 = P67;

        /* 第 3 行：累计路程（脉冲），用于标定每圈脉冲数 */
        sprintf(txt,"A:%d,%d M:%d    ",avg_l,avg_r,peak_l);
        lcd_showstr(0,3,(int8 *)txt);

        /* adc 用中断里已经采好的原始值，避免主循环和中断同时操作 ADC */
        adc1 = raw_lx;
        adc2 = raw_rx;
        adc3 = raw_ly;
        adc4 = raw_ry;

        sprintf(txt,"adc1:%d  adc2:%d         ",adc1,adc2);
        lcd_showstr(0,4,(int8 *)txt);

        sprintf(txt,"adc3:%d  adc4:%d         ",adc3,adc4);
        lcd_showstr(0,5,(int8 *)txt);
        /* ===================== 以上为 CarDemo1 原有显示 ===================== */

        /* ------------------- L4 / L5 调试显示（接在下面几行） ------------------- */
        sprintf(txt,"N:%d %d %d %d      ",lh,rh,lv,rv);
        lcd_showstr(0,6,(int8 *)txt);

        sprintf(txt,"T:%d C:%d E:%d ",force_start_tick,curve_flag,error);
        lcd_showstr(0,7,(int8 *)txt);

        sprintf(txt,"D:%d V:%d      ",duty_diff,speed_now);
        lcd_showstr(0,8,(int8 *)txt);


        delay_ms(1);
#endif
        /* 屏幕打印已关闭：LCD_DEBUG = 0 时不执行，节省主循环时间 */
    }
}
