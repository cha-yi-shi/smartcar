/*********************************************************************************************************************
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 ********************************************************************************************************************/
#include "headfile.h"
#include "encoder.h"
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
		if(wireless_module_uart_handler != NULL)
		{
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

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;
}

void TM0_Isr() interrupt 1
{
}
void TM1_Isr() interrupt 3
{
}
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;
}
void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG;
}

/*-------------------------------------------------------------------------------------------------------------------
 *  TM4：5ms 控制周期
 *      1. 采 4 路电感 -> 窗口去极值平均 -> 一阶低通 -> 归一化
 *      2. 差比和 -> error
 *      3. 曲率系数 curv -> 预判增益 K
 *      4. error * K -> 输入死区 -> 位置式 PID -> 舵机
 *      5. 圆环开环强制打角（到点覆盖）
 *      6. 编码器 -> 一阶低通 -> 增量式 PID -> 电机（两轮同速）
 *  中断里不延时、不等待、不打印
 *------------------------------------------------------------------------------------------------------------------*/
void TM4_Isr() interrupt 20
{
    int16  tl, tr, sl, sr, dl, dr;
    int16  out;
    int16  ead;

    TIM4_CLEAR_FLAG;                        /* 清除中断标志 */

    if(run_tick < 60000) run_tick++;        /* 上电计时（5ms 一拍） */

    /* 1. 四路电感：采集 -> 去极值平均 -> 低通 -> 归一化 */
    lh = adc_proc(&ch_lx);
    rh = adc_proc(&ch_rx);
    lv = adc_proc(&ch_ly);
    rv = adc_proc(&ch_ry);

    raw_lx = ch_lx.val;
    raw_rx = ch_rx.val;
    raw_ly = ch_ly.val;
    raw_ry = ch_ry.val;

    sv_now = (uint16)(raw_ly + raw_ry);     /* 竖向和（原始） */
    ss_now = (uint16)(raw_lx + raw_rx);     /* 横向和（原始） */

    /* 2. 差比和 */
    error = calc_error(lh, rh);

    /* 3. 曲率系数 + 预判增益 */
    curv_update();

    /* 4. 方向环：进 PID 前乘预判增益 */
    err_in = (int16)((float)error * k_pred);
    if(err_in >  100) err_in =  100;
    if(err_in < -100) err_in = -100;

    Positional_PID_Calc(&servo_pid, 0.0f, (float)err_in);

    /* 输入死区（自适应）：直道死区大（不打角），弯道死区自动缩小到 0 */
    ead = (int16)(E_DEAD * (1.0f - curv));
    if((err_in > -ead) && (err_in < ead))
    {
        servo_pid.output = 0.0f;
    }
    out = (int16)servo_pid.output;

    if(out >  (int16)SERVO_MAX_OUT) out =  (int16)SERVO_MAX_OUT;
    if(out < -(int16)SERVO_MAX_OUT) out = -(int16)SERVO_MAX_OUT;

    servo_out  = out;
    servo_duty = (int16)(SERVO_MID + out);
    Set_Servo_PWM((int32)servo_duty);

    /* 5. 圆环开环：到点强制打角（覆盖上面） */
    ring_force();

    /* 6. 编码器 -> 低通 -> 速度环 */
    speed_l = (int16)encoder_get_count(ENCODER1, 0);
    speed_r = (int16)encoder_get_count(ENCODER2, 1);
    L_encoder = speed_l;
    R_encoder = speed_r;

    enc_acc_l += speed_l;
    enc_acc_r += speed_r;
    spd_div++;

    if(spd_div >= SPEED_SAMPLE_DIV)
    {
        spd_div = 0;
        sl = enc_acc_l;  enc_acc_l = 0;
        sr = enc_acc_r;  enc_acc_r = 0;

        /* 编码器一阶低通（对 10ms 累加值滤波） */
        enc_f_l = enc_f_l + ENC_ALPHA * ((float)sl - enc_f_l);
        enc_f_r = enc_f_r + ENC_ALPHA * ((float)sr - enc_f_r);

        /* 目标：两轮同速（无差速） */
        tl = (int16)(TARGET_SPEED_BASE * SPEED_SAMPLE_DIV);
        tr = tl;

        sl = (int16)enc_f_l;
        sr = (int16)enc_f_r;

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

        motor_l = (int16)left_pid.output;
        motor_r = (int16)right_pid.output;
        Motor_Set(motor_l, motor_r);
    }
}
