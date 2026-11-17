#include "motor.h"


void Motor_Init(void)
{
    pwm_init(PWML,10000,0);
	pwm_init(PWMR,10000,0);
    STBY = 1;
}

void Motor_Set(int motorA,int motorB)
{
    if (motorA >= 0)
    {
        AIN1 = 1;
		AIN2 = 0;
		pwm_duty(PWML,motorA);
    }
    else if (motorA < 0)
    {
        AIN1 = 0;
		AIN2 = 1;
		pwm_duty(PWML,-motorA);
    }
	
	if (motorB >= 0)
    {
        BIN1 = 1;
		BIN2 = 0;
		pwm_duty(PWMR,motorB);
    }
    else if (motorB < 0)
    {
        BIN1 = 0;
		BIN2 = 1;
		pwm_duty(PWMR,-motorB);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 *  L4 4.7.3  舵机打角
 *  duty 刻度 0~10000，50Hz 下 1 个单位 = 2 微秒
 *      250 -> 0.5ms     750 -> 1.5ms（中位）     1250 -> 2.5ms
 *  两条纪律：必须限幅（不能让舵机长期顶死）；动过机械就要重测中位
 *------------------------------------------------------------------------------------------------------------------*/
void Set_Servo_PWM(int32 duty)
{
    if(duty < SERVO_LEFT)   duty = SERVO_LEFT;
    if(duty > SERVO_RIGHT)  duty = SERVO_RIGHT;
	  P_SW2 |= 0x80;
    pwm_duty(SERVO_PWM_CH, (uint32)duty);
}
