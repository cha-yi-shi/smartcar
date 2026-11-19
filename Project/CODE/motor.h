#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "headfile.h"

#define PWMR	PWMA_CH1P_P60
#define BIN2	P06
#define BIN1	P07
#define PWML	PWMA_CH2P_P62
#define AIN2	P63
#define AIN1	P61
#define STBY	P05

/*======================= ¶æ»ú£¨L4 4.7£© =======================*/

#define SERVO_PWM_CH    PWMB_CH1_P74
#define SERVO_MID       630
#define SERVO_LEFT      200
#define SERVO_RIGHT     950

void Motor_Init(void);
void Motor_Set(int motorA,int motorB);
void Set_Servo_PWM(int32 duty);

#endif // !_MOTOR_H_
