#ifndef CODE_KEY_H_
#define CODE_KEY_H_

#include "headfile.h"
#include "encoder.h"

#define KEY_NUMBER 4 // 按键数目
#define KEY_PINS               \
    {                          \
        P7_0, P7_1, P7_2, P7_3 \
    } // 按键对应引脚
	
extern int key_cnt1,key_cnt2,key_cnt3,key_cnt4;

sbit key_1 = P7 ^ 0;
sbit key_2 = P7 ^ 1;
sbit key_3 = P7 ^ 2;
sbit key_4 = P7 ^ 3;

#define PRESS_THRESHOLD 10     // 按键消抖阈值
#define HOLD_THRESHOLD 300     // 按键长按阈值
#define INTERVAL_THRESHOLD 150 // 按键连按阈值

//========================按键结构体参数========================//
typedef struct
{
    uint8 value, last_value;         // 按键当前值 / 上一次值
    uint32 hold_time, interval_time; // 按键长按时间 / 按键连按时间
    uint32 click_count;              // 按键连按次数

    enum
    {
        RELEASE,   // 按键释放
        DITHER,    // 按键抖动
        PRE_CLICK, // 预双击键
        IN_CLICK,  // 已双击键
        PRESS,     // 按键按下
        HOLD,      // 按键长按
        MULTI,     // 按键连按
    } state;       // 按键状态

    uint8 is_press; // 按键是否按下
    uint8 is_hold;  // 按键是否长按
    uint8 is_multi; // 按键是否连按
} key_struct;
//========================按键结构体参数========================//

//========================按键基础函数========================//
uint8 key_get_value(uint8 i);
void key_handler(void);
void key_action(void);
void key_list_init(uint8 pit_timer);
//========================按键基础函数========================//

#endif /* CODE_KEY_H_ */
