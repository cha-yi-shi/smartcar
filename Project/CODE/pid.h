#ifndef  __PID_H
#define  __PID_H

#include "headfile.h"

typedef struct
{
    float kp,ki,kd;                         //三个系数
    float error,lastError,lastlastError;    //误差、上次误差、上上次误差
    float integral,maxIntegral;             //积分、积分限幅
    float output,maxOutput;                 //输出、输出限幅
} PID;


/**
 * @brief 初始化增量式 PID 控制器
 * 
 * 该函数用于初始化增量式 PID 控制器的参数，包括比例系数、积分系数、微分系数和输出限幅。
 * 
 * @param pid 指向 PID 结构体的指针，该结构体包含 PID 控制器的各种参数。
 * @param p 比例系数，用于调整当前误差的影响程度。
 * @param i 积分系数，用于调整历史误差累积的影响程度。
 * @param d 微分系数，用于调整误差变化率的影响程度。
 * @param maxOutput 输出限幅，限制 PID 控制器的最大输出值。
 */
void Incremental_PID_Init(PID *pid, float p, float i, float d, float maxOutput);

/**
 * @brief 执行增量式 PID 计算
 * 
 * 该函数根据设定值和实际值，使用增量式 PID 算法计算控制量。
 * 
 * @param pid 指向 PID 结构体的指针，该结构体包含 PID 控制器的各种参数。
 * @param set_value 设定值，即期望的目标值。
 * @param get_value 实际值，即当前系统的反馈值。
 */
void Incremental_PID_Calc(PID *pid, float set_value, float get_value);

/**
 * @brief 初始化位置式 PID 控制器
 * 
 * 该函数用于初始化位置式 PID 控制器的参数，包括比例系数、积分系数、微分系数、积分限幅和输出限幅。
 * 
 * @param pid 指向 PID 结构体的指针，该结构体包含 PID 控制器的各种参数。
 * @param p 比例系数，用于调整当前误差的影响程度。
 * @param i 积分系数，用于调整历史误差累积的影响程度。
 * @param d 微分系数，用于调整误差变化率的影响程度。
 * @param maxI 积分限幅，限制积分项的最大累积值。
 * @param maxOutput 输出限幅，限制 PID 控制器的最大输出值。
 */
void Positional_PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOutput);

/**
 * @brief 执行位置式 PID 计算
 * 
 * 该函数根据设定值和实际值，使用位置式 PID 算法计算控制量。
 * 
 * @param pid 指向 PID 结构体的指针，该结构体包含 PID 控制器的各种参数。
 * @param set_value 设定值，即期望的目标值。
 * @param get_value 实际值，即当前系统的反馈值。
 */
void Positional_PID_Calc(PID *pid, float set_value, float get_value);


#endif

