#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"
#include <math.h>

// PID参数结构体
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float error;            // 当前误差
    float error_last;       // 上次误差
    float error_sum;        // 误差积分
    float output_max;       // 输出最大值
    float output_min;       // 输出最小值
    float error_sum_max;    // 积分限幅最大值
    float error_sum_min;    // 积分限幅最小值
} PIDParam_t;

// 函数声明
void PIDInit(PIDParam_t *pid_param, float kp, float ki, float kd, 
             float output_max, float output_min);                    // PID初始化
float PID(PIDParam_t *pid_param, float feedback, float target);     // 标准PID计算
float PD(PIDParam_t *pid_param, float feedback, float target);      // PD控制
float PID_AntiWindup(PIDParam_t *pid_param, float feedback, float target); // 抗积分饱和PID
float PID_WithFilter(PIDParam_t *pid_param, float feedback, 
                     float target, float alpha);                    // 带滤波的PID
void PID_Reset(PIDParam_t *pid_param);                              // 重置PID状态

// 增量式PID（可选）
typedef struct {
    float kp, ki, kd;      // PID参数
    float error[3];         // 误差队列 e(k), e(k-1), e(k-2)
    float output_max;       // 输出最大值
    float output_min;       // 输出最小值
} IncrementalPID_t;

void IncrementalPID_Init(IncrementalPID_t *pid, float kp, float ki, float kd,
                         float output_max, float output_min);      // 增量式PID初始化
float IncrementalPID_Calculate(IncrementalPID_t *pid, float feedback, float target); // 增量式PID计算

#endif /* __PID_H__ */