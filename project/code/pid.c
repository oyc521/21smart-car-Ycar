#include "pid.h"
#include <math.h>

/**
 * @brief PID初始化
 */
void PIDInit(PIDParam_t *pid_param, float kp, float ki, float kd, 
             float output_max, float output_min)
{
    pid_param->kp = kp;
    pid_param->ki = ki;
    pid_param->kd = kd;
    pid_param->output_max = output_max;
    pid_param->output_min = output_min;
    pid_param->error = 0;
    pid_param->error_last = 0;
    pid_param->error_sum = 0;
    
    // 设置默认的积分限幅值
    if (fabsf(ki) > 1e-6) {
        pid_param->error_sum_max = output_max / ki;
        pid_param->error_sum_min = output_min / ki;
    } else {
        pid_param->error_sum_max = 1000.0f;
        pid_param->error_sum_min = -1000.0f;
    }
}

/**
 * @brief 标准PID计算
 */
float PID(PIDParam_t *pid_param, float feedback, float target)
{
    pid_param->error = target - feedback;
    
    // 更新积分项
    pid_param->error_sum += pid_param->error;
    
    // 积分限幅
    if (pid_param->error_sum > pid_param->error_sum_max) {
        pid_param->error_sum = pid_param->error_sum_max;
    } else if (pid_param->error_sum < pid_param->error_sum_min) {
        pid_param->error_sum = pid_param->error_sum_min;
    }
    
    // 计算微分项
    float derivative = pid_param->error - pid_param->error_last;
    
    // 计算PID输出
    float output = pid_param->kp * pid_param->error + 
                   pid_param->ki * pid_param->error_sum + 
                   pid_param->kd * derivative;
    
    // 输出限幅
    if (output > pid_param->output_max) {
        output = pid_param->output_max;
    } else if (output < pid_param->output_min) {
        output = pid_param->output_min;
    }
    
    // 保存当前误差
    pid_param->error_last = pid_param->error;
    
    return output;
}

/**
 * @brief PD控制
 */
float PD(PIDParam_t *pid_param, float feedback, float target)
{
    pid_param->error = target - feedback;
    
    // 计算微分项
    float derivative = pid_param->error - pid_param->error_last;
    
    // 计算PD输出
    float output = pid_param->kp * pid_param->error + 
                   pid_param->kd * derivative;
    
    // 输出限幅
    if (output > pid_param->output_max) {
        output = pid_param->output_max;
    } else if (output < pid_param->output_min) {
        output = pid_param->output_min;
    }
    
    // 保存当前误差
    pid_param->error_last = pid_param->error;
    
    return output;
}

/**
 * @brief 抗积分饱和PID
 */
float PID_AntiWindup(PIDParam_t *pid_param, float feedback, float target)
{
    pid_param->error = target - feedback;
    
    // 计算比例项和微分项
    float p_term = pid_param->kp * pid_param->error;
    float derivative = pid_param->error - pid_param->error_last;
    float d_term = pid_param->kd * derivative;
    
    // 计算积分项（带抗饱和）
    float i_term = pid_param->ki * pid_param->error_sum;
    float output_pre_sat = p_term + i_term + d_term;
    
    // 输出饱和检查与抗积分饱和处理
    if (output_pre_sat > pid_param->output_max) {
        output_pre_sat = pid_param->output_max;
        // 只有误差与积分项符号相反时才累积积分
        if (pid_param->error * pid_param->ki <= 0) {
            pid_param->error_sum += pid_param->error;
        }
    } else if (output_pre_sat < pid_param->output_min) {
        output_pre_sat = pid_param->output_min;
        // 只有误差与积分项符号相反时才累积积分
        if (pid_param->error * pid_param->ki >= 0) {
            pid_param->error_sum += pid_param->error;
        }
    } else {
        // 输出未饱和，正常累积积分
        pid_param->error_sum += pid_param->error;
    }
    
    // 积分项限幅
    if (pid_param->error_sum > pid_param->error_sum_max) {
        pid_param->error_sum = pid_param->error_sum_max;
    } else if (pid_param->error_sum < pid_param->error_sum_min) {
        pid_param->error_sum = pid_param->error_sum_min;
    }
    
    pid_param->error_last = pid_param->error;
    
    return output_pre_sat;
}

/**
 * @brief 带滤波微分项的PID
 */
float PID_WithFilter(PIDParam_t *pid_param, float feedback, float target, float alpha)
{
    pid_param->error = target - feedback;
    
    // 计算比例项
    float p_term = pid_param->kp * pid_param->error;
    
    // 计算积分项
    pid_param->error_sum += pid_param->error;
    if (pid_param->error_sum > pid_param->error_sum_max) {
        pid_param->error_sum = pid_param->error_sum_max;
    } else if (pid_param->error_sum < pid_param->error_sum_min) {
        pid_param->error_sum = pid_param->error_sum_min;
    }
    float i_term = pid_param->ki * pid_param->error_sum;
    
    // 计算滤波后的微分项
    float derivative = (pid_param->error - pid_param->error_last);
    static float filtered_derivative = 0;
    filtered_derivative = alpha * filtered_derivative + (1 - alpha) * derivative;
    float d_term = pid_param->kd * filtered_derivative;
    
    // 计算输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > pid_param->output_max) {
        output = pid_param->output_max;
    } else if (output < pid_param->output_min) {
        output = pid_param->output_min;
    }
    
    pid_param->error_last = pid_param->error;
    
    return output;
}

/**
 * @brief 重置PID状态
 */
void PID_Reset(PIDParam_t *pid_param)
{
    pid_param->error = 0;
    pid_param->error_last = 0;
    pid_param->error_sum = 0;
}

/**
 * @brief 增量式PID初始化
 */
void IncrementalPID_Init(IncrementalPID_t *pid, float kp, float ki, float kd,
                         float output_max, float output_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_max = output_max;
    pid->output_min = output_min;
    
    for (int i = 0; i < 3; i++) {
        pid->error[i] = 0;
    }
}

/**
 * @brief 增量式PID计算
 */
float IncrementalPID_Calculate(IncrementalPID_t *pid, float feedback, float target)
{
    // 更新误差队列
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = target - feedback;
    
    // 计算增量
    float delta_output = pid->kp * (pid->error[0] - pid->error[1]) +
                        pid->ki * pid->error[0] +
                        pid->kd * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
    
    // 注意：增量式PID需要外部维护输出值
    // 这里只返回增量，需要外部累加并限幅
    
    return delta_output;
}