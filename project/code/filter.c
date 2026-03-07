#include "filter.h"
#include <math.h>

/**
 * @brief 初始化一阶低通滤波器
 * @param filter 滤波器结构体指针
 * @param alpha 滤波系数 (0-1，越小滤波越强)
 */
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL) return;
    
    filter->prev_value = 0.0f;
    filter->alpha = alpha;
    filter->initialized = 0;
}

/**
 * @brief 更新一阶低通滤波器（使用结构体）
 * @param new_value 新输入值
 * @param filter 滤波器结构体指针
 * @return 滤波后的输出值
 */
float LowPassFilter_Update(float new_value, LowPassFilter_t *filter)
{
    if (filter == NULL) return new_value;
    
    float output;
    
    if (!filter->initialized) {
        // 第一次调用，直接使用输入值
        output = new_value;
        filter->initialized = 1;
    } else {
        // 一阶低通滤波：y[n] = α * x[n] + (1-α) * y[n-1]
        output = filter->alpha * new_value + (1.0f - filter->alpha) * filter->prev_value;
    }
    
    filter->prev_value = output;
    return output;
}

/**
 * @brief 一阶低通滤波器（直接参数版本）
 * @param input 输入值
 * @param prev_output 上一次输出值的指针
 * @param alpha 滤波系数 (0-1，越小滤波越强)
 * @return 滤波后的输出值
 */
float LowPassFilter_Process(float input, float *prev_output, float alpha)
{
    static uint8_t initialized = 0;
    float output;
    
    if (!initialized || *prev_output == 0) {
        // 第一次调用
        output = input;
        initialized = 1;
    } else {
        // 一阶低通滤波：y[n] = α * x[n] + (1-α) * y[n-1]
        output = alpha * input + (1.0f - alpha) * (*prev_output);
    }
    
    *prev_output = output;
    return output;
}

/**
 * @brief 重置滤波器状态
 * @param filter 滤波器结构体指针
 */
void LowPassFilter_Reset(LowPassFilter_t *filter)
{
    if (filter == NULL) return;
    
    filter->prev_value = 0.0f;
    filter->initialized = 0;
}

/**
 * @brief 二阶低通滤波器初始化
 * @param filter 二阶滤波器结构体指针
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_freq 采样频率 (Hz)
 */
void LowPassFilter2_Init(LowPassFilter2_t *filter, float cutoff_freq, float sample_freq)
{
    if (filter == NULL) return;
    
    filter->x[0] = filter->x[1] = 0.0f;
    filter->y[0] = filter->y[1] = 0.0f;
    filter->initialized = 0;
    
    // 计算二阶巴特沃斯滤波器系数
    float dt = 1.0f / sample_freq;
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);
    float alpha = dt / (RC + dt);
    
    // 二阶低通滤波器系数（简化版）
    filter->a0 = alpha * alpha;
    filter->a1 = 2.0f * filter->a0;
    filter->a2 = filter->a0;
    filter->b1 = 2.0f * (1.0f - alpha);
    filter->b2 = (1.0f - alpha) * (1.0f - alpha);
}

/**
 * @brief 更新二阶低通滤波器
 * @param new_value 新输入值
 * @param filter 二阶滤波器结构体指针
 * @return 滤波后的输出值
 */
float LowPassFilter2_Update(float new_value, LowPassFilter2_t *filter)
{
    if (filter == NULL) return new_value;
    
    float output;
    
    if (!filter->initialized) {
        // 第一次调用，初始化为输入值
        filter->x[0] = filter->x[1] = new_value;
        filter->y[0] = filter->y[1] = new_value;
        filter->initialized = 1;
        output = new_value;
    } else {
        // 二阶差分方程：y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
        output = filter->a0 * new_value + 
                filter->a1 * filter->x[0] + 
                filter->a2 * filter->x[1] - 
                filter->b1 * filter->y[0] - 
                filter->b2 * filter->y[1];
        
        // 更新历史值
        filter->x[1] = filter->x[0];
        filter->x[0] = new_value;
        filter->y[1] = filter->y[0];
        filter->y[0] = output;
    }
    
    return output;
}

/**
 * @brief 移动平均滤波器初始化
 * @param filter 移动平均滤波器结构体指针
 * @param window_size 窗口大小
 */
void MovingAverage_Init(MovingAverage_t *filter, uint16_t window_size)
{
    if (filter == NULL || window_size == 0 || window_size > MAX_MOVING_AVG_SIZE) return;
    
    filter->window_size = window_size;
    filter->index = 0;
    filter->sum = 0.0f;
    filter->count = 0;
    filter->initialized = 0;
    
    // 初始化缓冲区为0
    for (uint16_t i = 0; i < window_size; i++) {
        filter->buffer[i] = 0.0f;
    }
}

/**
 * @brief 更新移动平均滤波器
 * @param new_value 新输入值
 * @param filter 移动平均滤波器结构体指针
 * @return 滤波后的输出值
 */
float MovingAverage_Update(float new_value, MovingAverage_t *filter)
{
    if (filter == NULL || filter->window_size == 0) return new_value;
    
    // 减去最旧的值，加上最新的值
    filter->sum -= filter->buffer[filter->index];
    filter->sum += new_value;
    filter->buffer[filter->index] = new_value;
    
    // 更新索引
    filter->index = (filter->index + 1) % filter->window_size;
    
    // 更新计数
    if (filter->count < filter->window_size) {
        filter->count++;
    }
    
    filter->initialized = 1;
    
    // 返回平均值
    return filter->sum / filter->count;
}