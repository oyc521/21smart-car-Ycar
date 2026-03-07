#ifndef __FILTER_H__
#define __FILTER_H__

#include "zf_common_headfile.h"

// 滤波器类型定义
#define MAX_MOVING_AVG_SIZE 50  // 移动平均最大窗口大小

// 数学常量定义（如果math.h中没有）
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 一阶低通滤波器结构体
typedef struct {
    float prev_value;    // 上一次输出值
    float alpha;         // 滤波系数 (0-1，越小滤波越强)
    uint8_t initialized; // 初始化标志
} LowPassFilter_t;

// 二阶低通滤波器结构体
typedef struct {
    float x[2];          // 输入历史值 [n-1, n-2]
    float y[2];          // 输出历史值 [n-1, n-2]
    float a0, a1, a2;    // 输入系数
    float b1, b2;        // 输出系数
    uint8_t initialized; // 初始化标志
} LowPassFilter2_t;

// 移动平均滤波器结构体
typedef struct {
    float buffer[MAX_MOVING_AVG_SIZE]; // 数据缓冲区
    uint16_t window_size;               // 窗口大小
    uint16_t index;                     // 当前索引
    float sum;                          // 总和
    uint16_t count;                     // 有效数据计数
    uint8_t initialized;                // 初始化标志
} MovingAverage_t;

// 滤波器接口函数声明

// ==================== 一阶低通滤波器 ====================
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha);
float LowPassFilter_Update(float new_value, LowPassFilter_t *filter);
float LowPassFilter_Process(float input, float *prev_output, float alpha);
void LowPassFilter_Reset(LowPassFilter_t *filter);

// ==================== 二阶低通滤波器 ====================
void LowPassFilter2_Init(LowPassFilter2_t *filter, float cutoff_freq, float sample_freq);
float LowPassFilter2_Update(float new_value, LowPassFilter2_t *filter);

// ==================== 移动平均滤波器 ====================
void MovingAverage_Init(MovingAverage_t *filter, uint16_t window_size);
float MovingAverage_Update(float new_value, MovingAverage_t *filter);

// ==================== 辅助函数 ====================
static inline float constrain_float(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static inline float clamp_alpha(float alpha) {
    return constrain_float(alpha, 0.001f, 0.999f);
}

#endif /* __FILTER_H__ */