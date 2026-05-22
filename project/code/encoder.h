#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "zf_common_headfile.h"
#include "filter.h"

// 编码器宏定义
#define ENCODER_1                       (QTIMER1_ENCODER1)
#define ENCODER_1_LSB                   (QTIMER1_ENCODER1_CH1_C0)  // A相
#define ENCODER_1_DIR                   (QTIMER1_ENCODER1_CH2_C1)  // B相

#define ENCODER_2                       (QTIMER1_ENCODER2)
#define ENCODER_2_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_DIR                   (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_3                       (QTIMER2_ENCODER1)
#define ENCODER_3_LSB                   (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_DIR                   (QTIMER2_ENCODER1_CH2_C4)

// 编码器参数
#define ENCODER_PPR         1000        // 编码器每转脉冲数
#define WHEEL_RADIUS        0.025f      // 轮子半径 25mm
#define ENCODER_GEAR_RATIO  30.0f       // 电机减速比
#define WHEEL_CIRCUMFERENCE (2.0f * M_PI * WHEEL_RADIUS) // 周长

// 滤波选择
#define ENCODER_FILTER_TYPE 1           // 1-一阶低通，0-无滤波
#define ENCODER_LPF_ALPHA   0.3f        // 低通滤波系数

#ifdef DEBUG
void encoder_send_thread_entry(void *parameter);
#endif
// 编码器数据
extern float enc1, enc2, enc3;           // 当前编码器值（脉冲数）
extern float wheel_speed_mps[3];         // 轮子速度（m/s，滤波后）
extern volatile uint8_t encoder_updated_flag; // 数据更新标志

// 函数声明
void EncoderInit(void);
void EncoderUpdate(void);
void EncoderReset(void);
void EncoderGetCounts(float counts[3]);
void EncoderGetSpeeds(float speeds_mps[3]);
void EncoderGetDeltas(int16_t deltas[3]);

#endif