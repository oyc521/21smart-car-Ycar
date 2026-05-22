#include "encoder.h"
#include <rtthread.h>
#include <math.h>

// 全局变量
float enc1 = 0, enc2 = 0, enc3 = 0;
float wheel_speed_mps[3] = {0, 0, 0};     // 保留，兼容旧接口
volatile uint8_t encoder_updated_flag = 0;

// 内部状态
static int16_t last_raw[3] = {0, 0, 0};
static uint32_t last_update_time = 0;
static const uint32_t update_interval_ms = 10;   // 10ms更新

// 滤波器实例（保留，供旧接口使用）
static LowPassFilter_t speed_lpf[3];

// 新增：保存最近一次脉冲增量
static int16_t g_delta_pulses[3] = {0, 0, 0};

// 编码器初始化
void EncoderInit(void)
{
    encoder_quad_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR);
    encoder_quad_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR);
    encoder_quad_init(ENCODER_3, ENCODER_3_LSB, ENCODER_3_DIR);

    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
    encoder_clear_count(ENCODER_3);

    last_raw[0] = encoder_get_count(ENCODER_1);
    last_raw[1] = encoder_get_count(ENCODER_2);
    last_raw[2] = encoder_get_count(ENCODER_3);
    last_update_time = rt_tick_get();

    for (int i = 0; i < 3; i++) {
        LowPassFilter_Init(&speed_lpf[i], ENCODER_LPF_ALPHA);
    }
}

// 编码器更新（每10ms在PIT中断中调用）
void EncoderUpdate(void)
{
    uint32_t now = rt_tick_get();
    uint32_t delta_ms = now - last_update_time;
    if (delta_ms < update_interval_ms) return;
    last_update_time = now;

    int16_t raw[3] = {
        encoder_get_count(ENCODER_1),
        encoder_get_count(ENCODER_2),
        encoder_get_count(ENCODER_3)
    };

    int16_t delta[3];
    for (int i = 0; i < 3; i++) {
        delta[i] = raw[i] - last_raw[i];
        if (delta[i] > 16384) delta[i] -= 32768;
        else if (delta[i] < -16384) delta[i] += 32768;
        last_raw[i] = raw[i];
    }

    // 保存原始脉冲增量，供新接口使用
    for (int i = 0; i < 3; i++) {
        g_delta_pulses[i] = delta[i];
    }

    float dt = delta_ms / 1000.0f;

    enc1 = raw[0];
    enc2 = raw[1];
    enc3 = raw[2];

    // 旧速度计算（保留兼容）
    float raw_speed_pps[3];
    for (int i = 0; i < 3; i++) {
        raw_speed_pps[i] = delta[i] / dt;
    }

    float pps_to_mps = WHEEL_CIRCUMFERENCE / (ENCODER_PPR * ENCODER_GEAR_RATIO);
    for (int i = 0; i < 3; i++) {
        raw_speed_pps[i] *= pps_to_mps;
    }

    for (int i = 0; i < 3; i++) {
        wheel_speed_mps[i] = LowPassFilter_Update(raw_speed_pps[i], &speed_lpf[i]);
    }

    encoder_updated_flag = 1;
}

// 新增：获取脉冲增量（线程安全）
void EncoderGetDeltas(int16_t deltas[3])
{
    rt_enter_critical();
    deltas[0] = g_delta_pulses[0];
    deltas[1] = g_delta_pulses[1];
    deltas[2] = g_delta_pulses[2];
    rt_exit_critical();
}

// 旧接口保留
void EncoderGetCounts(float counts[3])
{
    counts[0] = enc1;
    counts[1] = enc2;
    counts[2] = enc3;
}

void EncoderGetSpeeds(float speeds_mps[3])
{
    speeds_mps[0] = wheel_speed_mps[0];
    speeds_mps[1] = wheel_speed_mps[1];
    speeds_mps[2] = wheel_speed_mps[2];
}

void EncoderReset(void)
{
    rt_enter_critical();

    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
    encoder_clear_count(ENCODER_3);
    last_raw[0] = last_raw[1] = last_raw[2] = 0;
    enc1 = enc2 = enc3 = 0;
    g_delta_pulses[0] = g_delta_pulses[1] = g_delta_pulses[2] = 0;
    for (int i = 0; i < 3; i++) {
        wheel_speed_mps[i] = 0;
        LowPassFilter_Init(&speed_lpf[i], ENCODER_LPF_ALPHA);
    }
    last_update_time = rt_tick_get();
    encoder_updated_flag = 0;

    rt_exit_critical();
}

void pit_handler(void)
{
    EncoderUpdate();
}