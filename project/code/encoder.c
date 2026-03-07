#include "encoder.h"
#include <rtthread.h>
#include <math.h>

// 全局变量
float enc1 = 0, enc2 = 0, enc3 = 0;
float wheel_speed_mps[3] = {0, 0, 0};
volatile uint8_t encoder_updated_flag = 0;

// 内部状态
static int16_t last_raw[3] = {0, 0, 0};
static uint32_t last_update_time = 0;
static const uint32_t update_interval_ms = 10;   // 10ms更新

// 滤波器实例
static LowPassFilter_t speed_lpf[3];

// 编码器初始化
void EncoderInit(void)
{
    // 初始化硬件
    encoder_quad_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR);
    encoder_quad_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR);
    encoder_quad_init(ENCODER_3, ENCODER_3_LSB, ENCODER_3_DIR);

    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
    encoder_clear_count(ENCODER_3);

    // 初始化 last_raw 为当前值，避免第一次跳变
    last_raw[0] = encoder_get_count(ENCODER_1);
    last_raw[1] = encoder_get_count(ENCODER_2);
    last_raw[2] = encoder_get_count(ENCODER_3);
    last_update_time = rt_tick_get();

    // 初始化滤波器
    for (int i = 0; i < 3; i++) {
        LowPassFilter_Init(&speed_lpf[i], ENCODER_LPF_ALPHA);
    }

    rt_kprintf("Encoder Init OK\n");
}

// 编码器更新（每10ms在PIT中断中调用）
void EncoderUpdate(void)
{
    uint32_t now = rt_tick_get();
    uint32_t delta_ms = now - last_update_time;
    if (delta_ms < update_interval_ms) return;
    last_update_time = now;

    // 读取当前计数值
    int16_t raw[3] = {
        encoder_get_count(ENCODER_1),
        encoder_get_count(ENCODER_2),
        encoder_get_count(ENCODER_3)
    };

    // 计算增量，处理16位溢出
    int16_t delta[3];
    for (int i = 0; i < 3; i++) {
        delta[i] = raw[i] - last_raw[i];
        if (delta[i] > 16384) delta[i] -= 32768;
        else if (delta[i] < -16384) delta[i] += 32768;
        last_raw[i] = raw[i];
    }

    float dt = delta_ms / 1000.0f;   // 秒

    // 保存原始计数值（供外部读取）
    enc1 = raw[0];
    enc2 = raw[1];
    enc3 = raw[2];

    // 计算速度（脉冲/秒）
    float raw_speed_pps[3];
    for (int i = 0; i < 3; i++) {
        raw_speed_pps[i] = delta[i] / dt;
    }

    // 转换为 m/s
    float pps_to_mps = WHEEL_CIRCUMFERENCE / ENCODER_PPR;
    for (int i = 0; i < 3; i++) {
        raw_speed_pps[i] *= pps_to_mps;
    }

    // 低通滤波
    for (int i = 0; i < 3; i++) {
        wheel_speed_mps[i] = LowPassFilter_Update(raw_speed_pps[i], &speed_lpf[i]);
    }

    // 置位更新标志
    encoder_updated_flag = 1;
}

// 其他接口函数
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
    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
    encoder_clear_count(ENCODER_3);
    last_raw[0] = last_raw[1] = last_raw[2] = 0;
    enc1 = enc2 = enc3 = 0;
    for (int i = 0; i < 3; i++) {
        wheel_speed_mps[i] = 0;
    }
    rt_kprintf("Encoder Reset\n");
}

// PIT中断处理函数（在isr.c中调用）
void pit_handler(void)
{
    EncoderUpdate();
}
#ifdef DEBUG
#include "zf_device_wireless_uart.h"

void encoder_send_thread_entry(void *parameter)
{
    //wireless_uart_init(); // 如果已在主函数初始化，可注释掉
    while (1)
    {
        if (encoder_updated_flag)
        {
            encoder_updated_flag = 0;
            float speeds[3];
            EncoderGetSpeeds(speeds);
            int speed_int[3];
            for (int i = 0; i < 3; i++) {
                speed_int[i] = (int)(speeds[i] * 1000.0f);
            }
            char buf[64];
            rt_sprintf(buf, "SPD:%d,%d,%d\r\n",
                       speed_int[0], speed_int[1], speed_int[2]);
            wireless_uart_send_string(buf);
        }
        rt_thread_mdelay(5);
    }
}
#endif