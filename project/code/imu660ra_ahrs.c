#include "imu660ra_ahrs.h"
#include "filter.h"
#include <math.h>

// 全局变量
Attitude_t attitude = {0, 0, 0};
float dt = 0.005f;

// 原始传感器数据（物理单位，未减零偏）
static float acc_x_g = 0, acc_y_g = 0, acc_z_g = 0;
static float gyro_x_dps = 0, gyro_y_dps = 0, gyro_z_dps = 0;

// 零偏校准值
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
static float acc_bias_x = 0, acc_bias_y = 0, acc_bias_z = 0;

// 时间戳
static uint32_t last_time_ms = 0;
static uint8_t time_initialized = 0;

// 低通滤波器
static LowPassFilter_t acc_lpf_x_filter, acc_lpf_y_filter, acc_lpf_z_filter;

// 诊断输出宏（可通过无线模块打印调试信息）
#define AHRS_DEBUG 0   // 设置为1可输出诊断数据，调试完成后可关闭
#if AHRS_DEBUG
    #include "zf_device_wireless_uart.h"
    #define AHRS_PRINT(fmt, ...) do { \
        char dbg_buf[128]; \
        int len = sprintf(dbg_buf, fmt, ##__VA_ARGS__); \
        wireless_uart_send_string(dbg_buf); \
    } while(0)
#else
    #define AHRS_PRINT(...)
#endif

uint32_t GetSystemTimeMs(void)
{
    rt_tick_t tick = rt_tick_get();
#if RT_TICK_PER_SECOND == 1000
    return (uint32_t)tick;
#else
    return (uint32_t)(tick * 1000 / RT_TICK_PER_SECOND);
#endif
}

/**
 * @brief 陀螺仪零偏校准（增强版）
 * @param sample_count 采样次数（建议500以上）
 */
void IMU660RA_Calibrate(uint16_t sample_count)
{
    int32_t sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;

    //AHRS_PRINT("Calibrating gyro, keep sensor still...\r\n");
    rt_thread_mdelay(400);  // 等待传感器稳定

    for (uint16_t i = 0; i < sample_count; i++)
    {
        imu660ra_get_gyro();
        sum_gyro_x += imu660ra_gyro_x;
        sum_gyro_y += imu660ra_gyro_y;
        sum_gyro_z += imu660ra_gyro_z;
        rt_thread_mdelay(5);  // 5ms间隔，提高采样密度
    }

    gyro_bias_x = (float)sum_gyro_x / sample_count / imu660ra_transition_factor[1];
    gyro_bias_y = (float)sum_gyro_y / sample_count / imu660ra_transition_factor[1];
    gyro_bias_z = (float)sum_gyro_z / sample_count / imu660ra_transition_factor[1];

    // 加速度零偏置0（如需可后续手动设置）
    acc_bias_x = acc_bias_y = acc_bias_z = 0.0f;

   // AHRS_PRINT("gyro_bias: %.4f %.4f %.4f\r\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

void IMU660RA_GetData(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();

    acc_x_g = (float)imu660ra_acc_x / imu660ra_transition_factor[0];
    acc_y_g = (float)imu660ra_acc_y / imu660ra_transition_factor[0];
    acc_z_g = (float)imu660ra_acc_z / imu660ra_transition_factor[0];
    gyro_x_dps = (float)imu660ra_gyro_x / imu660ra_transition_factor[1];
    gyro_y_dps = (float)imu660ra_gyro_y / imu660ra_transition_factor[1];
    gyro_z_dps = (float)imu660ra_gyro_z / imu660ra_transition_factor[1];
}

void IMU660RA_AHRS_Init(void)
{
    if (imu660ra_init() != 0)
    {
        while (1);  // 初始化失败
    }
    rt_thread_mdelay(50);

    LowPassFilter_Init(&acc_lpf_x_filter, 0.2f);
    LowPassFilter_Init(&acc_lpf_y_filter, 0.2f);
    LowPassFilter_Init(&acc_lpf_z_filter, 0.2f);

    // 高精度校准
    IMU660RA_Calibrate(200);  // 200次采样

    last_time_ms = GetSystemTimeMs();
    time_initialized = 1;
}

void AHRS_Update(void)
{
    uint32_t current_time_ms = GetSystemTimeMs();

    if (time_initialized)
    {
        if (current_time_ms >= last_time_ms)
            dt = (current_time_ms - last_time_ms) / 1000.0f;
        else
            dt = (0xFFFFFFFF - last_time_ms + current_time_ms) / 1000.0f;
        last_time_ms = current_time_ms;
        if (dt > 0.1f) dt = 0.1f;
        if (dt < 0.001f) dt = 0.005f;
    }
    else
    {
        last_time_ms = current_time_ms;
        time_initialized = 1;
        return;
    }

    IMU660RA_GetData();

    // 扣除陀螺仪零偏
    float gx = gyro_x_dps - gyro_bias_x;
    float gy = gyro_y_dps - gyro_bias_y;
    float gz = gyro_z_dps - gyro_bias_z;

    // 加速度计数据（未减零偏）
    float ax = acc_x_g;
    float ay = acc_y_g;
    float az = acc_z_g;

    // 低通滤波
    ax = LowPassFilter_Update(ax, &acc_lpf_x_filter);
    ay = LowPassFilter_Update(ay, &acc_lpf_y_filter);
    az = LowPassFilter_Update(az, &acc_lpf_z_filter);

    // 加速度计角度（Z轴向上，X向前，Y向右）
    float acc_pitch_rad = atan2f(ax, az);
    float acc_roll_rad  = atan2f(ay, az);
    float acc_pitch = acc_pitch_rad * RAD_TO_DEG;
    float acc_roll  = acc_roll_rad * RAD_TO_DEG;

    // 陀螺仪积分
    float gyro_pitch = attitude.pitch + gy * dt;
    float gyro_roll  = attitude.roll  + gx * dt;
    float gyro_yaw   = attitude.yaw   + gz * dt;

    // 互补滤波融合
    float alpha = COMPLEMENTARY_FILTER_ALPHA;
    attitude.pitch = alpha * gyro_pitch + (1 - alpha) * acc_pitch;
    attitude.roll  = alpha * gyro_roll  + (1 - alpha) * acc_roll;
    attitude.yaw   = gyro_yaw;

    // 角度范围限制
    if (attitude.pitch > 180.0f) attitude.pitch -= 360.0f;
    else if (attitude.pitch < -180.0f) attitude.pitch += 360.0f;
    if (attitude.roll > 180.0f) attitude.roll -= 360.0f;
    else if (attitude.roll < -180.0f) attitude.roll += 360.0f;
    if (attitude.yaw > 180.0f) attitude.yaw -= 360.0f;
    else if (attitude.yaw < -180.0f) attitude.yaw += 360.0f;

    // 诊断输出（每秒约20次，可选择只输出部分信息）
    static uint32_t last_print = 0;
    if (AHRS_DEBUG && (current_time_ms - last_print > 500)) // 每500ms输出一次
    {
        AHRS_PRINT("acc_angle: %.2f %.2f | gyro_raw: %.2f %.2f %.2f\r\n",
                   acc_pitch, acc_roll,
                   gx, gy, gz);
        last_print = current_time_ms;
    }
}

/**
 * @brief 获取俯仰角
 * @return 俯仰角 (度)
 */
float AHRS_GetPitch(void)
{
    return attitude.pitch;
}

/**
 * @brief 获取横滚角
 * @return 横滚角 (度)
 */
float AHRS_GetRoll(void)
{
    return attitude.roll;
}

/**
 * @brief 获取偏航角
 * @return 偏航角 (度)
 */
float AHRS_GetYaw(void)
{
    return attitude.yaw;
}

/**
 * @brief 重置偏航角（置零）
 */
void AHRS_ResetYaw(void)
{
    attitude.yaw = 0.0f;
}

/**
 * @brief 设置固定采样时间
 * @param sampling_time 采样时间 (秒)
 */
void AHRS_SetSamplingTime(float sampling_time)
{
    dt = sampling_time;
}

// 以下函数是为了兼容原ICM42688接口而保留（可选）
void InitICM42688(void)
{
    IMU660RA_AHRS_Init();
}

void Get_Acc_ICM42688(void)
{
    IMU660RA_GetData();
}

void Get_Gyro_ICM42688(void)
{
    IMU660RA_GetData();
}

// 获取原始物理单位数据（未减零偏）
void IMU660RA_GetRawData(float *acc, float *gyro)
{
    IMU660RA_GetData();

    if (acc != NULL)
    {
        acc[0] = acc_x_g;
        acc[1] = acc_y_g;
        acc[2] = acc_z_g;
    }

    if (gyro != NULL)
    {
        gyro[0] = gyro_x_dps;
        gyro[1] = gyro_y_dps;
        gyro[2] = gyro_z_dps;
    }
}