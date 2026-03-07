#include "imu660ra_ahrs.h"
#include "filter.h"  // 添加滤波器头文件
#include <math.h>

// 全局变量定义
Attitude_t attitude = {0, 0, 0};
float dt = 0.005f;  // 默认5ms采样周期

// 原始传感器数据（物理单位）
static float acc_x_g = 0, acc_y_g = 0, acc_z_g = 0;      // 加速度 (g)
static float gyro_x_dps = 0, gyro_y_dps = 0, gyro_z_dps = 0; // 角速度 (°/s)

// 零偏校准值
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
static float acc_bias_x = 0, acc_bias_y = 0, acc_bias_z = 0;

// 时间戳相关
static uint32_t last_time_ms = 0;
static uint8_t time_initialized = 0;

// 低通滤波器状态（改为使用滤波器结构体）
static LowPassFilter_t acc_lpf_x_filter = {0, 0.2f, 0};
static LowPassFilter_t acc_lpf_y_filter = {0, 0.2f, 0};
static LowPassFilter_t acc_lpf_z_filter = {0, 0.2f, 0};

/**
 * @brief 获取系统时间（毫秒）
 * @return 系统时间（毫秒）
 * @note RT-Thread版本
 */
uint32_t GetSystemTimeMs(void) {
    // RT-Thread环境下，使用rt_tick_get获取tick数，然后转换为毫秒
    rt_tick_t tick = rt_tick_get();
    
    #if RT_TICK_PER_SECOND == 1000
    return (uint32_t)tick;
    #else
    return (uint32_t)(tick * 1000 / RT_TICK_PER_SECOND);
    #endif
}

/**
 * @brief 初始化IMU660RA AHRS系统
 * @note 包含传感器初始化和校准
 */
void IMU660RA_AHRS_Init(void) {
    uint8_t init_status;
    
    // 1. 初始化IMU660RA传感器
    init_status = imu660ra_init();
    
    if (init_status != 0) {
        // 初始化失败处理
        while (1);
    }
    
    // 2. 等待传感器稳定
    system_delay_ms(50);
    
    // 3. 初始化低通滤波器
    LowPassFilter_Init(&acc_lpf_x_filter, 0.2f);
    LowPassFilter_Init(&acc_lpf_y_filter, 0.2f);
    LowPassFilter_Init(&acc_lpf_z_filter, 0.2f);
    
    // 4. 校准传感器（零偏）
    IMU660RA_Calibrate(100);  // 采集100个样本进行校准
    
    // 5. 初始化时间戳
    last_time_ms = GetSystemTimeMs();
    time_initialized = 1;
}

/**
 * @brief 获取IMU660RA传感器数据（原始数据转物理单位）
 */
void IMU660RA_GetData(void) {
    // 读取原始数据
    imu660ra_get_acc();
    imu660ra_get_gyro();
    
    // 转换为物理单位（根据imu660ra_transition_factor）
    acc_x_g = (float)imu660ra_acc_x / imu660ra_transition_factor[0];
    acc_y_g = (float)imu660ra_acc_y / imu660ra_transition_factor[0];
    acc_z_g = (float)imu660ra_acc_z / imu660ra_transition_factor[0];
    
    // 陀螺仪数据并应用零偏校准
    gyro_x_dps = (float)imu660ra_gyro_x / imu660ra_transition_factor[1] - gyro_bias_x;
    gyro_y_dps = (float)imu660ra_gyro_y / imu660ra_transition_factor[1] - gyro_bias_y;
    gyro_z_dps = (float)imu660ra_gyro_z / imu660ra_transition_factor[1] - gyro_bias_z;
    
    // 加速度计低通滤波（使用新的滤波器接口）
    acc_x_g = LowPassFilter_Update(acc_x_g, &acc_lpf_x_filter);
    acc_y_g = LowPassFilter_Update(acc_y_g, &acc_lpf_y_filter);
    acc_z_g = LowPassFilter_Update(acc_z_g, &acc_lpf_z_filter);
}

/**
 * @brief 传感器校准（计算零偏）
 * @param sample_count 校准采样次数
 */
void IMU660RA_Calibrate(uint16_t sample_count) {
    float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
    float sum_acc_x = 0, sum_acc_y = 0, sum_acc_z = 0;
    
    // 确保传感器静止时进行校准
    for (uint16_t i = 0; i < sample_count; i++) {
        IMU660RA_GetData();
        
        sum_gyro_x += gyro_x_dps;
        sum_gyro_y += gyro_y_dps;
        sum_gyro_z += gyro_z_dps;
        
        // 加速度计校准（假设传感器水平放置，Z轴向上）
        sum_acc_x += acc_x_g;
        sum_acc_y += acc_y_g;
        sum_acc_z += acc_z_g;
        
        system_delay_ms(10);  // 10ms采样间隔
    }
    
    // 计算平均值作为零偏
    gyro_bias_x = sum_gyro_x / sample_count;
    gyro_bias_y = sum_gyro_y / sample_count;
    gyro_bias_z = sum_gyro_z / sample_count;
    
    // 加速度计校准（假设Z轴重力为1g）
    acc_bias_x = sum_acc_x / sample_count;
    acc_bias_y = sum_acc_y / sample_count;
    acc_bias_z = (sum_acc_z / sample_count) - 1.0f;  // 减去重力加速度
}

/**
 * @brief 互补滤波姿态解算
 * @note 使用加速度计校正陀螺仪积分漂移
 */
void AHRS_Update(void) {
    uint32_t current_time_ms;
    
    // 如果没有初始化时间，使用固定时间间隔
    if (!time_initialized) {
        last_time_ms = GetSystemTimeMs();
        time_initialized = 1;
    }
    
    // 1. 获取传感器数据
    IMU660RA_GetData();
    
    // 2. 计算实际时间间隔
    current_time_ms = GetSystemTimeMs();
    
    // 计算时间差（毫秒转秒）
    if (current_time_ms >= last_time_ms) {
        dt = (current_time_ms - last_time_ms) / 1000.0f;
    } else {
        // 处理时间溢出（32位计数器约49.7天溢出一次）
        dt = (0xFFFFFFFF - last_time_ms + current_time_ms) / 1000.0f;
    }
    
    last_time_ms = current_time_ms;
    
    // 限制dt在合理范围内
    if (dt > 0.1f) dt = 0.1f;       // 最大100ms
    if (dt < 0.001f) dt = 0.005f;   // 默认5ms
    
    // 3. 从加速度计计算俯仰角和横滚角
    // 应用加速度计校准
    float ax = acc_x_g - acc_bias_x;
    float ay = acc_y_g - acc_bias_y;
    float az = acc_z_g - acc_bias_z;
    
    // 计算加速度计姿态角（弧度）
    float acc_pitch_rad = atan2(-ax, sqrt(ay*ay + az*az));
    float acc_roll_rad = atan2(ay, az);
    
    // 转换为度
    float acc_pitch = acc_pitch_rad * RAD_TO_DEG;
    float acc_roll = acc_roll_rad * RAD_TO_DEG;
    
    // 4. 从陀螺仪积分得到角度（度）
    float gyro_pitch = attitude.pitch + (gyro_x_dps * DEG_TO_RAD) * dt * RAD_TO_DEG;
    float gyro_roll = attitude.roll + (gyro_y_dps * DEG_TO_RAD) * dt * RAD_TO_DEG;
    float gyro_yaw = attitude.yaw + (gyro_z_dps * DEG_TO_RAD) * dt * RAD_TO_DEG;
    
    // 5. 互补滤波融合
    float alpha = COMPLEMENTARY_FILTER_ALPHA;
    attitude.pitch = alpha * gyro_pitch + (1 - alpha) * acc_pitch;
    attitude.roll = alpha * gyro_roll + (1 - alpha) * acc_roll;
    attitude.yaw = gyro_yaw;  // 仅陀螺仪积分
    
    // 6. 角度范围限制到[-180, 180]度
    if (attitude.pitch > 180.0f) attitude.pitch -= 360.0f;
    else if (attitude.pitch < -180.0f) attitude.pitch += 360.0f;
    
    if (attitude.roll > 180.0f) attitude.roll -= 360.0f;
    else if (attitude.roll < -180.0f) attitude.roll += 360.0f;
    
    if (attitude.yaw > 180.0f) attitude.yaw -= 360.0f;
    else if (attitude.yaw < -180.0f) attitude.yaw += 360.0f;
}

/**
 * @brief 获取俯仰角
 * @return 俯仰角 (度)
 */
float AHRS_GetPitch(void) {
    return attitude.pitch;
}

/**
 * @brief 获取横滚角
 * @return 横滚角 (度)
 */
float AHRS_GetRoll(void) {
    return attitude.roll;
}

/**
 * @brief 获取偏航角
 * @return 偏航角 (度)
 */
float AHRS_GetYaw(void) {
    return attitude.yaw;
}

/**
 * @brief 重置偏航角（置零）
 */
void AHRS_ResetYaw(void) {
    attitude.yaw = 0.0f;
}

/**
 * @brief 设置固定采样时间
 * @param sampling_time 采样时间 (秒)
 */
void AHRS_SetSamplingTime(float sampling_time) {
    dt = sampling_time;
}

// 以下函数是为了兼容原ICM42688接口而保留（可选）
void InitICM42688(void) {
    IMU660RA_AHRS_Init();
}

void Get_Acc_ICM42688(void) {
    IMU660RA_GetData();
}

void Get_Gyro_ICM42688(void) {
    IMU660RA_GetData();
}

// 新增加的函数，用于获取原始物理单位数据
void IMU660RA_GetRawData(float *acc, float *gyro) {
    IMU660RA_GetData();
    
    if (acc != NULL) {
        acc[0] = acc_x_g;
        acc[1] = acc_y_g;
        acc[2] = acc_z_g;
    }
    
    if (gyro != NULL) {
        gyro[0] = gyro_x_dps;
        gyro[1] = gyro_y_dps;
        gyro[2] = gyro_z_dps;
    }
}