#ifndef __IMU660RA_AHRS_H__
#define __IMU660RA_AHRS_H__

#include "zf_common_headfile.h"
#include "zf_device_imu660ra.h"
#include "filter.h"  // 添加滤波器头文件
#include <rtthread.h>

// 姿态角结构体
typedef struct {
    float pitch;        // 俯仰角 (°)
    float roll;         // 横滚角 (°)
    float yaw;          // 偏航角 (°)
} Attitude_t;

// 传感器数据单位定义
#define GRAVITY         9.80665f     // 重力加速度 m/s2
#define DEG_TO_RAD      (M_PI / 180.0f)  // 度转弧度
#define RAD_TO_DEG      (180.0f / M_PI)  // 弧度转度

// 互补滤波系数
#ifndef COMPLEMENTARY_FILTER_ALPHA
#define COMPLEMENTARY_FILTER_ALPHA  0.98f   // 互补滤波系数 (0-1)
#endif

// 全局变量声明
extern Attitude_t attitude;
extern float dt;        // 采样时间间隔 (s)

// 函数声明
void IMU660RA_AHRS_Init(void);
void IMU660RA_GetData(void);
void IMU660RA_Calibrate(uint16_t sample_count);
void AHRS_Update(void);
float AHRS_GetPitch(void);
float AHRS_GetRoll(void);
float AHRS_GetYaw(void);
void AHRS_ResetYaw(void);
float AHRS_GetGyroZ(void);
void AHRS_SetSamplingTime(float sampling_time);
void IMU660RA_GetRawData(float *acc, float *gyro);
// RT-Thread时间获取函数
uint32_t GetSystemTimeMs(void);

// 兼容原ICM42688接口
void InitICM42688(void);
void Get_Acc_ICM42688(void);
void Get_Gyro_ICM42688(void);

#endif /* __IMU660RA_AHRS_H__ */