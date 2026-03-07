#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <math.h>
#include "zf_common_headfile.h"

// Y车模参数
#define WHEEL_RADIUS           0.025f      // 轮子半径 25mm
#define CHASSIS_RADIUS         0.1f        // 车体半径 100mm
#define MAX_WHEEL_SPEED_MPS    2.0f        // 最大轮子线速度 m/s

// 轮子角度（弧度）
#define WHEEL1_ANGLE           0.0f        // 0°
#define WHEEL2_ANGLE           2.094395f   // 120° = 2π/3
#define WHEEL3_ANGLE           4.188790f   // 240° = 4π/3

// 数学常量
#ifndef M_PI
#define M_PI                   3.14159265358979323846f
#endif

#define DEG_TO_RAD             (M_PI / 180.0f)
#define RAD_TO_DEG             (180.0f / M_PI)

// 运动学结构体
typedef struct {
    float vx;                   // 车体X方向速度 (m/s)
    float vy;                   // 车体Y方向速度 (m/s)
    float omega;                // 车体角速度 (rad/s)
    
    float wheel_speed_mps[3];   // 三个轮子的线速度 (m/s)
    float wheel_speed_rpm[3];   // 三个轮子的转速 (RPM)
    
    float position_x;           // X方向位置 (m)
    float position_y;           // Y方向位置 (m)
    float orientation;          // 车体朝向 (rad)
} Kinematics_t;

// 函数声明
void Kinematics_Init(Kinematics_t *kinematics);                     // 初始化运动学
void Kinematics_Inverse(float vx, float vy, float omega, 
                        float wheel_speeds[3]);                     // 逆运动学
void Kinematics_Forward(const float wheel_speeds[3],
                        float *vx, float *vy, float *omega);        // 正运动学
void Kinematics_UpdatePosition(Kinematics_t *kinematics, 
                               float delta_time_ms);                // 更新位置

// 速度限制函数
void Kinematics_LimitWheelSpeeds(float wheel_speeds[3], 
                                 float max_speed_mps);              // 限制轮子速度
void Kinematics_LimitBodySpeeds(float *vx, float *vy, float *omega,
                                float max_linear_speed, 
                                float max_angular_speed);           // 限制车体速度

// 单位转换
void Kinematics_WheelSpeedToRPM(const float wheel_speeds_mps[3],
                                float wheel_speeds_rpm[3]);         // 轮子速度转RPM
float Kinematics_SpeedMPSToRPM(float speed_mps);                    // m/s转RPM
float Kinematics_RPMToSpeedMPS(float rpm);                          // RPM转m/s

#endif /* __KINEMATICS_H__ */