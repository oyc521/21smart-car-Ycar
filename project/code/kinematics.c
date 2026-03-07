#include "kinematics.h"
#include <math.h>

// 轮子安装角度：0°, 120°, 240°（相对于车头方向，即x轴正方向）
static const float sin_0   = 0.0f;
static const float cos_0   = 1.0f;
static const float sin_120 = 0.8660254f;   
static const float cos_120 = -0.5f;
static const float sin_240 = -0.8660254f;
static const float cos_240 = -0.5f;

void Kinematics_Init(Kinematics_t *kinematics)
{
    kinematics->vx = 0.0f;
    kinematics->vy = 0.0f;
    kinematics->omega = 0.0f;
    for (int i = 0; i < 3; i++) {
        kinematics->wheel_speed_mps[i] = 0.0f;
        kinematics->wheel_speed_rpm[i] = 0.0f;
    }
    kinematics->position_x = 0.0f;
    kinematics->position_y = 0.0f;
    kinematics->orientation = 0.0f;
}

/**
 * @brief 逆运动学：车体速度 -> 轮子线速度 (m/s)
 */
void Kinematics_Inverse(float vx, float vy, float omega, float wheel_speeds[3])
{
    float R_omega = CHASSIS_RADIUS * omega;  // 旋转引起的线速度分量
    // 轮子线速度 = (vx,vy) 在轮子滚动方向上的投影 + Rω
    wheel_speeds[0] = -sin_0 * vx + cos_0 * vy + R_omega;   // = vy + Rω
    wheel_speeds[1] = -sin_120 * vx + cos_120 * vy + R_omega; // = -0.866vx -0.5vy + Rω
    wheel_speeds[2] = -sin_240 * vx + cos_240 * vy + R_omega; // = 0.866vx -0.5vy + Rω
}

/**
 * @brief 正运动学：轮子线速度 (m/s) -> 车体速度
 */
void Kinematics_Forward(const float wheel_speeds[3], float *vx, float *vy, float *omega)
{
    float v0 = wheel_speeds[0];
    float v1 = wheel_speeds[1];
    float v2 = wheel_speeds[2];
    
    // 根据最小二乘推导的公式
    *vx = ( -sin_0 * v0 - sin_120 * v1 - sin_240 * v2 ) / 3.0f;
    *vy = ( cos_0 * v0 + cos_120 * v1 + cos_240 * v2 ) / 3.0f;
    *omega = ( v0 + v1 + v2 ) / (3.0f * CHASSIS_RADIUS);
    
    // 展开后便于理解：
    // *vx = (0.866f * (v2 - v1)) / 3.0f = 0.288675f * (v2 - v1);
    // *vy = (v0 - 0.5f * v1 - 0.5f * v2) / 3.0f;
    // *omega = (v0 + v1 + v2) / (3.0f * CHASSIS_RADIUS);
}

/**
 * @brief 限制轮子速度
 */
void Kinematics_LimitWheelSpeeds(float wheel_speeds[3], float max_speed_mps)
{
    float max_abs = 0.0f;
    for (int i = 0; i < 3; i++) {
        float abs_val = fabsf(wheel_speeds[i]);
        if (abs_val > max_abs) max_abs = abs_val;
    }
    if (max_abs > max_speed_mps) {
        float scale = max_speed_mps / max_abs;
        for (int i = 0; i < 3; i++) {
            wheel_speeds[i] *= scale;
        }
    }
}

/**
 * @brief 更新位置（积分）
 */
void Kinematics_UpdatePosition(Kinematics_t *kinematics, float delta_time_ms)
{
    if (delta_time_ms <= 0) return;
    float dt = delta_time_ms / 1000.0f;
    
    kinematics->position_x += kinematics->vx * dt;
    kinematics->position_y += kinematics->vy * dt;
    kinematics->orientation += kinematics->omega * dt;
    
    // 角度归一化到 [-π, π]
    if (kinematics->orientation > M_PI)
        kinematics->orientation -= 2.0f * M_PI;
    else if (kinematics->orientation < -M_PI)
        kinematics->orientation += 2.0f * M_PI;
}

float Kinematics_SpeedMPSToRPM(float speed_mps)
{
    return speed_mps / WHEEL_RADIUS * 60.0f / (2.0f * M_PI);
}

float Kinematics_RPMToSpeedMPS(float rpm)
{
    return rpm * (2.0f * M_PI / 60.0f) * WHEEL_RADIUS;
}

void Kinematics_WheelSpeedToRPM(const float wheel_speeds_mps[3], float wheel_speeds_rpm[3])
{
    for (int i = 0; i < 3; i++) {
        wheel_speeds_rpm[i] = Kinematics_SpeedMPSToRPM(wheel_speeds_mps[i]);
    }
}