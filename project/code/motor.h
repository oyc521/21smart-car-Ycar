#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"
#include "pid.h"
#include "kinematics.h"

// 电机参数
#define MAX_MOTOR_SPEED        500.0f    // 最大电机速度（编码器单位）
#define MAX_MOTOR_SPEED_MPS    2.0f      // 最大电机速度（m/s）
#define PWM_FREQUENCY          17000     // PWM频率 17kHz

// 电机引脚定义（根据实际接线修改）
#define MOTOR1_PWM_PIN         (PWM2_MODULE1_CHA_C8)
#define MOTOR1_DIR_PIN         C9
#define MOTOR2_PWM_PIN         (PWM2_MODULE0_CHA_C6)
#define MOTOR2_DIR_PIN         C7
#define MOTOR3_PWM_PIN         (PWM2_MODULE3_CHB_D3)
#define MOTOR3_DIR_PIN         D2

// 电机方向定义
#define MOTOR_FORWARD          1
#define MOTOR_BACKWARD         0

// 电机控制器结构体
typedef struct {
    PIDParam_t pid;                 // PID控制器
    float target_speed_mps;         // 目标速度（m/s）
    float current_speed_mps;        // 当前速度（m/s）
    float pwm_duty;                 // PWM占空比（0-100%）
    uint32_t encoder_count;         // 编码器计数
    uint32_t last_encoder_count;    // 上次编码器计数
    uint32_t last_update_time;      // 上次更新时间（ms）
    float speed_filtered;           // 滤波后的速度
    uint8_t motor_id;               // 电机ID（0,1,2）
} MotorController_t;

// 车体控制器结构体
typedef struct {
    MotorController_t motors[3];    // 三个电机控制器
    float target_vx, target_vy, target_omega;  // 车体目标速度
    float current_vx, current_vy, current_omega; // 车体当前速度
    Kinematics_t kinematics;        // 运动学状态
} CarController_t;

// 全局变量声明
extern CarController_t car_ctrl;
extern float pid_kp, pid_ki, pid_kd;

// 调试变量
extern float dbg_enc1, dbg_enc2, dbg_enc3;
extern float dbg_tar1, dbg_tar2, dbg_tar3;
extern uint8_t dbg_new;

// 函数声明
void MotorInit(void);                           // 电机初始化
void MotorController_Init(void);                // 电机控制器初始化
void CarController_Init(void);                  // 车体控制器初始化

// 速度控制函数
void CarController_SetSpeed(float vx, float vy, float omega);  // 设置车体速度
void CarController_Update(void);                // 更新车体控制器（需定期调用）
void CarController_Stop(void);                  // 停止车体
void MotorController_SetSpeed(int motor_id, float speed_mps);  // 设置单个电机速度
//void CarController_Update(void);   // <-- 添加声明
// PID参数设置
void PIDController_SetParams(int motor_id, float kp, float ki, float kd);
void PIDController_Reset(int motor_id);         // 重置PID控制器

// 测试函数
void Test_Square(float side_length, float speed);      // 正方形测试
void Test_Rotation(float angle_deg, float speed_deg_per_sec);  // 旋转测试
void Test_Movement(float vx, float vy, float omega, float duration_ms);  // 运动测试

// 兼容旧接口（暂时保留）
void CarSpeedSet(float speed_x, float speed_y, float speed_z);
void CarStop(void);
void pid_init(float kp, float ki, float kd);

#endif /* __MOTOR_H__ */