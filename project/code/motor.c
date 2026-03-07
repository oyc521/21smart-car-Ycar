#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
#include <rtthread.h>
#include <math.h>

// 全局车体控制器
CarController_t car_ctrl;

// PID参数
float pid_kp = 80.0f, pid_ki = 2.0f, pid_kd = 5.0f;

// 调试变量
float dbg_enc1 = 0, dbg_enc2 = 0, dbg_enc3 = 0;
float dbg_tar1 = 0, dbg_tar2 = 0, dbg_tar3 = 0;
uint8_t dbg_new = 0;

// 私有函数声明
static void SetMotorPWM(int motor_id, float pwm_duty);
static void UpdateMotorSpeeds(void);
static void ApplySpeedLimits(float wheel_speeds[3]);

/**
 * @brief 电机初始化
 */
void MotorInit(void)
{
    // 初始化PWM
    pwm_init(MOTOR1_PWM_PIN, PWM_FREQUENCY, 0);
    pwm_init(MOTOR2_PWM_PIN, PWM_FREQUENCY, 0);
    pwm_init(MOTOR3_PWM_PIN, PWM_FREQUENCY, 0);
    
    // 初始化方向引脚
    gpio_init(MOTOR1_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR2_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR3_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    printf("Motor Hardware Initialized\n");
}

/**
 * @brief 电机控制器初始化
 */
void MotorController_Init(void)
{
    // 初始化每个电机的PID控制器
    for (int i = 0; i < 3; i++) {
        PIDInit(&car_ctrl.motors[i].pid, pid_kp, pid_ki, pid_kd, 100.0f, -100.0f);
        car_ctrl.motors[i].target_speed_mps = 0.0f;
        car_ctrl.motors[i].current_speed_mps = 0.0f;
        car_ctrl.motors[i].pwm_duty = 0.0f;
        car_ctrl.motors[i].encoder_count = 0;
        car_ctrl.motors[i].last_encoder_count = 0;
        car_ctrl.motors[i].speed_filtered = 0.0f;
        car_ctrl.motors[i].motor_id = i;
        car_ctrl.motors[i].last_update_time = rt_tick_get();
    }
    
    // 初始化运动学
    Kinematics_Init(&car_ctrl.kinematics);
    
    // 初始化目标速度
    car_ctrl.target_vx = 0.0f;
    car_ctrl.target_vy = 0.0f;
    car_ctrl.target_omega = 0.0f;
    
    car_ctrl.current_vx = 0.0f;
    car_ctrl.current_vy = 0.0f;
    car_ctrl.current_omega = 0.0f;
    
    printf("Motor Controller Initialized\n");
}

/**
 * @brief 车体控制器初始化
 */
void CarController_Init(void)
{
    // 初始化硬件
    MotorInit();
    
    // 初始化控制器
    MotorController_Init();
    
    printf("Car Controller Initialized\n");
}

/**
 * @brief 设置车体速度
 */
void CarController_SetSpeed(float vx, float vy, float omega)
{
    car_ctrl.target_vx = vx;
    car_ctrl.target_vy = vy;
    car_ctrl.target_omega = omega;
    
    float wheel_speeds[3];
    Kinematics_Inverse(vx, vy, omega, wheel_speeds);
    Kinematics_LimitWheelSpeeds(wheel_speeds, MAX_WHEEL_SPEED_MPS);
    
    for (int i = 0; i < 3; i++) {
        car_ctrl.motors[i].target_speed_mps = wheel_speeds[i];
    }
}

/**
 * @brief 设置单个电机速度
 */
void MotorController_SetSpeed(int motor_id, float speed_mps)
{
    if (motor_id < 0 || motor_id > 2) {
        return;
    }
    
    car_ctrl.motors[motor_id].target_speed_mps = speed_mps;
}

/**
 * @brief 更新车体控制器
 */
static void UpdateMotorSpeeds(void)
{
    float current_speeds[3];
    EncoderGetSpeeds(current_speeds);  // 获取当前轮子线速度 (m/s)
    
    for (int i = 0; i < 3; i++) {
        car_ctrl.motors[i].current_speed_mps = current_speeds[i];
        // 低通滤波
        float alpha = 0.3f;
        car_ctrl.motors[i].speed_filtered = alpha * current_speeds[i] + 
                                           (1-alpha) * car_ctrl.motors[i].speed_filtered;
    }
    
    // 通过正运动学计算车体速度
    float filtered[3] = {
        car_ctrl.motors[0].speed_filtered,
        car_ctrl.motors[1].speed_filtered,
        car_ctrl.motors[2].speed_filtered
    };
    Kinematics_Forward(filtered, 
                      &car_ctrl.current_vx,
                      &car_ctrl.current_vy,
                      &car_ctrl.current_omega);
}

/**
 * @brief 停止车体
 */
void CarController_Stop(void)
{
    // 设置目标速度为0
    CarController_SetSpeed(0, 0, 0);
    
    // 等待速度降到接近0
    float speed_threshold = 0.05f; // 5cm/s
    while (fabsf(car_ctrl.current_vx) > speed_threshold ||
           fabsf(car_ctrl.current_vy) > speed_threshold ||
           fabsf(car_ctrl.current_omega) > speed_threshold * 2) {
        CarController_Update();
        rt_thread_mdelay(10);
    }
    
    printf("Car Stopped\n");
}

/**
 * @brief 设置电机PWM
 */
static void SetMotorPWM(int motor_id, float pwm_duty)
{
    uint16_t pwm_pin;
    uint8_t dir_pin;
    
    // 选择对应的引脚
    switch (motor_id) {
        case 0:
            pwm_pin = MOTOR1_PWM_PIN;
            dir_pin = MOTOR1_DIR_PIN;
            break;
        case 1:
            pwm_pin = MOTOR2_PWM_PIN;
            dir_pin = MOTOR2_DIR_PIN;
            break;
        case 2:
            pwm_pin = MOTOR3_PWM_PIN;
            dir_pin = MOTOR3_DIR_PIN;
            break;
        default:
            return;
    }
    
    // 确保PWM占空比在有效范围内
    float duty = pwm_duty;
    if (duty > 100.0f) duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;
    
    // 设置方向和PWM
    if (duty >= 0) {
        gpio_set_level(dir_pin, MOTOR_FORWARD);
        pwm_set_duty(pwm_pin, (uint16_t)duty);
    } else {
        gpio_set_level(dir_pin, MOTOR_BACKWARD);
        pwm_set_duty(pwm_pin, (uint16_t)(-duty));
    }
}
/**
 * @brief 更新电机速度
 */
void CarController_Update(void)
{
    // 更新编码器数据
    EncoderUpdate();
    
    // 更新电机速度（包含正运动学计算）
    UpdateMotorSpeeds();
    
    // PID控制并输出PWM
    for (int i = 0; i < 3; i++) {
        MotorController_t *motor = &car_ctrl.motors[i];
        float pid_output = PID(&motor->pid, motor->speed_filtered, motor->target_speed_mps);
        // 限制输出
        if (pid_output > 100.0f) pid_output = 100.0f;
        if (pid_output < -100.0f) pid_output = -100.0f;
        SetMotorPWM(i, pid_output);
        motor->pwm_duty = pid_output;
    }
    
    // 调试输出等
}
/**
 * @brief 更新电机速度，备用方案
 */
/*static void UpdateMotorSpeeds(void)
{
    // 获取当前编码器速度
    float current_speeds[3];
    EncoderGetSpeeds(current_speeds);
    
    // 更新每个电机的速度
    for (int i = 0; i < 3; i++) {
        MotorController_t *motor = &car_ctrl.motors[i];
        
        // 更新当前速度
        motor->current_speed_mps = current_speeds[i];
        
        // 低通滤波
        float alpha = 0.3f;
        motor->speed_filtered = alpha * motor->current_speed_mps + 
                               (1 - alpha) * motor->speed_filtered;
    }
}*/

/**
 * @brief 应用速度限制
 */
static void ApplySpeedLimits(float wheel_speeds[3])
{
    // 使用运动学函数限制速度
    Kinematics_LimitWheelSpeeds(wheel_speeds, MAX_MOTOR_SPEED_MPS);
}

/**
 * @brief 设置PID参数
 */
void PIDController_SetParams(int motor_id, float kp, float ki, float kd)
{
    if (motor_id < 0 || motor_id > 2) {
        return;
    }
    
    car_ctrl.motors[motor_id].pid.kp = kp;
    car_ctrl.motors[motor_id].pid.ki = ki;
    car_ctrl.motors[motor_id].pid.kd = kd;
    
    // 更新全局PID参数
    if (motor_id == 0) {
        pid_kp = kp;
        pid_ki = ki;
        pid_kd = kd;
    }
}

/**
 * @brief 重置PID控制器
 */
void PIDController_Reset(int motor_id)
{
    if (motor_id < 0 || motor_id > 2) {
        return;
    }
    
    PID_Reset(&car_ctrl.motors[motor_id].pid);
}

/**
 * @brief 测试：正方形路径
 */
void Test_Square(float side_length, float speed)
{
    printf("\n=== Square Test: side=%.2fm, speed=%.2fm/s ===\n", side_length, speed);
    
    for (int i = 0; i < 4; i++) {
        // 前进
        printf("Side %d: Forward %.2fm\n", i + 1, side_length);
        CarController_SetSpeed(speed, 0, 0);
        
        // 计算前进时间
        float move_time = side_length / speed * 1000; // 毫秒
        
        uint32_t start_time = rt_tick_get();
        while (rt_tick_get() - start_time < move_time) {
            CarController_Update();
            rt_thread_mdelay(10);
        }
        
        // 停止
        CarController_Stop();
        rt_thread_mdelay(500);
        
        // 旋转90度
        printf("Side %d: Rotate 90 degrees\n", i + 1);
        
        // 旋转90度所需时间
        float rotate_speed = M_PI / 2.0f; // 90度/秒
        CarController_SetSpeed(0, 0, rotate_speed);
        
        start_time = rt_tick_get();
        while (rt_tick_get() - start_time < 1000) { // 1秒旋转90度
            CarController_Update();
            rt_thread_mdelay(10);
        }
        
        // 停止
        CarController_Stop();
        rt_thread_mdelay(500);
    }
    
    printf("=== Square Test Completed ===\n");
}

/**
 * @brief 测试：旋转
 */
void Test_Rotation(float angle_deg, float speed_deg_per_sec)
{
    printf("\n=== Rotation Test: angle=%.1f°, speed=%.1f°/s ===\n", 
           angle_deg, speed_deg_per_sec);
    
    // 转换为弧度
    float angle_rad = angle_deg * M_PI / 180.0f;
    float speed_rad_per_sec = speed_deg_per_sec * M_PI / 180.0f;
    
    // 设置旋转
    CarController_SetSpeed(0, 0, speed_rad_per_sec);
    
    // 计算旋转时间
    float rotate_time = fabsf(angle_rad / speed_rad_per_sec) * 1000; // 毫秒
    
    uint32_t start_time = rt_tick_get();
    while (rt_tick_get() - start_time < rotate_time) {
        CarController_Update();
        rt_thread_mdelay(10);
    }
    
    CarController_Stop();
    printf("=== Rotation Test Completed ===\n");
}

/**
 * @brief 测试：基本运动
 */
void Test_Movement(float vx, float vy, float omega, float duration_ms)
{
    printf("\n=== Movement Test: vx=%.2f, vy=%.2f, ω=%.2f, time=%.0fms ===\n",
           vx, vy, omega, duration_ms);
    
    CarController_SetSpeed(vx, vy, omega);
    
    uint32_t start_time = rt_tick_get();
    while (rt_tick_get() - start_time < duration_ms) {
        CarController_Update();
        rt_thread_mdelay(10);
    }
    
    CarController_Stop();
    printf("=== Movement Test Completed ===\n");
}

/**
 * @brief 兼容旧接口：设置车体速度
 */
/*void CarSpeedSet(float speed_x, float speed_y, float speed_z)
{
    // 假设传入的速度是电机速度（编码器单位）
    // 需要转换为车体速度
    
    // 这里简单地将传入的参数视为电机速度
    // 实际使用时需要根据实际情况调整
    
    printf("Warning: Using old CarSpeedSet interface\n");
    
    // 临时设置电机速度
    for (int i = 0; i < 3; i++) {
        car_ctrl.motors[i].target_speed_mps = speed_x / 100.0f; // 粗略转换
    }
}*/

/**
 * @brief 兼容旧接口：停止
 */
/*void CarStop(void)
{
    CarController_Stop();
}*/

/**
 * @brief 兼容旧接口：PID初始化
 */
/*void pid_init(float kp, float ki, float kd)
{
    pid_kp = kp;
    pid_ki = ki;
    pid_kd = kd;
    
    // 更新所有电机的PID参数
    for (int i = 0; i < 3; i++) {
        PIDController_SetParams(i, kp, ki, kd);
    }
}*/