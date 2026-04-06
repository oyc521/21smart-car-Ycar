#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
#include <rtthread.h>
#include <math.h>
#include "zf_device_wireless_uart.h"

// 目标轮速数组（m/s）
static float target_wheel_speeds[3] = {0, 0, 0};

// 轮子速度PID控制器
static PIDParam_t wheel_pid[3];

// 全局车体控制器
CarController_t car_ctrl;

// PID参数
float pid_kp = 0.0f, pid_ki = 0.0f, pid_kd = 0.0f;

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
    pwm_init(MOTOR1_PWM_PIN, PWM_FREQUENCY, 0);
    pwm_init(MOTOR2_PWM_PIN, PWM_FREQUENCY, 0);
    pwm_init(MOTOR3_PWM_PIN, PWM_FREQUENCY, 0);
    
    // 初始化方向引脚（默认低电平）
    gpio_init(MOTOR1_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR2_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR3_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

/**
 * @brief 电机控制器初始化
 */
void MotorController_Init(void)
{
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
    
    Kinematics_Init(&car_ctrl.kinematics);
    
    car_ctrl.target_vx = 0.0f;
    car_ctrl.target_vy = 0.0f;
    car_ctrl.target_omega = 0.0f;
    
    car_ctrl.current_vx = 0.0f;
    car_ctrl.current_vy = 0.0f;
    car_ctrl.current_omega = 0.0f;
    
    //rt_kprintf("Motor Controller Initialized\n");
}

/**
 * @brief 车体控制器初始化
 */
void CarController_Init(void)
{
    // 初始化每个轮子的PID控制器
    for (int i = 0; i < 3; i++) {
        PIDInit(&wheel_pid[i], INNER_KP, INNER_KI, INNER_KD, 100, -100);
    }
    
    // 初始化硬件
    MotorInit();
    
    // 初始化控制器状态
    MotorController_Init();
}

/**
 * @brief 设置车体速度
 */
void CarController_SetSpeed(float vx, float vy, float omega)
{		
		
		//vx = -vx;
    car_ctrl.target_vx = vx;
    car_ctrl.target_vy = vy;
    car_ctrl.target_omega = omega;
    
    float wheel_speeds[3];
    Kinematics_Inverse(vx, vy, omega, wheel_speeds);
    Kinematics_LimitWheelSpeeds(wheel_speeds, MAX_WHEEL_SPEED_MPS);
    
    // 根据实际电机角度重新映射
    car_ctrl.motors[0].target_speed_mps = wheel_speeds[2]; // 电机1 (240°) ← wheel_speeds[2]
    car_ctrl.motors[1].target_speed_mps = wheel_speeds[1]; // 电机2 (120°) ← wheel_speeds[1]
    car_ctrl.motors[2].target_speed_mps = wheel_speeds[0]; // 电机3 (0°)   ← wheel_speeds[0]

    // 更新目标轮速数组（供PID使用）
    target_wheel_speeds[0] = car_ctrl.motors[0].target_speed_mps;
    target_wheel_speeds[1] = car_ctrl.motors[1].target_speed_mps;
    target_wheel_speeds[2] = car_ctrl.motors[2].target_speed_mps;
    
    // 调试打印（乘以1000转为整数）
    /*char buf[128];
    rt_sprintf(buf, "wheel_speeds: %d %d %d\r\n",
               (int)(wheel_speeds[0]*1000), (int)(wheel_speeds[1]*1000), (int)(wheel_speeds[2]*1000));
    wireless_uart_send_string(buf);
    rt_sprintf(buf, "target_wheel: %d %d %d\r\n",
               (int)(target_wheel_speeds[0]*1000), (int)(target_wheel_speeds[1]*1000), (int)(target_wheel_speeds[2]*1000));
    wireless_uart_send_string(buf);*/
}

/**
 * @brief 设置单个电机速度
 */
void MotorController_SetSpeed(int motor_id, float speed_mps)
{
    if (motor_id < 0 || motor_id > 2) return;
    car_ctrl.motors[motor_id].target_speed_mps = speed_mps;
}

/**
 * @brief 更新车体控制器（内部）
 */
static void UpdateMotorSpeeds(void)
{
    float current_speeds[3];
    EncoderGetSpeeds(current_speeds);
    
    for (int i = 0; i < 3; i++) {
        car_ctrl.motors[i].current_speed_mps = current_speeds[i];
        float alpha = 0.3f;
        car_ctrl.motors[i].speed_filtered = alpha * current_speeds[i] + 
                                           (1-alpha) * car_ctrl.motors[i].speed_filtered;
    }
    
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
    CarController_SetSpeed(0, 0, 0);
    float speed_threshold = 0.05f;
    while (fabsf(car_ctrl.current_vx) > speed_threshold ||
           fabsf(car_ctrl.current_vy) > speed_threshold ||
           fabsf(car_ctrl.current_omega) > speed_threshold * 2) {
        CarController_Update();
        rt_thread_mdelay(10);
    }
}

/**
 * @brief 设置电机PWM（最终驱动函数）
 */
static void SetMotorPWM(int motor_id, float pwm_duty)
{
    uint16_t pwm_pin;
    uint8_t dir_pin;
    uint8_t forward_level, backward_level;

    // 根据电机ID选择引脚和方向电平
    switch (motor_id) {
        case 0:
            pwm_pin = MOTOR1_PWM_PIN;
            dir_pin = MOTOR1_DIR_PIN;
            forward_level = MOTOR_BACKWARD;
            backward_level = MOTOR_FORWARD;
            break;
        case 1:
            pwm_pin = MOTOR2_PWM_PIN;
            dir_pin = MOTOR2_DIR_PIN;
						forward_level = MOTOR_FORWARD;
            backward_level = MOTOR_BACKWARD;
            break;
        case 2:
            pwm_pin = MOTOR3_PWM_PIN;
            dir_pin = MOTOR3_DIR_PIN;
            forward_level = MOTOR_FORWARD;
            backward_level = MOTOR_BACKWARD;
            break;
        default:
            return;
    }

    // 限幅
    float duty = pwm_duty;
    if (duty > 100.0f) duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;

    // 转换为PWM原始值（假设 PWM_DUTY_MAX=10000）
    uint32_t duty_raw = (uint32_t)(fabsf(duty) * (PWM_DUTY_MAX / 100.0f));

    // 设置方向和PWM
    if (duty >= 0) {
        gpio_set_level(dir_pin, forward_level);
    } else {
        gpio_set_level(dir_pin, backward_level);
    }
    pwm_set_duty(pwm_pin, duty_raw);

    // 调试打印：占空比乘以100显示（-10000 ~ 10000）
    /*char buf[64];
    rt_sprintf(buf, "SetPWM id=%d duty=%d\r\n", motor_id, (int)(duty*100));
    wireless_uart_send_string(buf);*/
}

/**
 * @brief 更新电机速度（主控制循环调用）
 */
void CarController_Update(void)
{
    float actual_wheel_speeds[3];
    EncoderGetSpeeds(actual_wheel_speeds);

    char buf[128];
    for (int i = 0; i < 3; i++) {
        // 打印PID参数、目标速度、实际速度（乘以1000）
        /*rt_sprintf(buf, "i=%d: kp=%d tar=%d fb=%d\r\n",
                   i,
                   (int)wheel_pid[i].kp,
                   (int)(target_wheel_speeds[i] * 1000),
                   (int)(actual_wheel_speeds[i] * 1000));
        wireless_uart_send_string(buf);*/

        // 计算PID输出
        float output = PID(&wheel_pid[i], actual_wheel_speeds[i], target_wheel_speeds[i]);
        if (output > 100.0f) output = 100.0f;
        else if (output < -100.0f) output = -100.0f;

        // 打印输出值（整数 -100~100）
        /*rt_sprintf(buf, "out%d=%d\r\n", i, (int)output);
        wireless_uart_send_string(buf);*/

        // 直接调用PWM设置函数
        SetMotorPWM(i, output);
    }
}

/**
 * @brief 设置PID参数（单个电机，保留兼容）
 */
void PIDController_SetParams(int motor_id, float kp, float ki, float kd)
{
    if (motor_id < 0 || motor_id > 2) return;
    car_ctrl.motors[motor_id].pid.kp = kp;
    car_ctrl.motors[motor_id].pid.ki = ki;
    car_ctrl.motors[motor_id].pid.kd = kd;
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
    if (motor_id < 0 || motor_id > 2) return;
    PID_Reset(&car_ctrl.motors[motor_id].pid);
}

/**
 * @brief 测试：正方形路径
 */
void Test_Square(float side_length, float speed)
{
    //rt_kprintf("\n=== Square Test: side=%.2fm, speed=%.2fm/s ===\n", side_length, speed);
    for (int i = 0; i < 4; i++) {
        //rt_kprintf("Side %d: Forward %.2fm\n", i + 1, side_length);
        CarController_SetSpeed(speed, 0, 0);
        float move_time = side_length / speed * 1000;
        uint32_t start_time = rt_tick_get();
        while (rt_tick_get() - start_time < move_time) {
            CarController_Update();
            rt_thread_mdelay(10);
        }
        CarController_Stop();
        rt_thread_mdelay(500);
        
        //rt_kprintf("Side %d: Rotate 90 degrees\n", i + 1);
        float rotate_speed = M_PI / 2.0f;
        CarController_SetSpeed(0, 0, rotate_speed);
        start_time = rt_tick_get();
        while (rt_tick_get() - start_time < 1000) {
            CarController_Update();
            rt_thread_mdelay(10);
        }
        CarController_Stop();
        rt_thread_mdelay(500);
    }
    rt_kprintf("=== Square Test Completed ===\n");
}

/**
 * @brief 测试：旋转
 */
void Test_Rotation(float angle_deg, float speed_deg_per_sec)
{
    //rt_kprintf("\n=== Rotation Test: angle=%.1f°, speed=%.1f°/s ===\n", angle_deg, speed_deg_per_sec);
    float angle_rad = angle_deg * M_PI / 180.0f;
    float speed_rad_per_sec = speed_deg_per_sec * M_PI / 180.0f;
    CarController_SetSpeed(0, 0, speed_rad_per_sec);
    float rotate_time = fabsf(angle_rad / speed_rad_per_sec) * 1000;
    uint32_t start_time = rt_tick_get();
    while (rt_tick_get() - start_time < rotate_time) {
        CarController_Update();
        rt_thread_mdelay(10);
    }
    CarController_Stop();
    //rt_kprintf("=== Rotation Test Completed ===\n");
}

/**
 * @brief 测试：基本运动
 */
void Test_Movement(float vx, float vy, float omega, float duration_ms)
{
    //rt_kprintf("\n=== Movement Test: vx=%.2f, vy=%.2f, ω=%.2f, time=%.0fms ===\n", vx, vy, omega, duration_ms);
    CarController_SetSpeed(vx, vy, omega);
    uint32_t start_time = rt_tick_get();
    while (rt_tick_get() - start_time < duration_ms) {
        CarController_Update();
        rt_thread_mdelay(10);
    }
    CarController_Stop();
    //rt_kprintf("=== Movement Test Completed ===\n");
}

/**
 * @brief 更新所有轮子的PID参数（用于动态调参）
 */
void MotorPID_SetGlobalParams(float kp, float ki, float kd)
{
    pid_kp = kp;
    pid_ki = ki;
    pid_kd = kd;
    for (int i = 0; i < 3; i++) {
        wheel_pid[i].kp = kp;
        wheel_pid[i].ki = ki;
        wheel_pid[i].kd = kd;
        PID_Reset(&wheel_pid[i]);
    }
}

/**
 * @brief 外部接口：直接设置PWM
 */
void Motor_SetPWM(int motor_id, float duty)
{
    SetMotorPWM(motor_id, duty);
}