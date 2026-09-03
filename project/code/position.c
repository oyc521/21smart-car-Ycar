#include "position.h"
#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
#include "filter.h"
#include <math.h>
#include "planner.h"
#include "imu660ra_ahrs.h"
#include "zf_common_headfile.h"
#include "hybrid_controller.h"

extern HybridController g_ctrl;   // 外部混合控制器实例

Position_t position = {0};        // 全局位置结构体
extern CarController_t car_ctrl;
extern float gyro_bias_z;

// ========== 姿态融合变量 ==========
static uint8_t yaw_fusion_inited = 0;
static float  yaw_fused_rad = 0.0f;

// 位置低通滤波器，用于平滑位置信息
static LowPassFilter_t disp_x_filter;
static LowPassFilter_t disp_y_filter;
static uint8_t disp_filter_inited = 0;

// 航向滑动平均滤波参数（窗口大小1，仅做单点平滑，可调整）
#define YAW_WINDOW_SIZE 1
static float yaw_buffer[YAW_WINDOW_SIZE] = {0};
static uint8_t yaw_buf_idx = 0;
static uint8_t yaw_buf_filled = 0;

// ========== PID控制器参数 ==========
PIDParam_t angle_trace_param;        // 角度跟踪PID（用于导航）
PIDParam_t pos_x_param, pos_y_param; // 位置环X/Y PID
float Kp_rot = 1.2f;                 // 旋转比例系数

// 角度 PID 控制器（不同模式）
PIDParam_t angle_pid_nav;            // 导航模式（直行）
PIDParam_t angle_pid_lateral;        // 侧移模式
static PIDParam_t angle_pid_align;   // 对准模式

// 内部状态
static uint8_t position_initialized = 0;

/**
 * @brief 位置模块初始化
 * @details 清零所有状态，初始化PID参数和低通滤波器
 */
void Position_Init(void)
{
    position.x_dis = 0;
    position.y_dis = 0;
    position.yaw = 0;
    position.speed_x = 0;
    position.speed_y = 0;
    position.speed_z = 0;
    position.x_target = 0;
    position.y_target = 0;
    position.yaw_target = 0;
    position.flag_dis_count = 0;
    position.position_reached = 0;
    position.angle_reached = 0;
    
    PositionPIDParamInit();
    
    // 初始化位置低通滤波器（截止频率0.1）
    LowPassFilter_Init(&disp_x_filter, 0.1f);
    LowPassFilter_Init(&disp_y_filter, 0.1f);
    disp_filter_inited = 1;
    
    position_initialized = 1;
}

/**
 * @brief 初始化位置环和角度环PID参数
 */
void PositionPIDParamInit(void)
{
    // 位置环 PID（全局坐标系）
    float kp_adjust = 2.0f, ki_adjust = 0.04f, kd_adjust = 0.4f;
    PIDInit(&pos_x_param, kp_adjust, ki_adjust, kd_adjust, SPEED_MAX, -SPEED_MAX);
    PIDInit(&pos_y_param, kp_adjust, ki_adjust, kd_adjust, SPEED_MAX, -SPEED_MAX);

    // 导航 PID（直行）
    float nav_kp = 1.4f;
    float nav_kd = 1.4f;
    PIDInit(&angle_pid_nav, nav_kp, 0.0f, nav_kd, 30.0f, -30.0f);

    // 侧移 PID
    float lat_kp = 1.4f;
    float lat_kd = 1.4f;
    PIDInit(&angle_pid_lateral, lat_kp, 0.0f, lat_kd, 30.0f, -30.0f);

    // 对准 PID
    float align_kp = 1.4f;
    float align_kd = 1.4f;
    PIDInit(&angle_pid_align, align_kp, 0.0f, align_kd, 20.0f, -20.0f);

    angle_trace_param = angle_pid_nav;   // 默认使用导航模式
}

/**
 * @brief 位置更新函数（需周期性调用）
 * @details 读取编码器增量，经过运动学正解得到车体速度，
 *          结合AHRS航向进行坐标累积，同时进行航向滤波。
 * @note 该函数通常在控制循环中调用，频率建议200Hz
 */
void Position_Update(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = rt_tick_get();

    if (last_tick == 0) {
        last_tick = now;
        if (!yaw_fusion_inited) {
            yaw_fused_rad = AHRS_GetYaw() * DEG_TO_RAD;
            yaw_fusion_inited = 1;
            for (int i = 0; i < YAW_WINDOW_SIZE; i++) {
                yaw_buffer[i] = yaw_fused_rad;
            }
            yaw_buf_filled = 1;
        }
        return;
    }

    // ===== 只有编码器更新时才进行位置累积 =====
    if (!encoder_updated_flag) return;

    // 获取编码器增量（临界区保护）
    int16_t deltas[3];
    rt_enter_critical();
    encoder_updated_flag = 0;
    EncoderGetDeltas(deltas);
    rt_exit_critical();

    // 将脉冲数转换为每个轮子的位移（米）
    float meters_per_pulse = WHEEL_CIRCUMFERENCE / (ENCODER_PPR * ENCODER_GEAR_RATIO);
    float wheel_disp[3];
    wheel_disp[0] = deltas[2] * meters_per_pulse;
    wheel_disp[1] = deltas[1] * meters_per_pulse;
    wheel_disp[2] = deltas[0] * meters_per_pulse;

    // 运动学正解：从轮子位移计算车体速度（vx, vy, omega）
    float vx_body, vy_body, omega;
    Kinematics_Forward(wheel_disp, &vx_body, &vy_body, &omega);

    // 双轮标定系数（根据实际轮径调整）
    float scale_x = 4.13f;
    float scale_y = 4.13f;
    vx_body *= scale_x;
    vy_body *= scale_y;

    // AHRS更新 + 航向滑动平均滤波
    AHRS_Update();
    float ahrs_yaw_rad = AHRS_GetYaw() * DEG_TO_RAD;

    yaw_buffer[yaw_buf_idx] = ahrs_yaw_rad;
    yaw_buf_idx = (yaw_buf_idx + 1) % YAW_WINDOW_SIZE;
    float sum = 0;
    int count = yaw_buf_filled ? YAW_WINDOW_SIZE : yaw_buf_idx + 1;
    for (int i = 0; i < count; i++) {
        sum += yaw_buffer[i];
    }
    float filtered_yaw_rad = sum / (float)count;

    // 角度归一化到 [-PI, PI]
    while (filtered_yaw_rad >  M_PI) filtered_yaw_rad -= 2*M_PI;
    while (filtered_yaw_rad < -M_PI) filtered_yaw_rad += 2*M_PI;
    yaw_fused_rad = filtered_yaw_rad;

    // 将车体系速度转换到全局坐标系
    float v_forward =  vx_body;
    float v_left    = -vy_body;
    float cos_yaw = cosf(filtered_yaw_rad);
    float sin_yaw = sinf(filtered_yaw_rad);
    float vx_global = v_forward * cos_yaw - v_left * sin_yaw;
    float vy_global = v_forward * sin_yaw + v_left * cos_yaw;

    // 积分得到全局坐标（米）
    position.x_m += vx_global;
    position.y_m -= vy_global;

    // 更新位置结构体（同时保存角度）
    position.yaw_rad = filtered_yaw_rad;
    position.x_dis = position.x_m * 100.0f;   // 转换为厘米
    position.y_dis = position.y_m * 100.0f;
    position.yaw   = filtered_yaw_rad * RAD_TO_DEG;
}

/**
 * @brief 使用IMU精确旋转到目标角度（单位：度）
 * @param target_angle 目标角度（-180~180）
 * @return 1成功，0超时失败
 * @details 采用分段速度控制，最后一段为比例控制，并带有稳定判断
 */
int RotateToAngleIMU(float target_angle)
{
    float error;
    uint32_t start_tick = rt_tick_get();
    const uint32_t timeout_ms = 10000;
    const uint32_t stable_required_ms = 200;

    target_angle = fmodf(target_angle, 360.0f);
    if (target_angle > 180.0f) target_angle -= 360.0f;
    else if (target_angle < -180.0f) target_angle += 360.0f;

    float current_yaw = AHRS_GetYaw();
    error = target_angle - current_yaw;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    if (fabsf(error) < 2.0f) {
        wireless_uart_send_string("[Rotate] Already exactly aligned.\r\n");
        return 1;
    }

    uint32_t stable_start = 0;

    do {
        uint32_t now = rt_tick_get();
        if (now - start_tick > timeout_ms) {
            wireless_uart_send_string("[Rotate] Timeout!\r\n");
            CarController_Stop();
            return 0;
        }

        AHRS_Update();
        current_yaw = AHRS_GetYaw();
        error = target_angle - current_yaw;
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        float abs_err = fabsf(error);

        if (abs_err < 2.0f) {
            CarController_Stop();
            wireless_uart_send_string("[Rotate] Done (precise).\r\n");
            return 1;
        }

        float omega_deg;
        if (abs_err > 45.0f) {
            omega_deg = (error > 0) ? 60.0f : -60.0f;
        } else if (abs_err > 20.0f) {
            omega_deg = (error > 0) ? 40.0f : -40.0f;
        } else if (abs_err > 5.0f) {
            omega_deg = (error > 0) ? 20.0f : -20.0f;
        } else {
            omega_deg = Kp_rot * error;
            if (omega_deg > 15.0f) omega_deg = 15.0f;
            else if (omega_deg < -15.0f) omega_deg = -15.0f;
        }

        float omega_rad = omega_deg * DEG_TO_RAD;

        CarController_SetSpeed(0, 0, omega_rad);
        CarController_Update();
        rt_thread_mdelay(10);

        if (fabsf(error) < 3.5f) {
            if (stable_start == 0) stable_start = now;
        } else {
            stable_start = 0;
        }

    } while (stable_start == 0 || (rt_tick_get() - stable_start) < stable_required_ms);

    CarController_Stop();
    wireless_uart_send_string("[Rotate] Done.\r\n");
    return 1;
}

/**
 * @brief 使用IMU移动到目标坐标（单位：厘米）
 * @param x_cm 目标X坐标（厘米）
 * @param y_cm 目标Y坐标（厘米）
 * @param max_speed 最大移动速度（米/秒）
 * @details 先调整航向至0°，然后使用位置PID控制移动，全程保持航向稳定
 */
void MoveToPositionIMU(float x_cm, float y_cm, float max_speed)
{
    // 先确保车头朝向0°
    float cur_yaw = AHRS_GetYaw();
    float yaw_err_deg = 0.0f - cur_yaw;
    while (yaw_err_deg > 180.0f) yaw_err_deg -= 360.0f;
    while (yaw_err_deg < -180.0f) yaw_err_deg += 360.0f;
    if (fabsf(yaw_err_deg) > 5.0f) {
        wireless_uart_send_string("[MoveTo] Re-align to 0...\r\n");
        RotateToAngleIMU(0.0f);
    }

    // 临时调整角度PID参数（更柔顺）
    float saved_kp = angle_trace_param.kp;
    float saved_ki = angle_trace_param.ki;
    float saved_kd = angle_trace_param.kd;
    angle_trace_param.kp = 0.5f;
    angle_trace_param.ki = 0.0f;
    angle_trace_param.kd = 0.15f;
    PID_Reset(&angle_trace_param);

    // 位置环PID微调（增加积分项提高静态精度）
    PIDParam_t local_pos_x = pos_x_param;
    PIDParam_t local_pos_y = pos_y_param;
    local_pos_x.ki = 0.01f;
    local_pos_y.ki = 0.01f;
    local_pos_x.error_sum_max = 0.05f;
    local_pos_x.error_sum_min = -0.05f;
    local_pos_y.error_sum_max = 0.05f;
    local_pos_y.error_sum_min = -0.05f;
    PID_Reset(&local_pos_x);
    PID_Reset(&local_pos_y);

    float target_yaw = AHRS_GetYaw();   // 锁定当前航向

    float err_x_global, err_y_global;
    float vx_body, vy_body, omega_rad;
    uint32_t last_print = 0;

    do {
        // 计算全局误差（单位：米）
        err_x_global = (x_cm - position.x_dis) / 100.0f;
        err_y_global = (y_cm - position.y_dis) / 100.0f;

        // 位置PID得到期望全局速度
        float vx_global_des = PID(&local_pos_x, 0, err_x_global);
        float vy_global_des = PID(&local_pos_y, 0, err_y_global);

        // 限速
        float speed = sqrtf(vx_global_des * vx_global_des + vy_global_des * vy_global_des);
        if (speed > max_speed) {
            float scale = max_speed / speed;
            vx_global_des *= scale;
            vy_global_des *= scale;
        }

        // 将期望全局速度转换到车体系
        float current_yaw_rad = AHRS_GetYaw() * DEG_TO_RAD;
        float cos_yaw = cosf(current_yaw_rad);
        float sin_yaw = sinf(current_yaw_rad);
        vx_body =  vx_global_des * cos_yaw + vy_global_des * sin_yaw;
        vy_body = -vx_global_des * sin_yaw + vy_global_des * cos_yaw;

        // 航向保持控制
        float current_yaw_deg = AHRS_GetYaw();
        float angle_error = target_yaw - current_yaw_deg;
        while (angle_error > 180.0f) angle_error -= 360.0f;
        while (angle_error < -180.0f) angle_error += 360.0f;

        float omega_deg = 0.0f;
        if (fabsf(angle_error) > 2.5f) {
            omega_deg = angle_trace_param.kp * angle_error;
            // 动态限幅
            float limit = 10.0f + fminf(fabsf(angle_error) * 0.5f, 5.0f);
            if (omega_deg > limit) omega_deg = limit;
            else if (omega_deg < -limit) omega_deg = -limit;
        }
        omega_rad = omega_deg * DEG_TO_RAD;

        CarController_SetSpeed(vx_body, vy_body, omega_rad);
        CarController_Update();
        Position_Update();   // 更新位置（内部会读取编码器）

        // 调试打印
        if (rt_tick_get() - last_print > RT_TICK_PER_SECOND / 10) {
            last_print = rt_tick_get();
            char buf[64];
            rt_sprintf(buf, "POS:%d,%d\r\n", (int)position.x_dis, (int)position.y_dis);
            wireless_uart_send_string(buf);
        }

        rt_thread_mdelay(10);

    } while (fabsf(err_x_global) > 0.02f || fabsf(err_y_global) > 0.02f); // 误差小于2cm退出

    CarController_Stop();
    // 恢复原角度PID参数
    angle_trace_param.kp = saved_kp;
    angle_trace_param.ki = saved_ki;
    angle_trace_param.kd = saved_kd;
    PID_Reset(&angle_trace_param);
}

/**
 * @brief 重置航向角（用于初始化或强制校准）
 */
void Position_ResetYaw(void)
{
    yaw_fused_rad = AHRS_GetYaw() * DEG_TO_RAD;
    position.yaw_rad = yaw_fused_rad;
    position.yaw = yaw_fused_rad * RAD_TO_DEG;
}

/**
 * @brief 切换角度PID模式
 * @param mode 0：导航模式，1：对准模式
 */
void AnglePID_SwitchMode(int mode)
{
    switch (mode) {
        case 0: angle_trace_param = angle_pid_nav; break;
        case 1: angle_trace_param = angle_pid_align; break;
        default: return;
    }
    PID_Reset(&angle_trace_param);
}

/**
 * @brief 手动设置位置（用于外部修正）
 * @param x_m 全局X坐标（米）
 * @param y_m 全局Y坐标（米）
 * @param yaw_rad 航向角（弧度）
 */
void Position_Set(float x_m, float y_m, float yaw_rad)
{
    position.x_m = x_m;
    position.y_m = y_m;
    position.yaw_rad = yaw_rad;
    position.x_dis = x_m * 100.0f;
    position.y_dis = y_m * 100.0f;
    position.yaw = yaw_rad * RAD_TO_DEG;

    // 重置滤波器状态，避免历史值影响
    LowPassFilter_Init(&disp_x_filter, 0.1f);
    LowPassFilter_Init(&disp_y_filter, 0.1f);
}

/**
 * @brief 设置角度跟踪PID参数（动态调整）
 */
void AnglePID_SetParams(float kp, float ki, float kd)
{
    angle_trace_param.kp = kp;
    angle_trace_param.ki = ki;
    angle_trace_param.kd = kd;
    PID_Reset(&angle_trace_param);
}