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

// 全局变量定义
Position_t position = {0};
extern CarController_t car_ctrl;
extern float gyro_bias_z;

// 航向融合变量
static uint8_t yaw_fusion_inited = 0;
static float  yaw_fused_rad = 0.0f;
// 位移低通滤波器（用于平滑位移增量）
static LowPassFilter_t disp_x_filter;
static LowPassFilter_t disp_y_filter;
static uint8_t disp_filter_inited = 0;
// 滑动平均滤波器参数
#define YAW_WINDOW_SIZE 5
static float yaw_buffer[YAW_WINDOW_SIZE] = {0};
static uint8_t yaw_buf_idx = 0;
static uint8_t yaw_buf_filled = 0;

// PID控制器定义
PIDParam_t angle_trace_param;
PIDParam_t pos_x_param, pos_y_param;

// 角度 PID 参数组
static PIDParam_t angle_pid_nav;
static PIDParam_t angle_pid_align;
static PIDParam_t angle_pid_push;

// 内部状态
static uint8_t position_initialized = 0;

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
    
    // 重置位移低通滤波器
    LowPassFilter_Init(&disp_x_filter, 0.1f);
    LowPassFilter_Init(&disp_y_filter, 0.1f);
    disp_filter_inited = 1;
    
    position_initialized = 1;
}

void PositionPIDParamInit(void)
{
    // 位置环 PID（备用）
    float kp_adjust = 2.0f, kd_adjust = 0.8f;
    PIDInit(&pos_x_param, kp_adjust, 0, kd_adjust, SPEED_MAX, -SPEED_MAX);
    PIDInit(&pos_y_param, kp_adjust, 0, kd_adjust, SPEED_MAX, -SPEED_MAX);

    // 导航用角度 PID
    float nav_kp = 0.8f;
    float nav_kd = 0.2f;
    PIDInit(&angle_pid_nav, nav_kp, 0.0f, nav_kd, 30.0f, -30.0f);

    // 精对准用
    float align_kp = 0.5f;
    float align_kd = 0.2f;
    PIDInit(&angle_pid_align, align_kp, 0.0f, align_kd, 20.0f, -20.0f);

    // 推动用
    float push_kp = 0.5f;
    float push_kd = 0.2f;
    PIDInit(&angle_pid_push, push_kp, 0.0f, push_kd, 40.0f, -40.0f);

    angle_trace_param = angle_pid_nav;
}

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

    // ===== 只在编码器有新数据时处理，避免重复累加 =====
    if (!encoder_updated_flag) return;

    // 读取脉冲增量并立即清除标志
    int16_t deltas[3];
    rt_enter_critical();
    encoder_updated_flag = 0;
    EncoderGetDeltas(deltas);
    rt_exit_critical();

    // 脉冲增量 -> 轮子位移增量（米）
    float meters_per_pulse = WHEEL_CIRCUMFERENCE / (ENCODER_PPR * ENCODER_GEAR_RATIO);
    float wheel_disp[3];
    wheel_disp[0] = deltas[2] * meters_per_pulse;
    wheel_disp[1] = deltas[1] * meters_per_pulse;
    wheel_disp[2] = deltas[0] * meters_per_pulse;

    // 运动学正解：轮子位移增量 → 车体位移增量
    float vx_body, vy_body, omega;
    Kinematics_Forward(wheel_disp, &vx_body, &vy_body, &omega);

    // 双轴独立标定因子（需重新标定）
    float scale_x = 4.13f;
    float scale_y = 4.13f;
    vx_body *= scale_x;
    vy_body *= scale_y;

    // 位移增量直接使用，不再进行低通滤波

    // AHRS 更新 + 滑动平均航向
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

    // 角度归一化
    while (filtered_yaw_rad >  M_PI) filtered_yaw_rad -= 2*M_PI;
    while (filtered_yaw_rad < -M_PI) filtered_yaw_rad += 2*M_PI;
    yaw_fused_rad = filtered_yaw_rad;

    // 旋转到位移全局坐标系（直接使用原始位移增量）
    float v_forward =  vx_body;
    float v_left    = -vy_body;
    float cos_yaw = cosf(filtered_yaw_rad);
    float sin_yaw = sinf(filtered_yaw_rad);
    float vx_global = v_forward * cos_yaw - v_left * sin_yaw;
    float vy_global = v_forward * sin_yaw + v_left * cos_yaw;

    // 积分
    position.x_m += vx_global;
    position.y_m -= vy_global;

    // 输出
    position.yaw_rad = filtered_yaw_rad;
    position.x_dis = position.x_m * 100.0f;
    position.y_dis = position.y_m * 100.0f;
    position.yaw   = filtered_yaw_rad * RAD_TO_DEG;

    // 调试打印
    static uint32_t last_debug_tick = 0;
    if (now - last_debug_tick > RT_TICK_PER_SECOND / 5) {
        last_debug_tick = now;
        char buf[128];
        rt_sprintf(buf, "PLUSE: dx=%d dy=%d x=%d y=%d yaw=%d\r\n",
                   (int)(vx_body * 1000),   // 使用原始位移增量，单位 mm
                   (int)(vy_body * 1000),
                   (int)(position.x_m * 1000),
                   (int)(position.y_m * 1000),
                   (int)(filtered_yaw_rad * RAD_TO_DEG));
        wireless_uart_send_string(buf);
    }
}

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

    float Kp_rot = 1.2f;
    uint32_t stable_start = 0;
    uint32_t last_print = 0;

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

        if (now - last_print > 1000) {
            last_print = now;
            char buf[64];
            rt_sprintf(buf, "[Rotate] cur=%d err=%d omega=%d\r\n",
                       (int)current_yaw, (int)error, (int)omega_deg);
            wireless_uart_send_string(buf);
        }

    } while (stable_start == 0 || (rt_tick_get() - stable_start) < stable_required_ms);

    CarController_Stop();
    wireless_uart_send_string("[Rotate] Done.\r\n");
    return 1;
}

void MoveToPositionIMU(float x_cm, float y_cm, float max_speed)
{
    float cur_yaw = AHRS_GetYaw();
    float yaw_err_deg = 0.0f - cur_yaw;
    while (yaw_err_deg > 180.0f) yaw_err_deg -= 360.0f;
    while (yaw_err_deg < -180.0f) yaw_err_deg += 360.0f;
    if (fabsf(yaw_err_deg) > 5.0f) {
        wireless_uart_send_string("[MoveTo] Re-align to 0...\r\n");
        RotateToAngleIMU(0.0f);
    }

    float saved_kp = angle_trace_param.kp;
    float saved_ki = angle_trace_param.ki;
    float saved_kd = angle_trace_param.kd;
    angle_trace_param.kp = 0.5f;
    angle_trace_param.ki = 0.0f;
    angle_trace_param.kd = 0.15f;
    PID_Reset(&angle_trace_param);

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

    float target_yaw = AHRS_GetYaw();

    float err_x_global, err_y_global;
    float vx_body, vy_body, omega_rad;
    uint32_t last_print = 0;

    do {
        err_x_global = (x_cm - position.x_dis) / 100.0f;
        err_y_global = (y_cm - position.y_dis) / 100.0f;

        float vx_global_des = PID(&local_pos_x, 0, err_x_global);
        float vy_global_des = PID(&local_pos_y, 0, err_y_global);

        float speed = sqrtf(vx_global_des * vx_global_des + vy_global_des * vy_global_des);
        if (speed > max_speed) {
            float scale = max_speed / speed;
            vx_global_des *= scale;
            vy_global_des *= scale;
        }

        float current_yaw_rad = AHRS_GetYaw() * DEG_TO_RAD;
        float cos_yaw = cosf(current_yaw_rad);
        float sin_yaw = sinf(current_yaw_rad);
        vx_body =  vx_global_des * cos_yaw + vy_global_des * sin_yaw;
        vy_body = -vx_global_des * sin_yaw + vy_global_des * cos_yaw;

        float current_yaw_deg = AHRS_GetYaw();
        float angle_error = target_yaw - current_yaw_deg;
        while (angle_error > 180.0f) angle_error -= 360.0f;
        while (angle_error < -180.0f) angle_error += 360.0f;

        float omega_deg = 0.0f;
        if (fabsf(angle_error) > 2.5f) {
            omega_deg = angle_trace_param.kp * angle_error;
            float limit = 10.0f + fminf(fabsf(angle_error) * 0.5f, 5.0f);
            if (omega_deg > limit) omega_deg = limit;
            else if (omega_deg < -limit) omega_deg = -limit;
        }
        omega_rad = omega_deg * DEG_TO_RAD;

        CarController_SetSpeed(vx_body, vy_body, omega_rad);
        CarController_Update();
        Position_Update();

        if (rt_tick_get() - last_print > RT_TICK_PER_SECOND / 10) {
            last_print = rt_tick_get();
            char buf[64];
            rt_sprintf(buf, "POS:%d,%d\r\n", (int)position.x_dis, (int)position.y_dis);
            wireless_uart_send_string(buf);
        }

        rt_thread_mdelay(10);

    } while (fabsf(err_x_global) > 0.02f || fabsf(err_y_global) > 0.02f);

    CarController_Stop();
    angle_trace_param.kp = saved_kp;
    angle_trace_param.ki = saved_ki;
    angle_trace_param.kd = saved_kd;
    PID_Reset(&angle_trace_param);
}

void Position_ResetYaw(void)
{
    yaw_fused_rad = AHRS_GetYaw() * DEG_TO_RAD;
    position.yaw_rad = yaw_fused_rad;
    position.yaw = yaw_fused_rad * RAD_TO_DEG;
}

void AnglePID_SwitchMode(int mode)
{
    switch (mode) {
        case 0: angle_trace_param = angle_pid_nav; break;
        case 1: angle_trace_param = angle_pid_align; break;
        case 2: angle_trace_param = angle_pid_push; break;
        default: return;
    }
    PID_Reset(&angle_trace_param);
}

void Position_Set(float x_m, float y_m, float yaw_rad)
{
    position.x_m = x_m;
    position.y_m = y_m;
    position.yaw_rad = yaw_rad;
    position.x_dis = x_m * 100.0f;
    position.y_dis = y_m * 100.0f;
    position.yaw = yaw_rad * RAD_TO_DEG;

    // 重置滤波器，避免旧状态影响新位置
    LowPassFilter_Init(&disp_x_filter, 0.1f);
    LowPassFilter_Init(&disp_y_filter, 0.1f);
}

void AnglePID_SetParams(float kp, float ki, float kd)
{
    angle_trace_param.kp = kp;
    angle_trace_param.ki = ki;
    angle_trace_param.kd = kd;
    PID_Reset(&angle_trace_param);
}