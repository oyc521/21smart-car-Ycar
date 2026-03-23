#include "position.h"
#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
#include "filter.h"  // 添加滤波器头文件
#include <math.h>
#include "planner.h"
#include "imu660ra_ahrs.h"
#include "zf_common_headfile.h"
// 全局变量定义
Position_t position = {0};
uint8_t disaim_flag = 0;
float disaim_x_dis = 0, disaim_y_dis = 0;
extern CarController_t car_ctrl;   // 引用 motor.c 中的全局变量
// 声明外部全局变量
extern GridMap g_grid_map;
extern GameState g_game_state;
// PID控制器定义
PIDParam_t line_trace_param, angle_trace_param;
PIDParam_t disaim_x_param, disaim_y_param;
PIDParam_t pos_x_param, pos_y_param;

// 低通滤波器（使用新的滤波器结构体）
static LowPassFilter_t speed_x_filter = {0, 0.3f, 0};
static LowPassFilter_t speed_y_filter = {0, 0.3f, 0};
static LowPassFilter_t speed_z_filter = {0, 0.3f, 0};

// 内部状态
static uint8_t position_initialized = 0;

/**
 * @brief 位置控制器初始化
 */
void Position_Init(void)
{
    // 初始化位置状态
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
    
    // 初始化PID参数
    PositionPIDParamInit();
    
    // 初始化低通滤波器
    LowPassFilter_Init(&speed_x_filter, 0.3f);
    LowPassFilter_Init(&speed_y_filter, 0.3f);
    LowPassFilter_Init(&speed_z_filter, 0.3f);
    
    position_initialized = 1;
    
    //printf("Position Controller Initialized\n");
}

/**
 * @brief PID参数初始化
 */
void PositionPIDParamInit(void)
{
    // 使用更保守的参数以减少抖动
    float kp_x = 0.8f, kd_x = 0.3f;
    float kp_angle_track = 1.5f, ki_angle_track = 0.01f, kd_angle_track = 0.4f;
    float kp_dis_x = 3.0f, kp_dis_y = 2.0f, kd_dis_x = 3.0f, kd_dis_y = 2.0f;
    float kp_adjust = 2.0f, kd_adjust = 0.8f;
    
    PIDInit(&line_trace_param, kp_x, 0, kd_x, 50, -50);
    PIDInit(&angle_trace_param, kp_angle_track, ki_angle_track, kd_angle_track, 50, -50);
    PIDInit(&disaim_x_param, kp_dis_x, 0, kd_dis_x, 60, -60);
    PIDInit(&disaim_y_param, kp_dis_y, 0, kd_dis_y, 60, -60);
    PIDInit(&pos_x_param, kp_adjust, 0, kd_adjust, SPEED_MAX, -SPEED_MAX);
    PIDInit(&pos_y_param, kp_adjust, 0, kd_adjust, SPEED_MAX, -SPEED_MAX);
}

/**
 * @brief 更新位置信息
 */
void Position_Update(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = rt_tick_get();
    if (last_tick == 0) {
        last_tick = now;
        return;
    }
    float dt = (now - last_tick) / (float)RT_TICK_PER_SECOND;
    if (dt > 1.0f) dt = 1.0f;
    last_tick = now;

    float encoder_speeds[3];
    EncoderGetSpeeds(encoder_speeds); // encoder_speeds[0]=电机1, [1]=电机2, [2]=电机3

    // 按照运动学索引重新排列
    float wheel_speeds[3];
    wheel_speeds[0] = encoder_speeds[2]; // 电机3 (0°)
    wheel_speeds[1] = encoder_speeds[1]; // 电机2 (120°)
    wheel_speeds[2] = encoder_speeds[0]; // 电机1 (240°)

    float vx, vy, omega;
    Kinematics_Forward(wheel_speeds, &vx, &vy, &omega);

    // 标定因子：使积分位移与实际一致（根据推动1米POS显示20cm得出）
    float scale = 4.63f;   // 100 / 63 ≈ 1.587，原 scale=5.0，新 scale = 5.0 * 1.587 ≈ 7.94
    vx *= scale;
    vy *= scale;

    // 调试打印（可选）
    /*static uint32_t last_print = 0;
    if (now - last_print > 200) {
        last_print = now;
        char buf[64];
        rt_sprintf(buf, "VX:%d VY:%d\r\n", (int)(vx * 1000), (int)(vy * 1000));
        wireless_uart_send_string(buf);
    }*/

    // 积分
    position.x_m += vx * dt;
    position.y_m += vy * dt;
    position.yaw_rad += omega * dt;

    // 角度归一化
    if (position.yaw_rad > M_PI) position.yaw_rad -= 2 * M_PI;
    else if (position.yaw_rad < -M_PI) position.yaw_rad += 2 * M_PI;

    position.x_dis = position.x_m * 100.0f;
    position.y_dis = position.y_m * 100.0f;
    position.yaw   = position.yaw_rad * RAD_TO_DEG;
		// 在 Position_Update 函数末尾添加（在更新 x_dis, y_dis 之后）
		static uint32_t last_debug_tick = 0;
		uint32_t now_tick = rt_tick_get();
		if (now_tick - last_debug_tick > RT_TICK_PER_SECOND / 5) {  // 每200ms打印一次
				last_debug_tick = now_tick;
				char dbg_buf[128];
				rt_sprintf(dbg_buf, "DBG: vx=%d vy=%d omega=%d dt=%d x_m=%d y_m=%d\r\n",
									 (int)(vx*1000), (int)(vy*1000), (int)(omega*1000), (int)(dt*1000),
									 (int)(position.x_m*1000), (int)(position.y_m*1000));
				wireless_uart_send_string(dbg_buf);
		}
}
/**
 * @brief 设置目标位置
 */
void Position_SetTarget(float x_cm, float y_cm, float angle_deg)
{
    position.x_target = x_cm;
    position.y_target = y_cm;
    position.yaw_target = angle_deg;
    
    position.position_reached = 0;
    position.angle_reached = 0;
    
    //printf("Position Target Set: x=%.1fcm, y=%.1fcm, angle=%.1f°\n", 
          // x_cm, y_cm, angle_deg);
}

/**
 * @brief 检查是否到达目标位置
 */
uint8_t Position_IsTargetReached(void)
{
    float pos_error = sqrtf(powf(position.x_target - position.x_dis, 2) + 
                           powf(position.y_target - position.y_dis, 2));
    float angle_error = fabsf(position.yaw_target - position.yaw);
    
    if (angle_error > 180.0f) {
        angle_error = 360.0f - angle_error;
    }
    
    position.position_reached = (pos_error < POSITION_TOLERANCE);
    position.angle_reached = (angle_error < ANGLE_TOLERANCE);
    
    return (position.position_reached && position.angle_reached);
}

/**
 * @brief 移动到指定位置
 */
void MoveToPosition(float x_cm, float y_cm, float max_speed)
{
    //printf("Moving to position: (%.1f, %.1f) cm\n", x_cm, y_cm);
    
    Position_SetTarget(x_cm, y_cm, position.yaw);  // 保持当前角度为目标
    
    // 启用位移计数
    position.flag_dis_count = 1;
    
    // 记录目标角度（当前角度，也可考虑传入参数）
    float target_yaw = position.yaw;
    
    // 控制循环
    while (!Position_IsTargetReached()) {
        // 计算位置误差（米）
        float error_x = (position.x_target - position.x_dis) / 100.0f;
        float error_y = (position.y_target - position.y_dis) / 100.0f;
        
        // 位置PID得到期望速度（车体系pos_x_param期望输入是米）
        float vx = PID(&pos_x_param, 0, error_x);
        float vy = PID(&pos_y_param, 0, error_y);
        
        // 限制最大速度
        float speed = sqrtf(vx * vx + vy * vy);
        if (speed > max_speed) {
            float scale = max_speed / speed;
            vx *= scale;
            vy *= scale;
        }
        
        // --- 新增：角度保持控制 ---
        float angle_error = target_yaw - position.yaw;
        // 归一化到 [-180,180]
        while (angle_error > 180.0f) angle_error -= 360.0f;
        while (angle_error < -180.0f) angle_error += 360.0f;
        
        // 使用角度PID计算角速度（rad/s）
        float omega_rad = PD(&angle_trace_param, 0, angle_error * DEG_TO_RAD);
        // 限幅角速度
        float max_omega_rad = SPEED_MAX_DPS * DEG_TO_RAD;
        if (fabsf(omega_rad) > max_omega_rad) {
            omega_rad = (omega_rad > 0) ? max_omega_rad : -max_omega_rad;
        }
        
        // 设置车体速度（包含旋转）
        CarController_SetSpeed(vx, vy, omega_rad);
        
        // 更新位置和控制器
        Position_Update();
        CarController_Update();
        
        // 短暂延时
        rt_thread_mdelay(10);
    }
    
    // 到达目标，停止
    CarController_Stop();
    position.flag_dis_count = 0;
    
    //printf("Reached target position\n");
}

/**
 * @brief 旋转到指定角度
 */
void RotateToAngle(float target_angle, float max_speed_dps)
{
    //printf("Rotating to angle: %.1f°\n", target_angle);
    
    // 角度归一化
    target_angle = fmodf(target_angle, 360.0f);
    if (target_angle < -180.0f) target_angle += 360.0f;
    if (target_angle > 180.0f) target_angle -= 360.0f;
    
    Position_SetTarget(position.x_dis, position.y_dis, target_angle);
    
    // 控制循环
    while (!position.angle_reached) {
        // 计算角度误差
        float error = position.yaw_target - position.yaw;
        
        // 角度误差处理
        if (error > 180.0f) error -= 360.0f;
        else if (error < -180.0f) error += 360.0f;
        
        // 使用PID计算期望角速度
        float omega_rad = PD(&angle_trace_param, 0, error * M_PI / 180.0f);
        
        // 限制最大角速度
        float max_omega_rad = max_speed_dps * M_PI / 180.0f;
        if (fabsf(omega_rad) > max_omega_rad) {
            omega_rad = (omega_rad > 0) ? max_omega_rad : -max_omega_rad;
        }
        
        // 设置车体速度（原地旋转）
        CarController_SetSpeed(0, 0, omega_rad);
        
        // 更新位置和控制器
        Position_Update();
        CarController_Update();
        
        // 短暂延时
        rt_thread_mdelay(10);
    }
    
    // 停止
    CarController_Stop();
    
    //printf("Reached target angle\n");
}
/**
 * @brief 使用IMU的偏航角旋转到指定角度
 * @param target_angle 目标角度（度），范围 -180~180
 */
void RotateToAngleIMU(float target_angle)
{
    float error;
    float omega_deg, omega_rad;

    // 角度归一化
    target_angle = fmodf(target_angle, 360.0f);
    if (target_angle > 180.0f) target_angle -= 360.0f;
    else if (target_angle < -180.0f) target_angle += 360.0f;

    do {
        float current_yaw = AHRS_GetYaw();
        error = target_angle - current_yaw;

        // 归一化误差到 [-180,180]
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        // 角度PD计算角速度（度/秒）
        omega_deg = PD(&angle_trace_param, 0, error);
        // 限幅
        if (omega_deg > 50.0f) omega_deg = 50.0f;
        else if (omega_deg < -50.0f) omega_deg = -50.0f;

        omega_rad = omega_deg * DEG_TO_RAD;

        // 设置原地旋转
        CarController_SetSpeed(0, 0, omega_rad);
        CarController_Update();
        rt_thread_mdelay(10);
    } while (fabsf(error) > 2.0f); // 误差小于2度时停止

    CarController_Stop();
}
/**
 * @brief 使用IMU的角度保持移动到指定位置（厘米）
 * @param x_cm      目标X坐标（厘米）
 * @param y_cm      目标Y坐标（厘米）
 * @param max_speed 最大速度（m/s）
 */
void MoveToPositionIMU(float x_cm, float y_cm, float max_speed)
{
    // 记录起始位置（可选，用于调试）
    float start_x = position.x_dis;
    float start_y = position.y_dis;

    // 目标角度锁定为当前航向（保持方向）
    float target_yaw = AHRS_GetYaw();

    float error_x, error_y, vx, vy, omega_rad;
    do {
        // 计算位置误差（米）
        error_x = (x_cm - position.x_dis) / 100.0f;
        error_y = (y_cm - position.y_dis) / 100.0f;

        // 位置PID得到期望速度（车体系）
        vx = PID(&pos_x_param, 0, error_x);
        vy = PID(&pos_y_param, 0, error_y);

        // 限制最大速度
        float speed = sqrtf(vx*vx + vy*vy);
        if (speed > max_speed) {
            vx *= max_speed / speed;
            vy *= max_speed / speed;
        }

        // 角度保持（使用IMU）
        float current_yaw = AHRS_GetYaw();
        float angle_error = target_yaw - current_yaw;
        // 归一化到 [-180,180]
        while (angle_error > 180.0f) angle_error -= 360.0f;
        while (angle_error < -180.0f) angle_error += 360.0f;

        float omega_deg = PD(&angle_trace_param, 0, angle_error);
        // 限幅角速度（度/秒）
        if (omega_deg > 50.0f) omega_deg = 50.0f;
        else if (omega_deg < -50.0f) omega_deg = -50.0f;

        omega_rad = omega_deg * DEG_TO_RAD;

        // 设置车体速度
        CarController_SetSpeed(vx, vy, omega_rad);
        CarController_Update();
        Position_Update();   // 更新编码器位置

        // 可选的调试输出（每100ms发送一次当前位置）
        static uint32_t last_print = 0;
        if (rt_tick_get() - last_print > RT_TICK_PER_SECOND / 10) { // 100ms
            last_print = rt_tick_get();
            char buf[64];
            rt_sprintf(buf, "POS:%d,%d\r\n", (int)position.x_dis, (int)position.y_dis);
            wireless_uart_send_string(buf);
        }

        rt_thread_mdelay(10);
    } while (fabsf(error_x) > 0.02 || fabsf(error_y) > 0.02); // 容差2cm

    CarController_Stop();
}
/**
 * @brief 直线移动
 */
void MoveStraight(float distance_cm, float speed_cmps)
{
    //printf("Moving straight: %.1fcm at %.1fcm/s\n", distance_cm, speed_cmps);
    
    // 计算目标位置
    float distance_rad = position.yaw * M_PI / 180.0f;
    float target_x = position.x_dis + distance_cm * cosf(distance_rad);
    float target_y = position.y_dis + distance_cm * sinf(distance_rad);
    
    // 移动到目标位置
    MoveToPosition(target_x, target_y, speed_cmps / 100.0f); // 转换为m/s
}

/**
 * @brief 横向移动
 */
void MoveLateral(float distance_cm, float speed_cmps)
{
    //printf("Moving lateral: %.1fcm at %.1fcm/s\n", distance_cm, speed_cmps);
    
    // 计算目标位置（垂直于当前方向）
    float distance_rad = (position.yaw + 90.0f) * M_PI / 180.0f;
    float target_x = position.x_dis + distance_cm * cosf(distance_rad);
    float target_y = position.y_dis + distance_cm * sinf(distance_rad);
    
    // 移动到目标位置
    MoveToPosition(target_x, target_y, speed_cmps / 100.0f); // 转换为m/s
}

// 辅助函数：寻找粗网格(r,c)旁边的可通行点
static int find_coarse_adjacent_point(int r, int c, float* out_x, float* out_y)
{
    int dr[4] = {-1, 1, 0, 0};
    int dc[4] = {0, 0, -1, 1};
    for (int d = 0; d < 4; d++) {
        int nr = r + dr[d];
        int nc = c + dc[d];
        if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
        int base_x = nc * 4;
        int base_y = nr * 4;
        int blocked = 0;
        for (int dy = 0; dy < 4 && !blocked; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                uint8_t occ = g_grid_map.occupancy[base_y + dy][base_x + dx];
                if (occ == OCC_WALL || occ == OCC_BOX) {
                    blocked = 1;
                    break;
                }
            }
        }
        if (!blocked) {
            *out_x = (nc + 0.5f) * CELL_SIZE;
            *out_y = (nr + 0.5f) * CELL_SIZE;
            return 1;
        }
    }
    return 0;
}

// 移动到目的地 i 附近
int move_to_destination(int dest_id)
{
    if (dest_id < 0 || dest_id >= g_game_state.num_destinations) return 0;
    Destination* dest = &g_game_state.destinations[dest_id];
    int r = (int)(dest->y / CELL_SIZE);
    int c = (int)(dest->x / CELL_SIZE);
    float target_x, target_y;
    if (!find_coarse_adjacent_point(r, c, &target_x, &target_y))
        return 0;
    // MoveToPosition 接受厘米单位，转换
    MoveToPosition(target_x * 100, target_y * 100, 0.3);
    return 1;
}

// 移动到箱子 i 附近
int move_to_box(int box_id)
{
    if (box_id < 0 || box_id >= g_game_state.num_boxes) return 0;
    Box* box = &g_game_state.boxes[box_id];
    int r = (int)(box->y / CELL_SIZE);
    int c = (int)(box->x / CELL_SIZE);
    float target_x, target_y;
    if (!find_coarse_adjacent_point(r, c, &target_x, &target_y))
        return 0;
    MoveToPosition(target_x * 100, target_y * 100, 0.3);
    return 1;
}
/**
 * @brief 手动设置位置和航向（单位：米，弧度）
 */
void Position_Set(float x_m, float y_m, float yaw_rad)
{
    position.x_m = x_m;
    position.y_m = y_m;
    position.yaw_rad = yaw_rad;
    position.x_dis = x_m * 100.0f;
    position.y_dis = y_m * 100.0f;
    position.yaw = yaw_rad * RAD_TO_DEG;
}
// 位置环动态调参
void AnglePID_SetParams(float kp, float ki, float kd)
{
    angle_trace_param.kp = kp;
    angle_trace_param.ki = ki;
    angle_trace_param.kd = kd;
    PID_Reset(&angle_trace_param); // 重置积分项，防止突变
}