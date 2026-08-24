/**
 * @file main.c
 * @brief 智能车全向移动控制系统主程序
 * 
 * @note 开源声明
 * ============================================================================
 * 本项目基于【逐飞科技 RT1064 开源库】进行二次开发。
 * 底层芯片驱动库（包括但不限于 UART、PWM、ENCODER、IMU 等外设驱动）
 * 由【逐飞科技】开源提供，在此表示感谢。
 * 
 * 逐飞科技开源库仓库地址：https://github.com/seekfree/seekfree_library
 * 
 * 本项目（智能车全向移动控制系统）在原库基础上新增/修改的内容包括：
 * - 混合控制器（Hybrid Controller）
 * - A* 路径规划器 + 路径抽稀算法
 * - Sokoban 推箱子 / 炸弹爆破墙规划器
 * - 任务管理器（Task Manager）状态机
 * - IMU/里程计互补滤波融合定位
 * - UART 通信协议解析
 * - 三轮全向底盘运动学建模
 * 
 * 本项目遵循逐飞科技开源库的原始许可证进行开源。
 * ============================================================================
 */

#include <rtthread.h>
#include <math.h>
#include "imu660ra_ahrs.h"
#include "position.h"
#include "pid.h"
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "motor.h"
#include "encoder.h"
#include "planner.h"
#include "hybrid_controller.h"
#include "uart_receiver.h"
#include "task_manager.h"
#include "seekfree_assistant.h"
#include "planner.h"
#include "kinematics.h"
#include "filter.h"
#include "uart4_recognition.h"
#include "debug.h"

// ========== 硬件引脚与参数定义 ==========
#define KEY_START_PIN       C12        // 启动按键（KEY4/KEY3共用此定义？实际代码中分别检测KEY_4和KEY_3，此处为历史遗留）
#define LED_CONFIRM_PIN     B9         // 确认指示灯（LED）
#define MAP_REQ_INTERVAL_MS   200      // 地图请求间隔（毫秒）
#define MAP_REQ_MAX_ATTEMPTS  25       // 地图请求最大重试次数
#define BEEP_PIN            B11        // 蜂鸣器引脚

// ========== 全局变量 ==========
GridMap g_grid_map;                     // 全局栅格地图
GameState g_game_state;                 // 游戏状态（任务进度等）
HybridController g_ctrl;               // 混合控制器实例
uint8_t system_started = 0;            // 系统启动标志（按键触发后置1）
uint8_t yaw_initialized = 0;           // 航向角是否已初始化（锁定）
static uint32_t last_map_req_tick = 0; // 上次发送地图请求的时刻（tick）
volatile uint8_t waiting_map = 0;      // 是否正在等待地图响应
volatile uint8_t need_map_update = 0;  // 是否需要主动请求更新地图
static uint8_t map_req_attempts = 0;   // 当前地图请求尝试次数

// 系统航向锁定值（用于稳定行驶方向）
float g_locked_yaw = 0.0f;

extern uint8_t g_map_updated;          // 外部（中断/串口解析）置位，表示新地图已收到
rt_mutex_t g_map_mutex = RT_NULL;      // 地图数据互斥锁，防止并发访问

// ========== YAW 测试相关（调试用） ==========
static uint8_t yaw_test_active = 0;
static float yaw_test_ref = 0.0f;

/**
 * @brief 初始化蜂鸣器引脚
 */
static void beep_init(void)
{
    gpio_init(BEEP_PIN, GPO, 0, GPO_PUSH_PULL);
}

/**
 * @brief 蜂鸣器短鸣一声（用于提示操作成功）
 */
void BeepOnce(void)
{
    gpio_set_level(BEEP_PIN, 1);   // 拉高，蜂鸣器响
    rt_thread_mdelay(100);
    gpio_set_level(BEEP_PIN, 0);   // 拉低，关闭
}

/**
 * @brief LED闪烁一次（用于视觉反馈）
 */
static void led_blink_once(void)
{
    gpio_set_level(LED_CONFIRM_PIN, 0);
    rt_thread_mdelay(100);
    gpio_set_level(LED_CONFIRM_PIN, 1);
}

/**
 * @brief 向 OpenArt请求新地图
 * @note 通过串口发送 "MAP_REQ\n" 指令
 */
void request_new_map(void)
{
    uart_write_string(UART_1, "MAP_REQ\n");
}

/**
 * @brief 控制线程主循环
 * @param parameter 线程入口参数（未使用）
 * @details 该线程负责：
 *          - 更新 AHRS 和位置信息
 *          - 处理地图请求超时重试
 *          - 检测按键启动不同阶段
 *          - 调用混合控制器计算速度指令
 *          - 更新电机控制
 *          - 打印模式切换等信息
 */
static void control_thread_entry(void *parameter)
{
    wireless_uart_send_string("Control thread started.\r\n");

    uint32_t tick = rt_tick_get();
    char buf[128];
    static float last_omega_cmd = 0.0f;

    while (1) {
        // 更新姿态和位置（基于IMU和里程计）
        AHRS_Update();
        Position_Update();

#if DEBUG_ENABLE
        debug_seekfree_loop();   // 调试辅助循环
#endif

        // ===== 检查是否需要请求地图更新 =====
        if (need_map_update) {
            if (g_ctrl.mode == CTRL_MODE_PATH_FOLLOWING) {
                // 若当前处于路径跟随模式，则不主动请求（避免干扰）
            } else {
                need_map_update = 0;
                waiting_map = 1;
                map_req_attempts = 0;
                request_new_map();
            }
        }

        // 按键扫描（由底层中断或定时器更新状态）
        key_scanner();

        // 检测 KEY4 短按 → 启动第一阶段（出库）
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_4);
            if (!system_started) {
                system_started = 1;
                task_manager_set_mode(TASK_MODE_STAGE1);   // 切换任务管理器到阶段1
                led_blink_once();
                //BeepOnce();  // 可选的蜂鸣反馈
                wireless_uart_send_string("Stage1 started. Moving out of garage...\r\n");
                g_task_mgr.state = TASK_STATE_MOVE_OUT;
                rt_event_send(g_task_mgr.event, 0x80);     // 触发任务管理器事件
                AHRS_ResetYaw();
                Position_ResetYaw();               // 同时重置位置模块的航向
                g_locked_yaw = AHRS_GetYaw();
                yaw_initialized = 1;
            }
        }

        // 检测 KEY3 短按 → 启动第二阶段（复杂任务）
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_3);
            if (!system_started) {
                system_started = 1;
                task_manager_set_mode(TASK_MODE_STAGE2);   // 切换任务管理器到阶段2
                led_blink_once();
                BeepOnce();                                // 蜂鸣反馈
                wireless_uart_send_string("Stage2 started. Moving out of garage...\r\n");
                g_task_mgr.state = TASK_STATE_MOVE_OUT;
                rt_event_send(g_task_mgr.event, 0x80);
                AHRS_ResetYaw();
                Position_ResetYaw();
                g_locked_yaw = AHRS_GetYaw();
                yaw_initialized = 1;
            }
        }

        // 处理地图请求超时重传
        if (system_started && waiting_map) {
            uint32_t now = rt_tick_get();
            if (last_map_req_tick == 0 || (now - last_map_req_tick) >= RT_TICK_PER_SECOND * MAP_REQ_INTERVAL_MS / 1000) {
                if (map_req_attempts >= MAP_REQ_MAX_ATTEMPTS) {
                    wireless_uart_send_string("Map request timeout, reset waiting_map.\r\n");
                    waiting_map = 0;
                    map_req_attempts = 0;
                    last_map_req_tick = 0;
                } else {
                    request_new_map();
                    map_req_attempts++;
                    last_map_req_tick = now;
                }
            }
        }

        // 若收到新地图，则通知任务管理器
        if (waiting_map && g_map_updated) {
            g_map_updated = 0;
            waiting_map = 0;
            map_req_attempts = 0;
            last_map_req_tick = 0;
            wireless_uart_send_string("New map received, sending event to task manager.\r\n");
            rt_event_send(g_task_mgr.event, TASK_EVENT_MAP_READY);
        }

        // ========== 正常控制循环（系统已启动且航向已初始化且不处于等待地图状态） ==========
        else if (system_started && yaw_initialized && !waiting_map) {
            float vx_out, vy_out, omega;
            float current_time = (float)rt_tick_get() / 1000.0f;
            // 调用混合控制器，根据当前位置、姿态和目标路径计算速度指令
            HybridController_ComputeControl(&g_ctrl, position.x_m, position.y_m, position.yaw_rad,
                                            0.005f, current_time, &vx_out, &vy_out, &omega);

            // 设置电机速度
            CarController_SetSpeed(vx_out, vy_out, omega);
            CarController_Update();

            // 调试：输出角速度指令变化
            if (fabsf(omega - last_omega_cmd) > 0.01f) {
                last_omega_cmd = omega;
                char dbg[32];
                rt_sprintf(dbg, "CMD_OMEGA:%d\r\n", (int)(omega * 1000));
                wireless_uart_send_string(dbg);
            }
        } else {
            // 未启动或未初始化时停止小车
            CarController_SetSpeed(0, 0, 0);
            CarController_Update();
        }

        // 打印控制器模式变化（调试信息）
        static int last_mode = -1;
        if (g_ctrl.mode != last_mode) {
            last_mode = g_ctrl.mode;
            char dbg[32];
            rt_sprintf(dbg, "[Mode] changed to %d\r\n", g_ctrl.mode);
            wireless_uart_send_string(dbg);
        }

        // 循环周期 5ms（200Hz）
        rt_thread_mdelay(5);
    }
}

/**
 * @brief 主函数，系统初始化入口
 * @return 永不返回（调度器启动后）
 */
int main(void)
{
    // 系统时钟初始化（600MHz，根据硬件配置）
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    // 无线串口初始化（用于与上位机通信）
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);

    // GPIO初始化
    gpio_init(LED_CONFIRM_PIN, GPO, 1, GPO_PUSH_PULL);  // 默认LED高电平熄灭
    beep_init();    // 蜂鸣器
    key_init(10);   // 按键扫描周期10ms

    // 电机与编码器初始化
    MotorInit();
    EncoderInit();
    CarController_Init();
    MotorPID_SetGlobalParams(100.0f, 2.5f, 1.0f); // 设置全局PID参数（比例、积分、微分）

    // IMU与位置估计初始化
    IMU660RA_AHRS_Init();
    Position_Init();

    // 任务管理器初始化并启动
    task_manager_init();
    task_manager_start();

    // 清空全局状态和地图数据
    memset(&g_game_state, 0, sizeof(g_game_state));
    memset(&g_grid_map, 0, sizeof(g_grid_map));

    // 初始化混合控制器，绑定地图和状态
    HybridController_Init(&g_ctrl, &g_grid_map, &g_game_state);
    g_ctrl.max_speed = 0.10f;        // 最大线速度 0.1 m/s
    g_ctrl.path_tolerance = 0.15f;   // 路径跟踪容差 0.15 m

    debug_module_init();             // 调试模块初始化

    // 串口接收解析线程初始化（用于处理上位机指令）
    uart_receive_init();
    uart4_recognition_init();        // 特定串口4识别功能

    // 创建地图数据互斥锁
    g_map_mutex = rt_mutex_create("map_mutex", RT_IPC_FLAG_FIFO);
    if (g_map_mutex == RT_NULL) {
        wireless_uart_send_string("Failed to create map mutex.\r\n");
    }

    // 创建串口数据解析线程
    rt_thread_t uart_thread = rt_thread_create("uart_parse",
                                               parse_uart_data_thread_entry,
                                               RT_NULL,
                                               4096,
                                               5,
                                               20);
    if (uart_thread) rt_thread_startup(uart_thread);
    else wireless_uart_send_string("Failed to create uart thread.\r\n");

    // 启动 PIT 定时器（10ms 周期，用于周期任务触发）
    pit_ms_init(PIT_CH0, 10);
    interrupt_global_enable(0);   // 开启全局中断

    // 创建控制线程（优先级中等，栈大小8192）
    rt_thread_t ctrl_thread = rt_thread_create("control", control_thread_entry, NULL,
                                               8192, RT_THREAD_PRIORITY_MAX / 2, 20);
    if (ctrl_thread) {
        rt_thread_startup(ctrl_thread);
        wireless_uart_send_string("Control thread created and started.\r\n");
    } else {
        wireless_uart_send_string("Failed to create control thread.\r\n");
    }

    wireless_uart_send_string("System ready. Press KEY4 to start.\r\n");

    // 启动 RT-Thread 调度器（主线程不再返回）
    rt_system_scheduler_start();

    return 0;  // 永远不会执行到这里
}