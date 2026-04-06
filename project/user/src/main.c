/*********************************************************************************************************************
* 描述：集成UART地图接收、自动任务执行、编码器重置。混合控制器统一控制。支持动作测试模式。
********************************************************************************************************************/

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
#include "seekfree_assistant.h"      // 逐飞助手支持

// ========== 引脚定义 ==========
#define KEY_START_PIN       C12
#define LED_CONFIRM_PIN     B9
#define MAP_REQ_INTERVAL_MS   200   // 重发间隔 200ms
#define MAP_REQ_MAX_ATTEMPTS  25    // 最多重试 5 秒（25*200ms）

// ========== 全局变量 ==========
GridMap g_grid_map;
GameState g_game_state;
HybridController g_ctrl;
static uint8_t system_started = 0;
static float angle_kp = 1.5f, angle_ki = 0.01f, angle_kd = 0.4f;
static uint32_t last_map_req_tick = 0;
uint8_t waiting_map = 0;      // 正在等待新地图
uint8_t need_map_update = 0;   // 需要请求新地图（由任务管理器或混合控制器设置）

// 动作测试模式变量
static uint8_t action_test_mode = 0;
static uint8_t action_executing = 0;
static uint32_t action_start_tick = 0;
static float action_vx = 0, action_vy = 0, action_omega = 0;
static uint32_t action_duration_ms = 1000;   // 每个动作默认执行1秒

// ========== 外部变量声明（来自 uart_receiver.c） ==========
extern uint8_t g_map_updated;

// ========== LED 闪烁 ==========
static void led_blink_once(void) {
    gpio_set_level(LED_CONFIRM_PIN, 0);
    rt_thread_mdelay(100);
    gpio_set_level(LED_CONFIRM_PIN, 1);
}

// ========== 发送地图请求 ==========
static void request_new_map(void) {
    wireless_uart_send_string("request_new_map called.\r\n");
    uart_write_string(UART_1, "MAP_REQ\n");
    wireless_uart_send_string("MAP_REQ sent.\r\n");
}

// ========== 主控制线程 ==========
static void control_thread_entry(void *parameter) {
    wireless_uart_send_string("Control thread started.\r\n");

    uint32_t tick = rt_tick_get();
    char buf[128];
    float target_yaw = 0.0f;
    uint8_t yaw_initialized = 0;
    float current_omega_rad = 0.0f;

    while (1) {
        // 更新位置（基于编码器积分）
        Position_Update();

        key_scanner();

        // KEY4 启动系统
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_4);
            if (!system_started) {
                system_started = 1;
                led_blink_once();
                wireless_uart_send_string("System started. Requesting initial map...\r\n");
                waiting_map = 1;
                request_new_map();
                AHRS_ResetYaw();
                target_yaw = 0.0f;
                yaw_initialized = 1;
            }
        }

        // KEY1 切换动作测试模式
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_1);
            action_test_mode = !action_test_mode;
            if (action_test_mode) {
                wireless_uart_send_string("Action test mode ON. Send parameter via Assistant.\r\n");
                action_executing = 0;
                CarController_SetSpeed(0, 0, 0);
            } else {
                wireless_uart_send_string("Action test mode OFF.\r\n");
                action_executing = 0;
                CarController_SetSpeed(0, 0, 0);
            }
        }

        // KEY2 重置位置（调试用）
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_2);
            if (!system_started) {
                wireless_uart_send_string("System not started.\r\n");
                continue;
            }
            Position_Set(0.3f, 1.2f, 0.0f);
            wireless_uart_send_string("Position reset to (0.3,1.2) yaw=0\r\n");
        }

        // ========== 地图请求逻辑：持续重发直到收到地图 ==========
        if (system_started && waiting_map) {
            uint32_t now = rt_tick_get();
            if (last_map_req_tick == 0 || (now - last_map_req_tick) >= RT_TICK_PER_SECOND * MAP_REQ_INTERVAL_MS / 1000) {
                static uint8_t req_attempts = 0;
                if (req_attempts >= MAP_REQ_MAX_ATTEMPTS) {
                    wireless_uart_send_string("Map request timeout, reset waiting_map.\r\n");
                    waiting_map = 0;
                    req_attempts = 0;
                    last_map_req_tick = 0;
                } else {
                    request_new_map();
                    req_attempts++;
                    last_map_req_tick = now;
                }
            }
        }

        // 检查地图是否已更新（UART接收线程解析完成）
        if (waiting_map && g_map_updated) {
            g_map_updated = 0;
            waiting_map = 0;
            last_map_req_tick = 0;
            wireless_uart_send_string("New map received, task manager will handle planning.\r\n");
        }

        // ========== 接收逐飞助手参数（动作码） ==========
        seekfree_assistant_data_analysis();
        if (seekfree_assistant_parameter_update_flag[0]) {
            seekfree_assistant_parameter_update_flag[0] = 0;
            int action_code = (int)(seekfree_assistant_parameter[0] + 0.5f);
            rt_sprintf(buf, "Received action code: %d\r\n", action_code);
            wireless_uart_send_string(buf);

            if (action_test_mode) {
                switch(action_code) {
                    case 1:
                        action_vx = 0.2f; action_vy = 0; action_omega = 0;
                        wireless_uart_send_string("Action: FORWARD\r\n");
                        break;
                    case 2:
                        action_vx = -0.2f; action_vy = 0; action_omega = 0;
                        wireless_uart_send_string("Action: BACKWARD\r\n");
                        break;
                    case 3:
                        action_vx = 0; action_vy = -0.2f; action_omega = 0;
                        wireless_uart_send_string("Action: LEFT\r\n");
                        break;
                    case 4:
                        action_vx = 0; action_vy = 0.2f; action_omega = 0;
                        wireless_uart_send_string("Action: RIGHT\r\n");
                        break;
                    case 5:
                        action_vx = 0; action_vy = 0; action_omega = 0.5f;
                        wireless_uart_send_string("Action: TURN LEFT\r\n");
                        break;
                    case 6:
                        action_vx = 0; action_vy = 0; action_omega = -0.5f;
                        wireless_uart_send_string("Action: TURN RIGHT\r\n");
                        break;
                    case 7:
                        wireless_uart_send_string("Manual push: FORWARD (action=4)\r\n");
                        if (g_ctrl.current_box_id >= 0) {
                            apply_push(&g_ctrl, g_ctrl.current_box_id, ACTION_PUSH_UP);
                        } else {
                            wireless_uart_send_string("No current box\r\n");
                        }
                        break;
                    case 8:
                        wireless_uart_send_string("Manual push: BACKWARD (action=6)\r\n");
                        if (g_ctrl.current_box_id >= 0) {
                            apply_push(&g_ctrl, g_ctrl.current_box_id, ACTION_PUSH_DOWN);
                        } else {
                            wireless_uart_send_string("No current box\r\n");
                        }
                        break;
                    case 9:
                        wireless_uart_send_string("Manual push: LEFT (action=7)\r\n");
                        if (g_ctrl.current_box_id >= 0) {
                            apply_push(&g_ctrl, g_ctrl.current_box_id, ACTION_PUSH_LEFT);
                        } else {
                            wireless_uart_send_string("No current box\r\n");
                        }
                        break;
                    case 10:
                        wireless_uart_send_string("Manual push: RIGHT (action=5)\r\n");
                        if (g_ctrl.current_box_id >= 0) {
                            apply_push(&g_ctrl, g_ctrl.current_box_id, ACTION_PUSH_RIGHT);
                        } else {
                            wireless_uart_send_string("No current box\r\n");
                        }
                        break;
                    case 0:
                        action_vx = 0; action_vy = 0; action_omega = 0;
                        action_executing = 0;
                        wireless_uart_send_string("Action: STOP\r\n");
                        break;
                    default:
                        wireless_uart_send_string("Unknown action code\r\n");
                        break;
                }
                if (action_code >= 1 && action_code <= 6) {
                    action_executing = 1;
                    action_start_tick = rt_tick_get();
                    CarController_SetSpeed(action_vx, action_vy, action_omega);
                }
            }
        }

        // ========== 控制逻辑：动作测试模式优先 ==========
        if (action_test_mode) {
            // 关键：必须调用 CarController_Update() 来执行PID并输出PWM
            CarController_Update();

            if (action_executing) {
                uint32_t elapsed = rt_tick_get() - action_start_tick;
                if (elapsed >= action_duration_ms * RT_TICK_PER_SECOND / 1000) {
                    CarController_SetSpeed(0, 0, 0);
                    action_executing = 0;
                    wireless_uart_send_string("Action finished.\r\n");
                }
            }
        } else if (system_started && yaw_initialized && !waiting_map) {
            float vx, vy, omega;
            float current_time = (float)rt_tick_get() / 1000.0f;
            HybridController_ComputeControl(&g_ctrl, position.x_m, position.y_m, position.yaw_rad,
                                            0.005f, current_time, &vx, &vy, &omega);
            CarController_SetSpeed(vx, vy, omega);
            CarController_Update();
            current_omega_rad = car_ctrl.current_omega;
        } else {
            // 未启动、未初始化或等待地图时停止电机
            CarController_SetSpeed(0, 0, 0);
            CarController_Update();
            current_omega_rad = 0.0f;
        }

        // 监控数据打印（每50ms）
        if (rt_tick_get() - tick > RT_TICK_PER_SECOND / 20) {
            tick = rt_tick_get();
            float actual[3], target[3];
            EncoderGetSpeeds(actual);
            for (int i = 0; i < 3; i++) target[i] = car_ctrl.motors[i].target_speed_mps;

            rt_sprintf(buf,
                "mode=%d vx=%d vy=%d omega=%d "
                "ch1:%d ch2:%d ch3:%d ch4:%d ch5:%d ch6:%d\r\n",
                g_ctrl.mode,
                (int)(target[0]*1000), (int)(target[1]*1000), (int)(current_omega_rad*1000),
                (int)(target[0]*1000), (int)(target[1]*1000), (int)(target[2]*1000),
                (int)(actual[0]*1000), (int)(actual[1]*1000), (int)(actual[2]*1000));
            wireless_uart_send_string(buf);
        }

        rt_thread_mdelay(5);
    }
}

// ========== 主函数 ==========
int main(void) {
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);

    gpio_init(LED_CONFIRM_PIN, GPO, 1, GPO_PUSH_PULL);
    key_init(10);

    MotorInit();
    EncoderInit();
    CarController_Init();
    MotorPID_SetGlobalParams(100.0f, 2.5f, 1.0f);

    IMU660RA_AHRS_Init();
    Position_Init();
    task_manager_init();
    task_manager_start();

    // 初始地图为空
    memset(&g_game_state, 0, sizeof(g_game_state));
    memset(&g_grid_map, 0, sizeof(g_grid_map));

    HybridController_Init(&g_ctrl, &g_grid_map, &g_game_state);
    g_ctrl.max_speed = 0.10f;
    g_ctrl.path_tolerance = 0.15f;

    // 初始化 UART1 接收
    uart_receive_init();
    rt_thread_t uart_thread = rt_thread_create("uart_parse",
                                               parse_uart_data_thread_entry,
                                               RT_NULL,
                                               4096,
                                               5,
                                               20);
    if (uart_thread) rt_thread_startup(uart_thread);
    else wireless_uart_send_string("Failed to create uart thread.\r\n");

    pit_ms_init(PIT_CH0, 10);
    interrupt_global_enable(0);

    // 创建控制线程
    rt_thread_t ctrl_thread = rt_thread_create("control", control_thread_entry, NULL,
                                               8192, RT_THREAD_PRIORITY_MAX / 2, 20);
    if (ctrl_thread) {
        rt_thread_startup(ctrl_thread);
        wireless_uart_send_string("Control thread created and started.\r\n");
    } else {
        wireless_uart_send_string("Failed to create control thread.\r\n");
    }

    wireless_uart_send_string("System ready. Press KEY4 to start. Press KEY1 for action test mode.\r\n");

    rt_system_scheduler_start();

    return 0;
}