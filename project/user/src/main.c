/*********************************************************************************************************************
* 文件名：main.c
* 描述：集成速度控制、方向保持和路径跟踪。按 KEY4 启动，按 KEY3 执行硬编码直线路径测试。
*       所有控制在一个线程中完成，避免多线程冲突。
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

// ========== 引脚定义 ==========
#define KEY_START_PIN       C12
#define LED_CONFIRM_PIN     B9

// ========== 全局变量 ==========
GridMap g_grid_map;
GameState g_game_state;
HybridController g_ctrl;
static uint8_t system_started = 0;        // 系统启动标志
static float angle_kp = 1.5f;
static float angle_ki = 0.01f;
static float angle_kd = 0.4f;
static uint8_t forward_mode = 0;          // 空闲时的前进模式

// ========== 外部函数声明 ==========
extern void AnglePID_SetParams(float kp, float ki, float kd);
extern void RotateToAngleIMU(float target_angle);
// follow_path 已在 hybrid_controller.h 中声明，无需再次声明

// ========== LED 闪烁 ==========
static void led_blink_once(void) {
    gpio_set_level(LED_CONFIRM_PIN, 0);
    rt_thread_mdelay(100);
    gpio_set_level(LED_CONFIRM_PIN, 1);
}

// ========== 主控制线程（集成所有控制） ==========
static void control_thread_entry(void *parameter) {
    uint32_t tick = rt_tick_get();
    char buf[128];
    float target_yaw = 0.0f;
    uint8_t yaw_initialized = 0;
    // 空闲模式不输出速度
    float vx_target = 0.0f;
    float vy_target = 0.0f;

    while (1) {
        key_scanner();

        // KEY4 启动系统（仅标志，不控制电机）
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_4);
            if (!system_started) {
                system_started = 1;
                led_blink_once();
                wireless_uart_send_string("System started (idle).\r\n");
                AHRS_ResetYaw();
                target_yaw = 0.0f;
                yaw_initialized = 1;
            }
        }

                // KEY2 多箱子顺序规划测试
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_1);
            if (!system_started) {
                wireless_uart_send_string("System not started. Press KEY4 first.\r\n");
                continue;
            }

            wireless_uart_send_string("Multi-box sequential planning...\r\n");

            // 复制当前游戏状态（规划过程可能修改，但我们需要保持原始状态？实际执行中会修改，无需复制）
            // 依次处理每个未完成的箱子
            int total_boxes = g_game_state.num_boxes;
            int success_count = 0;
            for (int box_id = 0; box_id < total_boxes; box_id++) {
                if (g_game_state.boxes[box_id].state == 1) {
                    char buf[64];
                    rt_sprintf(buf, "Box %d already pushed, skip.\r\n", box_id);
                    wireless_uart_send_string(buf);
                    continue;
                }

                char buf[128];
                rt_sprintf(buf, "Processing box %d at (%.2f,%.2f) to dest %d\r\n",
                           box_id,
                           g_game_state.boxes[box_id].x,
                           g_game_state.boxes[box_id].y,
                           g_game_state.boxes[box_id].dest_id);
                wireless_uart_send_string(buf);

                // 获取当前小车位置
                float car_x = position.x_m;
                float car_y = position.y_m;
                // 如果小车位置为0（未移动），使用默认起点
                if (car_x < 0.01f && car_y < 0.01f) {
                    car_x = 0.5f;
                    car_y = 0.5f;
                    wireless_uart_send_string("Using default start (0.5,0.5)\r\n");
                }

                // 规划推箱子
                int actions[200];
                int action_count = light_sokoban_plan(&g_game_state, &g_grid_map, box_id,
                                                      car_x, car_y, actions, 200);
                if (action_count <= 0) {
                    rt_sprintf(buf, "Planning failed for box %d\r\n", box_id);
                    wireless_uart_send_string(buf);
                    continue;
                }

                // 注意：light_sokoban_plan 返回的动作已经是 4~7，不需要再加 4
                // 转换为世界坐标路径
                float path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
                int path_len = actions_to_world_path(&g_game_state, &g_grid_map, box_id,
                                                     car_x, car_y, actions, action_count,
                                                     path_x, path_y, MAX_PATH_POINTS);
                if (path_len <= 0) {
                    rt_sprintf(buf, "Path conversion failed for box %d\r\n", box_id);
                    wireless_uart_send_string(buf);
                    continue;
                }

                // 将路径存入混合控制器
                for (int i = 0; i < path_len; i++) {
                    g_ctrl.current_path[i][0] = path_x[i];
                    g_ctrl.current_path[i][1] = path_y[i];
                }
                g_ctrl.path_len = path_len;
                g_ctrl.path_following = 1;
                g_ctrl.is_bomb_path = 0;
                g_ctrl.path_stuck_counter = 0;
                g_ctrl.mode = CTRL_MODE_PATH_FOLLOWING;

                rt_sprintf(buf, "Executing path for box %d, length %d\r\n", box_id, path_len);
                wireless_uart_send_string(buf);

                // 执行路径跟踪
                int wait_count = 0;
                float dist_to_end = 100.0f;
                while (g_ctrl.mode == CTRL_MODE_PATH_FOLLOWING && g_ctrl.path_following && wait_count < 500) {
                    Position_Update();
                    float cur_x = position.x_m;
                    float cur_y = position.y_m;
                    float cur_angle = position.yaw_rad;
                    float vx, vy, omega;
                    if (follow_path(&g_ctrl, cur_x, cur_y, cur_angle, &vx, &vy, &omega, &dist_to_end)) {
                        CarController_SetSpeed(vx, vy, omega);
                        CarController_Update();
                        if (dist_to_end < g_ctrl.path_tolerance) {
                            break;
                        }
                    } else {
                        break;
                    }
                    rt_thread_mdelay(10);
                    wait_count++;
                }
                if (wait_count >= 500)
                    wireless_uart_send_string("Timeout.\r\n");
                else
                    wireless_uart_send_string("Path finished.\r\n");

                // 停止小车，等待下一个箱子
                CarController_Stop();
                g_ctrl.mode = CTRL_MODE_IDLE;

                // 注意：箱子状态应该在执行推动动作时由混合控制器自动更新（apply_push 中调用）
                // 但为了确保，我们再次检查状态，若未更新则手动刷新地图
                if (g_game_state.boxes[box_id].state == 0) {
                    // 可能推动未完成，但路径执行完毕，强制刷新地图
                    refresh_grid_map(&g_game_state, &g_grid_map);
                }

                success_count++;
            }

            char buf[64];
            rt_sprintf(buf, "Multi-box test finished, success %d/%d\r\n", success_count, total_boxes);
            wireless_uart_send_string(buf);
        }
        // KEY2 重置位置到 (0.5, 0.5)
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_2);
            if (!system_started) {
                wireless_uart_send_string("System not started. Press KEY4 first.\r\n");
                continue;
            }
            Position_Set(0.3f, 1.2f, 0.0f);
            wireless_uart_send_string("Position reset to (0.5,0.5) yaw=0\r\n");
        }


               // KEY3 A* 规划路径测试（从当前位置向前移动1米）
        // KEY3 推箱子测试（箱子 (0.5,0.5) 推到 (1.5,0.5)）
                // KEY3 推箱子测试（调试版）
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            key_clear_state(KEY_3);
            if (!system_started) {
                wireless_uart_send_string("System not started. Press KEY4 first.\r\n");
                continue;
            }

            // 手动设置箱子和目的地（运动坐标）
            float box_x = 0.5f;
            float box_y = 0.5f;
            float dest_x = 1.5f;
            float dest_y = 0.5f;

            // 清空游戏状态中的箱子和目的地，并添加测试数据
            g_game_state.num_boxes = 1;
            g_game_state.boxes[0].x = box_x;
            g_game_state.boxes[0].y = box_y;
            g_game_state.boxes[0].grid_x = (int)(box_x / CELL_SIZE) * 4 + 2; // 近似
            g_game_state.boxes[0].grid_y = (int)(box_y / CELL_SIZE) * 4 + 2;
            g_game_state.boxes[0].state = 0;
            g_game_state.boxes[0].dest_id = 0;
            g_game_state.boxes[0].type = BOX_TYPE_UNKNOWN;

            g_game_state.num_destinations = 1;
            g_game_state.destinations[0].x = dest_x;
            g_game_state.destinations[0].y = dest_y;
            g_game_state.destinations[0].grid_x = (int)(dest_x / CELL_SIZE) * 4 + 2;
            g_game_state.destinations[0].grid_y = (int)(dest_y / CELL_SIZE) * 4 + 2;
            g_game_state.destinations[0].assigned_box_id = 0;
            g_game_state.destinations[0].required_digit = 0;

            // 刷新网格地图
            refresh_grid_map(&g_game_state, &g_grid_map);

            // 获取当前小车位置
            float car_x = position.x_m;
            float car_y = position.y_m;

									char buf[128];
			rt_sprintf(buf, "Car pos (%d,%d) Box (%d,%d) Dest (%d,%d)\r\n",
								 (int)(car_x*1000), (int)(car_y*1000),
								 (int)(box_x*1000), (int)(box_y*1000),
								 (int)(dest_x*1000), (int)(dest_y*1000));
			wireless_uart_send_string(buf);
            // 调用轻量级推箱子规划器
            int actions[200];
            wireless_uart_send_string("Calling light_sokoban_plan...\r\n");
            int action_count = light_sokoban_plan(&g_game_state, &g_grid_map, 0,
                                                  car_x, car_y, actions, 200);
            rt_sprintf(buf, "light_sokoban_plan returned %d\r\n", action_count);
            wireless_uart_send_string(buf);

            if (action_count <= 0) {
                wireless_uart_send_string("Sokoban planning failed!\r\n");
                continue;
            }

            // 打印动作序列
            for (int i = 0; i < action_count; i++) {
                rt_sprintf(buf, "action[%d]=%d\r\n", i, actions[i]);
                wireless_uart_send_string(buf);
            }

            // 去掉加4的循环
// for (int i = 0; i < action_count; i++) {
//     actions[i] += 4;
// }


            // 转换为世界坐标路径
            float path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
            int path_len = actions_to_world_path(&g_game_state, &g_grid_map, 0,
                                                 car_x, car_y, actions, action_count,
                                                 path_x, path_y, MAX_PATH_POINTS);
            rt_sprintf(buf, "path_len=%d\r\n", path_len);
            wireless_uart_send_string(buf);

            if (path_len <= 0) {
                wireless_uart_send_string("Path conversion failed!\r\n");
                continue;
            }

            // 将路径存入混合控制器
            for (int i = 0; i < path_len; i++) {
                g_ctrl.current_path[i][0] = path_x[i];
                g_ctrl.current_path[i][1] = path_y[i];
            }
            g_ctrl.path_len = path_len;
            g_ctrl.path_following = 1;
            g_ctrl.is_bomb_path = 0;
            g_ctrl.path_stuck_counter = 0;
            g_ctrl.mode = CTRL_MODE_PATH_FOLLOWING;

            wireless_uart_send_string("Executing Sokoban path...\r\n");

            // 执行路径跟踪
            int wait_count = 0;
            float dist_to_end = 100.0f;
            while (g_ctrl.mode == CTRL_MODE_PATH_FOLLOWING && g_ctrl.path_following && wait_count < 500) {
                Position_Update();
                float cur_x = position.x_m;
                float cur_y = position.y_m;
                float cur_angle = position.yaw_rad;
                float vx, vy, omega;
                if (follow_path(&g_ctrl, cur_x, cur_y, cur_angle, &vx, &vy, &omega, &dist_to_end)) {
                    CarController_SetSpeed(vx, vy, omega);
                    CarController_Update();
                    if (dist_to_end < g_ctrl.path_tolerance) {
                        break;
                    }
                } else {
                    break;
                }
                rt_thread_mdelay(10);
                wait_count++;
            }
            if (wait_count >= 500)
                wireless_uart_send_string("Timeout.\r\n");
            else
                wireless_uart_send_string("Finished.\r\n");

            CarController_Stop();
            g_ctrl.mode = CTRL_MODE_IDLE;
            wireless_uart_send_string("Sokoban test done.\r\n");
        }
        // ========== 控制逻辑 ==========
        if (system_started && yaw_initialized) {
            if (g_ctrl.mode == CTRL_MODE_PATH_FOLLOWING) {
                // 路径跟踪模式
                float car_x = position.x_m;
                float car_y = position.y_m;
                float car_angle = position.yaw_rad;
                float dist_to_end;
                float vx, vy, omega;

                if (follow_path(&g_ctrl, car_x, car_y, car_angle, &vx, &vy, &omega, &dist_to_end)) {
                    CarController_SetSpeed(vx, vy, omega);
                    CarController_Update();

                    // 到达终点则自动退出
                    if (dist_to_end < g_ctrl.path_tolerance) {
                        g_ctrl.mode = CTRL_MODE_IDLE;
                        g_ctrl.path_following = 0;
                        wireless_uart_send_string("Auto finished.\r\n");
                    }
                } else {
                    g_ctrl.mode = CTRL_MODE_IDLE;
                    g_ctrl.path_following = 0;
                    wireless_uart_send_string("Follow failed.\r\n");
                }
            } else {
                // 空闲模式：方向保持 + 可选前进
                float current_yaw = AHRS_GetYaw();
                float angle_error = target_yaw - current_yaw;
                while (angle_error > 180.0f) angle_error -= 360.0f;
                while (angle_error < -180.0f) angle_error += 360.0f;

                float omega_deg = PD(&angle_trace_param, 0, angle_error);
                if (omega_deg > 50.0f) omega_deg = 50.0f;
                else if (omega_deg < -50.0f) omega_deg = -50.0f;
                float omega_rad = omega_deg * DEG_TO_RAD;

                CarController_SetSpeed(vx_target, vy_target, omega_rad);
                CarController_Update();
            }
        } else {
            // 未启动或未初始化时停止电机
            for (int i = 0; i < 3; i++) Motor_SetPWM(i, 0);
        }

        // 监控数据打印（每50ms）
        if (rt_tick_get() - tick > RT_TICK_PER_SECOND / 20) {
            tick = rt_tick_get();
            float actual[3], target[3];
            EncoderGetSpeeds(actual);
            for (int i = 0; i < 3; i++) target[i] = car_ctrl.motors[i].target_speed_mps;

            rt_sprintf(buf,
                "mode=%d kp=%d ki=%d kd=%d akp=%d aki=%d akd=%d "
                "ch1:%d ch2:%d ch3:%d ch4:%d ch5:%d ch6:%d\r\n",
                g_ctrl.mode,
                (int)(pid_kp * 10), (int)(pid_ki * 10), (int)(pid_kd * 10),
                (int)(angle_trace_param.kp * 10), (int)(angle_trace_param.ki * 10), (int)(angle_trace_param.kd * 10),
                (int)(target[0] * 1000), (int)(target[1] * 1000), (int)(target[2] * 1000),
                (int)(actual[0] * 1000), (int)(actual[1] * 1000), (int)(actual[2] * 1000));
            wireless_uart_send_string(buf);
        }

        rt_thread_mdelay(5);   // 控制周期5ms
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

    // 加载测试地图（内部空地）
    const char* test_map_text =
        "################\n"
        "#--------------#\n"
        "#--------------#\n"
        "#--------------#\n"
        "#---------.----#\n"
        "#------$-------#\n"
        "#--------------#\n"
        "#--------------#\n"
        "#--------------#\n"
        "#--------------#\n"
        "#--------------#\n"
        "################\n";
    load_map_from_text(test_map_text, &g_grid_map, &g_game_state);
		// 清空原有箱子和目的地
		g_game_state.num_boxes = 0;
		g_game_state.num_destinations = 0;

		// 添加箱子1
		g_game_state.boxes[0].x = 0.5f;
		g_game_state.boxes[0].y = 0.5f;
		g_game_state.boxes[0].state = 0;
		g_game_state.boxes[0].dest_id = 0;  // 关联目的地0
		g_game_state.boxes[0].type = BOX_TYPE_UNKNOWN;
		g_game_state.num_boxes = 1;

		// 添加箱子2
		g_game_state.boxes[1].x = 1.0f;
		g_game_state.boxes[1].y = 1.0f;
		g_game_state.boxes[1].state = 0;
		g_game_state.boxes[1].dest_id = 1;  // 关联目的地1
		g_game_state.boxes[1].type = BOX_TYPE_UNKNOWN;
		g_game_state.num_boxes = 2;

		// 添加目的地1
		g_game_state.destinations[0].x = 1.5f;
		g_game_state.destinations[0].y = 0.5f;
		g_game_state.destinations[0].assigned_box_id = 0;
		g_game_state.num_destinations = 1;

		// 添加目的地2
		g_game_state.destinations[1].x = 2.0f;
		g_game_state.destinations[1].y = 1.0f;
		g_game_state.destinations[1].assigned_box_id = 1;
		g_game_state.num_destinations = 2;
		
    HybridController_Init(&g_ctrl, &g_grid_map, &g_game_state);
    g_ctrl.max_speed = 0.1f;
    g_ctrl.path_tolerance = 0.05f;

    pit_ms_init(PIT_CH0, 10);
    interrupt_global_enable(0);

    // 创建控制线程
    rt_thread_t tid = rt_thread_create("control", control_thread_entry, NULL,
                                       8192, RT_THREAD_PRIORITY_MAX / 2, 20);
    if (tid) rt_thread_startup(tid);

    // 主线程负责动态调参
    while (1) {
        seekfree_assistant_data_analysis();
        for (uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
            if (seekfree_assistant_parameter_update_flag[i]) {
                seekfree_assistant_parameter_update_flag[i] = 0;
                float val = seekfree_assistant_parameter[i];
                switch (i) {
                    case 0: MotorPID_SetGlobalParams(val, pid_ki, pid_kd); break;
                    case 1: MotorPID_SetGlobalParams(pid_kp, val, pid_kd); break;
                    case 2: MotorPID_SetGlobalParams(pid_kp, pid_ki, val); break;
                    case 3: AnglePID_SetParams(val, angle_ki, angle_kd); break;
                    case 4: AnglePID_SetParams(angle_kp, val, angle_kd); break;
                    case 5: AnglePID_SetParams(angle_kp, angle_ki, val); break;
                }
                char msg[64];
                rt_sprintf(msg, "ch%d updated to %d\r\n", i+1, (int)(val * 1000));
                wireless_uart_send_string(msg);
            }
        }
        rt_thread_mdelay(10);
    }
}