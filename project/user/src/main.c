#include <math.h>
#include <rtthread.h>
#include <stdarg.h>
#include "planner.h"
#include "hybrid_controller.h"
#include "zf_device_wireless_uart.h"

// ========== 全局变量 ==========
GridMap g_grid_map;
GameState g_game_state;
HybridController g_ctrl;

// 路径缓冲区（静态全局，避免栈溢出）
static float path_x[MAX_PATH_POINTS];
static float path_y[MAX_PATH_POINTS];
static int test_path_x[MAX_PATH_POINTS];
static int test_path_y[MAX_PATH_POINTS];
static float bomb_path_x[MAX_PATH_POINTS];
static float bomb_path_y[MAX_PATH_POINTS];
static float box_path_x[MAX_PATH_POINTS];
static float box_path_y[MAX_PATH_POINTS];


// 测试地图（与PC端一致）
static const char* test_map =
"################\n"
"#--------------#\n"
"#--------------#\n"
"#--------------#\n"
"#----$---------#\n"
"#----###########\n"
"#---*#-.-------#\n"
"#----###########\n"
"#--------------#\n"
"#--------------#\n"
"#--------------#\n"
"################\n";

// ========== 无线串口输出函数（不使用浮点格式化） ==========

/**
 * @brief 直接发送字符串
 */
static void uart_send(const char *str) {
    wireless_uart_send_string(str);
}

/**
 * @brief 发送字符串并换行
 */
static void uart_send_line(const char *str) {
    wireless_uart_send_string(str);
    wireless_uart_send_string("\r\n");
}

/**
 * @brief 格式化输出（仅支持整数、字符串等，不支持 %f）
 *        浮点数需提前转换为整数（例如乘以1000）
 */
static void uart_printf(const char *fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    rt_vsprintf(buffer, fmt, args);   // RT-Thread 格式化函数
    va_end(args);
    wireless_uart_send_string(buffer);
}

// ========== 辅助函数：将浮点距离转换为整数（毫米）输出 ==========
static void print_distance(const char *prefix, float dist) {
    int mm = (int)(dist * 1000.0f);
    char buf[64];
    rt_sprintf(buf, "%s%d", prefix, mm);
    wireless_uart_send_string(buf);
}

static void print_point(const char *prefix, float x, float y) {
    int x_mm = (int)(x * 1000.0f);
    int y_mm = (int)(y * 1000.0f);
    char buf[128];
    rt_sprintf(buf, "%s(%d,%d)", prefix, x_mm, y_mm);
    wireless_uart_send_string(buf);
}

// ========== 自动分配目的地 ==========
static void auto_assign_destinations(GameState* state) {
    for (int i = 0; i < state->num_boxes; i++) {
        if (state->boxes[i].dest_id >= 0) continue;
        int best_dest = -1;
        float min_dist_sq = 1e9f;
        for (int j = 0; j < state->num_destinations; j++) {
            if (state->destinations[j].assigned_box_id >= 0) continue;
            float dx = state->boxes[i].x - state->destinations[j].x;
            float dy = state->boxes[i].y - state->destinations[j].y;
            float dist_sq = dx*dx + dy*dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_dest = j;
            }
        }
        if (best_dest >= 0) {
            state->boxes[i].dest_id = best_dest;
            state->destinations[best_dest].assigned_box_id = i;
            // 距离转换为毫米输出
            int dist_mm = (int)(sqrtf(min_dist_sq) * 1000.0f);
            uart_printf("自动分配: 箱子%d -> 目的地%d (距离=%dmm)\r\n",
                        i, best_dest, dist_mm);
        }
    }
}

// ========== 寻找箱子周围可通行粗网格中心 ==========
static int find_coarse_adjacent_target(GameState* state, GridMap* grid_map, int box_id,
                                       float* out_x, float* out_y) {
    int box_r = (int)(state->boxes[box_id].y / CELL_SIZE);
    int box_c = (int)(state->boxes[box_id].x / CELL_SIZE);
    int dr[4] = {-1, 1, 0, 0};
    int dc[4] = {0, 0, -1, 1};

    for (int d = 0; d < 4; d++) {
        int nr = box_r + dr[d];
        int nc = box_c + dc[d];
        if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;

        int base_x = nc * 4;
        int base_y = nr * 4;
        int blocked = 0;
        for (int dy = 0; dy < 4 && !blocked; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                uint8_t occ = grid_map->occupancy[base_y + dy][base_x + dx];
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

// ========== 主函数 ==========
int main(void) {
    // 初始化无线串口
    wireless_uart_init();

    // 发送启动消息，确认串口工作
    uart_send_line("Path Planning Test Start");

    // 加载地图
    load_map_from_text(test_map, &g_grid_map, &g_game_state);
    uart_printf("地图加载完成: 箱子=%d, 目的地=%d, 炸弹=%d\r\n",
                g_game_state.num_boxes, g_game_state.num_destinations, g_game_state.num_bombs);

    auto_assign_destinations(&g_game_state);

    // 小车初始位置
    float car_x = 0.2f, car_y = 0.2f;
    int start_x, start_y;
    world_to_grid(car_x, car_y, &start_x, &start_y);
    uart_printf("小车起点: (%d,%d)mm 网格(%d,%d)\r\n",
                (int)(car_x*1000), (int)(car_y*1000), start_x, start_y);

    // 收集未完成的箱子
    int remaining_boxes[MAX_BOXES];
    int num_remaining = 0;
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].state == 0) {
            remaining_boxes[num_remaining++] = i;
        }
    }

    for (int idx = 0; idx < num_remaining; idx++) {
        int box_id = remaining_boxes[idx];
        uart_printf("\r\n========== 处理箱子 %d ==========\r\n", box_id);

        float target_x, target_y;
        if (!find_coarse_adjacent_target(&g_game_state, &g_grid_map, box_id, &target_x, &target_y)) {
            uart_printf("箱子 %d 周围无可通行粗网格，跳过\r\n", box_id);
            continue;
        }
        int target_grid_x, target_grid_y;
        world_to_grid(target_x, target_y, &target_grid_x, &target_grid_y);
        uart_printf("目标邻近粗网格中心: (%d,%d)mm -> 细网格(%d,%d)\r\n",
                    (int)(target_x*1000), (int)(target_y*1000),
                    target_grid_x, target_grid_y);

        AStarParams params = {5000, 2.0f};
        int test_len = astar_plan_path(&g_grid_map, start_x, start_y,
                                       target_grid_x, target_grid_y,
                                       test_path_x, test_path_y,
                                       MAX_PATH_POINTS, &params);
        uart_printf("直接A*测试结果: %d\r\n", test_len);

        int need_bomb = 0;
        if (test_len <= 0) {
            uart_printf("小车无法到达目标点，可能需要炸墙。\r\n");
            need_bomb = 1;
        } else {
            // 尝试推箱子规划
            int box_path_len = sokoban_plan_for_box(&g_game_state, &g_grid_map, box_id,
                                                     car_x, car_y,
                                                     box_path_x, box_path_y,
                                                     MAX_PATH_POINTS);
            if (box_path_len > 0) {
                uart_printf("箱子 %d 规划成功，路径点数: %d\r\n", box_id, box_path_len);
                for (int i = 0; i < box_path_len; i++) {
                    uart_printf("  (%d,%d)mm\r\n",
                                (int)(box_path_x[i]*1000), (int)(box_path_y[i]*1000));
                }
                g_game_state.boxes[box_id].state = 1;
                uart_printf("箱子 %d 已完成！\r\n", box_id);
                continue;
            } else {
                uart_printf("小车可到达，但推箱子规划失败，可能需要炸墙。\r\n");
                need_bomb = 1;
            }
        }

        if (need_bomb && g_game_state.num_bombs > 0) {
            float bomb_target_x, bomb_target_y;
						int best_wall = select_best_wall_to_destroy(&g_game_state, &g_grid_map,
                                            start_x, start_y,
                                            box_id,                       // 关键修正：用 box_id 替换 target_grid_x, target_grid_y
                                            &bomb_target_x, &bomb_target_y);
            if (best_wall >= 0) {
                uart_printf("发现可炸墙体 %d，尝试利用炸弹...\r\n", best_wall);
                int bomb_id = -1;
                for (int i = 0; i < g_game_state.num_bombs; i++) {
                    if (g_game_state.bombs[i].active) {
                        bomb_id = i;
                        break;
                    }
                }
                if (bomb_id >= 0) {
                    int bomb_path_len = plan_bomb_to_target(&g_game_state, &g_grid_map, bomb_id,
                                                             car_x, car_y,
                                                             bomb_target_x, bomb_target_y,
                                                             bomb_path_x, bomb_path_y,
                                                             MAX_PATH_POINTS);
                    if (bomb_path_len > 0) {
                        uart_printf("炸弹 %d 移动路径 (%d 点):\r\n", bomb_id, bomb_path_len);
                        for (int i = 0; i < bomb_path_len; i++) {
                            uart_printf("  (%d,%d)mm\r\n",
                                        (int)(bomb_path_x[i]*1000), (int)(bomb_path_y[i]*1000));
                        }
                        explode_bomb(&g_game_state, &g_grid_map, bomb_id);
                        uart_printf("炸弹 %d 已爆炸，剩余墙体数量: %d\r\n", bomb_id, g_game_state.num_walls);

                        // 炸墙后重新尝试推箱子
                        int box_path_len = sokoban_plan_for_box(&g_game_state, &g_grid_map, box_id,
                                                                 car_x, car_y,
                                                                 box_path_x, box_path_y,
                                                                 MAX_PATH_POINTS);
                        if (box_path_len > 0) {
                            uart_printf("炸墙后箱子 %d 规划成功，路径点数: %d\r\n", box_id, box_path_len);
                            for (int i = 0; i < box_path_len; i++) {
                                uart_printf("  (%d,%d)mm\r\n",
                                            (int)(box_path_x[i]*1000), (int)(box_path_y[i]*1000));
                            }
                            g_game_state.boxes[box_id].state = 1;
                            uart_printf("箱子 %d 已完成！\r\n", box_id);
                        } else {
                            uart_printf("炸墙后箱子 %d 仍然规划失败。\r\n", box_id);
                        }
                    } else {
                        uart_printf("炸弹 %d 无法规划到目标\r\n", bomb_id);
                    }
                } else {
                    uart_printf("没有可用炸弹\r\n");
                }
            } else {
                uart_printf("无可炸墙体\r\n");
            }
        } else if (need_bomb && g_game_state.num_bombs == 0) {
            uart_printf("无炸弹可用，跳过箱子 %d\r\n", box_id);
        }
    }

    uart_send_line("\r\n所有箱子处理完毕，测试结束。");
		// 获取内存信息并打印
		rt_size_t total, used, max_used;
		rt_memory_info(&total, &used, &max_used);

		char buf[128];
		rt_sprintf(buf, "\r\n内存信息: 总大小 %d, 已用 %d, 最大已用 %d\r\n", total, used, max_used);
wireless_uart_send_string(buf);
    while(1);  // 防止退出
    return 0;
}