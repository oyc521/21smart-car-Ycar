#include "task_manager.h"
#include "hybrid_controller.h"
#include "position.h"
#include "planner.h"
#include "zf_device_wireless_uart.h"
#include <stdio.h>

// 全局变量
TaskManager g_task_mgr;
extern HybridController g_ctrl;
extern GameState g_game_state;
extern Position_t position;
extern GridMap g_grid_map; 
extern uint8_t need_map_update;

// 静态函数声明
static void task_state_machine(void);
static void remove_current_box(void);
static void skip_current_box(void);

void task_manager_init(void)
{
    g_task_mgr.state = TASK_STATE_IDLE;
    g_task_mgr.event = rt_event_create("task", RT_IPC_FLAG_FIFO);
    g_task_mgr.current_box_id = -1;
    g_task_mgr.current_bomb_id = -1;
    g_task_mgr.pending_box_count = 0;
    g_task_mgr.retry_count = 0;
    //wireless_uart_send_string("Task manager initialized.\r\n");
}

void task_manager_start(void)
{
    rt_thread_t tid = rt_thread_create("task_mgr",
                                       task_manager_thread_entry,
                                       RT_NULL,
                                       2048,
                                       RT_THREAD_PRIORITY_MAX / 2 + 2,
                                       20);
    if (tid) {
        rt_thread_startup(tid);
        wireless_uart_send_string("Task manager thread started.\r\n");
    } else {
        wireless_uart_send_string("Failed to create task manager thread.\r\n");
    }
}

void assign_boxes_to_destinations(void)
{
    wireless_uart_send_string("assign_boxes_to_destinations() called.\r\n");
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].state == 0 && g_game_state.boxes[i].dest_id == -1) {
            int best_dest = -1;
            float best_dist = 1e9;
            for (int j = 0; j < g_game_state.num_destinations; j++) {
                if (g_game_state.destinations[j].assigned_box_id == -1) {
                    float dx = g_game_state.boxes[i].x - g_game_state.destinations[j].x;
                    float dy = g_game_state.boxes[i].y - g_game_state.destinations[j].y;
                    float dist = dx*dx + dy*dy;
                    if (dist < best_dist) {
                        best_dist = dist;
                        best_dest = j;
                    }
                }
            }
            if (best_dest >= 0) {
                g_game_state.boxes[i].dest_id = best_dest;
                g_game_state.destinations[best_dest].assigned_box_id = i;
                char buf[64];
                rt_sprintf(buf, "分配箱子 %d -> 目的地 %d (距离 %d)\r\n",
                           i, best_dest, (int)(sqrtf(best_dist) + 0.5f));
                wireless_uart_send_string(buf);
            }
        }
    }
}

static void remove_current_box(void)
{
    if (g_task_mgr.current_box_id < 0) return;
    for (int i = 0; i < g_task_mgr.pending_box_count; i++) {
        if (g_task_mgr.pending_box_ids[i] == g_task_mgr.current_box_id) {
            for (int j = i; j < g_task_mgr.pending_box_count - 1; j++) {
                g_task_mgr.pending_box_ids[j] = g_task_mgr.pending_box_ids[j+1];
            }
            g_task_mgr.pending_box_count--;
            char buf[64];
            rt_sprintf(buf, "Box %d removed from pending, remaining=%d\r\n",
                       g_task_mgr.current_box_id, g_task_mgr.pending_box_count);
            wireless_uart_send_string(buf);
            need_map_update = 1;
            break;
        }
    }
    g_task_mgr.current_box_id = -1;
    g_task_mgr.current_bomb_id = -1;
}

static void skip_current_box(void)
{
    wireless_uart_send_string("Skip current box.\r\n");
    remove_current_box();
    g_task_mgr.retry_count = 0;
    g_task_mgr.state = TASK_STATE_PLANNING_BOX;
}

static int find_active_bomb(void)
{
    for (int i = 0; i < g_game_state.num_bombs; i++) {
        if (g_game_state.bombs[i].active) return i;
    }
    return -1;
}

// 线程入口
void task_manager_thread_entry(void *parameter)
{
    rt_uint32_t recv_set;
    wireless_uart_send_string("Task manager thread running.\r\n");

    while (1) {
        if (rt_event_recv(g_task_mgr.event,
                          TASK_EVENT_CONTROLLER_IDLE | TASK_EVENT_MAP_READY,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_TICK_PER_SECOND / 10,
                          &recv_set) == RT_EOK) {
            if (recv_set & TASK_EVENT_CONTROLLER_IDLE) {
                wireless_uart_send_string("Event: CONTROLLER_IDLE received.\r\n");
                if (g_task_mgr.state == TASK_STATE_EXECUTE_BOX) {
                    int box_id = g_task_mgr.current_box_id;
                    // 检查箱子是否真的到达目的地
                    if (box_id >= 0 && box_id < g_game_state.num_boxes && g_game_state.boxes[box_id].state == 1) {
                        wireless_uart_send_string("Box reached destination, removing.\r\n");
                        remove_current_box();
                        g_task_mgr.retry_count = 0;
                        g_task_mgr.state = TASK_STATE_PLANNING_BOX;
                    } else {
                        // 箱子未到达，重新规划（继续推动）
                        wireless_uart_send_string("Box not at destination, replanning.\r\n");
                        g_task_mgr.state = TASK_STATE_PLANNING_BOX;
                    }
                } else if (g_task_mgr.state == TASK_STATE_EXECUTE_BOMB) {
                    wireless_uart_send_string("Bomb executed, retry Sokoban for box.\r\n");
                    g_task_mgr.state = TASK_STATE_PLANNING_BOX;
                }
            }
            if (recv_set & TASK_EVENT_MAP_READY) {
                wireless_uart_send_string("Event: MAP_READY received.\r\n");
                g_task_mgr.state = TASK_STATE_IDLE;
                g_task_mgr.current_box_id = -1;
                g_task_mgr.current_bomb_id = -1;
                g_task_mgr.pending_box_count = 0;
                g_task_mgr.retry_count = 0;
                HybridController_Reset(&g_ctrl);
                wireless_uart_send_string("Task manager reset due to new map.\r\n");
            }
        }

        task_state_machine();
        rt_thread_mdelay(10);
    }
}

static void task_state_machine(void)
{
    float car_x, car_y;
    static TaskState last_state = -1;

    if (g_task_mgr.state != last_state) {
        char buf[32];
        rt_sprintf(buf, "State machine: state=%d\r\n", g_task_mgr.state);
        wireless_uart_send_string(buf);
        last_state = g_task_mgr.state;
    }

    switch (g_task_mgr.state) {
        case TASK_STATE_IDLE:
            g_task_mgr.pending_box_count = 0;
            for (int i = 0; i < g_game_state.num_boxes; i++) {
                if (g_game_state.boxes[i].state == 0 && g_game_state.boxes[i].dest_id >= 0) {
                    g_task_mgr.pending_box_ids[g_task_mgr.pending_box_count++] = i;
                }
            }
            if (g_task_mgr.pending_box_count > 0) {
                wireless_uart_send_string("Found pending boxes, start planning.\r\n");
                g_task_mgr.state = TASK_STATE_PLANNING_BOX;
            }
            break;

        case TASK_STATE_PLANNING_BOX:
            if (g_task_mgr.pending_box_count == 0) {
                g_task_mgr.state = TASK_STATE_IDLE;
                wireless_uart_send_string("All boxes done, back to IDLE.\r\n");
                break;
            }

            int box_id = g_task_mgr.pending_box_ids[0];
            car_x = position.x_m;
            car_y = position.y_m;
            char buf[128];
            rt_sprintf(buf, "Plan path to box %d, car=(%d,%d)\r\n",
                       box_id, (int)car_x, (int)car_y);
            wireless_uart_send_string(buf);

            if (HybridController_PlanPathToBox(&g_ctrl, car_x, car_y, box_id)) {
                g_task_mgr.retry_count = 0;
                g_task_mgr.current_box_id = box_id;
                g_task_mgr.state = TASK_STATE_EXECUTE_BOX;
                wireless_uart_send_string("Navigation plan success, enter EXECUTE_BOX.\r\n");
            } else {
                g_task_mgr.retry_count++;
                rt_sprintf(buf, "Plan failed, retry_count=%d\r\n", g_task_mgr.retry_count);
                wireless_uart_send_string(buf);
                if (g_task_mgr.retry_count >= 3) {
                    wireless_uart_send_string("Retry limit reached, skip box.\r\n");
                    skip_current_box();
                    g_task_mgr.retry_count = 0;
                    break;
                }
                // 尝试炸弹（省略，保持原有）
                wireless_uart_send_string("Direct plan failed, try bomb.\r\n");
                int bomb_id = find_active_bomb();
                if (bomb_id >= 0) {
                    float car_img_x = car_y;
                    float car_img_y = car_x;
                    int car_fine_x = (int)(car_img_x / RESOLUTION + 0.5f);
                    int car_fine_y = (int)(car_img_y / RESOLUTION + 0.5f);
                    float bomb_target_x, bomb_target_y;
                    int best_wall = select_best_wall_to_destroy(&g_game_state, &g_grid_map,
                                                                car_fine_x, car_fine_y,
                                                                box_id,
                                                                &bomb_target_x, &bomb_target_y);
                    if (best_wall >= 0) {
                        rt_sprintf(buf, "Found best wall %d, bomb target (%d,%d)\r\n",
                                   best_wall, (int)bomb_target_x, (int)bomb_target_y);
                        wireless_uart_send_string(buf);
                        if (HybridController_PlanBomb(&g_ctrl, bomb_id, car_x, car_y,
                                                      bomb_target_x, bomb_target_y)) {
                            g_task_mgr.retry_count = 0;
                            g_task_mgr.current_box_id = box_id;
                            g_task_mgr.current_bomb_id = bomb_id;
                            g_task_mgr.state = TASK_STATE_PLANNING_BOMB;
                            wireless_uart_send_string("Bomb plan success, enter PLANNING_BOMB.\r\n");
                        } else {
                            g_task_mgr.retry_count++;
                            if (g_task_mgr.retry_count >= 3) {
                                skip_current_box();
                                g_task_mgr.retry_count = 0;
                            }
                        }
                    } else {
                        g_task_mgr.retry_count++;
                        if (g_task_mgr.retry_count >= 3) skip_current_box();
                    }
                } else {
                    g_task_mgr.retry_count++;
                    if (g_task_mgr.retry_count >= 3) skip_current_box();
                }
            }
            break;

        case TASK_STATE_EXECUTE_BOX:
            break;

        case TASK_STATE_PLANNING_BOMB:
            g_task_mgr.state = TASK_STATE_EXECUTE_BOMB;
            wireless_uart_send_string("Enter EXECUTE_BOMB state.\r\n");
            break;

        case TASK_STATE_EXECUTE_BOMB:
            break;

        default:
            g_task_mgr.state = TASK_STATE_IDLE;
            break;
    }
}