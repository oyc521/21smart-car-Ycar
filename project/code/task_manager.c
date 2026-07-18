#include "task_manager.h"
#include "hybrid_controller.h"
#include "position.h"
#include "planner.h"
#include "mapping.h"
#include "zf_device_wireless_uart.h"
#include "motor.h"
#include "uart_receiver.h"
#include "encoder.h"
#include <stdio.h>

/* ========== 回库功能配置 ========== */
#define ENABLE_RETURN_HOME   1
#define HOME_X               0.2f
#define HOME_Y               1.1f

extern HybridController g_ctrl;
extern GameState g_game_state;
extern Position_t position;
extern GridMap g_grid_map;
extern volatile uint8_t need_map_update;
extern volatile uint8_t waiting_map;
extern rt_mutex_t g_map_mutex;
extern int select_best_wall_to_destroy(GameState* state, GridMap* grid_map,
                                       int car_fine_x, int car_fine_y,
                                       int box_id, float* out_x, float* out_y,
                                       int* out_push_dir);
extern int is_boundary_wall(Wall* wall);
extern void request_new_map(void);
extern void AHRS_ResetYaw(void);   // 重置航向（陀螺仪）

DigitMap_t g_digit_map[MAX_DIGITS];
int g_digit_map_count = 0;
TaskManager g_task_mgr;

static void task_state_machine(void);
static void rebuild_pending_boxes(void);
static int calculate_push_stance_for_dir(GameState* state, int obj_id, int obj_type,
                                          int push_dir, float* out_x, float* out_y);
static void assign_single_box(int box_id);
static void system_reset_for_stage2(void);
static void system_reset_for_mode(TaskMode_t target_mode);
static int find_active_bomb(void);   // 查找活跃炸弹

/* ---------- 辅助函数 ---------- */
int find_unrecognized_destination(int *out_idx) {
    for (int i = 0; i < g_game_state.num_destinations; i++) {
        if (!g_game_state.destinations[i].recognized) {
            *out_idx = i;
            return 1;
        }
    }
    return 0;
}

int find_unrecognized_box(int *out_idx) {
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (!g_game_state.boxes[i].recognized) {
            *out_idx = i;
            return 1;
        }
    }
    return 0;
}

int find_unpushed_box_for_dest_digit(int dest_digit, int *out_idx) {
    BoxTypeEnum_t target_type = mapping_get_box_type_for_digit(dest_digit);
    if (target_type == BOX_TYPE_UNKNOWN) return 0;
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].state == 0 &&
            g_game_state.boxes[i].recognized &&
            g_game_state.boxes[i].type == target_type) {
            *out_idx = i;
            return 1;
        }
    }
    return 0;
}

static uint8_t recog_fail_count = 0;

// 查找活跃炸弹
static int find_active_bomb(void) {
    for (int i = 0; i < g_game_state.num_bombs; i++) {
        if (g_game_state.bombs[i].active) return i;
    }
    return -1;
}

// 为单个箱子分配目的地（根据箱子类型匹配数字映射）
static void assign_single_box(int box_id) {
    BoxTypeEnum_t box_type = g_game_state.boxes[box_id].type;
    for (int i = 0; i < g_digit_map_count; i++) {
        int digit = g_digit_map[i].digit;
        BoxTypeEnum_t target_type = mapping_get_box_type_for_digit(digit);
        if (target_type == box_type) {
            int dest_id = g_digit_map[i].dest_id;
            if (g_game_state.destinations[dest_id].assigned_box_id == -1) {
                g_game_state.boxes[box_id].dest_id = dest_id;
                g_game_state.destinations[dest_id].assigned_box_id = box_id;
                char buf[64];
                rt_sprintf(buf, "[Single] Box %d (type %d) -> dest %d (digit %d)\r\n",
                           box_id, box_type, dest_id, digit);
                wireless_uart_send_string(buf);
                return;
            }
        }
    }
    int best_dest = -1;
    float best_dist = 1e9;
    for (int j = 0; j < g_game_state.num_destinations; j++) {
        if (g_game_state.destinations[j].assigned_box_id == -1) {
            float dx = g_game_state.boxes[box_id].x - g_game_state.destinations[j].x;
            float dy = g_game_state.boxes[box_id].y - g_game_state.destinations[j].y;
            float dist = dx*dx + dy*dy;
            if (dist < best_dist) {
                best_dist = dist;
                best_dest = j;
            }
        }
    }
    if (best_dest >= 0) {
        g_game_state.boxes[box_id].dest_id = best_dest;
        g_game_state.destinations[best_dest].assigned_box_id = box_id;
        char buf[64];
        rt_sprintf(buf, "[Single] Box %d -> dest %d (distance)\r\n", box_id, best_dest);
        wireless_uart_send_string(buf);
    } else {
        wireless_uart_send_string("[Single] No available destination for box!\r\n");
    }
}

void start_next_recognition(void) {
    int target_id;
    int target_grid_x, target_grid_y;
    RecognTargetType_t recog_type;

    if (!g_task_mgr.all_dest_recognized) {
        if (find_unrecognized_destination(&target_id)) {
            Destination* dest = &g_game_state.destinations[target_id];
            float img_x, img_y;
            motion_to_image(dest->x, dest->y, &img_x, &img_y);
            target_grid_x = (int)(img_x / CELL_SIZE);
            target_grid_y = (int)(img_y / CELL_SIZE);
            recog_type = RECOG_TARGET_DEST;
        } else {
            g_task_mgr.all_dest_recognized = 1;
            wireless_uart_send_string("[TaskMgr] All destinations recognized.\r\n");
            start_next_recognition();
            return;
        }
    }
    else if (!g_task_mgr.all_box_recognized) {
        if (find_unrecognized_box(&target_id)) {
            Box* box = &g_game_state.boxes[target_id];
            float img_x, img_y;
            motion_to_image(box->x, box->y, &img_x, &img_y);
            target_grid_x = (int)(img_x / CELL_SIZE);
            target_grid_y = (int)(img_y / CELL_SIZE);
            recog_type = RECOG_TARGET_BOX;
        } else {
            g_task_mgr.all_box_recognized = 1;
            wireless_uart_send_string("[TaskMgr] All boxes recognized.\r\n");
            g_task_mgr.state = TASK_STATE_ASSIGN_BOXES;
            return;
        }
    } else {
        g_task_mgr.state = TASK_STATE_ASSIGN_BOXES;
        return;
    }

    g_task_mgr.current_recog_target_id = target_id;
    g_task_mgr.current_recog_type = recog_type;

    if (HybridController_NavigateAndRecognize(&g_ctrl, target_grid_x, target_grid_y,
                                              recog_type, target_id)) {
        char buf[64];
        rt_sprintf(buf, "[TaskMgr] Navigating for %s recog, id=%d\r\n",
                   recog_type == RECOG_TARGET_DEST ? "dest" : "box", target_id);
        wireless_uart_send_string(buf);
        recog_fail_count = 0;
    } else {
        recog_fail_count++;
        if (recog_fail_count >= 3) {
            if (recog_type == RECOG_TARGET_DEST) {
                g_game_state.destinations[target_id].recognized = 1;
            } else {
                g_game_state.boxes[target_id].recognized = 1;
            }
            recog_fail_count = 0;
            wireless_uart_send_string("[TaskMgr] Recognition failed repeatedly, skip.\r\n");
        } else if (recog_fail_count == 1) {
            // 首次失败：尝试用炸弹炸开挡路的墙（排除边界墙）
            int bomb_id = find_active_bomb();
            if (bomb_id >= 0) {
                float bomb_ix, bomb_iy;
                motion_to_image(g_game_state.bombs[bomb_id].x, g_game_state.bombs[bomb_id].y, &bomb_ix, &bomb_iy);
                int bomb_r = (int)(bomb_iy / CELL_SIZE);
                int bomb_c = (int)(bomb_ix / CELL_SIZE);
                int best_wall = -1, best_bomb_dist = 1e9;
                float best_target_x = 0, best_target_y = 0;
                int car_fx = (int)(position.x_m / RESOLUTION + 0.5f);
                int car_fy = (int)(position.y_m / RESOLUTION + 0.5f);
                if (car_fx < 0) car_fx = 0; if (car_fx >= FINE_COLS) car_fx = FINE_COLS - 1;
                if (car_fy < 0) car_fy = 0; if (car_fy >= FINE_ROWS) car_fy = FINE_ROWS - 1;

                // 加锁保护临时修改地图
                if (g_map_mutex) rt_mutex_take(g_map_mutex, RT_WAITING_FOREVER);

                for (int w = 0; w < g_game_state.num_walls; w++) {
                    Wall* wall = &g_game_state.walls[w];
                    if (is_boundary_wall(wall)) continue;
                    float wx, wy;
                    motion_to_image(wall->x1, wall->y1, &wx, &wy);
                    int wr = (int)(wy / CELL_SIZE + 0.5f);
                    int wc = (int)(wx / CELL_SIZE + 0.5f);
                    int base_x = wc * 4, base_y = wr * 4;
                    uint8_t saved[4][4];
                    for (int dy = 0; dy < 4; dy++)
                        for (int dx = 0; dx < 4; dx++)
                            saved[dy][dx] = g_grid_map.occupancy[base_y+dy][base_x+dx];
                    // 临时移除墙壁
                    for (int dy = 0; dy < 4; dy++)
                        for (int dx = 0; dx < 4; dx++)
                            g_grid_map.occupancy[base_y+dy][base_x+dx] = OCC_FREE;

                    const int dr[4] = {-1, 0, 1, 0};
                    const int dc[4] = {0, 1, 0, -1};
                    int stance_r = -1, stance_c = -1;
                    for (int d = 0; d < 4 && stance_r < 0; d++) {
                        int nr = target_grid_y + dr[d], nc = target_grid_x + dc[d];
                        if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
                        int occ = 0;
                        for (int dy = 0; dy < 4 && !occ; dy++)
                            for (int dx = 0; dx < 4; dx++)
                                if (g_grid_map.occupancy[nr*4+dy][nc*4+dx] != OCC_FREE) occ = 1;
                        if (!occ) { stance_r = nr; stance_c = nc; }
                    }

                    if (stance_r >= 0) {
                        int gx = stance_c * 4 + 2, gy = stance_r * 4 + 2;
                        static int qx[FINE_COLS * FINE_ROWS];
                        static int qy[FINE_COLS * FINE_ROWS];
                        static uint8_t vis[FINE_ROWS][FINE_COLS];
                        memset(vis, 0, sizeof(vis));
                        const int bdx[8] = {-1,0,1,-1,1,-1,0,1};
                        const int bdy[8] = {-1,-1,-1,0,0,1,1,1};
                        int head = 0, tail = 0;
                        qx[tail] = car_fx; qy[tail] = car_fy; tail++;
                        vis[car_fy][car_fx] = 1;
                        while (head < tail) {
                            int x = qx[head], y = qy[head]; head++;
                            if (x == gx && y == gy) {
                                int bomb_dist = abs(bomb_r - wr) + abs(bomb_c - wc);
                                if (bomb_dist < best_bomb_dist) {
                                    best_bomb_dist = bomb_dist;
                                    best_wall = w;
                                    best_target_x = (wc + 0.5f) * CELL_SIZE;
                                    best_target_y = (wr + 0.5f) * CELL_SIZE;
                                }
                                break;
                            }
                            for (int d = 0; d < 8; d++) {
                                int nx = x + bdx[d], ny = y + bdy[d];
                                if (nx < 0 || nx >= FINE_COLS || ny < 0 || ny >= FINE_ROWS) continue;
                                if (vis[ny][nx]) continue;
                                uint8_t occ = g_grid_map.occupancy[ny][nx];
                                if (occ == OCC_WALL || occ == OCC_BOX || occ == OCC_BOMB) continue;
                                if (bdx[d] != 0 && bdy[d] != 0) {
                                    if (g_grid_map.occupancy[y][nx] == OCC_WALL ||
                                        g_grid_map.occupancy[y][nx] == OCC_BOX) continue;
                                    if (g_grid_map.occupancy[ny][x] == OCC_WALL ||
                                        g_grid_map.occupancy[ny][x] == OCC_BOX) continue;
                                }
                                vis[ny][nx] = 1;
                                if (tail < FINE_COLS * FINE_ROWS) {
                                    qx[tail] = nx; qy[tail] = ny; tail++;
                                } else {
                                    // 队列满，强制退出（实际不会发生）
                                    break;
                                }
                            }
                        }
                    }

                    // 恢复墙壁
                    for (int dy = 0; dy < 4; dy++)
                        for (int dx = 0; dx < 4; dx++)
                            g_grid_map.occupancy[base_y+dy][base_x+dx] = saved[dy][dx];
                }

                if (g_map_mutex) rt_mutex_release(g_map_mutex);

                if (best_wall >= 0) {
                    float bomb_target_x, bomb_target_y;
                    image_to_motion(best_target_x, best_target_y, &bomb_target_x, &bomb_target_y);
                    int bomb_success = EvaluateBestStance(&g_game_state, &g_grid_map,
                                        bomb_id, OBJ_BOMB,
                                        bomb_target_x, bomb_target_y,
                                        &g_ctrl.precomputed_stand_x,
                                        &g_ctrl.precomputed_stand_y,
                                        g_ctrl.precomputed_actions,
                                        &g_ctrl.precomputed_count);
                    if (bomb_success) {
                        memcpy(g_task_mgr.action_queue, g_ctrl.precomputed_actions,
                               g_ctrl.precomputed_count * sizeof(int));
                        g_task_mgr.action_total = g_ctrl.precomputed_count;
                        g_task_mgr.action_index = 0;
                        g_task_mgr.current_bomb_id = bomb_id;
                        g_ctrl.current_box_id = -1;
                        g_ctrl.current_bomb_id = bomb_id;
                        g_ctrl.is_bomb_path = 1;
                        g_ctrl.bomb_target_pos[0] = bomb_target_x;
                        g_ctrl.bomb_target_pos[1] = bomb_target_y;
                        if (HybridController_PlanPathToPoint(&g_ctrl,
                                position.x_m, position.y_m,
                                g_ctrl.precomputed_stand_x,
                                g_ctrl.precomputed_stand_y)) {
                            recog_fail_count = 0;   // 重置失败计数
                            g_task_mgr.state = TASK_STATE_EXECUTE_BOX;
                            wireless_uart_send_string("[TaskMgr] Bomb path for recog.\r\n");
                            return;
                        }
                    }
                }
            }
            wireless_uart_send_string("[TaskMgr] Recognition failed, will retry.\r\n");
            return;
        } else {
            wireless_uart_send_string("[TaskMgr] Recognition failed, will retry.\r\n");
            return;
        }
        start_next_recognition();
    }
}

void assign_boxes_to_destinations(void) {
    wireless_uart_send_string("assign_boxes_to_destinations() called.\r\n");

    for (int i = 0; i < g_digit_map_count; i++) {
        int digit = g_digit_map[i].digit;
        int dest_id = g_digit_map[i].dest_id;
        int box_idx;
        if (find_unpushed_box_for_dest_digit(digit, &box_idx)) {
            g_game_state.boxes[box_idx].dest_id = dest_id;
            g_game_state.destinations[dest_id].assigned_box_id = box_idx;
            char buf[64];
            rt_sprintf(buf, "Mapped box %d (digit %d) -> dest %d\r\n", box_idx, digit, dest_id);
            wireless_uart_send_string(buf);
        }
    }

    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].state == 0 && g_game_state.boxes[i].dest_id == -1) {
            if (g_task_mgr.mode == TASK_MODE_STAGE2 &&
                g_game_state.boxes[i].type == BOX_TYPE_UNKNOWN) continue;
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
                rt_sprintf(buf, "Box %d -> dest %d (distance)\r\n", i, best_dest);
                wireless_uart_send_string(buf);
            }
        }
    }
}

static void rebuild_pending_boxes(void) {
    if (g_map_mutex && rt_mutex_take(g_map_mutex, 500) == RT_EOK) {
        g_task_mgr.pending_box_count = 0;
        for (int i = 0; i < g_game_state.num_boxes; i++) {
            if (g_game_state.boxes[i].state == 0 && g_game_state.boxes[i].dest_id >= 0) {
                g_task_mgr.pending_box_ids[g_task_mgr.pending_box_count++] = i;
            }
        }
        rt_mutex_release(g_map_mutex);
    } else {
        wireless_uart_send_string("[TaskMgr] rebuild_pending_boxes: mutex timeout!\r\n");
    }
}

static int calculate_push_stance_for_dir(GameState* state, int obj_id, int obj_type,
                                          int push_dir, float* out_x, float* out_y) {
    float obj_x, obj_y;
    if (obj_type == OBJ_BOX) {
        obj_x = state->boxes[obj_id].x;
        obj_y = state->boxes[obj_id].y;
    } else {
        obj_x = state->bombs[obj_id].x;
        obj_y = state->bombs[obj_id].y;
    }
    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};
    const int opposite[4] = {2, 3, 0, 1};
    int back_dir = opposite[push_dir];
    float img_x, img_y;
    motion_to_image(obj_x, obj_y, &img_x, &img_y);
    int obj_c = (int)(img_x / CELL_SIZE);
    int obj_r = (int)(img_y / CELL_SIZE);
    int nr = obj_r + dr[back_dir];
    int nc = obj_c + dc[back_dir];
    if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) return 0;

    int blocked = 0;
    if (g_map_mutex && rt_mutex_take(g_map_mutex, 500) == RT_EOK) {
        int base_x = nc * 4, base_y = nr * 4;
        for (int dy = 0; dy < 4 && !blocked; dy++)
            for (int dx = 0; dx < 4; dx++)
                if (g_grid_map.occupancy[base_y+dy][base_x+dx] == OCC_WALL ||
                    g_grid_map.occupancy[base_y+dy][base_x+dx] == OCC_BOX)
                    blocked = 1;
        rt_mutex_release(g_map_mutex);
    } else {
        wireless_uart_send_string("[TaskMgr] calculate_push_stance: mutex timeout!\r\n");
        return 0;
    }
    if (blocked) return 0;

    float new_img_x = (nc + 0.5f) * CELL_SIZE;
    float new_img_y = (nr + 0.5f) * CELL_SIZE;
    image_to_motion(new_img_x, new_img_y, out_x, out_y);
    return 1;
}

uint8_t IsControllerBusy(void) {
    return (g_ctrl.mode != CTRL_MODE_IDLE);
}

void task_manager_init(void) {
    g_task_mgr.state = TASK_STATE_IDLE;
    g_task_mgr.mode = TASK_MODE_STAGE1;
    g_task_mgr.event = rt_event_create("task", RT_IPC_FLAG_FIFO);
    g_task_mgr.current_box_id = -1;
    g_task_mgr.current_bomb_id = -1;
    g_task_mgr.pending_box_count = 0;
    g_task_mgr.retry_count = 0;
    g_task_mgr.wait_start_tick = 0;
    g_task_mgr.current_recog_target_id = -1;
    g_task_mgr.all_dest_recognized = 0;
    g_task_mgr.all_box_recognized = 0;
    g_task_mgr.action_total = 0;
    g_task_mgr.action_index = 0;
    g_task_mgr.last_push_end_idx = 0;
    recog_fail_count = 0;
}

void task_manager_set_mode(TaskMode_t mode) {
    g_task_mgr.mode = mode;
}

void task_manager_start(void) {
    rt_thread_t tid = rt_thread_create("task_mgr",
                                       task_manager_thread_entry,
                                       RT_NULL,
                                       8192,
                                       RT_THREAD_PRIORITY_MAX / 2 + 2,
                                       20);
    if (tid) {
        rt_thread_startup(tid);
        wireless_uart_send_string("Task manager thread started.\r\n");
    } else {
        wireless_uart_send_string("Failed to create task manager thread.\r\n");
    }
}

void task_manager_thread_entry(void *parameter) {
    rt_uint32_t recv_set;
    wireless_uart_send_string("Task manager thread running.\r\n");

    while (1) {
        if (rt_event_recv(g_task_mgr.event,
                          TASK_EVENT_CONTROLLER_IDLE | TASK_EVENT_MAP_READY |
                          TASK_EVENT_RECOG_DONE | TASK_EVENT_RECOG_FAILED |
                          0x80,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER,
                          &recv_set) == RT_EOK) {

            if (recv_set & 0x80) {
            }

            if (recv_set & TASK_EVENT_RECOG_DONE) {
                if (g_task_mgr.state == TASK_STATE_RECOGNIZE_DEST) {
                    start_next_recognition();
                } else if (g_task_mgr.state == TASK_STATE_RECOGNIZE_BOX) {
                    g_task_mgr.state = TASK_STATE_ASSIGN_BOXES;
                }
            }

            if (recv_set & TASK_EVENT_RECOG_FAILED) {
                if (g_task_mgr.state == TASK_STATE_RECOGNIZE_DEST ||
                    g_task_mgr.state == TASK_STATE_RECOGNIZE_BOX) {
                    start_next_recognition();
                }
            }

            if (recv_set & TASK_EVENT_CONTROLLER_IDLE) {
                if (g_task_mgr.state == TASK_STATE_MOVE_OUT) {
                    if (g_task_mgr.mode == TASK_MODE_STAGE1) {
                        g_task_mgr.state = TASK_STATE_WAIT_MAP;
                    } else {
                        g_task_mgr.state = TASK_STATE_IDLE;
                    }
                    rt_thread_mdelay(500);//停稳等待地图更新
                    waiting_map = 1;//等待请求
                    request_new_map();
                    wireless_uart_send_string("[TaskMgr] Moved out, requesting map...\r\n");
                } else if (g_task_mgr.state == TASK_STATE_RETURNING) {
                    if (g_ctrl.complete_reason == CTRL_COMPLETE_SUCCESS) {
                        if (g_task_mgr.mode == TASK_MODE_STAGE1) {
                            system_reset_for_mode(TASK_MODE_STAGE2);
                        } else if (g_task_mgr.mode == TASK_MODE_STAGE2) {
                            system_reset_for_mode(TASK_MODE_STAGE3);
                        } else {
                            RotateToAngleIMU(0.0f);
                            g_task_mgr.state = TASK_STATE_IDLE;
                            wireless_uart_send_string("[Return] All stages completed.\r\n");
                        }
                    } else {
                        wireless_uart_send_string("[Return] Failed, will retry.\r\n");
                        g_task_mgr.retry_count++;
                        if (g_task_mgr.retry_count >= 3) {
                            wireless_uart_send_string("[Return] Abort.\r\n");
                            g_task_mgr.state = TASK_STATE_IDLE;
                        } else {
                            g_task_mgr.state = TASK_STATE_RETURNING;
                        }
                    }
                }
            }

            if (recv_set & TASK_EVENT_MAP_READY) {
                if (g_task_mgr.state == TASK_STATE_WAIT_MAP) {
                    // 视觉校正
                    if (is_vision_valid()) {
                        float vision_x, vision_y;
                        get_vision_position(&vision_x, &vision_y);
                        Position_Set(vision_x, vision_y, position.yaw_rad);
                        EncoderReset();
                        wireless_uart_send_string("[TaskMgr] Vision reset on map ready.\r\n");
                        clear_vision_valid();
                    }

                    rebuild_pending_boxes();

                    if (g_task_mgr.action_total > 0 && g_task_mgr.action_index < g_task_mgr.action_total) {
                        g_task_mgr.state = TASK_STATE_EXECUTE_BOX;
                    } else if (g_task_mgr.pending_box_count > 0) {
                        g_task_mgr.state = TASK_STATE_PLANNING_BOX;
                    } else {
                        if (g_task_mgr.mode == TASK_MODE_STAGE1) {
                            g_task_mgr.state = TASK_STATE_RETURNING;
                            wireless_uart_send_string("[TaskMgr] Stage1 done. Returning home.\r\n");
                        } else {
                            g_task_mgr.state = TASK_STATE_RETURNING;
                            wireless_uart_send_string("[TaskMgr] Stage done. Returning home.\r\n");
                        }
                    }
                } else if (g_task_mgr.state == TASK_STATE_IDLE) {
                    HybridController_Reset(&g_ctrl);
                    if (g_task_mgr.mode == TASK_MODE_STAGE1) {
                        rebuild_pending_boxes();
                        if (g_task_mgr.pending_box_count > 0) {
                            g_task_mgr.state = TASK_STATE_PLANNING_BOX;
                        }
                    } else { // STAGE2/STAGE3
                        g_task_mgr.all_dest_recognized = 0;
                        g_task_mgr.all_box_recognized = 0;
                        g_digit_map_count = 0;
                        for (int i = 0; i < g_game_state.num_destinations; i++)
                            g_game_state.destinations[i].recognized = 0;
                        for (int i = 0; i < g_game_state.num_boxes; i++)
                            g_game_state.boxes[i].recognized = 0;
                        g_task_mgr.state = TASK_STATE_RECOGNIZE_DEST;
                        start_next_recognition();
                    }
                }
            }
        }

        task_state_machine();
        rt_thread_mdelay(10);
    }
}

static void task_state_machine(void) {
    float car_x, car_y;

    switch (g_task_mgr.state) {
        case TASK_STATE_IDLE:
        case TASK_STATE_WAIT_MAP:
        case TASK_STATE_RECOGNIZE_DEST:
        case TASK_STATE_RECOGNIZE_BOX:
            break;

        case TASK_STATE_MOVE_OUT: {
            if (g_ctrl.mode != CTRL_MODE_IDLE) break;
            float cx = position.x_m;
            float cy = position.y_m;
            g_ctrl.current_path[0][0] = cx;
            g_ctrl.current_path[0][1] = cy;
            g_ctrl.current_path[1][0] = cx + 0.20f;
            g_ctrl.current_path[1][1] = cy;
            g_ctrl.path_len = 2;
            g_ctrl.path_following = 1;
            g_ctrl.path_purpose = PATH_PURPOSE_MOVE_ACTION;
            g_ctrl.use_tangent_heading = 0;
            g_ctrl.path_locked_yaw = 0.0f;
            g_ctrl.max_speed = 0.10f;
            g_ctrl.min_speed = 0.06f;
            g_ctrl.path_tolerance = 0.03f;
            g_ctrl.push_smoothed_speed = 0.0f;
            g_ctrl.axial_tracking = 0;
            PID_Reset(&angle_trace_param);
            g_ctrl.mode = CTRL_MODE_PATH_FOLLOWING;
            wireless_uart_send_string("[TaskMgr] Moving out of garage...\r\n");
            break;
        }

        case TASK_STATE_RETURNING: {
#if ENABLE_RETURN_HOME
            if (waiting_map) break;
            if (g_ctrl.mode != CTRL_MODE_IDLE) break;

            car_x = position.x_m;
            car_y = position.y_m;

            if (!HybridController_PlanPathToPoint(&g_ctrl, car_x, car_y, HOME_X, HOME_Y)) {
                g_task_mgr.retry_count++;
                if (g_task_mgr.retry_count >= 3) {
                    wireless_uart_send_string("[Return] Navigation failed, abort.\r\n");
                    g_task_mgr.state = TASK_STATE_IDLE;
                } else {
                    rt_thread_mdelay(200);
                }
            } else {
                g_task_mgr.retry_count = 0;
                g_ctrl.current_box_id = -1;
                g_ctrl.is_bomb_path = 0;
                wireless_uart_send_string("[Return] Going home...\r\n");
            }
#endif
            break;
        }

        case TASK_STATE_ASSIGN_BOXES: {
            if (g_map_mutex && rt_mutex_take(g_map_mutex, 500) == RT_EOK) {
                if (g_task_mgr.mode == TASK_MODE_STAGE2 &&
                    g_task_mgr.current_recog_target_id >= 0 &&
                    !g_task_mgr.all_box_recognized) {
                    int box_id = g_task_mgr.current_recog_target_id;
                    if (g_game_state.boxes[box_id].recognized && g_game_state.boxes[box_id].dest_id == -1) {
                        assign_single_box(box_id);
                    }
                    g_task_mgr.current_recog_target_id = -1;
                } else {
                    assign_boxes_to_destinations();
                }
                rt_mutex_release(g_map_mutex);
            } else {
                wireless_uart_send_string("[TaskMgr] assign_boxes: mutex timeout!\r\n");
            }
            rebuild_pending_boxes();
            if (g_task_mgr.pending_box_count > 0) {
                g_task_mgr.state = TASK_STATE_PLANNING_BOX;
            } else {
                g_task_mgr.state = TASK_STATE_IDLE;
            }
            break;
        }

        case TASK_STATE_PLANNING_BOX: {
            if (waiting_map) break;
            if (g_task_mgr.pending_box_count == 0) {
                g_task_mgr.state = TASK_STATE_IDLE;
                break;
            }

            int box_id = g_task_mgr.pending_box_ids[0];
            car_x = position.x_m;
            car_y = position.y_m;

            static uint8_t plan_retry = 0;
            int success = 0;
            if (g_ctrl.precomputed_count == 0) {
                success = EvaluateBestStance(&g_game_state, &g_grid_map,
                                             box_id, OBJ_BOX,
                                             0, 0,
                                             &g_ctrl.precomputed_stand_x,
                                             &g_ctrl.precomputed_stand_y,
                                             g_ctrl.precomputed_actions,
                                             &g_ctrl.precomputed_count);
            }

            if (success) {
                plan_retry = 0;
                memcpy(g_task_mgr.action_queue, g_ctrl.precomputed_actions,
                       g_ctrl.precomputed_count * sizeof(int));
                g_task_mgr.action_total = g_ctrl.precomputed_count;
                g_task_mgr.action_index = 0;

                if (HybridController_PlanPathToPoint(&g_ctrl, car_x, car_y,
                                                     g_ctrl.precomputed_stand_x,
                                                     g_ctrl.precomputed_stand_y)) {
                    g_ctrl.path_locked_yaw = 0.0f;
                    g_task_mgr.retry_count = 0;
                    g_task_mgr.current_box_id = box_id;
                    g_task_mgr.current_bomb_id = -1;
                    g_ctrl.current_box_id = box_id;
                    g_ctrl.current_bomb_id = -1;
                    g_ctrl.is_bomb_path = 0;
                    g_task_mgr.state = TASK_STATE_EXECUTE_BOX;
                } else {
                    plan_retry++;
                    if (plan_retry >= 3) {
                        g_task_mgr.state = TASK_STATE_IDLE;
                        g_ctrl.complete_reason = CTRL_COMPLETE_FAIL_PLAN;
                        plan_retry = 0;
                    } else {
                        rt_thread_mdelay(200);
                    }
                }
            } else {
                int bomb_id = find_active_bomb();
                if (bomb_id >= 0) {
                    float bomb_target_x, bomb_target_y;
                    int push_dir;
                    int wall_idx = select_best_wall_to_destroy(&g_game_state, &g_grid_map,
                                                               (int)(car_x / RESOLUTION + 0.5f),
                                                               (int)(car_y / RESOLUTION + 0.5f),
                                                               box_id,
                                                               &bomb_target_x, &bomb_target_y,
                                                               &push_dir);
                    if (wall_idx >= 0) {
                        int bomb_success = EvaluateBestStance(&g_game_state, &g_grid_map,
                                                              bomb_id, OBJ_BOMB,
                                                              bomb_target_x, bomb_target_y,
                                                              &g_ctrl.precomputed_stand_x,
                                                              &g_ctrl.precomputed_stand_y,
                                                              g_ctrl.precomputed_actions,
                                                              &g_ctrl.precomputed_count);
                        if (bomb_success) {
                            memcpy(g_task_mgr.action_queue, g_ctrl.precomputed_actions,
                                   g_ctrl.precomputed_count * sizeof(int));
                            g_task_mgr.action_total = g_ctrl.precomputed_count;
                            g_task_mgr.action_index = 0;

                            if (HybridController_PlanPathToPoint(&g_ctrl, car_x, car_y,
                                                                 g_ctrl.precomputed_stand_x,
                                                                 g_ctrl.precomputed_stand_y)) {
                                g_ctrl.path_locked_yaw = 0.0f;
                                g_task_mgr.retry_count = 0;
                                g_task_mgr.current_box_id = box_id;
                                g_task_mgr.current_bomb_id = bomb_id;
                                g_ctrl.current_box_id = -1;
                                g_ctrl.current_bomb_id = bomb_id;
                                g_ctrl.is_bomb_path = 1;
                                g_ctrl.bomb_target_pos[0] = bomb_target_x;
                                g_ctrl.bomb_target_pos[1] = bomb_target_y;
                                g_task_mgr.state = TASK_STATE_EXECUTE_BOX;
                            } else {
                                plan_retry++;
                                if (plan_retry >= 3) {
                                    g_game_state.boxes[box_id].state = 2;
                                    need_map_update = 1;
    g_task_mgr.state = TASK_STATE_MOVE_OUT;
                                    plan_retry = 0;
                                } else {
                                    rt_thread_mdelay(200);
                                }
                            }
                        } else {
                            plan_retry++;
                            if (plan_retry >= 3) {
                                g_game_state.boxes[box_id].state = 2;
                                need_map_update = 1;
                                g_task_mgr.state = TASK_STATE_WAIT_MAP;
                                plan_retry = 0;
                            } else {
                                rt_thread_mdelay(200);
                            }
                        }
                    } else {
                        plan_retry++;
                        if (plan_retry >= 3) {
                            g_game_state.boxes[box_id].state = 2;
                            need_map_update = 1;
                            g_task_mgr.state = TASK_STATE_WAIT_MAP;
                            plan_retry = 0;
                        } else {
                            rt_thread_mdelay(200);
                        }
                    }
                } else {
                    plan_retry++;
                    if (plan_retry >= 3) {
                        g_game_state.boxes[box_id].state = 2;
                        need_map_update = 1;
                        g_task_mgr.state = TASK_STATE_WAIT_MAP;
                        plan_retry = 0;
                    } else {
                        rt_thread_mdelay(200);
                    }
                }
            }
            break;
        }

        case TASK_STATE_EXECUTE_BOX: {
            if (waiting_map) break;
            if (g_ctrl.mode != CTRL_MODE_IDLE) break;

            if (g_task_mgr.action_index > g_task_mgr.last_push_end_idx) {
                g_task_mgr.last_push_end_idx = g_task_mgr.action_index;

                if (g_task_mgr.action_index >= g_task_mgr.action_total) {
                    if (g_ctrl.is_bomb_path) {
                        if (g_task_mgr.current_bomb_id >= 0) {
                            g_game_state.bombs[g_task_mgr.current_bomb_id].active = 0;
                        }
                    } else {
                        if (g_task_mgr.current_box_id >= 0) {
                            g_game_state.boxes[g_task_mgr.current_box_id].state = 1;
                            char buf[64];
                            rt_sprintf(buf, "[TaskMgr] Box %d pushed, state=1.\r\n", g_task_mgr.current_box_id);
                            wireless_uart_send_string(buf);
                        }
                    }
                    need_map_update = 1;
                    g_ctrl.precomputed_count = 0;
                    if (g_task_mgr.mode == TASK_MODE_STAGE2 && g_ctrl.is_bomb_path) {
                        g_ctrl.current_box_id = -1;
                        g_ctrl.current_bomb_id = -1;
                        g_ctrl.is_bomb_path = 0;
                        g_task_mgr.action_total = 0;
                        g_task_mgr.action_index = 0;
                        g_task_mgr.state = TASK_STATE_RECOGNIZE_DEST;
                        start_next_recognition();
                        break;
                    }
                    g_ctrl.current_box_id = -1;
                    g_ctrl.current_bomb_id = -1;
                    g_ctrl.is_bomb_path = 0;
                    g_task_mgr.action_total = 0;
                    g_task_mgr.action_index = 0;
                    g_task_mgr.state = TASK_STATE_WAIT_MAP;
                    break;
                }

                // 段间请求真实地图纠正
                g_ctrl.precomputed_count = 0;
                g_task_mgr.action_total = 0;
                g_task_mgr.action_index = 0;
                g_task_mgr.last_push_end_idx = 0;
                need_map_update = 1;
                g_task_mgr.state = TASK_STATE_WAIT_MAP;
                wireless_uart_send_string("[TaskMgr] Segment done, requesting map...\r\n");
                break;
            }

            int seg_dir = g_task_mgr.action_queue[g_task_mgr.action_index] - 4;
            int seg_steps = 0;
            int idx = g_task_mgr.action_index;
            while (idx < g_task_mgr.action_total &&
                   (g_task_mgr.action_queue[idx] - 4) == seg_dir) {
                seg_steps++;
                idx++;
            }

            int obj_type = g_ctrl.is_bomb_path ? OBJ_BOMB : OBJ_BOX;
            int obj_id = g_ctrl.is_bomb_path ? g_task_mgr.current_bomb_id : g_task_mgr.current_box_id;

            float stand_x, stand_y;
            if (!calculate_push_stance_for_dir(&g_game_state, obj_id, obj_type, seg_dir,
                                               &stand_x, &stand_y)) {
                wireless_uart_send_string("[TaskMgr] Cannot calculate push stance, wait map.\r\n");
                g_task_mgr.state = TASK_STATE_WAIT_MAP;
                break;
            }

            float dist = sqrtf(powf(stand_x - position.x_m, 2) + powf(stand_y - position.y_m, 2));
            if (dist > 0.04f) {
                if (!HybridController_PlanPathToPoint(&g_ctrl, position.x_m, position.y_m,
                                                       stand_x, stand_y)) {
                    if (!HybridController_PlanPathToPoint(&g_ctrl, position.x_m, position.y_m,
                                                           stand_x, stand_y)) {
                        g_task_mgr.state = TASK_STATE_IDLE;
                        break;
                    }
                }
            } else {
                if (!HybridController_PlanPushPath(&g_ctrl, position.x_m, position.y_m,
                                                    g_task_mgr.action_queue + g_task_mgr.action_index,
                                                    seg_steps)) {
                    if (!HybridController_PlanPushPath(&g_ctrl, position.x_m, position.y_m,
                                                        g_task_mgr.action_queue + g_task_mgr.action_index,
                                                        seg_steps)) {
                        g_task_mgr.state = TASK_STATE_IDLE;
                        break;
                    }
                }
                g_task_mgr.action_index = idx;
            }
            break;
        }

        default:
            g_task_mgr.state = TASK_STATE_IDLE;
            break;
    }
}

static void system_reset_for_stage2(void)
{
    system_reset_for_mode(TASK_MODE_STAGE2);
}

static void system_reset_for_mode(TaskMode_t target_mode)
{
    char buf[64];
    rt_sprintf(buf, "[Reset] Switching to Stage %d...\r\n", (int)target_mode + 1);
    wireless_uart_send_string(buf);

    CarController_Stop();
    CarController_Update();
    rt_thread_mdelay(50);

    EncoderReset();
    Position_Init();
    Position_ResetYaw();
    AHRS_ResetYaw();

    g_task_mgr.state = TASK_STATE_MOVE_OUT;
    g_task_mgr.mode = target_mode;
    g_task_mgr.current_box_id = -1;
    g_task_mgr.current_bomb_id = -1;
    g_task_mgr.action_total = 0;
    g_task_mgr.action_index = 0;
    g_task_mgr.pending_box_count = 0;
    g_task_mgr.retry_count = 0;
    g_task_mgr.all_dest_recognized = 0;
    g_task_mgr.all_box_recognized = 0;
    g_digit_map_count = 0;
    recog_fail_count = 0;

    memset(&g_game_state, 0, sizeof(g_game_state));
    memset(&g_grid_map, 0, sizeof(g_grid_map));

    HybridController_Reset(&g_ctrl);

    clear_vision_valid();

    rt_sprintf(buf, "[Reset] Stage %d ready, will move out...\r\n", (int)target_mode + 1);
    wireless_uart_send_string(buf);
}