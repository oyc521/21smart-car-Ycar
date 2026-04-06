#include "hybrid_controller.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "kinematics.h"
#include <rtthread.h>
#include "task_manager.h"
#include "zf_device_wireless_uart.h"
#include "position.h"
// 外部函数声明（来自规划模块）
void world_to_grid(float wx, float wy, int* gx, int* gy);
void grid_to_world(int gx, int gy, float* wx, float* wy);
int astar_plan_path(GridMap* map, int start_x, int start_y, int goal_x, int goal_y,
                    int* out_path_x, int* out_path_y, int max_path_len, AStarParams* params);
void refresh_grid_map(GameState* state, GridMap* map);
void explode_bomb(GameState* state, GridMap* map, int bomb_id);

// 推箱子规划器
int light_sokoban_plan(GameState* state, GridMap* grid_map, int box_id,
                       float car_x, float car_y, int* out_actions, int max_actions);
int actions_to_world_path(GameState* state, GridMap* grid_map, int box_id,
                          float start_car_x, float start_car_y,
                          const int* actions, int action_count,
                          float* out_x, float* out_y, int max_len);

// 炸弹规划器（来自 bomb_planner.c）
int light_push_plan(GridMap* map, int start_r, int start_c, int goal_r, int goal_c,
                    int car_start_r, int car_start_c, int* out_actions, int max_actions);
int bomb_actions_to_world_path(GameState* state, GridMap* grid_map, int bomb_id,
                               float start_car_x, float start_car_y,
                               const int* actions, int action_count,
                               float* out_x, float* out_y, int max_len);

static int g_plan_actions[200];
#define PATH_DIR_MODE 0

extern uint8_t need_map_update;
extern TaskManager g_task_mgr;

// 将图像坐标系下的动作转换为运动坐标系下的动作
static int convert_action_to_motion(int action) {
    static const int motion_map[8] = {
        3,   // 0 (UP)    -> LEFT (3)
        0,   // 1 (RIGHT) -> UP (0)
        1,   // 2 (DOWN)  -> RIGHT (1)
        2,   // 3 (LEFT)  -> DOWN (2)
        7,   // 4 (PUSH_UP)    -> PUSH_LEFT (7)
        4,   // 5 (PUSH_RIGHT) -> PUSH_UP (4)
        5,   // 6 (PUSH_DOWN)  -> PUSH_RIGHT (5)
        6    // 7 (PUSH_LEFT)  -> PUSH_DOWN (6)
    };
    if (action >= 0 && action < 8) return motion_map[action];
    return action;
}

static int find_coarse_adjacent_target(GameState* state, GridMap* grid_map, int box_id,
                                       float* out_x, float* out_y) {
    Box* box = &state->boxes[box_id];
    float car_x = position.x_m;
    float car_y = position.y_m;
    int box_r = (int)(box->x / CELL_SIZE);
    int box_c = (int)(box->y / CELL_SIZE);
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    float best_dist = 1e9;
    int best_nr = -1, best_nc = -1;
    for (int d = 0; d < 4; d++) {
        int nr = box_r + dr[d];
        int nc = box_c + dc[d];
        if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
        // 检查空闲
        int img_r = nc;
        int img_c = nr;
        int base_x = img_c * 4;
        int base_y = img_r * 4;
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
            float target_x = (nr + 0.5f) * CELL_SIZE;
            float target_y = (nc + 0.5f) * CELL_SIZE;
            float dist = (target_x - car_x)*(target_x - car_x) + (target_y - car_y)*(target_y - car_y);
            if (dist < best_dist) {
                best_dist = dist;
                best_nr = nr;
                best_nc = nc;
            }
        }
    }
    if (best_nr >= 0) {
        *out_x = (best_nr + 0.5f) * CELL_SIZE;
        *out_y = (best_nc + 0.5f) * CELL_SIZE;
        return 1;
    }
    return 0;
}

static int is_straight_path_safe(GridMap* grid_map, float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist < 1e-3f) return 1;
    int num_samples = (int)(dist / (RESOLUTION * 0.5f)) + 2;
    for (int i = 0; i <= num_samples; i++) {
        float t = (float)i / num_samples;
        float wx = x1 + t * dx;
        float wy = y1 + t * dy;
        int gx, gy;
        world_to_grid(wx, wy, &gx, &gy);
        if (gx < 0 || gx >= grid_map->width || gy < 0 || gy >= grid_map->height) return 0;
        uint8_t occ = grid_map->occupancy[gy][gx];
        if (occ == OCC_WALL || occ == OCC_BOX) return 0;
    }
    return 1;
}

static int find_nearest_point_on_path(const float path[][2], int len, float x, float y, float* min_dist) {
    int idx = 0;
    *min_dist = 1e9f;
    for (int i = 0; i < len; i++) {
        float dx = path[i][0] - x;
        float dy = path[i][1] - y;
        float d = dx*dx + dy*dy;
        if (d < *min_dist) {
            *min_dist = d;
            idx = i;
        }
    }
    *min_dist = sqrtf(*min_dist);
    return idx;
}

static void get_lookahead_point(const float path[][2], int len, int start_idx,
                                float x, float y, float lookahead, float* out_x, float* out_y) {
    if (start_idx >= len - 1) {
        *out_x = path[len-1][0];
        *out_y = path[len-1][1];
        return;
    }
    float cumulative = 0.0f;
    for (int i = start_idx; i < len - 1; i++) {
        float seg_len = sqrtf((path[i+1][0]-path[i][0])*(path[i+1][0]-path[i][0]) +
                              (path[i+1][1]-path[i][1])*(path[i+1][1]-path[i][1]));
        if (cumulative + seg_len >= lookahead) {
            float ratio = (lookahead - cumulative) / seg_len;
            *out_x = path[i][0] + ratio * (path[i+1][0] - path[i][0]);
            *out_y = path[i][1] + ratio * (path[i+1][1] - path[i][1]);
            return;
        }
        cumulative += seg_len;
    }
    *out_x = path[len-1][0];
    *out_y = path[len-1][1];
}

int follow_path(HybridController* ctrl, float car_x, float car_y, float car_angle,
                float* vx, float* vy, float* omega, float* dist_to_end) {
    if (ctrl->path_len < 2) return 0;

    float min_dist;
    int nearest = find_nearest_point_on_path(ctrl->current_path, ctrl->path_len, car_x, car_y, &min_dist);
    *dist_to_end = sqrtf((ctrl->current_path[ctrl->path_len-1][0] - car_x) *
                         (ctrl->current_path[ctrl->path_len-1][0] - car_x) +
                         (ctrl->current_path[ctrl->path_len-1][1] - car_y) *
                         (ctrl->current_path[ctrl->path_len-1][1] - car_y));

    float target_x, target_y;
    if (ctrl->path_len == 2 || *dist_to_end < 0.5f) {
        target_x = ctrl->current_path[ctrl->path_len-1][0];
        target_y = ctrl->current_path[ctrl->path_len-1][1];
    } else {
        float lookahead = ctrl->lookahead_dist;
        if (ctrl->enable_adaptive_lookahead) {
            lookahead = fmaxf(ctrl->min_lookahead,
                              fminf(ctrl->max_lookahead,
                                    ctrl->lookahead_dist + 0.2f * min_dist));
        }
        get_lookahead_point(ctrl->current_path, ctrl->path_len, nearest,
                            car_x, car_y, lookahead, &target_x, &target_y);
    }

    float dx = target_x - car_x;
    float dy = target_y - car_y;
    float dist_err = sqrtf(dx*dx + dy*dy);

    // 调试打印：输出起点、目标点、位移差（单位毫米）
    /*char dbg[128];
    rt_sprintf(dbg, "follow: car=(%d,%d) target=(%d,%d) dx=%d dy=%d dist_err=%d\r\n",
               (int)(car_x*1000), (int)(car_y*1000),
               (int)(target_x*1000), (int)(target_y*1000),
               (int)(dx*1000), (int)(dy*1000), (int)(dist_err*1000));
    wireless_uart_send_string(dbg);*/

    if (dist_err < 1e-3f) {
        *vx = 0; *vy = 0; *omega = 0;
        return 1;
    }

    float dir_x = dx / dist_err;
    float dir_y = dy / dist_err;
    float desired_speed = fminf(ctrl->max_speed, 1.5f * dist_err);
    desired_speed = fmaxf(ctrl->min_speed, desired_speed);

    // 确保 PATH_DIR_MODE 未定义，以使用标准方向映射
    #ifdef PATH_DIR_MODE
        #if PATH_DIR_MODE == 0
            *vx = desired_speed * dir_x;
            *vy = desired_speed * dir_y;
        #elif PATH_DIR_MODE == 1
            *vx = desired_speed * dir_y;
            *vy = desired_speed * dir_x;
        #elif PATH_DIR_MODE == 2
            *vx = -desired_speed * dir_x;
            *vy = desired_speed * dir_y;
        #elif PATH_DIR_MODE == 3
            *vx = desired_speed * dir_x;
            *vy = -desired_speed * dir_y;
        #elif PATH_DIR_MODE == 4
            *vx = -desired_speed * dir_x;
            *vy = -desired_speed * dir_y;
        #endif
    #else
        *vx = desired_speed * dir_x;
        *vy = desired_speed * dir_y;
    #endif

    // 打印最终速度指令
    /*rt_sprintf(dbg, "follow: vx=%d vy=%d omega=%d\r\n",
               (int)(*vx*1000), (int)(*vy*1000), (int)(*omega*1000));
    wireless_uart_send_string(dbg);*/

    return 1;
}

static void computeVisualAlignControl(HybridController* ctrl,
                                      float car_x, float car_y, float car_angle,
                                      float target_x, float target_y,
                                      float* out_vx, float* out_vy, float* out_omega) {
    float dx = target_x - car_x;
    float dy = target_y - car_y;
    float target_angle = atan2f(dy, dx);
    float angle_error = target_angle - car_angle;
    while (angle_error > M_PI) angle_error -= 2*M_PI;
    while (angle_error < -M_PI) angle_error += 2*M_PI;

    const float ANGLE_TOLERANCE_VAL = 0.1f;   // 避免与宏冲突
    if (fabsf(angle_error) < ANGLE_TOLERANCE_VAL) {
        *out_vx = 0; *out_vy = 0; *out_omega = 0;
        ctrl->visual_align_complete = 1;
        return;
    }

    float Kp_angle = 2.0f;
    float max_omega = 1.0f;
    float omega = Kp_angle * angle_error;
    if (omega > max_omega) omega = max_omega;
    if (omega < -max_omega) omega = -max_omega;

    *out_vx = 0;
    *out_vy = 0;
    *out_omega = omega;
}

static void check_path_stuck(HybridController* ctrl, float car_x, float car_y, float car_angle, float dt) {
    float disp = sqrtf((car_x - ctrl->last_path_pos[0])*(car_x - ctrl->last_path_pos[0]) +
                       (car_y - ctrl->last_path_pos[1])*(car_y - ctrl->last_path_pos[1]));
    float angle_change = fabsf(car_angle - ctrl->last_path_angle);
    if (angle_change > M_PI) angle_change = 2*M_PI - angle_change;

    if (disp < 0.01f && angle_change < 0.01f) {
        ctrl->path_stuck_counter++;
    } else {
        ctrl->path_stuck_counter = 0;
    }

    ctrl->last_path_pos[0] = car_x;
    ctrl->last_path_pos[1] = car_y;
    ctrl->last_path_angle = car_angle;

    if (ctrl->path_stuck_counter > ctrl->path_stuck_threshold) {
        rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
        ctrl->path_stuck_counter = 0;
    }
}

void apply_push(HybridController* ctrl, int box_id, int action) {
    char buf[128];
    rt_sprintf(buf, "apply_push: box=%d, action=%d\r\n", box_id, action);
    wireless_uart_send_string(buf);

    // 获取小车当前位置（运动坐标）
    extern Position_t position;
    float car_x = position.x_m;
    float car_y = position.y_m;

    float dx = 0, dy = 0;
    switch (action) {
        case ACTION_PUSH_UP:    dx = CELL_SIZE; dy = 0; break;   // 向前推
        case ACTION_PUSH_DOWN:  dx = -CELL_SIZE; dy = 0; break;  // 向后推
        case ACTION_PUSH_LEFT:  dx = 0; dy = -CELL_SIZE; break;  // 向左推
        case ACTION_PUSH_RIGHT: dx = 0; dy = CELL_SIZE; break;   // 向右推
        default: return;
    }

    // 计算箱子新位置：小车前方一格
    float new_box_x = car_x + dx;
    float new_box_y = car_y + dy;

    Box* box = &ctrl->game_state->boxes[box_id];
    rt_sprintf(buf, "Before push: box=(%d,%d), car=(%d,%d)\r\n",
               (int)(box->x*1000), (int)(box->y*1000),
               (int)(car_x*1000), (int)(car_y*1000));
    wireless_uart_send_string(buf);

    // 更新箱子坐标
    box->x = new_box_x;
    box->y = new_box_y;
    // 对齐到网格中心（可选）
    int r = (int)(box->y / CELL_SIZE);
    int c = (int)(box->x / CELL_SIZE);
    box->x = (c + 0.5f) * CELL_SIZE;
    box->y = (r + 0.5f) * CELL_SIZE;

    rt_sprintf(buf, "After push: box=(%d,%d)\r\n", (int)(box->x*1000), (int)(box->y*1000));
    wireless_uart_send_string(buf);
    refresh_grid_map(ctrl->game_state, ctrl->grid_map);
}
static int check_box_at_destination(HybridController* ctrl, int box_id) {
    if (box_id < 0 || box_id >= ctrl->game_state->num_boxes) return 0;
    Box* box = &ctrl->game_state->boxes[box_id];
    if (box->dest_id < 0) return 0;
    int box_r = (int)(box->y / CELL_SIZE);
    int box_c = (int)(box->x / CELL_SIZE);
    int dest_r = (int)(ctrl->game_state->destinations[box->dest_id].y / CELL_SIZE);
    int dest_c = (int)(ctrl->game_state->destinations[box->dest_id].x / CELL_SIZE);
    
    if (box_r == dest_r && box_c == dest_c) {
        int base_x = box_c * 4;
        int base_y = box_r * 4;
        for (int dy = 0; dy < 4; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                int fx = base_x + dx;
                int fy = base_y + dy;
                if (fx >= 0 && fx < ctrl->grid_map->width && fy >= 0 && fy < ctrl->grid_map->height) {
                    ctrl->grid_map->occupancy[fy][fx] = OCC_DEST;
                }
            }
        }
        box->state = 1;
        need_map_update = 1;
        return 1;
    }
    return 0;
}

// ---------- 公有函数 ----------
void HybridController_Init(HybridController* ctrl, GridMap* grid_map, GameState* game_state) {
    memset(ctrl, 0, sizeof(HybridController));
    ctrl->mode = CTRL_MODE_IDLE;
    ctrl->grid_map = grid_map;
    ctrl->game_state = game_state;

    ctrl->lookahead_dist = 0.3f;
    ctrl->min_lookahead = 0.15f;
    ctrl->max_lookahead = 0.5f;
    ctrl->max_speed = 0.25f;
    ctrl->min_speed = 0.02f;
    ctrl->path_tolerance = 0.15f;
    ctrl->enable_adaptive_lookahead = 1;

    ctrl->path_stuck_threshold = 300;
    ctrl->max_push_retries = 5;
    ctrl->plan_interval = 0.5f;
    ctrl->last_plan_time = 0.0f;
    ctrl->is_bomb_path = 0;
}

int HybridController_PlanPathToBox(HybridController* ctrl, float start_x, float start_y, int box_id) {
    if (box_id < 0 || box_id >= ctrl->game_state->num_boxes) return 0;
    
    float target_x, target_y;
    if (!find_coarse_adjacent_target(ctrl->game_state, ctrl->grid_map, box_id, &target_x, &target_y)) {
        return 0;
    }

    char buf[128];
    rt_sprintf(buf, "PlanPath: target motion (%d,%d)\r\n", (int)(target_x*1000), (int)(target_y*1000));
    wireless_uart_send_string(buf);
    
    float start_img_x, start_img_y, target_img_x, target_img_y;
    motion_to_image(start_x, start_y, &start_img_x, &start_img_y);
    motion_to_image(target_x, target_y, &target_img_x, &target_img_y);
    
    
    int start_gx, start_gy, goal_gx, goal_gy;
    world_to_grid(start_img_x, start_img_y, &start_gx, &start_gy);
    world_to_grid(target_img_x, target_img_y, &goal_gx, &goal_gy);
    
    
    int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
    AStarParams params = {5000, 2.0f};
    int len = astar_plan_path(ctrl->grid_map, start_gx, start_gy, goal_gx, goal_gy,
                              path_x, path_y, MAX_PATH_POINTS, &params);
    
    
    if (len > 0) {
        rt_sprintf(buf, "first path point: (%d,%d) last: (%d,%d)\r\n",
                   path_x[0], path_y[0], path_x[len-1], path_y[len-1]);
        wireless_uart_send_string(buf);
        
        for (int i = 0; i < len; i++) {
            float wx, wy;
            grid_to_world(path_x[i], path_y[i], &wx, &wy);
            float mx, my;
            image_to_motion(wx, wy, &mx, &my);
            ctrl->current_path[i][0] = mx;
            ctrl->current_path[i][1] = my;
        }
        
        rt_sprintf(buf, "first motion: (%d,%d) last: (%d,%d)\r\n",
                   (int)(ctrl->current_path[0][0]*1000), (int)(ctrl->current_path[0][1]*1000),
                   (int)(ctrl->current_path[len-1][0]*1000), (int)(ctrl->current_path[len-1][1]*1000));
        wireless_uart_send_string(buf);
        
        ctrl->path_len = len;
        ctrl->path_following = 1;
        ctrl->current_box_id = box_id;
        ctrl->mode = CTRL_MODE_PATH_FOLLOWING;
        ctrl->is_bomb_path = 0;
        ctrl->path_stuck_counter = 0;
        return 1;
    } else {
        wireless_uart_send_string("A* planning failed\r\n");
        return 0;
    }
}

int HybridController_PlanBomb(HybridController* ctrl, int bomb_id, float car_x, float car_y,
                              float target_x, float target_y) {
    if (bomb_id < 0 || bomb_id >= ctrl->game_state->num_bombs) return 0;
    Bomb* bomb = &ctrl->game_state->bombs[bomb_id];
    if (!bomb->active) return 0;

    float bomb_img_x, bomb_img_y, target_img_x, target_img_y, car_img_x, car_img_y;
    motion_to_image(bomb->x, bomb->y, &bomb_img_x, &bomb_img_y);
    motion_to_image(target_x, target_y, &target_img_x, &target_img_y);
    motion_to_image(car_x, car_y, &car_img_x, &car_img_y);

    int bomb_start_r = (int)(bomb_img_y / CELL_SIZE);
    int bomb_start_c = (int)(bomb_img_x / CELL_SIZE);
    int bomb_target_r = (int)(target_img_y / CELL_SIZE);
    int bomb_target_c = (int)(target_img_x / CELL_SIZE);
    int car_start_r = (int)(car_img_y / CELL_SIZE);
    int car_start_c = (int)(car_img_x / CELL_SIZE);

    int actions[200];
    int action_count = light_push_plan(ctrl->grid_map,
                                       bomb_start_r, bomb_start_c,
                                       bomb_target_r, bomb_target_c,
                                       car_start_r, car_start_c,
                                       actions, 200);
    if (action_count <= 0) return 0;

    for (int i = 0; i < action_count; i++) {
        actions[i] = convert_action_to_motion(actions[i]);
    }

    float path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
    int path_len = bomb_actions_to_world_path(ctrl->game_state, ctrl->grid_map, bomb_id,
                                              car_img_x, car_img_y,
                                              actions, action_count,
                                              path_x, path_y, MAX_PATH_POINTS);
    if (path_len <= 0) return 0;

    for (int i = 0; i < path_len; i++) {
        ctrl->current_path[i][0] = path_x[i];
        ctrl->current_path[i][1] = path_y[i];
    }
    ctrl->path_len = path_len;
    ctrl->path_following = 1;
    ctrl->current_bomb_id = bomb_id;
    ctrl->bomb_target_pos[0] = target_x;
    ctrl->bomb_target_pos[1] = target_y;
    ctrl->is_bomb_path = 1;
    ctrl->mode = CTRL_MODE_PATH_FOLLOWING;
    return 1;
}

int HybridController_PlanSokoban(HybridController* ctrl, int box_id, float car_x, float car_y) {
    char buf[128];
    rt_sprintf(buf, "PlanSokoban called for box %d, car=(%d,%d)\r\n",
               box_id, (int)car_x, (int)car_y);
    wireless_uart_send_string(buf);

    if (box_id < 0 || box_id >= ctrl->game_state->num_boxes) return 0;
    Box* box = &ctrl->game_state->boxes[box_id];
    if (box->dest_id < 0) return 0;

    float car_img_x, car_img_y;
    motion_to_image(car_x, car_y, &car_img_x, &car_img_y);

    int actions[200];
    int action_count = light_sokoban_plan(ctrl->game_state, ctrl->grid_map, box_id,
                                          car_img_x, car_img_y, actions, 200);
    if (action_count <= 0) {
        wireless_uart_send_string("light_sokoban_plan failed\r\n");
        return 0;
    }

    // 关键修改：直接使用原始动作，不再调用 convert_action_to_motion
    for (int i = 0; i < action_count && i < MAX_SOKOBAN_ACTIONS; i++) {
        ctrl->sokoban_actions[i] = actions[i];
    }
    ctrl->sokoban_action_count = action_count;
    ctrl->sokoban_action_index = 0;
    ctrl->sokoban_subpath_following = 0;
    ctrl->current_box_id = box_id;
    ctrl->mode = CTRL_MODE_SOKOBAN_EXECUTING;
    wireless_uart_send_string("PlanSokoban success\r\n");
    return 1;
}

void HybridController_ComputeControl(HybridController* ctrl,
                                     float car_x, float car_y, float car_angle,
                                     float dt, float current_time,
                                     float* out_vx, float* out_vy, float* out_omega) {
    *out_vx = 0; *out_vy = 0; *out_omega = 0;

    switch (ctrl->mode) {
        case CTRL_MODE_IDLE:
            break;

        case CTRL_MODE_PATH_FOLLOWING:
            if (!ctrl->path_following || ctrl->path_len == 0) {
                ctrl->mode = CTRL_MODE_IDLE;
                break;
            }

            check_path_stuck(ctrl, car_x, car_y, car_angle, dt);

            float dist_to_end;
            int ret = follow_path(ctrl, car_x, car_y, car_angle, out_vx, out_vy, out_omega, &dist_to_end);
            if (ret) {
                if (dist_to_end < ctrl->path_tolerance) {
                    ctrl->path_following = 0;
                    *out_vx = 0; *out_vy = 0; *out_omega = 0;
                    
                    // 添加调试打印：路径终点到达
                    char buf[128];
                    rt_sprintf(buf, "Path end reached, is_bomb=%d, box_id=%d\r\n",
                               ctrl->is_bomb_path, ctrl->current_box_id);
                    wireless_uart_send_string(buf);

                    if (ctrl->is_bomb_path) {
                        explode_bomb(ctrl->game_state, ctrl->grid_map, ctrl->current_bomb_id);
                        rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
                        ctrl->mode = CTRL_MODE_IDLE;
                        ctrl->current_bomb_id = -1;
                        ctrl->is_bomb_path = 0;
                    } else if (ctrl->current_box_id >= 0) {
                        rt_sprintf(buf, "Calling PlanSokoban for box %d\r\n", ctrl->current_box_id);
                        wireless_uart_send_string(buf);
                        if (!HybridController_PlanSokoban(ctrl, ctrl->current_box_id, car_x, car_y)) {
                            wireless_uart_send_string("PlanSokoban failed\r\n");
                            ctrl->mode = CTRL_MODE_IDLE;
                            ctrl->current_box_id = -1;
                        }
                    } else {
                        ctrl->mode = CTRL_MODE_IDLE;
                    }
                    // 发送空闲事件
                    rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
                }
            } else {
                ctrl->mode = CTRL_MODE_IDLE;
                rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
            }
            break;

        case CTRL_MODE_VISUAL_ALIGNING:
            computeVisualAlignControl(ctrl, car_x, car_y, car_angle,
                                      ctrl->align_target_x, ctrl->align_target_y,
                                      out_vx, out_vy, out_omega);
            if (ctrl->visual_align_complete) {
                ctrl->mode = CTRL_MODE_IDLE;
                rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
            }
            break;

       case CTRL_MODE_SOKOBAN_EXECUTING:
    if (ctrl->sokoban_action_index >= ctrl->sokoban_action_count) {
        if (check_box_at_destination(ctrl, ctrl->current_box_id)) {
            ctrl->mode = CTRL_MODE_IDLE;
            ctrl->current_box_id = -1;
            rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
        } else {
            if (HybridController_PlanSokoban(ctrl, ctrl->current_box_id, car_x, car_y)) {
                // 重新规划成功，继续执行
            } else {
                ctrl->mode = CTRL_MODE_IDLE;
                ctrl->current_box_id = -1;
                rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
            }
        }
        break;
    }

    if (!ctrl->sokoban_subpath_following) {
        int action = ctrl->sokoban_actions[ctrl->sokoban_action_index];
        int box_id = ctrl->current_box_id;
        float box_x = ctrl->game_state->boxes[box_id].x;
        float box_y = ctrl->game_state->boxes[box_id].y;

        float target_x, target_y;
        if (action >= ACTION_PUSH_UP) {
            switch (action) {
                case ACTION_PUSH_UP:    target_x = box_x; target_y = box_y + CELL_SIZE; break;
                case ACTION_PUSH_DOWN:  target_x = box_x; target_y = box_y - CELL_SIZE; break;
                case ACTION_PUSH_LEFT:  target_x = box_x + CELL_SIZE; target_y = box_y; break;
                case ACTION_PUSH_RIGHT: target_x = box_x - CELL_SIZE; target_y = box_y; break;
                default: target_x = box_x; target_y = box_y;
            }
        } else {
            switch (action) {
                case ACTION_UP:    target_x = box_x; target_y = box_y + CELL_SIZE; break;
                case ACTION_DOWN:  target_x = box_x; target_y = box_y - CELL_SIZE; break;
                case ACTION_LEFT:  target_x = box_x + CELL_SIZE; target_y = box_y; break;
                case ACTION_RIGHT: target_x = box_x - CELL_SIZE; target_y = box_y; break;
                default: target_x = box_x; target_y = box_y;
            }
        }

        ctrl->sokoban_target_pos[0] = target_x;
        ctrl->sokoban_target_pos[1] = target_y;

        refresh_grid_map(ctrl->game_state, ctrl->grid_map);

        float car_img_x, car_img_y, target_img_x, target_img_y;
        motion_to_image(car_x, car_y, &car_img_x, &car_img_y);
        motion_to_image(target_x, target_y, &target_img_x, &target_img_y);

        int start_gx, start_gy, goal_gx, goal_gy;
        world_to_grid(car_img_x, car_img_y, &start_gx, &start_gy);
        world_to_grid(target_img_x, target_img_y, &goal_gx, &goal_gy);

        int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
        AStarParams params = {5000, 2.0f};
        int len = astar_plan_path(ctrl->grid_map, start_gx, start_gy, goal_gx, goal_gy,
                                  path_x, path_y, MAX_PATH_POINTS, &params);

        char dbg[128];
        rt_sprintf(dbg, "SOKOBAN: action=%d, A* len=%d\n", action, len);
        wireless_uart_send_string(dbg);

        // 如果A*规划失败，直接使用直线路径（不再检查安全性）
        if (len <= 0) {
            rt_sprintf(dbg, "A* failed, using straight line from (%d,%d) to (%d,%d)\n",
                       (int)(car_x*1000), (int)(car_y*1000),
                       (int)(target_x*1000), (int)(target_y*1000));
            wireless_uart_send_string(dbg);
            ctrl->sokoban_subpath[0][0] = car_x;
            ctrl->sokoban_subpath[0][1] = car_y;
            ctrl->sokoban_subpath[1][0] = target_x;
            ctrl->sokoban_subpath[1][1] = target_y;
            ctrl->sokoban_subpath_len = 2;
        } else {
            for (int i = 0; i < len; i++) {
                float wx, wy;
                grid_to_world(path_x[i], path_y[i], &wx, &wy);
                float mx, my;
                image_to_motion(wx, wy, &mx, &my);
                ctrl->sokoban_subpath[i][0] = mx;
                ctrl->sokoban_subpath[i][1] = my;
            }
            ctrl->sokoban_subpath_len = len;
            rt_sprintf(dbg, "A* success, path len=%d\n", len);
            wireless_uart_send_string(dbg);
        }

        // 确保起点为当前小车位置
        if (ctrl->sokoban_subpath_len > 0) {
            ctrl->sokoban_subpath[0][0] = car_x;
            ctrl->sokoban_subpath[0][1] = car_y;
        }

        ctrl->sokoban_subpath_following = 1;
    }

    if (ctrl->sokoban_subpath_following) {
        float saved_path[MAX_PATH_POINTS][2];
        int saved_len = ctrl->path_len;
        int saved_following = ctrl->path_following;
        memcpy(saved_path, ctrl->current_path, sizeof(saved_path));

        memcpy(ctrl->current_path, ctrl->sokoban_subpath, sizeof(float)*2*ctrl->sokoban_subpath_len);
        ctrl->path_len = ctrl->sokoban_subpath_len;
        ctrl->path_following = 1;

        float dist_to_target;
        follow_path(ctrl, car_x, car_y, car_angle, out_vx, out_vy, out_omega, &dist_to_target);

        memcpy(ctrl->current_path, saved_path, sizeof(saved_path));
        ctrl->path_len = saved_len;
        ctrl->path_following = saved_following;

        float dx = ctrl->sokoban_target_pos[0] - car_x;
        float dy = ctrl->sokoban_target_pos[1] - car_y;
        float dist_to_goal = sqrtf(dx*dx + dy*dy);

        // 计算小车与箱子的实际距离
        int box_id = ctrl->current_box_id;
        float box_dx = ctrl->game_state->boxes[box_id].x - car_x;
        float box_dy = ctrl->game_state->boxes[box_id].y - car_y;
        float dist_to_box = sqrtf(box_dx*box_dx + box_dy*box_dy);

        char dbg2[128];
        rt_sprintf(dbg2, "SOKOBAN: dist_to_goal=%d mm, dist_to_box=%d mm, tol=%d mm\n",
                   (int)(dist_to_goal*1000), (int)(dist_to_box*1000), (int)(ctrl->path_tolerance*1000));
        wireless_uart_send_string(dbg2);

        // 到达目标点 或 紧贴箱子（距离小于0.15米）时执行推动
        if (dist_to_goal < ctrl->path_tolerance || dist_to_box < 0.15f) {
            int action = ctrl->sokoban_actions[ctrl->sokoban_action_index];
            rt_sprintf(dbg2, "SOKOBAN: reached target (dist=%d mm) or near box (dist=%d mm), action=%d\n",
                       (int)(dist_to_goal*1000), (int)(dist_to_box*1000), action);
            wireless_uart_send_string(dbg2);

            if (action >= ACTION_PUSH_UP) {
                if (ctrl->push_retry_count < ctrl->max_push_retries) {
                    apply_push(ctrl, ctrl->current_box_id, action);
                    ctrl->push_retry_count = 0;

                    if (check_box_at_destination(ctrl, ctrl->current_box_id)) {
                        ctrl->mode = CTRL_MODE_IDLE;
                        ctrl->current_box_id = -1;
                        ctrl->sokoban_action_index = ctrl->sokoban_action_count;
                        rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
                        break;
                    }

                    ctrl->sokoban_action_index++;
                    ctrl->sokoban_subpath_following = 0;
                } else {
                    ctrl->sokoban_action_index++;
                    ctrl->sokoban_subpath_following = 0;
                    ctrl->push_retry_count = 0;
                }
            } else {
                ctrl->sokoban_action_index++;
                ctrl->sokoban_subpath_following = 0;
            }
        }
    }
    break;

        default:
            ctrl->mode = CTRL_MODE_IDLE;
            break;
    }
}

void HybridController_Reset(HybridController* ctrl) {
    GridMap* saved_grid = ctrl->grid_map;
    GameState* saved_state = ctrl->game_state;
    memset(ctrl, 0, sizeof(HybridController));
    ctrl->grid_map = saved_grid;
    ctrl->game_state = saved_state;

    ctrl->mode = CTRL_MODE_IDLE;
    ctrl->lookahead_dist = 0.3f;
    ctrl->min_lookahead = 0.15f;
    ctrl->max_lookahead = 0.5f;
    ctrl->max_speed = 0.15f;
    ctrl->min_speed = 0.02f;
    ctrl->visual_align_complete = 0;
    ctrl->path_tolerance = 0.15f;
    ctrl->enable_adaptive_lookahead = 1;
    ctrl->path_stuck_threshold = 300;
    ctrl->max_push_retries = 5;
    ctrl->plan_interval = 0.5f;
    ctrl->last_plan_time = 0.0f;
    ctrl->is_bomb_path = 0;
    ctrl->bomb_action_count = 0;
    ctrl->bomb_action_index = 0;
    ctrl->bomb_subpath_following = 0;
}