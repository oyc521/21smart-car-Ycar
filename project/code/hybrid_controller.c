#include "hybrid_controller.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "kinematics.h"
#include <rtthread.h>
#include "task_manager.h"

// 外部函数声明（来自规划模块）
void world_to_grid(float wx, float wy, int* gx, int* gy);
void grid_to_world(int gx, int gy, float* wx, float* wy);
int astar_plan_path(GridMap* map, int start_x, int start_y, int goal_x, int goal_y,
                    int* out_path_x, int* out_path_y, int max_path_len, AStarParams* params);
void refresh_grid_map(GameState* state, GridMap* map);
void explode_bomb(GameState* state, GridMap* map, int bomb_id);

// 新规划器函数
int light_sokoban_plan(GameState* state, GridMap* grid_map, int box_id,
                       float car_x, float car_y,
                       int* out_actions, int max_actions);
int plan_bomb_to_target(GameState* state, GridMap* grid_map, int bomb_id,
                        float car_x, float car_y,
                        float target_x, float target_y,
                        float* out_path_x, float* out_path_y, int max_path_len);

extern rt_event_t g_task_event;
extern TaskManager g_task_mgr;

// ---------- 辅助函数 ----------

/**
 * 寻找箱子旁边的可通行粗网格中心点
 */
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

/**
 * 检查直线路径是否安全（无障碍）
 */
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

/**
 * 在路径上找最近点
 */
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

/**
 * 获取前视点
 */
static void get_lookahead_point(const float path[][2], int len, int start_idx,
                                float x, float y, float lookahead, float* out_x, float* out_y) {
    if (start_idx >= len - 1) {
        *out_x = path[len-1][0];
        *out_y = path[len-1][1];
        return;
    }
    float cumulative = 0.0f;
    for (int i = start_idx; i < len - 1; i++) {
        float seg_len = sqrtf( (path[i+1][0]-path[i][0])*(path[i+1][0]-path[i][0]) +
                               (path[i+1][1]-path[i][1])*(path[i+1][1]-path[i][1]) );
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

/**
 * 路径跟踪（全向平移控制）
 */
static int follow_path(HybridController* ctrl, float car_x, float car_y, float car_angle,
                       float* vx, float* vy, float* omega, float* dist_to_end) {
    if (ctrl->path_len < 2) return 0;

    float min_dist;
    int nearest = find_nearest_point_on_path(ctrl->current_path, ctrl->path_len, car_x, car_y, &min_dist);
    *dist_to_end = sqrtf( (ctrl->current_path[ctrl->path_len-1][0] - car_x) *
                          (ctrl->current_path[ctrl->path_len-1][0] - car_x) +
                          (ctrl->current_path[ctrl->path_len-1][1] - car_y) *
                          (ctrl->current_path[ctrl->path_len-1][1] - car_y) );

    float target_x, target_y;
    if (*dist_to_end < 0.5f) {
        // 直接瞄准终点
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

    if (dist_err < 1e-3f) {
        *vx = 0; *vy = 0; *omega = 0;
        return 1;
    }

    float dir_x = dx / dist_err;
    float dir_y = dy / dist_err;

    float desired_speed = fminf(ctrl->max_speed, 1.5f * dist_err);
    desired_speed = fmaxf(ctrl->min_speed, desired_speed);

    *vx = desired_speed * dir_x;
    *vy = desired_speed * dir_y;
    *omega = 0.0f;   // 全向平台无自转

    return 1;
}

/**
 * 视觉对准控制（只旋转，不移动或微动）
 */
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

    const float ANGLE_TOLERANCE = 0.1f;
    if (fabsf(angle_error) < ANGLE_TOLERANCE) {
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

/**
 * 死锁检测
 */
static void check_path_stuck(HybridController* ctrl, float car_x, float car_y, float car_angle, float dt) {
    float disp = sqrtf( (car_x - ctrl->last_path_pos[0])*(car_x - ctrl->last_path_pos[0]) +
                        (car_y - ctrl->last_path_pos[1])*(car_y - ctrl->last_path_pos[1]) );
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
        printf("路径跟踪死锁，尝试重新规划\n");
        if (ctrl->current_box_id >= 0) {
            HybridController_PlanPathToBox(ctrl, car_x, car_y, ctrl->current_box_id);
        } else if (ctrl->current_bomb_id >= 0) {
            HybridController_PlanBombPath(ctrl, car_x, car_y,
                                          ctrl->current_bomb_id,
                                          ctrl->bomb_target_pos[0],
                                          ctrl->bomb_target_pos[1]);
        }
        ctrl->path_stuck_counter = 0;
    }
}

/**
 * 执行推动动作（更新箱子位置并刷新地图）
 */
static void apply_push(HybridController* ctrl, int box_id, int action) {
    float dx = 0, dy = 0;
    switch (action) {
        case ACTION_PUSH_UP:    dy = CELL_SIZE; break;
        case ACTION_PUSH_DOWN:  dy = -CELL_SIZE; break;
        case ACTION_PUSH_LEFT:  dx = -CELL_SIZE; break;
        case ACTION_PUSH_RIGHT: dx = CELL_SIZE; break;
        default: return;
    }
    Box* box = &ctrl->game_state->boxes[box_id];
    box->x += dx;
    box->y += dy;
    // 对齐到粗网格中心
    int r = (int)(box->y / CELL_SIZE);
    int c = (int)(box->x / CELL_SIZE);
    box->x = (c + 0.5f) * CELL_SIZE;
    box->y = (r + 0.5f) * CELL_SIZE;
    refresh_grid_map(ctrl->game_state, ctrl->grid_map);
    printf("箱子%d推动成功，新位置 (%.2f,%.2f)\n", box_id, box->x, box->y);
}

/**
 * 检查箱子是否到达目的地
 */
static int check_box_at_destination(HybridController* ctrl, int box_id) {
    if (box_id < 0 || box_id >= ctrl->game_state->num_boxes) return 0;
    Box* box = &ctrl->game_state->boxes[box_id];
    if (box->dest_id < 0) return 0;
    int box_r = (int)(box->y / CELL_SIZE);
    int box_c = (int)(box->x / CELL_SIZE);
    int dest_r = (int)(ctrl->game_state->destinations[box->dest_id].y / CELL_SIZE);
    int dest_c = (int)(ctrl->game_state->destinations[box->dest_id].x / CELL_SIZE);
    if (box_r == dest_r && box_c == dest_c) {
        box->state = 1;
        printf("箱子%d到达目的地！\n", box_id);
        rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
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
    if (!find_coarse_adjacent_target(ctrl->game_state, ctrl->grid_map, box_id, &target_x, &target_y))
        return 0;

    int start_gx, start_gy, goal_gx, goal_gy;
    world_to_grid(start_x, start_y, &start_gx, &start_gy);
    world_to_grid(target_x, target_y, &goal_gx, &goal_gy);

    int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
    AStarParams params = {5000, 2.0f};
    int len = astar_plan_path(ctrl->grid_map, start_gx, start_gy, goal_gx, goal_gy,
                              path_x, path_y, MAX_PATH_POINTS, &params);
    if (len > 0) {
        for (int i = 0; i < len; i++) {
            grid_to_world(path_x[i], path_y[i], &ctrl->current_path[i][0], &ctrl->current_path[i][1]);
        }
        ctrl->path_len = len;
        ctrl->path_following = 1;
        ctrl->current_box_id = box_id;
        ctrl->mode = CTRL_MODE_PATH_FOLLOWING;
        ctrl->is_bomb_path = 0;
        ctrl->path_stuck_counter = 0;
        printf("规划到箱子%d旁边的路径成功，目标(%.2f,%.2f)，路径点数%d\n",
               box_id, target_x, target_y, len);
        return 1;
    }
    return 0;
}

int HybridController_PlanBombPath(HybridController* ctrl, float start_x, float start_y,
                                  int bomb_id, float target_x, float target_y) {
    if (bomb_id < 0 || bomb_id >= ctrl->game_state->num_bombs) return 0;
    if (!ctrl->game_state->bombs[bomb_id].active) return 0;

    float path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
    int len = plan_bomb_to_target(ctrl->game_state, ctrl->grid_map, bomb_id,
                                   start_x, start_y, target_x, target_y,
                                   path_x, path_y, MAX_PATH_POINTS);
    if (len <= 0) return 0;

    for (int i = 0; i < len; i++) {
        ctrl->current_path[i][0] = path_x[i];
        ctrl->current_path[i][1] = path_y[i];
    }
    ctrl->path_len = len;
    ctrl->path_following = 1;
    ctrl->current_bomb_id = bomb_id;
    ctrl->bomb_target_pos[0] = target_x;
    ctrl->bomb_target_pos[1] = target_y;
    ctrl->is_bomb_path = 1;
    ctrl->path_stuck_counter = 0;

    printf("炸弹%d路径规划成功，路径点数 %d\n", bomb_id, len);
    return 1;
}

int HybridController_PlanSokoban(HybridController* ctrl, int box_id, float car_x, float car_y) {
    if (box_id < 0 || box_id >= ctrl->game_state->num_boxes) return 0;
    Box* box = &ctrl->game_state->boxes[box_id];
    if (box->dest_id < 0) return 0;

    int actions[MAX_SOKOBAN_ACTIONS];
    int action_count = light_sokoban_plan(ctrl->game_state, ctrl->grid_map, box_id,
                                          car_x, car_y, actions, MAX_SOKOBAN_ACTIONS);
    if (action_count <= 0) {
        printf("箱子%d 轻量级规划失败\n", box_id);
        return 0;
    }

    for (int i = 0; i < action_count; i++) {
        ctrl->sokoban_actions[i] = actions[i];
    }
    ctrl->sokoban_action_count = action_count;
    ctrl->sokoban_action_index = 0;
    ctrl->sokoban_subpath_following = 0;
    ctrl->current_box_id = box_id;
    ctrl->mode = CTRL_MODE_SOKOBAN_EXECUTING;
    printf("箱子%d Sokoban规划成功，动作数 %d\n", box_id, action_count);
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
            if (follow_path(ctrl, car_x, car_y, car_angle, out_vx, out_vy, out_omega, &dist_to_end)) {
                if (dist_to_end < ctrl->path_tolerance) {
                    ctrl->path_following = 0;
                    if (ctrl->is_bomb_path) {
                        // 炸弹路径完成，触发爆炸
                        explode_bomb(ctrl->game_state, ctrl->grid_map, ctrl->current_bomb_id);
                        printf("炸弹%d到达目标点，已爆炸\n", ctrl->current_bomb_id);
                        ctrl->mode = CTRL_MODE_IDLE;
                        ctrl->current_bomb_id = -1;
                        ctrl->is_bomb_path = 0;
                    } else if (ctrl->current_box_id >= 0) {
                        // 箱子路径完成，启动Sokoban
                        if (!HybridController_PlanSokoban(ctrl, ctrl->current_box_id, car_x, car_y)) {
                            ctrl->mode = CTRL_MODE_IDLE;
                            ctrl->current_box_id = -1;
                        }
                    } else {
                        ctrl->mode = CTRL_MODE_IDLE;
                    }
                }
            } else {
                ctrl->mode = CTRL_MODE_IDLE;
            }
            break;

        case CTRL_MODE_VISUAL_ALIGNING:
            computeVisualAlignControl(ctrl,
                                      car_x, car_y, car_angle,
                                      ctrl->align_target_x, ctrl->align_target_y,
                                      out_vx, out_vy, out_omega);
            if (ctrl->visual_align_complete) {
                ctrl->mode = CTRL_MODE_IDLE;
            }
            break;

        case CTRL_MODE_SOKOBAN_EXECUTING:
            // 如果所有动作执行完毕
            if (ctrl->sokoban_action_index >= ctrl->sokoban_action_count) {
                if (check_box_at_destination(ctrl, ctrl->current_box_id)) {
                    ctrl->mode = CTRL_MODE_IDLE;
                    ctrl->current_box_id = -1;
                } else {
                    // 箱子未到达，重新规划
                    if (current_time - ctrl->last_plan_time > ctrl->plan_interval) {
                        if (HybridController_PlanSokoban(ctrl, ctrl->current_box_id, car_x, car_y)) {
                            ctrl->last_plan_time = current_time;
                        } else {
                            ctrl->mode = CTRL_MODE_IDLE;
                            ctrl->current_box_id = -1;
                        }
                    }
                }
                break;
            }

            // 如果没有正在跟踪的子路径，开始新动作
            if (!ctrl->sokoban_subpath_following) {
                int action = ctrl->sokoban_actions[ctrl->sokoban_action_index];
                int box_id = ctrl->current_box_id;
                float box_x = ctrl->game_state->boxes[box_id].x;
                float box_y = ctrl->game_state->boxes[box_id].y;

                float target_x, target_y;
                if (action >= ACTION_PUSH_UP) {
                    // 推动动作：小车应位于箱子后方（反方向）
                    switch (action) {
                        case ACTION_PUSH_UP:    target_x = box_x; target_y = box_y - CELL_SIZE; break;
                        case ACTION_PUSH_DOWN:  target_x = box_x; target_y = box_y + CELL_SIZE; break;
                        case ACTION_PUSH_LEFT:  target_x = box_x + CELL_SIZE; target_y = box_y; break;
                        case ACTION_PUSH_RIGHT: target_x = box_x - CELL_SIZE; target_y = box_y; break;
                        default: target_x = box_x; target_y = box_y;
                    }
                } else {
                    // 移动动作：小车目标为箱子位置
                    target_x = box_x;
                    target_y = box_y;
                }

                // 更新地图
                refresh_grid_map(ctrl->game_state, ctrl->grid_map);

                int start_gx, start_gy, goal_gx, goal_gy;
                world_to_grid(car_x, car_y, &start_gx, &start_gy);
                world_to_grid(target_x, target_y, &goal_gx, &goal_gy);

                int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
                AStarParams params = {5000, 2.0f};
                int len = astar_plan_path(ctrl->grid_map, start_gx, start_gy, goal_gx, goal_gy,
                                          path_x, path_y, MAX_PATH_POINTS, &params);

                if (len <= 0) {
                    // A*失败，尝试直线路径
                    if (is_straight_path_safe(ctrl->grid_map, car_x, car_y, target_x, target_y)) {
                        ctrl->sokoban_subpath[0][0] = car_x;
                        ctrl->sokoban_subpath[0][1] = car_y;
                        ctrl->sokoban_subpath[1][0] = target_x;
                        ctrl->sokoban_subpath[1][1] = target_y;
                        ctrl->sokoban_subpath_len = 2;
                        printf("动作%d A*失败，使用直线路径\n", action);
                    } else {
                        // 无法规划，重新规划整个Sokoban
                        printf("动作%d 无法规划子路径，重新规划Sokoban\n", action);
                        if (current_time - ctrl->last_plan_time > ctrl->plan_interval) {
                            if (HybridController_PlanSokoban(ctrl, ctrl->current_box_id, car_x, car_y)) {
                                ctrl->last_plan_time = current_time;
                            } else {
                                ctrl->mode = CTRL_MODE_IDLE;
                                ctrl->current_box_id = -1;
                            }
                        }
                        break;
                    }
                } else {
                    for (int i = 0; i < len; i++) {
                        grid_to_world(path_x[i], path_y[i], &ctrl->sokoban_subpath[i][0], &ctrl->sokoban_subpath[i][1]);
                    }
                    ctrl->sokoban_subpath_len = len;
                }

                ctrl->sokoban_subpath_following = 1;
                ctrl->sokoban_target_pos[0] = target_x;
                ctrl->sokoban_target_pos[1] = target_y;
                printf("开始执行动作 %d，子路径点数 %d\n", action, ctrl->sokoban_subpath_len);
            }

            // 跟踪当前子路径
            if (ctrl->sokoban_subpath_following) {
                // 临时保存原路径
                float saved_path[MAX_PATH_POINTS][2];
                int saved_len = ctrl->path_len;
                int saved_following = ctrl->path_following;
                memcpy(saved_path, ctrl->current_path, sizeof(saved_path));

                // 替换为子路径
                memcpy(ctrl->current_path, ctrl->sokoban_subpath, sizeof(float)*2*ctrl->sokoban_subpath_len);
                ctrl->path_len = ctrl->sokoban_subpath_len;
                ctrl->path_following = 1;

                float dist_to_target;
                follow_path(ctrl, car_x, car_y, car_angle, out_vx, out_vy, out_omega, &dist_to_target);

                // 恢复原路径
                memcpy(ctrl->current_path, saved_path, sizeof(saved_path));
                ctrl->path_len = saved_len;
                ctrl->path_following = saved_following;

                // 检查是否到达子路径终点
                float dx = ctrl->sokoban_target_pos[0] - car_x;
                float dy = ctrl->sokoban_target_pos[1] - car_y;
                float dist_to_goal = sqrtf(dx*dx + dy*dy);
                if (dist_to_goal < ctrl->path_tolerance) {
                    int action = ctrl->sokoban_actions[ctrl->sokoban_action_index];
                    if (action >= ACTION_PUSH_UP) {
                        if (ctrl->push_retry_count < ctrl->max_push_retries) {
                            apply_push(ctrl, ctrl->current_box_id, action);
                            ctrl->push_retry_count = 0;

                            if (check_box_at_destination(ctrl, ctrl->current_box_id)) {
                                ctrl->mode = CTRL_MODE_IDLE;
                                ctrl->current_box_id = -1;
                                ctrl->sokoban_action_index = ctrl->sokoban_action_count; // 强制结束
                                break;
                            }

                            ctrl->sokoban_action_index++;
                            ctrl->sokoban_subpath_following = 0;
                        } else {
                            printf("推动重试超限，跳过动作 %d\n", action);
                            ctrl->sokoban_action_index++;
                            ctrl->sokoban_subpath_following = 0;
                            ctrl->push_retry_count = 0;
                        }
                    } else {
                        // 移动动作，直接推进
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
    ctrl->max_speed = 0.25f;
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