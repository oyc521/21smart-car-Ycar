/**
 * @file hybrid_controller.c
 * @brief 混合控制器实现 - 路径规划、跟踪与目标操作
 * 当前版本中，该功能已暂时禁用（use_tangent_heading = 0），
 * 路径跟踪时锁定初始航向，避免频繁转向。
 * 若需启用切向航向（沿路径切线方向），需做以下修改：
 *   1. 在 PlanSokoban / PlanBomb 中设置 ctrl->use_tangent_heading = 1
 *   2. 在 follow_path 中取消注释计算目标航向的代码：
 *         target_yaw_deg = atan2f(dy, dx) * 180.0f / M_PI;
 *         target_yaw_deg = -target_yaw_deg;   // 根据坐标系取反
 *         while (target_yaw_deg > 180.0f) target_yaw_deg -= 360.0f;
 *         while (target_yaw_deg < -180.0f) target_yaw_deg += 360.0f;
 *   3. 在后续计算中使用该目标航向代替锁定航向
 * =================================================================
 */

#include "hybrid_controller.h"
#include <math.h>
#include <rtthread.h>
#include "position.h"
#include "task_manager.h"
#include "zf_device_wireless_uart.h"
#include "imu660ra_ahrs.h"
#include "motor.h"
#include "uart_receiver.h"
#include "uart4_recognition.h"

/* 外部全局变量声明 */
extern volatile uint8_t need_map_update;
extern TaskManager g_task_mgr;
extern Position_t position;
extern CarController_t car_ctrl;
extern rt_mutex_t g_map_mutex;
extern void BeepOnce(void);
extern PIDParam_t angle_trace_param;

/* ========== 导航模式选择（0=A*精细规划，1=BFS曼哈顿路径） ========== */
#ifndef USE_MANHATTAN_NAV
#define USE_MANHATTAN_NAV  0
#endif
#ifndef PUSH_INTERP_STEP
#define PUSH_INTERP_STEP   RESOLUTION
#endif

/* 外部函数（来自其他模块） */
extern void world_to_grid(float wx, float wy, int* gx, int* gy);
extern void grid_to_world(int gx, int gy, float* wx, float* wy);
extern int astar_plan_path(GridMap* map, int start_x, int start_y, int goal_x, int goal_y,
                           int* out_path_x, int* out_path_y, int max_path_len, AStarParams* params);
extern void refresh_grid_map(GameState* state, GridMap* map);
extern void explode_bomb(GameState* state, GridMap* map, int bomb_id);
extern int light_sokoban_plan(GameState* state, GridMap* grid_map, int box_id,
                              float car_x, float car_y, int* out_actions, int max_actions);
extern int light_push_plan(GridMap* map, int start_r, int start_c, int goal_r, int goal_c,
                           int car_start_r, int car_start_c, int* out_actions, int max_actions);

/* ========== 辅助工具函数 ========== */
/**
 * @brief 计算角度差（度），归一化到 [-180, 180]
 */
static float angle_diff(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

/**
 * @brief 在目标物体周围寻找空闲单元格（用于站位）
 * @param obj_x, obj_y 物体坐标（米）
 * @param grid_map 地图指针
 * @param out_x, out_y 输出站位坐标（米）
 * @return 1成功，0失败
 */
static int find_adjacent_free_cell(float obj_x, float obj_y, GridMap* grid_map,
                                   float* out_x, float* out_y) {
    float img_x, img_y;
    motion_to_image(obj_x, obj_y, &img_x, &img_y);
    int obj_r = (int)(img_y / CELL_SIZE);
    int obj_c = (int)(img_x / CELL_SIZE);
    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};
    for (int d = 0; d < 4; d++) {
        int nr = obj_r + dr[d];
        int nc = obj_c + dc[d];
        if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
        int base_x = nc * 4;
        int base_y = nr * 4;
        int blocked = 0;
        for (int dy = 0; dy < 4 && !blocked; dy++)
            for (int dx = 0; dx < 4; dx++)
                if (grid_map->occupancy[base_y+dy][base_x+dx] == OCC_WALL ||
                    grid_map->occupancy[base_y+dy][base_x+dx] == OCC_BOX ||
                    grid_map->occupancy[base_y+dy][base_x+dx] == OCC_BOMB)
                    blocked = 1;
        if (!blocked) {
            float target_img_x = (nc + 0.5f) * CELL_SIZE;
            float target_img_y = (nr + 0.5f) * CELL_SIZE;
            image_to_motion(target_img_x, target_img_y, out_x, out_y);
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 根据推的方向，计算炸弹的站位（在炸弹后方）
 */
static int find_bomb_stance_by_dir(GridMap* map, float bomb_x, float bomb_y,
                                   int push_dir, float* out_x, float* out_y) {
    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};
    const int opposite[4] = {2, 3, 0, 1};
    int back_dir = opposite[push_dir];

    float img_x, img_y;
    motion_to_image(bomb_x, bomb_y, &img_x, &img_y);
    int bomb_c = (int)(img_x / CELL_SIZE);
    int bomb_r = (int)(img_y / CELL_SIZE);
    int nr = bomb_r + dr[back_dir];
    int nc = bomb_c + dc[back_dir];
    if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) return 0;

    int base_x = nc * 4, base_y = nr * 4;
    for (int dy = 0; dy < 4; dy++)
        for (int dx = 0; dx < 4; dx++)
            if (map->occupancy[base_y+dy][base_x+dx] != OCC_FREE)
                return 0;

    float new_img_x = (nc + 0.5f) * CELL_SIZE;
    float new_img_y = (nr + 0.5f) * CELL_SIZE;
    image_to_motion(new_img_x, new_img_y, out_x, out_y);
    return 1;
}

/**
 * @brief 寻找识别目标时的站位（兼顾距离和航向成本）
 * @details 用于导航到数字/箱子识别点，选择最优的相邻空位。
 *          参数 RECOG_STANDOFF_DISTANCE 可调整站位偏移。
 *
 * 设计要点：
 * 1. 识别站位距离阈值 RECOG_STANDOFF_DISTANCE（默认0.10m），确保视野清晰。
 * 2. 综合考虑：
 *    - 与目标物的距离（经 OpenArt 视野标定，最佳距离约0.12~0.15m）
 *    - 航向对齐（使用 yaw_table 映射到 AHRS 0° 参考系）
 * 3. 安全性：确保站位所在的4x4区域无墙/箱/炸弹。
 * 4. yaw_table 映射关系：{90°, 0°, -90°, 180°} 对应四个方向。
 */
#define RECOG_STANDOFF_DISTANCE  0.0f
static int find_adjacent_for_recog(GridMap* map, int target_c, int target_r,
                                    float* out_x, float* out_y, float* out_yaw,
                                    float car_x, float car_y)
{
    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};
    const float yaw_table[4] = {90.0f, 0.0f, -90.0f, 180.0f};
    const float back_dir_x[4] = {0.0f, -1.0f, 0.0f, 1.0f};
    const float back_dir_y[4] = {1.0f, 0.0f, -1.0f, 0.0f};
    #define YAW_COST_PER_DEG 0.004f

    float best_score = 1e9f;
    int best_d = -1;
    float best_mx = 0, best_my = 0, best_yaw = 0;

    for (int d = 0; d < 4; d++) {
        int nr = target_r + dr[d];
        int nc = target_c + dc[d];
        if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
        int base_x = nc * 4;
        int base_y = nr * 4;
        int blocked = 0;
        for (int dy = 0; dy < 4 && !blocked; dy++)
            for (int dx = 0; dx < 4; dx++)
                if (map->occupancy[base_y+dy][base_x+dx] == OCC_WALL ||
                    map->occupancy[base_y+dy][base_x+dx] == OCC_BOX ||
                    map->occupancy[base_y+dy][base_x+dx] == OCC_BOMB)
                    blocked = 1;
        if (blocked) continue;
        float raw_img_x = (nc + 0.5f) * CELL_SIZE;
        float raw_img_y = (nr + 0.5f) * CELL_SIZE;
        float raw_motion_x, raw_motion_y;
        image_to_motion(raw_img_x, raw_img_y, &raw_motion_x, &raw_motion_y);
        float new_motion_x = raw_motion_x + back_dir_x[d] * RECOG_STANDOFF_DISTANCE;
        float new_motion_y = raw_motion_y + back_dir_y[d] * RECOG_STANDOFF_DISTANCE;
        float new_img_x, new_img_y;
        motion_to_image(new_motion_x, new_motion_y, &new_img_x, &new_img_y);
        int new_c = (int)(new_img_x / CELL_SIZE);
        int new_r = (int)(new_img_y / CELL_SIZE);
        if (new_r < 0 || new_r >= MAP_ROWS || new_c < 0 || new_c >= MAP_COLS) continue;
        int new_base_x = new_c * 4;
        int new_base_y = new_r * 4;
        int new_blocked = 0;
        for (int dy = 0; dy < 4 && !new_blocked; dy++)
            for (int dx = 0; dx < 4; dx++)
                if (map->occupancy[new_base_y+dy][new_base_x+dx] == OCC_WALL ||
                    map->occupancy[new_base_y+dy][new_base_x+dx] == OCC_BOX)
                    new_blocked = 1;
        if (new_blocked) continue;

        float dx = new_motion_x - car_x;
        float dy = new_motion_y - car_y;
        float dist = sqrtf(dx*dx + dy*dy);
        float yaw_err = fabsf(yaw_table[d]);
        if (yaw_err > 180.0f) yaw_err = 360.0f - yaw_err;
        float score = dist + yaw_err * YAW_COST_PER_DEG;
        if (score < best_score) {
            best_score = score; best_d = d;
            best_mx = new_motion_x; best_my = new_motion_y; best_yaw = yaw_table[d];
        }
    }
    if (best_d < 0) return 0;
    *out_x = best_mx; *out_y = best_my; *out_yaw = best_yaw;
    return 1;
}

/* ========== 评估最佳站位（箱子/炸弹） ========== */
/**
 * @brief 评估并选择最佳站位（用于推动箱子或炸弹）
 * @param state 游戏状态
 * @param grid_map 地图
 * @param obj_id 物体ID
 * @param obj_type 物体类型（OBJ_BOX / OBJ_BOMB）
 * @param bomb_target_x, bomb_target_y 炸弹目标位置（仅炸弹时有效）
 * @param out_stand_x, out_stand_y 输出最佳站位坐标
 * @param out_actions 输出动作序列（编码）
 * @param out_count 输出动作数量
 * @return 1成功，0失败
 */
int EvaluateBestStance(GameState* state, GridMap* grid_map,
                       int obj_id, int obj_type,
                       float bomb_target_x, float bomb_target_y,
                       float* out_stand_x, float* out_stand_y,
                       int* out_actions, int* out_count) {
    if (obj_type == OBJ_BOMB && !state->bombs[obj_id].active) return 0;

    float obj_x, obj_y;
    if (obj_type == OBJ_BOX) {
        obj_x = state->boxes[obj_id].x;
        obj_y = state->boxes[obj_id].y;
    } else {
        obj_x = state->bombs[obj_id].x;
        obj_y = state->bombs[obj_id].y;
    }

    /* 临时清除箱子占用，以便规划（后续恢复） */
    uint8_t saved[4][4] = {0};
    int saved_flag = 0;
    if (obj_type == OBJ_BOX) {
        if (g_map_mutex) rt_mutex_take(g_map_mutex, RT_WAITING_FOREVER);
        float img_x, img_y;
        motion_to_image(obj_x, obj_y, &img_x, &img_y);
        int obj_c = (int)(img_x / RESOLUTION);
        int obj_r = (int)(img_y / RESOLUTION);
        for (int dy = 0; dy < 4; dy++)
            for (int dx = 0; dx < 4; dx++) {
                int r = obj_r + dy, c = obj_c + dx;
                if (r < FINE_ROWS && c < FINE_COLS) {
                    saved[dy][dx] = grid_map->occupancy[r][c];
                    grid_map->occupancy[r][c] = OCC_FREE;
                }
            }
        if (g_map_mutex) rt_mutex_release(g_map_mutex);
        saved_flag = 1;
    }

    const int dr[4] = {-1,0,1,0};
    const int dc[4] = {0,1,0,-1};
    const int opposite[4] = {2,3,0,1};

    int best_count = 0, best_steps = 999;
    float best_x = 0, best_y = 0;
    int best_actions[200];

    /* 超时保护（避免死循环） */
    uint32_t eval_start_tick = rt_tick_get();

    for (int d = 0; d < 4; d++) {
        /* --- 超时检测，每个方向总耗时不超过 300ms --- */
        if (rt_tick_get() - eval_start_tick > 300) {
            wireless_uart_send_string("[Eval] timeout, skip remaining directions\r\n");
            break;
        }

        float stand_x, stand_y;
        if (obj_type == OBJ_BOX) {
            if (!find_coarse_adjacent_target(state, grid_map, obj_id,
                                             &stand_x, &stand_y, d))
                continue;
        } else {
            if (!find_bomb_stance_by_dir(grid_map, obj_x, obj_y, d, &stand_x, &stand_y))
                continue;
        }

        int test_act[200], test_cnt = 0;
        if (obj_type == OBJ_BOX) {
            float stand_img_x, stand_img_y;
            motion_to_image(stand_x, stand_y, &stand_img_x, &stand_img_y);

            int sgx = (int)(stand_img_x / CELL_SIZE);
            int sgy = (int)(stand_img_y / CELL_SIZE);
            char buf[64];
            rt_sprintf(buf, "[Eval] dir %d: stand grid (%d, %d)\r\n", d, sgx, sgy);
            wireless_uart_send_string(buf);

            test_cnt = light_sokoban_plan(state, grid_map, obj_id,
                                          stand_img_x, stand_img_y, test_act, 200);
        } else {
            float bomb_img_x, bomb_img_y, target_img_x, target_img_y, car_img_x, car_img_y;
            motion_to_image(obj_x, obj_y, &bomb_img_x, &bomb_img_y);
            motion_to_image(bomb_target_x, bomb_target_y, &target_img_x, &target_img_y);
            motion_to_image(stand_x, stand_y, &car_img_x, &car_img_y);
            int br = (int)(bomb_img_y / CELL_SIZE), bc = (int)(bomb_img_x / CELL_SIZE);
            int tr = (int)(target_img_y / CELL_SIZE), tc = (int)(target_img_x / CELL_SIZE);
            int cr = (int)(car_img_y / CELL_SIZE), cc = (int)(car_img_x / CELL_SIZE);

            char buf[64];
            rt_sprintf(buf, "[Eval] bomb dir %d: stand grid (%d, %d)\r\n", d, cc, cr);
            wireless_uart_send_string(buf);

            test_cnt = light_push_plan(grid_map, br, bc, tr, tc, cr, cc, test_act, 200);
            if (test_cnt > 0) {
                for (int i = 0; i < test_cnt; i++) test_act[i] += 4;  // 转为控制动作编码
            }
        }

        /* --- 丢弃步数超过150的动作序列（防止过长） --- */
        if (test_cnt > 150) {
            char buf[64];
            rt_sprintf(buf, "[Eval] dir %d ignored, steps=%d\r\n", d, test_cnt);
            wireless_uart_send_string(buf);
            continue;
        }

        if (test_cnt > 0 && test_cnt < best_steps) {
            int expected_first = 4 + d;
            if (test_act[0] == expected_first) {
                best_steps = test_cnt;
                best_x = stand_x; best_y = stand_y;
                best_count = test_cnt;
                memcpy(best_actions, test_act, test_cnt * sizeof(int));
            }
        }
    }

    /* 恢复箱子占据信息 */
    if (saved_flag) {
        if (g_map_mutex) rt_mutex_take(g_map_mutex, RT_WAITING_FOREVER);
        float img_x, img_y;
        motion_to_image(obj_x, obj_y, &img_x, &img_y);
        int obj_c = (int)(img_x / RESOLUTION);
        int obj_r = (int)(img_y / RESOLUTION);
        for (int dy = 0; dy < 4; dy++)
            for (int dx = 0; dx < 4; dx++) {
                int r = obj_r + dy, c = obj_c + dx;
                if (r < FINE_ROWS && c < FINE_COLS)
                    grid_map->occupancy[r][c] = saved[dy][dx];
            }
        if (g_map_mutex) rt_mutex_release(g_map_mutex);
    }

    if (best_count <= 0) return 0;

    *out_stand_x = best_x;
    *out_stand_y = best_y;
    *out_count = best_count;
    memcpy(out_actions, best_actions, best_count * sizeof(int));
    return 1;
}

/* ========== BFS 曼哈顿路径规划（可选） ========== */
#if USE_MANHATTAN_NAV
/**
 * @brief 使用BFS在网格上规划曼哈顿路径（仅上下左右移动）
 * @return 1成功，0失败
 */
static int bfs_plan_path(HybridController* ctrl,
                          float start_x, float start_y,
                          float target_x, float target_y,
                          float* out_path, int* out_len, int max_len) {
    float start_img_x, start_img_y, target_img_x, target_img_y;
    motion_to_image(start_x, start_y, &start_img_x, &start_img_y);
    motion_to_image(target_x, target_y, &target_img_x, &target_img_y);

    int sr = (int)(start_img_y / CELL_SIZE);
    int sc = (int)(start_img_x / CELL_SIZE);
    int gr = (int)(target_img_y / CELL_SIZE);
    int gc = (int)(target_img_x / CELL_SIZE);
    if (sr < 0) sr = 0; if (sr >= MAP_ROWS) sr = MAP_ROWS - 1;
    if (sc < 0) sc = 0; if (sc >= MAP_COLS) sc = MAP_COLS - 1;
    if (gr < 0) gr = 0; if (gr >= MAP_ROWS) gr = MAP_ROWS - 1;
    if (gc < 0) gc = 0; if (gc >= MAP_COLS) gc = MAP_COLS - 1;

    uint8_t obstacle[MAP_ROWS][MAP_COLS];
    memset(obstacle, 0, sizeof(obstacle));
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            int base_x = c * 4, base_y = r * 4;
            for (int dy = 0; dy < 4; dy++)
                for (int dx = 0; dx < 4; dx++)
                    if (ctrl->grid_map->occupancy[base_y+dy][base_x+dx] == OCC_WALL ||
                        ctrl->grid_map->occupancy[base_y+dy][base_x+dx] == OCC_BOX ||
                        ctrl->grid_map->occupancy[base_y+dy][base_x+dx] == OCC_BOMB)
                        { obstacle[r][c] = 1; goto next_cell; }
            next_cell:;
        }
    }
    obstacle[sr][sc] = 0; obstacle[gr][gc] = 0;

    typedef struct { int8_t r, c; int16_t parent; int8_t action; } NavNode;
    #define BFS_Q_SIZE (MAP_ROWS * MAP_COLS)
    static NavNode queue[BFS_Q_SIZE];
    static uint8_t visited[MAP_ROWS][MAP_COLS];
    memset(visited, 0, sizeof(visited));
    const int dr[4] = {-1,0,1,0}, dc[4] = {0,1,0,-1};

    int head = 0, tail = 0, found = -1;
    queue[tail].r = sr; queue[tail].c = sc;
    queue[tail].parent = -1; queue[tail].action = -1; tail++;
    visited[sr][sc] = 1;

    while (head < tail && found < 0) {
        NavNode cur = queue[head];
        if (cur.r == gr && cur.c == gc) { found = head; break; }
        for (int d = 0; d < 4; d++) {
            int nr = cur.r + dr[d], nc = cur.c + dc[d];
            if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
            if (visited[nr][nc] || obstacle[nr][nc]) continue;
            visited[nr][nc] = 1;
            if (tail < BFS_Q_SIZE) {
                queue[tail].r = nr; queue[tail].c = nc;
                queue[tail].parent = head; queue[tail].action = d;
                tail++;
            }
        }
        head++;
    }
    if (found < 0) return 0;

    int actions[200], act_cnt = 0, idx = found;
    while (idx >= 0) {
        if (queue[idx].action >= 0) actions[act_cnt++] = queue[idx].action;
        idx = queue[idx].parent;
    }
    for (int i = 0; i < act_cnt/2; i++) {
        int t = actions[i]; actions[i] = actions[act_cnt-1-i]; actions[act_cnt-1-i] = t;
    }

    float points[100][2]; int pt = 0;
    float cx = start_img_x, cy = start_img_y, mx, my;
    image_to_motion(cx, cy, &mx, &my);
    points[pt][0] = mx; points[pt][1] = my; pt++;
    for (int i = 0; i < act_cnt; i++) {
        int d = actions[i];
        if (d==0) cy -= CELL_SIZE; else if (d==1) cx += CELL_SIZE;
        else if (d==2) cy += CELL_SIZE; else if (d==3) cx -= CELL_SIZE;
        image_to_motion(cx, cy, &mx, &my);
        if (pt < 100) { points[pt][0] = mx; points[pt][1] = my; pt++; }
    }

    int out = 0;
    out_path[out*2+0] = points[0][0]; out_path[out*2+1] = points[0][1]; out++;
    for (int i = 1; i < pt && out < max_len; i++) {
        float dx = points[i][0] - points[i-1][0];
        float dy = points[i][1] - points[i-1][1];
        float seg = sqrtf(dx*dx + dy*dy);
        int steps = (int)(seg / PUSH_INTERP_STEP);
        if (steps < 1) steps = 1;
        float sx = dx/steps, sy = dy/steps;
        for (int j = 1; j <= steps && out < max_len; j++) {
            out_path[out*2+0] = points[i-1][0] + sx*j;
            out_path[out*2+1] = points[i-1][1] + sy*j;
            if (j == steps) { out_path[out*2+0] = points[i][0]; out_path[out*2+1] = points[i][1]; }
            out++;
        }
    }
    *out_len = out;
    return 1;
}
#endif

/* ========== 路径规划主入口（到目标点） ========== */
/**
 * @brief 规划从起点到目标点的路径（默认使用A*）
 * @param ctrl 控制器指针
 * @param start_x, start_y 起点坐标（米）
 * @param target_x, target_y 终点坐标（米）
 * @return 1成功，0失败
 */
int HybridController_PlanPathToPoint(HybridController* ctrl,
                                     float start_x, float start_y,
                                     float target_x, float target_y) {
#if USE_MANHATTAN_NAV
    float path[MAX_PATH_POINTS * 2];
    int plen;
    if (!bfs_plan_path(ctrl, start_x, start_y, target_x, target_y, path, &plen, MAX_PATH_POINTS))
        return 0;
    for (int i = 0; i < plen; i++) {
        ctrl->current_path[i][0] = path[i*2+0];
        ctrl->current_path[i][1] = path[i*2+1];
    }
    ctrl->path_len = plen;
    ctrl->path_target_idx = 1;
    ctrl->axial_tracking = 1;
#else
    float start_ix, start_iy, target_ix, target_iy;
    motion_to_image(start_x, start_y, &start_ix, &start_iy);
    motion_to_image(target_x, target_y, &target_ix, &target_iy);
    int start_gx, start_gy, goal_gx, goal_gy;
    world_to_grid(start_ix, start_iy, &start_gx, &start_gy);
    world_to_grid(target_ix, target_iy, &goal_gx, &goal_gy);

    int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
    AStarParams params = {5000, 2.0f};
    int len = astar_plan_path(ctrl->grid_map, start_gx, start_gy, goal_gx, goal_gy,
                              path_x, path_y, MAX_PATH_POINTS, &params);
    if (len <= 0) return 0;

    // 动态申请栈空间（此处用静态数组避免栈溢出）
    static int snapped_x[MAX_PATH_POINTS];
    static int snapped_y[MAX_PATH_POINTS];
    int snap_len = 0;
    for (int i = 0; i < len; i++) {
        int cc = path_x[i] / 4;
        int cr = path_y[i] / 4;
        int cx = cc * 4 + 2;
        int cy = cr * 4 + 2;
        if (snap_len > 0 && snapped_x[snap_len-1] == cx && snapped_y[snap_len-1] == cy)
            continue;
        snapped_x[snap_len] = cx;
        snapped_y[snap_len] = cy;
        snap_len++;
    }
    // 使用 4x4 单元格抽稀后的路径（检查是否安全），若不安全则用原始路径
    int use_snapped = (snap_len >= 2);
    if (use_snapped) {
        for (int i = 0; i < snap_len - 1; i++) {
            int dx = snapped_x[i+1] - snapped_x[i];
            int dy = snapped_y[i+1] - snapped_y[i];
            int steps = (abs(dx) > abs(dy)) ? abs(dx) : abs(dy);
            if (steps == 0) continue;
            float fx = (float)snapped_x[i];
            float fy = (float)snapped_y[i];
            float step_dx = (float)dx / steps;
            float step_dy = (float)dy / steps;
            int ok = 1;
            for (int s = 0; s <= steps && ok; s++) {
                int gx = (int)(fx + 0.5f);
                int gy = (int)(fy + 0.5f);
                for (int dy_off = -2; dy_off <= 1 && ok; dy_off++)
                    for (int dx_off = -2; dx_off <= 1 && ok; dx_off++) {
                        int nx = gx + dx_off, ny = gy + dy_off;
                        if (nx >= 0 && nx < FINE_COLS && ny >= 0 && ny < FINE_ROWS) {
                            uint8_t occ = ctrl->grid_map->occupancy[ny][nx];
                            if (occ == OCC_WALL || occ == OCC_BOX || occ == OCC_BOMB)
                                ok = 0;
                        }
                    }
                fx += step_dx;
                fy += step_dy;
            }
            if (!ok) { use_snapped = 0; break; }
        }
    }
    int out_len;
    int *straighten_x, *straighten_y;
    if (use_snapped) {
        straighten_x = snapped_x; straighten_y = snapped_y; out_len = snap_len;
    } else {
        straighten_x = path_x; straighten_y = path_y; out_len = len;
    }

    // 抽稀路径：删除冗余点，只保留关键转折点（通过 4x4 格子检测）
    #define MAX_COARSE_STEPS 8
    if (out_len > 2) {
        int kept_x[MAX_PATH_POINTS], kept_y[MAX_PATH_POINTS];
        int kept = 0;
        kept_x[kept] = straighten_x[0]; kept_y[kept] = straighten_y[0]; kept++;
        int last = 0;
        while (last < out_len - 1) {
            int next = last + 1;
            int max_j = last + MAX_COARSE_STEPS;
            if (max_j >= out_len) max_j = out_len - 1;
            for (int j = max_j; j > last; j--) {
                int dx = straighten_x[j] - straighten_x[last];
                int dy = straighten_y[j] - straighten_y[last];
                int steps = (abs(dx) > abs(dy)) ? abs(dx) : abs(dy);
                int ok = 1;
                if (steps > 0) {
                    float fx = (float)straighten_x[last];
                    float fy = (float)straighten_y[last];
                    float step_dx = (float)dx / steps;
                    float step_dy = (float)dy / steps;
                    for (int s = 0; s <= steps && ok; s++) {
                        int gx = (int)(fx + 0.5f);
                        int gy = (int)(fy + 0.5f);
                        for (int dy_off = -2; dy_off <= 1 && ok; dy_off++)
                            for (int dx_off = -2; dx_off <= 1 && ok; dx_off++) {
                                int nx = gx + dx_off, ny = gy + dy_off;
                                if (nx >= 0 && nx < FINE_COLS && ny >= 0 && ny < FINE_ROWS) {
                                    uint8_t occ = ctrl->grid_map->occupancy[ny][nx];
                                    if (occ == OCC_WALL || occ == OCC_BOX || occ == OCC_BOMB)
                                        ok = 0;
                                }
                            }
                        fx += step_dx; fy += step_dy;
                    }
                }
                if (ok) { next = j; break; }
            }
            if (kept < MAX_PATH_POINTS) {
                kept_x[kept] = straighten_x[next]; kept_y[kept] = straighten_y[next]; kept++;
            }
            last = next;
        }
        out_len = kept;
        for (int i = 0; i < kept; i++) {
            straighten_x[i] = kept_x[i]; straighten_y[i] = kept_y[i];
        }
    }

    // 将网格坐标转换为实际世界坐标（米）
    for (int i = 0; i < out_len; i++) {
        float wx, wy;
        grid_to_world(straighten_x[i], straighten_y[i], &wx, &wy);
        image_to_motion(wx, wy, &ctrl->current_path[i][0], &ctrl->current_path[i][1]);
    }
    ctrl->path_len = out_len;
    ctrl->path_target_idx = 1;
    ctrl->axial_tracking = 0;
#endif
    ctrl->path_following = 1;
    ctrl->path_purpose = PATH_PURPOSE_NAV_TO_BOX;
    ctrl->use_tangent_heading = 0;
    ctrl->path_locked_yaw = 0.0f;
    ctrl->push_smoothed_speed = 0.0f;
    ctrl->stuck_start_tick = 0;
    ctrl->path_stuck_counter = 0;
    PID_Reset(&angle_trace_param);
    PID_Reset(&pos_x_param);
    PID_Reset(&pos_y_param);
    ctrl->mode = CTRL_MODE_PATH_FOLLOWING;
    return 1;
}

/* ========== 规划推送路径（用于执行动作序列） ========== */
/**
 * @brief 根据动作序列（来自Sokoban/推炸弹规划）生成路径点
 * @param ctrl 控制器指针
 * @param start_x, start_y 起始位置（米）
 * @param actions 动作编码数组（编码值≥4表示方向）
 * @param action_count 动作数量
 * @return 1成功，0失败
 */
int HybridController_PlanPushPath(HybridController* ctrl,
                                  float start_x, float start_y,
                                  int* actions, int action_count) {
    if (action_count <= 0) return 0;
    float cur_x = start_x, cur_y = start_y;
    int pt = 0;
    // 第一个点为当前位置
    ctrl->current_path[pt][0] = cur_x;
    ctrl->current_path[pt][1] = cur_y;
    pt++;

    for (int i = 0; i < action_count; i++) {
        int dir = actions[i] - 4;
        if (dir == 0)      cur_y -= CELL_SIZE;
        else if (dir == 1) cur_x += CELL_SIZE;
        else if (dir == 2) cur_y += CELL_SIZE;
        else if (dir == 3) cur_x -= CELL_SIZE;
    }
    ctrl->current_path[pt][0] = cur_x;
    ctrl->current_path[pt][1] = cur_y;
    pt++;

    if (pt < 2) return 0;
    ctrl->path_len = pt;
    ctrl->path_target_idx = 1;
    ctrl->path_following = 1;
    ctrl->path_purpose = PATH_PURPOSE_PUSH_STANCE;
    ctrl->use_tangent_heading = 0;
    ctrl->path_locked_yaw = 0.0f;
    ctrl->push_smoothed_speed = 0.0f;
    ctrl->stuck_start_tick = 0;
    ctrl->path_stuck_counter = 0;
    PID_Reset(&angle_trace_param);
    PID_Reset(&pos_x_param);
    PID_Reset(&pos_y_param);
    ctrl->mode = CTRL_MODE_PATH_FOLLOWING;
    return 1;
}

/* ========== 寻找箱子旁边的空位（用于站位） ========== */
/**
 * @brief 根据指定方向，在箱子旁边寻找空闲站位
 * @param state 游戏状态
 * @param grid_map 地图
 * @param box_id 箱子ID
 * @param out_x, out_y 输出站位坐标（米）
 * @param preferred_dir 优先方向（0上，1右，2下，3左）
 * @return 1找到，0未找到
 */
int find_coarse_adjacent_target(GameState* state, GridMap* grid_map,
                                int box_id, float* out_x, float* out_y,
                                int preferred_dir) {
    Box* box = &state->boxes[box_id];
    float box_img_x, box_img_y;
    motion_to_image(box->x, box->y, &box_img_x, &box_img_y);
    int box_r = (int)(box_img_y / CELL_SIZE);
    int box_c = (int)(box_img_x / CELL_SIZE);
    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};
    const int opposite[4] = {2, 3, 0, 1};

    // 优先使用指定方向的反方向（即推箱子的后方）
    if (preferred_dir >= 0 && preferred_dir < 4) {
        int back_dir = opposite[preferred_dir];
        int nr = box_r + dr[back_dir];
        int nc = box_c + dc[back_dir];
        if (nr >= 0 && nr < MAP_ROWS && nc >= 0 && nc < MAP_COLS) {
            int base_x = nc * 4, base_y = nr * 4;
            int blocked = 0;
            for (int dy = 0; dy < 4 && !blocked; dy++)
                for (int dx = 0; dx < 4; dx++)
                    if (grid_map->occupancy[base_y+dy][base_x+dx] == OCC_WALL ||
                        grid_map->occupancy[base_y+dy][base_x+dx] == OCC_BOX ||
                        grid_map->occupancy[base_y+dy][base_x+dx] == OCC_BOMB)
                        blocked = 1;
            if (!blocked) {
                float target_img_x = (nc + 0.5f) * CELL_SIZE;
                float target_img_y = (nr + 0.5f) * CELL_SIZE;
                image_to_motion(target_img_x, target_img_y, out_x, out_y);
                return 1;
            }
        }
    }
    return 0;
}

/* ========== 路径跟踪相关辅助函数 ========== */
/**
 * @brief 寻找路径上离当前位置最近的点索引
 */
static int find_nearest_point_on_path(const float path[][2], int len, float x, float y, float* min_dist) {
    int idx = 0;
    *min_dist = 1e9f;
    for (int i = 0; i < len; i++) {
        float dx = path[i][0] - x, dy = path[i][1] - y;
        float d = dx*dx + dy*dy;
        if (d < *min_dist) { *min_dist = d; idx = i; }
    }
    *min_dist = sqrtf(*min_dist);
    return idx;
}

/**
 * @brief 获取路径上前视点（lookahead point）
 * @param path 路径点数组
 * @param len 路径长度
 * @param start_idx 起始索引
 * @param lookahead 前视距离（米）
 * @param out_x, out_y 输出前视点坐标
 */
static void get_lookahead_point(const float path[][2], int len, int start_idx,
                                float lookahead, float* out_x, float* out_y) {
    if (start_idx >= len - 1) {
        *out_x = path[len-1][0]; *out_y = path[len-1][1];
        return;
    }
    float cumulative = 0.0f;
    for (int i = start_idx; i < len - 1; i++) {
        float seg_len = sqrtf(powf(path[i+1][0]-path[i][0], 2) + powf(path[i+1][1]-path[i][1], 2));
        if (cumulative + seg_len >= lookahead) {
            float ratio = (lookahead - cumulative) / seg_len;
            *out_x = path[i][0] + ratio * (path[i+1][0] - path[i][0]);
            *out_y = path[i][1] + ratio * (path[i+1][1] - path[i][1]);
            return;
        }
        cumulative += seg_len;
    }
    *out_x = path[len-1][0]; *out_y = path[len-1][1];
}

/**
 * @brief 核心路径跟踪函数
 * @param ctrl 控制器指针
 * @param car_x, car_y, car_angle 当前位姿
 * @param vx, vy, omega 输出控制量
 * @param dist_to_end 输出到终点的距离
 * @return 1继续跟踪，0结束
 */
int follow_path(HybridController* ctrl, float car_x, float car_y, float car_angle,
                float* vx, float* vy, float* omega, float* dist_to_end) {
    AnglePID_SwitchMode(0);

    if (ctrl->path_len < 2) return 0;

    float current_yaw = position.yaw_rad * RAD_TO_DEG;

    /* 寻找最近点 */
    float min_dist;
    int nearest = find_nearest_point_on_path(ctrl->current_path, ctrl->path_len, car_x, car_y, &min_dist);
    *dist_to_end = sqrtf(powf(ctrl->current_path[ctrl->path_len-1][0] - car_x, 2) +
                         powf(ctrl->current_path[ctrl->path_len-1][1] - car_y, 2));

    float target_x, target_y;
    if (ctrl->path_target_idx < 0 || ctrl->path_target_idx >= ctrl->path_len)
        ctrl->path_target_idx = (nearest + 1 < ctrl->path_len) ? nearest + 1 : ctrl->path_len - 1;
    if (ctrl->path_target_idx >= ctrl->path_len)
        ctrl->path_target_idx = ctrl->path_len - 1;
    {
        float tdx = ctrl->current_path[ctrl->path_target_idx][0] - car_x;
        float tdy = ctrl->current_path[ctrl->path_target_idx][1] - car_y;
        if (sqrtf(tdx*tdx + tdy*tdy) < ctrl->path_tolerance &&
            ctrl->path_target_idx < ctrl->path_len - 1)
            ctrl->path_target_idx++;
    }
    target_x = ctrl->current_path[ctrl->path_target_idx][0];
    target_y = ctrl->current_path[ctrl->path_target_idx][1];

    float dx = target_x - car_x;
    float dy = target_y - car_y;
    float dist_err = sqrtf(dx*dx + dy*dy);
    if (dist_err < 1e-3f) {
        *vx = *vy = *omega = 0;
        return 1;
    }

    // 期望速度：与误差成正比，限制在最大/最小速度之间
    float desired_speed = fminf(ctrl->max_speed, 1.5f * dist_err);
    desired_speed = fmaxf(ctrl->min_speed, desired_speed);
    float vx_global = desired_speed * dx / dist_err;
    float vy_global = desired_speed * dy / dist_err;
    if (ctrl->axial_tracking) {
        float ax = fabsf(dx), ay = fabsf(dy);
        if (ax > ay) vy_global = 0.0f; else vx_global = 0.0f;
    }

    *vx = vx_global;
    *vy = vy_global;
    // 当误差小于8cm时，切换为PID位置控制（提高静态精度）
    uint8_t in_pid = 0;
    if (dist_err < 0.08f) {
        float limit = ctrl->max_speed;
        pos_x_param.output_max =  limit;
        pos_x_param.output_min = -limit;
        pos_y_param.output_max =  limit;
        pos_y_param.output_min = -limit;
        uint8_t is_end = (*dist_to_end < 0.30f);
        float saved_ki_x = pos_x_param.ki;
        float saved_ki_y = pos_y_param.ki;
        if (!is_end) {
            pos_x_param.ki = 0.0f;
            pos_y_param.ki = 0.0f;
            pos_x_param.error_sum_max = 1.0f;
            pos_x_param.error_sum_min = -1.0f;
            pos_y_param.error_sum_max = 1.0f;
            pos_y_param.error_sum_min = -1.0f;
        } else {
            pos_x_param.error_sum_max =  limit / pos_x_param.ki;
            pos_x_param.error_sum_min = -limit / pos_x_param.ki;
            pos_y_param.error_sum_max =  limit / pos_y_param.ki;
            pos_y_param.error_sum_min = -limit / pos_y_param.ki;
        }
        *vx = PID_AntiWindup(&pos_x_param, 0, dx);
        *vy = PID_AntiWindup(&pos_y_param, 0, dy);
        {
            static uint32_t last_pid_tick = 0;
            static uint16_t pid_call = 0;
            pid_call++;
            uint32_t now = rt_tick_get();
            if (now - last_pid_tick > RT_TICK_PER_SECOND / 4 || pid_call % 50 == 0) {
                last_pid_tick = now;
                char buf[128];
                rt_sprintf(buf, "[PID] e=%d i=%d/%d dE=%d dx=%d dy=%d vx=%d vy=%d sx=%d sy=%d\r\n",
                           is_end,
                           ctrl->path_target_idx, ctrl->path_len,
                           (int)(dist_err * 1000),
                           (int)(dx * 1000), (int)(dy * 1000),
                           (int)(*vx * 1000), (int)(*vy * 1000),
                           (int)(pos_x_param.error_sum * 1000),
                           (int)(pos_y_param.error_sum * 1000));
                wireless_uart_send_string(buf);
            }
        }
        pos_x_param.ki = saved_ki_x;
        pos_y_param.ki = saved_ki_y;
        if (*vx > limit) *vx = limit;
        else if (*vx < -limit) *vx = -limit;
        if (*vy > limit) *vy = limit;
        else if (*vy < -limit) *vy = -limit;
        in_pid = 1;
    }

    // 根据运动方向切换角度PID参数（导航/侧移）
    if (fabsf(dy) > fabsf(dx)) {
        angle_trace_param.kp = angle_pid_lateral.kp;
        angle_trace_param.kd = angle_pid_lateral.kd;
    } else {
        angle_trace_param.kp = angle_pid_nav.kp;
        angle_trace_param.kd = angle_pid_nav.kd;
    }

    // 航向控制：锁定初始航向（use_tangent_heading=0）
    float target_yaw_deg = ctrl->path_locked_yaw;
    float yaw_error = angle_diff(target_yaw_deg, current_yaw);
    if (fabsf(yaw_error) < 3.0f) yaw_error = 0.0f;
    float omega_corr_deg = PD(&angle_trace_param, 0, yaw_error);
    float max_omega_nav = 25.0f;
    if (omega_corr_deg > max_omega_nav) omega_corr_deg = max_omega_nav;
    else if (omega_corr_deg < -max_omega_nav) omega_corr_deg = -max_omega_nav;
    *omega = omega_corr_deg * DEG_TO_RAD;

    // 速度平滑（加速/减速）
    if (!in_pid) {
        float max_accel = 0.002f;
        if (desired_speed > ctrl->push_smoothed_speed) {
            ctrl->push_smoothed_speed += max_accel;
            if (ctrl->push_smoothed_speed > desired_speed)
                ctrl->push_smoothed_speed = desired_speed;
        } else {
            ctrl->push_smoothed_speed = desired_speed;
        }
        if (desired_speed > 1e-3f) {
            *vx = (*vx / desired_speed) * ctrl->push_smoothed_speed;
            *vy = (*vy / desired_speed) * ctrl->push_smoothed_speed;
        } else {
            *vx = *vy = 0;
        }
    }

    return 1;
}

/**
 * @brief 检测路径跟踪是否卡死（2秒内位移<2cm且角度变化<3°）
 */
static void check_path_stuck(HybridController* ctrl, float car_x, float car_y, float car_angle) {
    if (ctrl->path_purpose == PATH_PURPOSE_PUSH_STANCE)
        return;  // 推送过程中允许较慢移动

    uint32_t now = rt_tick_get();
    if (ctrl->stuck_start_tick == 0) {
        ctrl->stuck_start_tick = now;
        ctrl->stuck_start_pos[0] = car_x;
        ctrl->stuck_start_pos[1] = car_y;
        ctrl->stuck_start_angle = car_angle;
        return;
    }

    float total_disp = sqrtf(powf(car_x - ctrl->stuck_start_pos[0], 2) +
                             powf(car_y - ctrl->stuck_start_pos[1], 2));
    float total_angle = fabsf(car_angle - ctrl->stuck_start_angle);
    if (total_angle > M_PI) total_angle = 2*M_PI - total_angle;

    if (now - ctrl->stuck_start_tick > 2000) {
        if (total_disp < 0.02f && total_angle < 0.05f) {
            CarController_Stop();          // 停车并上报卡死
            ctrl->mode = CTRL_MODE_IDLE;
            ctrl->path_following = 0;
            ctrl->complete_reason = CTRL_COMPLETE_FAIL_STUCK;
            rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
        }
        ctrl->stuck_start_tick = 0;
    } else {
        if (total_disp > 0.03f || total_angle > 0.05f) {
            ctrl->stuck_start_tick = 0;
        }
    }
}

/* ========== 控制器初始化和重置 ========== */
void HybridController_Init(HybridController* ctrl, GridMap* grid_map, GameState* game_state) {
    memset(ctrl, 0, sizeof(HybridController));
    ctrl->mode = CTRL_MODE_IDLE;
    ctrl->grid_map = grid_map;
    ctrl->game_state = game_state;

    ctrl->lookahead_dist = LOOKAHEAD_DIST;
    ctrl->min_lookahead  = MIN_LOOKAHEAD;
    ctrl->max_lookahead  = MAX_LOOKAHEAD;
    ctrl->max_speed      = NAV_SPEED_MAX;
    ctrl->min_speed      = MIN_SPEED;
    ctrl->path_tolerance = PATH_TOLERANCE;
    ctrl->enable_adaptive_lookahead = ENABLE_ADAPTIVE_LOOKAHEAD;
    ctrl->path_stuck_threshold = 300;

    ctrl->push_speed_max = PUSH_SPEED_MAX_DFL;

    ctrl->current_box_id = -1;
    ctrl->current_bomb_id = -1;
    ctrl->is_bomb_path = 0;

    ctrl->path_purpose = PATH_PURPOSE_NAV_TO_BOX;
    ctrl->complete_reason = CTRL_COMPLETE_SUCCESS;

    ctrl->recog_pending = 0;
    ctrl->recog_target_id = -1;
    ctrl->use_tangent_heading = 0;
    ctrl->path_locked_yaw = 0.0f;
    ctrl->push_smoothed_speed = 0.0f;

    ctrl->precomputed_count = 0;

    // 其他成员变量初始化（需要时按需填充）
    ctrl->stop_start_tick = 0;
    ctrl->pending_path_purpose = PATH_PURPOSE_NAV_TO_BOX;
}

void HybridController_Reset(HybridController* ctrl) {
    GridMap*   saved_grid  = ctrl->grid_map;
    GameState* saved_state = ctrl->game_state;
    memset(ctrl, 0, sizeof(HybridController));
    ctrl->grid_map   = saved_grid;
    ctrl->game_state = saved_state;
    HybridController_Init(ctrl, saved_grid, saved_state);
}

/* ========== 识别导航（双模式支持） ========== */
/**
 * @brief 导航至识别站位并执行识别（数字/箱子类型）
 * @param ctrl 控制器指针
 * @param target_grid_x, target_grid_y 目标格子坐标（列/行）
 * @param target_type 识别目标类型（RECOG_TARGET_DEST 或 RECOG_TARGET_BOX）
 * @param target_id 目标ID
 * @return 1成功启动，0失败
 */
int HybridController_NavigateAndRecognize(HybridController* ctrl,
                                          int target_grid_x, int target_grid_y,
                                          RecognTargetType_t target_type, int target_id) {
    if (ctrl->mode != CTRL_MODE_IDLE) return 0;
    float target_x, target_y, target_yaw;
    float car_x = position.x_m, car_y = position.y_m;
    if (!find_adjacent_for_recog(ctrl->grid_map, target_grid_x, target_grid_y,
                                  &target_x, &target_y, &target_yaw,
                                  car_x, car_y))
        return 0;
    if (!HybridController_PlanPathToPoint(ctrl, car_x, car_y, target_x, target_y))
        return 0;
    ctrl->path_purpose = PATH_PURPOSE_RECOG_STANCE;
    ctrl->recog_pending = 1;
    ctrl->recog_target_type = target_type;
    ctrl->recog_target_id = target_id;
    ctrl->recog_stand_x = target_x;
    ctrl->recog_stand_y = target_y;
    ctrl->recog_align_yaw = target_yaw;
    ctrl->recog_original_yaw = position.yaw;
    return 1;
}

/* ========== 主控制计算函数（状态机） ========== */
void HybridController_ComputeControl(HybridController* ctrl,
                                     float car_x, float car_y, float car_angle,
                                     float dt, float current_time,
                                     float* out_vx, float* out_vy, float* out_omega) {
    *out_vx = *out_vy = *out_omega = 0;

    switch (ctrl->mode) {
        case CTRL_MODE_IDLE: {
            *out_vx = *out_vy = 0;
            float yaw_err = angle_diff(ctrl->path_locked_yaw, car_angle * RAD_TO_DEG);
            float omega_deg = PD(&angle_trace_param, 0, yaw_err);
            if (omega_deg > 25.0f) omega_deg = 25.0f;
            else if (omega_deg < -25.0f) omega_deg = -25.0f;
            *out_omega = omega_deg * DEG_TO_RAD;
            break;
        }

        case CTRL_MODE_PATH_FOLLOWING: {
            if (!ctrl->path_following || ctrl->path_len == 0) {
                ctrl->mode = CTRL_MODE_IDLE;
                break;
            }
            // 检测卡死
            check_path_stuck(ctrl, car_x, car_y, car_angle);

            float dist_to_end;
            int ret = follow_path(ctrl, car_x, car_y, car_angle,
                                  out_vx, out_vy, out_omega, &dist_to_end);

            if (ret && dist_to_end < ctrl->path_tolerance) {
                ctrl->path_following = 0;
                *out_vx = *out_vy = *out_omega = 0;
                CarController_Stop();                     // 停车
                ctrl->mode = CTRL_MODE_STOPPING;
                ctrl->stop_start_tick = rt_tick_get();
                ctrl->pending_path_purpose = ctrl->path_purpose;
            }
            break;
        }

        case CTRL_MODE_STOPPING: {
            float speed = sqrtf(car_ctrl.current_vx * car_ctrl.current_vx +
                                car_ctrl.current_vy * car_ctrl.current_vy);
            if (speed < 0.02f || (rt_tick_get() - ctrl->stop_start_tick > 300)) {
                if (ctrl->pending_path_purpose == PATH_PURPOSE_PUSH_STANCE) {
                    need_map_update = 1;
                }
                if (ctrl->pending_path_purpose == PATH_PURPOSE_RECOG_STANCE) {
                    ctrl->mode = CTRL_MODE_RECOGNIZING;
                } else {
                    ctrl->mode = CTRL_MODE_IDLE;
                    rt_event_send(g_task_mgr.event, TASK_EVENT_CONTROLLER_IDLE);
                }
            }
            *out_vx = *out_vy = 0;
            {
                float yaw_err = angle_diff(ctrl->path_locked_yaw, car_angle * RAD_TO_DEG);
                float omega_deg = PD(&angle_trace_param, 0, yaw_err);
                if (omega_deg > 25.0f) omega_deg = 25.0f;
                else if (omega_deg < -25.0f) omega_deg = -25.0f;
                *out_omega = omega_deg * DEG_TO_RAD;
            }
            break;
        }

        case CTRL_MODE_RECOGNIZING: {
            static uint8_t step = 0;
            static uint32_t timer = 0;
            switch (step) {
                case 0:
                    AnglePID_SwitchMode(1);
                    if (RotateToAngleIMU(ctrl->recog_align_yaw)) {
                        step = 1; timer = rt_tick_get();
                    } else {
                        ctrl->recog_pending = 0;
                        ctrl->mode = CTRL_MODE_IDLE;
                        rt_event_send(g_task_mgr.event, TASK_EVENT_RECOG_FAILED);
                        step = 0;
                    }
                    break;
                case 1:
                    if (rt_tick_get() - timer > 500) step = 2;
                    break;
                case 2: {
                    int success = 0;
                    if (ctrl->recog_target_type == RECOG_TARGET_DEST) {
                        int digit;
                        success = uart4_request_digit(&digit);
                        if (success) {
                            Destination* dest = &ctrl->game_state->destinations[ctrl->recog_target_id];
                            dest->required_digit = digit;
                            dest->recognized = 1;
                            g_digit_map[g_digit_map_count].digit = digit;
                            g_digit_map[g_digit_map_count].dest_id = ctrl->recog_target_id;
                            g_digit_map[g_digit_map_count].x = dest->x;
                            g_digit_map[g_digit_map_count].y = dest->y;
                            g_digit_map_count++;
                        }
                    } else if (ctrl->recog_target_type == RECOG_TARGET_BOX) {
                        BoxTypeEnum_t type;
                        success = uart4_request_box_type(&type);
                        if (success) {
                            ctrl->game_state->boxes[ctrl->recog_target_id].type = type;
                            ctrl->game_state->boxes[ctrl->recog_target_id].recognized = 1;
                        }
                    }
                    step = 3; timer = rt_tick_get();
                    break;
                }
                case 3:
                    AnglePID_SwitchMode(1);
                    RotateToAngleIMU(ctrl->recog_original_yaw);
                    step = 4;
                    break;
                case 4:
                    ctrl->recog_pending = 0;
                    ctrl->mode = CTRL_MODE_IDLE;
                    rt_event_send(g_task_mgr.event, TASK_EVENT_RECOG_DONE);
                    step = 0;
                    break;
            }
            *out_vx = *out_vy = 0;
            {
                float yaw_err = angle_diff(ctrl->path_locked_yaw, car_angle * RAD_TO_DEG);
                float omega_deg = PD(&angle_trace_param, 0, yaw_err);
                if (omega_deg > 25.0f) omega_deg = 25.0f;
                else if (omega_deg < -25.0f) omega_deg = -25.0f;
                *out_omega = omega_deg * DEG_TO_RAD;
            }
            break;
        }

        default:
            ctrl->mode = CTRL_MODE_IDLE;
            break;
    }
}