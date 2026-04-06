#include "planner.h"
#include <math.h>
#include <string.h>
#include <rtthread.h>
#include "zf_device_wireless_uart.h"

// ================= 多炸弹并发规划数据结构 =================

#define MAX_BOMB_PLANS 4
#define MAX_PATH_LENGTH MAX_BOMB_PATH_LENGTH

// 时间扩展状态空间 - 位图标记占用
static uint8_t time_occupancy[MAP_ROWS][MAP_COLS][MAX_TIME_SLOTS];  // [行][列][时间槽]

// 候选墙体结构（用于目标选择）
typedef struct {
    int wall_idx;
    float bomb_target_x, bomb_target_y;   // 运动坐标
    int push_steps;          // 炸墙后推箱子所需步数（越小越好）
    int box_id;              // 该候选是针对哪个箱子的
} WallCandidate;

// ================= 辅助函数 =================

// 评估指定箱子的所有墙体候选
static int evaluate_walls_for_box(GameState* state, GridMap* grid_map, int box_id,
                                  int car_start_r, int car_start_c,
                                  WallCandidate* candidates, int max_candidates) {
    int count = 0;
    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};

    for (int w = 0; w < state->num_walls && count < max_candidates; w++) {
        if (is_boundary_wall(&state->walls[w])) continue;

        Wall* wall = &state->walls[w];
        // 墙体运动坐标 -> 图像坐标
        float wall_img_x, wall_img_y;
        motion_to_image(wall->x1, wall->y1, &wall_img_x, &wall_img_y);
        int wall_r = (int)(wall_img_y / CELL_SIZE + 0.5f);
        int wall_c = (int)(wall_img_x / CELL_SIZE + 0.5f);

        for (int d = 0; d < 4; d++) {
            int nr = wall_r + dr[d];
            int nc = wall_c + dc[d];
            if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;

            int base_x = nc * 4;
            int base_y = nr * 4;
            int occupied = 0;
            for (int dy = 0; dy < 4 && !occupied; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    if (grid_map->occupancy[base_y + dy][base_x + dx] == OCC_WALL ||
                        grid_map->occupancy[base_y + dy][base_x + dx] == OCC_BOX) {
                        occupied = 1;
                        break;
                    }
                }
            }
            if (occupied) continue;

            // 炸弹目标点（图像坐标粗网格中心）
            float bomb_target_img_x = (nc + 0.5f) * CELL_SIZE;
            float bomb_target_img_y = (nr + 0.5f) * CELL_SIZE;
            float bomb_target_mx, bomb_target_my;
            image_to_motion(bomb_target_img_x, bomb_target_img_y, &bomb_target_mx, &bomb_target_my);

            // 模拟炸墙后推箱子
            int steps;
            // 注意：simulate_wall_destruction 期望小车细网格坐标（图像坐标），car_start_r/car_start_c 是图像粗网格索引，需转换为细网格坐标
            int car_fine_x = car_start_c * 4 + 2;
            int car_fine_y = car_start_r * 4 + 2;
            if (simulate_wall_destruction(state, grid_map, w,
                                          car_fine_x, car_fine_y, box_id, &steps)) {
                candidates[count].wall_idx = w;
                candidates[count].bomb_target_x = bomb_target_mx;
                candidates[count].bomb_target_y = bomb_target_my;
                candidates[count].push_steps = steps;
                candidates[count].box_id = box_id;
                count++;
                break;  // 每个墙体只取第一个可用格子
            }
        }
    }
    return count;
}

// 前向声明
int can_reach_fine_exclude_bomb(GridMap* map, int sx, int sy, int gx, int gy, int bomb_r, int bomb_c);

// ================= 轻量级炸弹规划器辅助函数 =================

// 细网格8方向BFS，检查起点(sx,sy)到终点(gx,gy)是否可达（使用图像坐标）
static int can_reach_fine(GridMap* map, int sx, int sy, int gx, int gy) {
    if (sx == gx && sy == gy) return 1;
    if (map->occupancy[sy][sx] == OCC_WALL || map->occupancy[sy][sx] == OCC_BOX ||
        map->occupancy[gy][gx] == OCC_WALL || map->occupancy[gy][gx] == OCC_BOX)
        return 0;

    const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static int qx[FINE_COLS * FINE_ROWS];
    static int qy[FINE_COLS * FINE_ROWS];
    static uint8_t visited[FINE_ROWS][FINE_COLS];
    memset(visited, 0, sizeof(visited));

    int head = 0, tail = 0;
    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sy][sx] = 1;

    while (head < tail) {
        int x = qx[head];
        int y = qy[head];
        head++;
        for (int d = 0; d < 8; d++) {
            int nx = x + dx[d];
            int ny = y + dy[d];
            if (nx < 0 || nx >= map->width || ny < 0 || ny >= map->height) continue;
            if (visited[ny][nx]) continue;
            if (dx[d] != 0 && dy[d] != 0) {
                if (map->occupancy[y][nx] == OCC_WALL || map->occupancy[y][nx] == OCC_BOX) continue;
                if (map->occupancy[ny][x] == OCC_WALL || map->occupancy[ny][x] == OCC_BOX) continue;
            }
            if (map->occupancy[ny][nx] == OCC_WALL || map->occupancy[ny][nx] == OCC_BOX) continue;
            if (nx == gx && ny == gy) return 1;
            visited[ny][nx] = 1;
            qx[tail] = nx; qy[tail] = ny; tail++;
            if (tail >= FINE_COLS * FINE_ROWS) tail = 0;
        }
    }
    return 0;
}

// 通用轻量级推箱子规划器（用于炸弹），所有粗网格坐标均为图像坐标系
int light_push_plan(GridMap* map, int start_r, int start_c, int goal_r, int goal_c,
                    int car_start_r, int car_start_c, int* out_actions, int max_actions) {
    //printf("\n-------- light_push_plan --------\n");
    //printf("炸弹起始粗网格(图像): (%d,%d)\n", start_c, start_r);
    //printf("炸弹目标粗网格(图像): (%d,%d)\n", goal_c, goal_r);
    //printf("小车起始粗网格(图像): (%d,%d)\n", car_start_c, car_start_r);

    // 构建障碍物地图（粗网格，图像坐标系）
    static uint8_t obstacle[MAP_ROWS][MAP_COLS];
    memset(obstacle, 0, sizeof(obstacle));
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            int base_x = c * 4;
            int base_y = r * 4;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    uint8_t occ = map->occupancy[base_y + dy][base_x + dx];
                    if (occ == OCC_WALL || occ == OCC_BOX) {
                        obstacle[r][c] = 1;
                        break;
                    }
                }
                if (obstacle[r][c]) break;
            }
        }
    }
    
    // 打印障碍物地图（可选，只打印目标点周围5x5）
    // printf("障碍物地图 (目标点周围5x5):\n");
    for (int r = goal_r - 2; r <= goal_r + 2; r++) {
        for (int c = goal_c - 2; c <= goal_c + 2; c++) {
            //if (r < 0 || r >= MAP_ROWS || c < 0 || c >= MAP_COLS)
                // printf(". ");
            //else
                // printf("%d ", obstacle[r][c]);
        }
        // printf("\n");
    }
    // printf("炸弹起始位置障碍物值: obstacle[%d][%d]=%d\n", start_r, start_c, obstacle[start_r][start_c]);
    // printf("炸弹目标位置障碍物值: obstacle[%d][%d]=%d\n", goal_r, goal_c, obstacle[goal_r][goal_c]);

    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};

    typedef struct {
        int br, bc;
        int pr, pc;
        int pushes;
        int parent;
        int action;
    } PushNode;

    static PushNode queue[200];
    int best_pushes[MAP_ROWS][MAP_COLS];
    memset(best_pushes, -1, sizeof(best_pushes));

    int head = 0, tail = 0;
    queue[tail].br = start_r;
    queue[tail].bc = start_c;
    queue[tail].pr = car_start_r;
    queue[tail].pc = car_start_c;
    queue[tail].pushes = 0;
    queue[tail].parent = -1;
    queue[tail].action = -1;
    tail++;
    best_pushes[start_r][start_c] = 0;

    int found_idx = -1;
    int max_queue = 200;
    int expanded = 0;

    while (head < tail && expanded < 1000) {
        PushNode cur = queue[head];
        expanded++;
        
        if (cur.br == goal_r && cur.bc == goal_c) {
            found_idx = head;
            //printf("BFS找到目标，扩展节点数=%d, 队列长度=%d\n", expanded, tail);
            break;
        }

        for (int d = 0; d < 4; d++) {
            int push_r = cur.br - dr[d];
            int push_c = cur.bc - dc[d];
            if (push_r < 0 || push_r >= MAP_ROWS || push_c < 0 || push_c >= MAP_COLS) continue;
            if (obstacle[push_r][push_c] == 1) continue;

            int nbr = cur.br + dr[d];
            int nbc = cur.bc + dc[d];
            if (nbr < 0 || nbr >= MAP_ROWS || nbc < 0 || nbc >= MAP_COLS) continue;
            if (obstacle[nbr][nbc] == 1) continue;

            // 小车能否到达推动位置（细网格坐标）
            int sx = cur.pc * 4 + 2;
            int sy = cur.pr * 4 + 2;
            int gx = push_c * 4 + 2;
            int gy = push_r * 4 + 2;
            
            // 打印尝试的推动方向（可选，减少输出）
            // printf("  尝试方向%d: push_pos(%d,%d), 小车从(%d,%d)到(%d,%d)\n", d, push_r, push_c, cur.pr, cur.pc, push_r, push_c);
            
            if (!can_reach_fine_exclude_bomb(map, sx, sy, gx, gy, cur.br, cur.bc))
                continue;

            int new_pushes = cur.pushes + 1;
            if (best_pushes[nbr][nbc] == -1 || new_pushes < best_pushes[nbr][nbc]) {
                best_pushes[nbr][nbc] = new_pushes;
                if (tail >= max_queue) {
                    printf("队列已满，终止BFS\n");
                    break;
                }
                queue[tail].br = nbr;
                queue[tail].bc = nbc;
                queue[tail].pr = cur.br;
                queue[tail].pc = cur.bc;
                queue[tail].pushes = new_pushes;
                queue[tail].parent = head;
                queue[tail].action = d;
                tail++;
            }
        }
        head++;
    }

    if (found_idx == -1) {
        //printf("BFS未找到目标，扩展节点数=%d, 队列长度=%d\n", expanded, tail);
        return -1;
    }

    // 回溯动作
    int actions[200];
    int act_cnt = 0;
    int idx = found_idx;
    while (idx != -1) {
        if (queue[idx].action != -1) {
            actions[act_cnt++] = queue[idx].action;
        }
        idx = queue[idx].parent;
    }
    for (int i = 0; i < act_cnt / 2; i++) {
        int tmp = actions[i];
        actions[i] = actions[act_cnt - 1 - i];
        actions[act_cnt - 1 - i] = tmp;
    }

    int n = (act_cnt < max_actions) ? act_cnt : max_actions;
    for (int i = 0; i < n; i++) out_actions[i] = actions[i];
    
    //printf("light_push_plan 成功，动作数=%d\n", n);
    //for (int i = 0; i < n; i++) printf("%d ", out_actions[i]);
    //printf("\n");
    
    return n;
}

// 细网格8方向BFS，检查起点(sx,sy)到终点(gx,gy)是否可达，排除炸弹所在粗网格（图像坐标）
int can_reach_fine_exclude_bomb(GridMap* map, int sx, int sy, int gx, int gy,
                                        int bomb_r, int bomb_c) {
    if (sx == gx && sy == gy) return 1;
    if (map->occupancy[sy][sx] == OCC_WALL || map->occupancy[sy][sx] == OCC_BOX ||
        map->occupancy[gy][gx] == OCC_WALL || map->occupancy[gy][gx] == OCC_BOX)
        return 0;

    const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static int qx[FINE_COLS * FINE_ROWS];
    static int qy[FINE_COLS * FINE_ROWS];
    static uint8_t visited[FINE_ROWS][FINE_COLS];
    memset(visited, 0, sizeof(visited));

    int head = 0, tail = 0;
    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sy][sx] = 1;

    while (head < tail) {
        int x = qx[head];
        int y = qy[head];
        head++;
        for (int d = 0; d < 8; d++) {
            int nx = x + dx[d];
            int ny = y + dy[d];
            if (nx < 0 || nx >= map->width || ny < 0 || ny >= map->height) continue;
            if (visited[ny][nx]) continue;
            if (ny/4 == bomb_r && nx/4 == bomb_c) continue;
            if (dx[d] != 0 && dy[d] != 0) {
                if (map->occupancy[y][nx] == OCC_WALL || map->occupancy[y][nx] == OCC_BOX) continue;
                if (map->occupancy[ny][x] == OCC_WALL || map->occupancy[ny][x] == OCC_BOX) continue;
            }
            if (map->occupancy[ny][nx] == OCC_WALL || map->occupancy[ny][nx] == OCC_BOX) continue;
            if (nx == gx && ny == gy) return 1;
            visited[ny][nx] = 1;
            qx[tail] = nx; qy[tail] = ny; tail++;
            if (tail >= FINE_COLS * FINE_ROWS) tail = 0;
        }
    }
    return 0;
}

// 判断墙体是否为边界墙（运动坐标）
int is_boundary_wall(Wall* w) {
    const float FIELD_WIDTH = 3.2f;
    const float FIELD_HEIGHT = 2.4f;
    const float EPS = 0.01f;
    if (fabsf(w->x1) < EPS || fabsf(w->x2 - FIELD_WIDTH) < EPS ||
        fabsf(w->y1) < EPS || fabsf(w->y2 - FIELD_HEIGHT) < EPS) {
        return 1;
    }
    return 0;
}

// 模拟炸墙（检查推箱子可行性），小车起点为图像细网格坐标 (start_x, start_y)
int simulate_wall_destruction(GameState* state, GridMap* grid_map,
                              int wall_idx,
                              int start_x, int start_y,
                              int box_id,
                              int* out_push_steps) {
    #if DEBUG
    printf("  模拟炸墙 %d: ", wall_idx);
    Wall* w = &state->walls[wall_idx];
    printf("墙体 (%.2f,%.2f)-(%.2f,%.2f)\n", w->x1, w->y1, w->x2, w->y2);
    #endif

    GameState temp_state = *state;
    Wall temp_walls[MAX_WALLS];
    memcpy(temp_walls, state->walls, state->num_walls * sizeof(Wall));

    int kept = 0;
    for (int i = 0; i < temp_state.num_walls; i++) {
        if (i != wall_idx) {
            temp_walls[kept++] = temp_walls[i];
        }
    }
    temp_state.num_walls = kept;
    memcpy(temp_state.walls, temp_walls, kept * sizeof(Wall));

    GridMap temp_map = *grid_map;
    refresh_grid_map(&temp_state, &temp_map);

    // 小车起点（图像坐标）
    float car_img_x = start_x * RESOLUTION;
    float car_img_y = start_y * RESOLUTION;
    int actions[200];
    int len = light_sokoban_plan(&temp_state, &temp_map, box_id, car_img_x, car_img_y, actions, 200);
    if (len > 0) {
        #if DEBUG
        printf("    推箱子成功，步数 %d\n", len);
        #endif
        if (out_push_steps) *out_push_steps = len;
        return 1;
    } else {
        #if DEBUG
        printf("    推箱子失败\n");
        #endif
        return 0;
    }
}

// 选择最佳墙体，传入小车图像细网格坐标 (start_x, start_y)
int select_best_wall_to_destroy(GameState* state, GridMap* grid_map,
                                int start_x, int start_y,   // 图像细网格坐标
                                int box_id,
                                float* out_bomb_target_x, float* out_bomb_target_y) {
    //printf("\n========== select_best_wall_to_destroy ==========\n");
    //printf("小车细网格(图像): (%d, %d)\n", start_x, start_y);
    
    // 小车图像粗网格索引
    int car_start_r = start_y / 4;
    int car_start_c = start_x / 4;
    //printf("小车粗网格(图像): (%d, %d)\n", car_start_c, car_start_r);

    int best_wall = -1;
    int best_steps = 1e9;
    int wall_count = 0;

    // 寻找可用炸弹
    int bomb_id = -1;
    for (int b = 0; b < state->num_bombs; b++) {
        if (state->bombs[b].active) {
            bomb_id = b;
            break;
        }
    }
    if (bomb_id < 0) {
        //printf("没有可用炸弹\n");
        return -1;
    }
    Bomb* bomb = &state->bombs[bomb_id];
    
    // 打印炸弹原始运动坐标
    //printf("炸弹运动坐标: (%.3f, %.3f)\n", bomb->x, bomb->y);
    float bomb_img_x, bomb_img_y;
    motion_to_image(bomb->x, bomb->y, &bomb_img_x, &bomb_img_y);
    int bomb_r = (int)(bomb_img_y / CELL_SIZE);
    int bomb_c = (int)(bomb_img_x / CELL_SIZE);
    //printf("炸弹图像坐标: (%.3f, %.3f) -> 粗网格(%d, %d)\n", bomb_img_x, bomb_img_y, bomb_c, bomb_r);

    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};

    for (int w = 0; w < state->num_walls; w++) {
        if (is_boundary_wall(&state->walls[w])) continue;
        wall_count++;

        Wall* wall = &state->walls[w];
        // 墙体运动坐标 -> 图像坐标
        float wall_img_x, wall_img_y;
        motion_to_image(wall->x1, wall->y1, &wall_img_x, &wall_img_y);
        int wall_r = (int)(wall_img_y / CELL_SIZE + 0.5f);
        int wall_c = (int)(wall_img_x / CELL_SIZE + 0.5f);
        //printf("\n墙体%d: 运动(%.2f,%.2f) -> 图像(%.2f,%.2f) -> 粗网格(%d,%d)\n",
        //       w, wall->x1, wall->y1, wall_img_x, wall_img_y, wall_c, wall_r);

        for (int d = 0; d < 4; d++) {
            int nr = wall_r + dr[d];
            int nc = wall_c + dc[d];
            if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) {
                //printf("  方向%d: 粗网格(%d,%d) 超出边界\n", d, nc, nr);
                continue;
            }

            // 检查该粗网格是否空闲
            int base_x = nc * 4;
            int base_y = nr * 4;
            int occupied = 0;
            for (int dy = 0; dy < 4 && !occupied; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    uint8_t occ = grid_map->occupancy[base_y + dy][base_x + dx];
                    if (occ == OCC_WALL || occ == OCC_BOX) {
                        occupied = 1;
                        break;
                    }
                }
            }
            if (occupied) {
                //printf("  方向%d: 粗网格(%d,%d) 被占据\n", d, nc, nr);
                continue;
            }

            // 炸弹目标点（图像坐标粗网格中心）
            float bomb_target_img_x = (nc + 0.5f) * CELL_SIZE;
            float bomb_target_img_y = (nr + 0.5f) * CELL_SIZE;
            float bomb_target_mx, bomb_target_my;
            image_to_motion(bomb_target_img_x, bomb_target_img_y, &bomb_target_mx, &bomb_target_my);
            //printf("  方向%d: 目标粗网格(%d,%d) 图像中心(%.2f,%.2f) 运动(%.2f,%.2f)\n",
            //       d, nc, nr, bomb_target_img_x, bomb_target_img_y, bomb_target_mx, bomb_target_my);

            // 检查炸弹能否推动到目标点
            int actions[100];
            int bomb_ok = light_push_plan(grid_map,
                                          bomb_r, bomb_c,   // 炸弹起始粗网格（图像坐标）
                                          nr, nc,           // 目标粗网格（图像坐标）
                                          car_start_r, car_start_c,
                                          actions, 100);
            if (bomb_ok <= 0) {
                //printf("  炸弹无法到达目标\n");
                continue;
            }

            // 模拟炸墙后推箱子
            int steps;
            if (simulate_wall_destruction(state, grid_map, w, start_x, start_y, box_id, &steps)) {
                //printf("  墙体%d可行，推箱子步数=%d\n", w, steps);
                if (steps < best_steps) {
                    best_steps = steps;
                    best_wall = w;
                    *out_bomb_target_x = bomb_target_mx;
                    *out_bomb_target_y = bomb_target_my;
                }
            } else {
                //printf("  炸墙后推箱子失败\n");
            }
        }
    }
    //printf("检查了%d个内部墙体，最佳墙体:%d (步数:%d)\n", wall_count, best_wall, best_steps);
    return best_wall;
}

// 炸弹动作转路径（输入动作已转换为运动坐标系，小车起始点为图像坐标）
int bomb_actions_to_world_path(GameState* state, GridMap* grid_map, int bomb_id,
                                      float start_car_x, float start_car_y,
                                      const int* actions, int action_count,
                                      float* out_x, float* out_y, int max_len) {
    // 炸弹运动坐标 → 图像坐标
    Bomb* bomb = &state->bombs[bomb_id];
    float bomb_img_x, bomb_img_y;
    motion_to_image(bomb->x, bomb->y, &bomb_img_x, &bomb_img_y);
    int br = (int)(bomb_img_y / CELL_SIZE);
    int bc = (int)(bomb_img_x / CELL_SIZE);
    // 小车起始点（图像坐标）→ 粗网格索引
    int pr = (int)(start_car_y / CELL_SIZE);
    int pc = (int)(start_car_x / CELL_SIZE);
    const int dr4[4] = {-1, 0, 1, 0};
    const int dc4[4] = {0, 1, 0, -1};

    static float points_x[MAX_ACT_POINTS];
    static float points_y[MAX_ACT_POINTS];
    int point_count = 0;

    #define ADD_POINT(x, y) do { \
        if (point_count < MAX_ACT_POINTS && (point_count == 0 || \
            fabsf((x) - points_x[point_count-1]) > 1e-4f || \
            fabsf((y) - points_y[point_count-1]) > 1e-4f)) { \
            points_x[point_count] = (x); \
            points_y[point_count] = (y); \
            point_count++; \
        } \
    } while(0)

    // 小车起点转换为运动坐标并添加
    float start_mx, start_my;
    image_to_motion(start_car_x, start_car_y, &start_mx, &start_my);
    ADD_POINT(start_mx, start_my);

    AStarParams params = {5000, 2.0f};
    for (int i = 0; i < action_count; i++) {
        int act = actions[i];
        if (act >= 4) {  // 推动动作
            int d = act - 4;
            int push_r = br - dr4[d];
            int push_c = bc - dc4[d];

            // 小车移动到推动位置的路径（细网格坐标）
            int sx = pc * 4 + 2;
            int sy = pr * 4 + 2;
            int gx = push_c * 4 + 2;
            int gy = push_r * 4 + 2;

            int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
            int plen = astar_plan_path(grid_map, sx, sy, gx, gy, path_x, path_y, MAX_PATH_POINTS, &params);
            if (plen > 0) {
                for (int k = 0; k < plen; k++) {
                    float wx, wy;
                    grid_to_world(path_x[k], path_y[k], &wx, &wy);
                    float mx, my;
                    image_to_motion(wx, wy, &mx, &my);
                    ADD_POINT(mx, my);
                }
            } else {
                // A*失败，直接添加推动位置的运动坐标
                float push_mx = (push_r + 0.5f) * CELL_SIZE;
                float push_my = (push_c + 0.5f) * CELL_SIZE;
                ADD_POINT(push_mx, push_my);
            }

            // 推动后，炸弹移动，小车进入炸弹原位置
            int old_br = br, old_bc = bc;
            br += dr4[d];
            bc += dc4[d];
            pr = old_br;
            pc = old_bc;
            float new_car_mx = (pr + 0.5f) * CELL_SIZE;
            float new_car_my = (pc + 0.5f) * CELL_SIZE;
            ADD_POINT(new_car_mx, new_car_my);
        } else {  // 移动动作（理论上不会出现，但保留）
            int d = act;
            int target_r = br;
            int target_c = bc;
            int sx = pc * 4 + 2;
            int sy = pr * 4 + 2;
            int gx = target_c * 4 + 2;
            int gy = target_r * 4 + 2;

            int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
            int plen = astar_plan_path(grid_map, sx, sy, gx, gy, path_x, path_y, MAX_PATH_POINTS, &params);
            if (plen > 0) {
                for (int k = 0; k < plen; k++) {
                    float wx, wy;
                    grid_to_world(path_x[k], path_y[k], &wx, &wy);
                    float mx, my;
                    image_to_motion(wx, wy, &mx, &my);
                    ADD_POINT(mx, my);
                }
            } else {
                float target_mx = (target_r + 0.5f) * CELL_SIZE;
                float target_my = (target_c + 0.5f) * CELL_SIZE;
                ADD_POINT(target_mx, target_my);
            }
            pr = target_r;
            pc = target_c;
            float new_car_mx = (pr + 0.5f) * CELL_SIZE;
            float new_car_my = (pc + 0.5f) * CELL_SIZE;
            ADD_POINT(new_car_mx, new_car_my);
        }
    }
    #undef ADD_POINT

    int n = (point_count < max_len) ? point_count : max_len;
    for (int i = 0; i < n; i++) {
        out_x[i] = points_x[i];
        out_y[i] = points_y[i];
    }
    return n;
}

// 炸弹推动规划（已废弃，建议使用 HybridController_PlanBomb）
int plan_bomb_to_target(GameState* state, GridMap* grid_map, int bomb_id,
                        float car_x, float car_y,
                        float target_x, float target_y,
                        float* out_path_x, float* out_path_y, int max_path_len) {
    // 将输入的运动坐标转换为图像坐标
    float car_img_x, car_img_y, target_img_x, target_img_y;
    motion_to_image(car_x, car_y, &car_img_x, &car_img_y);
    motion_to_image(target_x, target_y, &target_img_x, &target_img_y);

    Bomb* bomb = &state->bombs[bomb_id];
    float bomb_img_x, bomb_img_y;
    motion_to_image(bomb->x, bomb->y, &bomb_img_x, &bomb_img_y);
    int bomb_start_r = (int)(bomb_img_y / CELL_SIZE);
    int bomb_start_c = (int)(bomb_img_x / CELL_SIZE);
    int bomb_target_r = (int)(target_img_y / CELL_SIZE);
    int bomb_target_c = (int)(target_img_x / CELL_SIZE);
    int car_start_r = (int)(car_img_y / CELL_SIZE);
    int car_start_c = (int)(car_img_x / CELL_SIZE);

    int actions[1000];
    int action_count = light_push_plan(grid_map, bomb_start_r, bomb_start_c, bomb_target_r, bomb_target_c,
                                       car_start_r, car_start_c, actions, 1000);
    if (action_count <= 0) return -1;

    // 动作转换为运动坐标系
    for (int i = 0; i < action_count; i++) actions[i] += 4;

    return bomb_actions_to_world_path(state, grid_map, bomb_id, car_img_x, car_img_y,
                                   actions, action_count,
                                   out_path_x, out_path_y, max_path_len);
}

// ================= 多炸弹并发规划函数（以下函数保持原有，确保坐标统一） =================

// 计算炸弹优先级（基于运动坐标距离）
static int calculate_bomb_priority(GameState* state, int bomb_id, float player_x, float player_y) {
    Bomb* bomb = &state->bombs[bomb_id];
    float dx = bomb->x - player_x;
    float dy = bomb->y - player_y;
    float dist = sqrtf(dx*dx + dy*dy);
    return (int)(dist * 10.0f);
}

// 初始化时间占用位图
static void init_time_occupancy() {
    memset(time_occupancy, 0, sizeof(time_occupancy));
}

// 检查并标记时间槽占用
static int check_and_mark_occupancy(int r, int c, int start_time, int duration) {
    for (int t = start_time; t < start_time + duration && t < MAX_TIME_SLOTS; t++) {
        if (time_occupancy[r][c][t]) {
            return 0;
        }
    }
    for (int t = start_time; t < start_time + duration && t < MAX_TIME_SLOTS; t++) {
        time_occupancy[r][c][t] = 1;
    }
    return 1;
}

// 模拟炸弹路径并检查冲突
static int simulate_bomb_path_with_conflict(GameState* state, GridMap* grid_map, BombPlan* plan,
                                           int start_time, int* conflict_time) {
    Bomb* bomb = &state->bombs[plan->bomb_id];
    // 将炸弹运动坐标转为图像粗网格
    float bomb_img_x, bomb_img_y;
    motion_to_image(bomb->x, bomb->y, &bomb_img_x, &bomb_img_y);
    int current_r = (int)(bomb_img_y / CELL_SIZE);
    int current_c = (int)(bomb_img_x / CELL_SIZE);
    int time = start_time;

    for (int i = 0; i < plan->path_len; i++) {
        int action = plan->path_actions[i];
        int dr = 0, dc = 0;
        if (action >= 4) {
            int d = action - 4;
            const int dr4[4] = {-1,0,1,0}, dc4[4] = {0,1,0,-1};
            dr = dr4[d];
            dc = dc4[d];
        } else {
            int d = action;
            const int dr4[4] = {-1,0,1,0}, dc4[4] = {0,1,0,-1};
            dr = dr4[d];
            dc = dc4[d];
        }
        int next_r = current_r + dr;
        int next_c = current_c + dc;
        if (next_r < 0 || next_r >= MAP_ROWS || next_c < 0 || next_c >= MAP_COLS) {
            *conflict_time = time;
            return 0;
        }
        if (!check_and_mark_occupancy(next_r, next_c, time, 1)) {
            *conflict_time = time;
            return 0;
        }
        current_r = next_r;
        current_c = next_c;
        time++;
    }
    return 1;
}

// 回退策略：降低路径复杂度
static int reduce_path_complexity(BombPlan* plan) {
    if (plan->path_len > 2) {
        plan->path_len -= 1;
        return 1;
    }
    return 0;
}

// 多炸弹并发规划主函数，暂时用不到
int plan_multiple_bombs(GameState* state, GridMap* grid_map, float player_x, float player_y,
                        BombPlan* plans, int max_plans) {
    #if DEBUG
    printf("开始多炸弹并发规划，可用炸弹数量: %d\n", state->num_bombs);
    #endif

    init_time_occupancy();
    int plan_count = 0;

    // 收集所有活跃炸弹
    int active_bombs[MAX_BOMB_PLANS];
    int active_count = 0;
    for (int i = 0; i < state->num_bombs && active_count < MAX_BOMB_PLANS; i++) {
        if (state->bombs[i].active) {
            active_bombs[active_count++] = i;
        }
    }
    if (active_count == 0) return 0;

    // 计算优先级并排序
    typedef struct {
        int bomb_id;
        int priority;
    } BombPriority;
    BombPriority priorities[MAX_BOMB_PLANS];
    for (int i = 0; i < active_count; i++) {
        priorities[i].bomb_id = active_bombs[i];
        priorities[i].priority = calculate_bomb_priority(state, active_bombs[i], player_x, player_y);
    }
    for (int i = 0; i < active_count - 1; i++) {
        for (int j = 0; j < active_count - i - 1; j++) {
            if (priorities[j].priority > priorities[j+1].priority) {
                BombPriority temp = priorities[j];
                priorities[j] = priorities[j+1];
                priorities[j+1] = temp;
            }
        }
    }

    // 收集未完成箱子（运动坐标）
    int unfinished_boxes[MAX_BOXES];
    int box_count = 0;
    for (int i = 0; i < state->num_boxes; i++) {
        if (state->boxes[i].state == 0) {
            unfinished_boxes[box_count++] = i;
        }
    }

    // 评估所有候选墙体（需要小车图像粗网格）
    WallCandidate all_candidates[MAX_WALLS];
    int total_candidates = 0;
    // 将玩家运动坐标转为图像粗网格
    float player_img_x, player_img_y;
    motion_to_image(player_x, player_y, &player_img_x, &player_img_y);
    int car_start_r = (int)(player_img_y / CELL_SIZE);
    int car_start_c = (int)(player_img_x / CELL_SIZE);

    for (int i = 0; i < box_count && total_candidates < MAX_WALLS; i++) {
        int box_id = unfinished_boxes[i];
        int added = evaluate_walls_for_box(state, grid_map, box_id,
                                           car_start_r, car_start_c,
                                           all_candidates + total_candidates,
                                           MAX_WALLS - total_candidates);
        total_candidates += added;
    }

    // 按推箱子步数排序
    for (int i = 0; i < total_candidates - 1; i++) {
        for (int j = i + 1; j < total_candidates; j++) {
            if (all_candidates[j].push_steps < all_candidates[i].push_steps) {
                WallCandidate tmp = all_candidates[i];
                all_candidates[i] = all_candidates[j];
                all_candidates[j] = tmp;
            }
        }
    }

    uint8_t used_candidate[MAX_WALLS] = {0};
    int current_time = 0;
    for (int i = 0; i < active_count && plan_count < max_plans; i++) {
        int bomb_id = priorities[i].bomb_id;
        Bomb* bomb = &state->bombs[bomb_id];

        float target_x, target_y;
        int selected_candidate = -1;
        for (int j = 0; j < total_candidates; j++) {
            if (!used_candidate[j]) {
                selected_candidate = j;
                used_candidate[j] = 1;
                break;
            }
        }
        if (selected_candidate >= 0) {
            target_x = all_candidates[selected_candidate].bomb_target_x;
            target_y = all_candidates[selected_candidate].bomb_target_y;
        } else {
            target_x = player_x + (i + 1) * 0.5f;
            target_y = player_y + (i + 1) * 0.5f;
        }

        // 规划炸弹路径（使用运动坐标，内部会转换）
        int actions[MAX_PATH_LENGTH];
        int action_count = plan_bomb_to_target(state, grid_map, bomb_id,
                                               player_x, player_y,
                                               target_x, target_y,
                                               NULL, NULL, 0);
        if (action_count <= 0) continue;

        // 填充计划（使用原始动作，不转换）
        // 注意：plan_bomb_to_target 返回的是路径点，不是动作，此处需要修改。
        // 为简化，此处仅示意，实际应重新设计接口。
        // 由于多炸弹规划尚未使用，暂时保留原样，但需确保动作和坐标正确。
        plans[plan_count].bomb_id = bomb_id;
        plans[plan_count].priority = priorities[i].priority;
        plans[plan_count].path_len = action_count;
        plans[plan_count].target_x = target_x;
        plans[plan_count].target_y = target_y;
        plans[plan_count].start_time = current_time;
        // 此处需要存储实际的动作，但 plan_bomb_to_target 未返回动作，故需调整
        // 暂时跳过
        current_time += action_count;
        plan_count++;
    }
    return plan_count;
}