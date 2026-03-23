#include "planner.h"
#include <math.h>
#include <string.h>
#include <rtthread.h>
#include "zf_device_wireless_uart.h"

#define DEBUG_PRINT(...)  { char buf[128]; rt_sprintf(buf, __VA_ARGS__); wireless_uart_send_string(buf); }

// ================= 多炸弹并发规划数据结构 =================

#define MAX_BOMB_PLANS 4
#define MAX_PATH_LENGTH MAX_BOMB_PATH_LENGTH

// 时间扩展状态空间 - 位图标记占用
static uint8_t time_occupancy[MAP_ROWS][MAP_COLS][MAX_TIME_SLOTS];  // [行][列][时间槽]

// 候选墙体结构（用于目标选择）
typedef struct {
    int wall_idx;
    float bomb_target_x, bomb_target_y;
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
        int wall_r = (int)(wall->y1 / CELL_SIZE + 0.5f);
        int wall_c = (int)(wall->x1 / CELL_SIZE + 0.5f);

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

            float bomb_target_x = (nc + 0.5f) * CELL_SIZE;
            float bomb_target_y = (nr + 0.5f) * CELL_SIZE;

            // 模拟炸墙后推箱子
            int steps;
            if (simulate_wall_destruction(state, grid_map, w,
                                          car_start_r * 4 + 2, car_start_c * 4 + 2, box_id, &steps)) {
                candidates[count].wall_idx = w;
                candidates[count].bomb_target_x = bomb_target_x;
                candidates[count].bomb_target_y = bomb_target_y;
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

// 细网格8方向BFS，检查起点(sx,sy)到终点(gx,gy)是否可达
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

// 通用轻量级推箱子规划器（用于炸弹）
static int light_push_plan(GridMap* map, int start_r, int start_c, int goal_r, int goal_c,
                           int car_start_r, int car_start_c, int* out_actions, int max_actions) {
    #if DEBUG
    printf("      light_push_plan: 炸弹从(%d,%d)到(%d,%d), 车从(%d,%d)\n",
           start_r, start_c, goal_r, goal_c, car_start_r, car_start_c);
    #endif

    static uint8_t obstacle[MAP_ROWS][MAP_COLS];
    memset(obstacle, 0, sizeof(obstacle));
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            int base_x = c * 4;
            int base_y = r * 4;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    if (map->occupancy[base_y + dy][base_x + dx] == OCC_WALL ||
                        map->occupancy[base_y + dy][base_x + dx] == OCC_BOX) {
                        obstacle[r][c] = 1;
                        break;
                    }
                }
                if (obstacle[r][c]) break;
            }
        }
    }

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

    while (head < tail) {
        PushNode cur = queue[head];
        if (cur.br == goal_r && cur.bc == goal_c) {
            found_idx = head;
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

            int sx = cur.pc * 4 + 2;
            int sy = cur.pr * 4 + 2;
            int gx = push_c * 4 + 2;
            int gy = push_r * 4 + 2;
            if (!can_reach_fine_exclude_bomb(map, sx, sy, gx, gy, cur.br, cur.bc))
                continue;

            int new_pushes = cur.pushes + 1;
            if (best_pushes[nbr][nbc] == -1 || new_pushes < best_pushes[nbr][nbc]) {
                best_pushes[nbr][nbc] = new_pushes;
                if (tail >= 200) continue;
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

    if (found_idx == -1) return -1;

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
    #if DEBUG
    printf("light_push_plan 找到目标，路径长度: %d\n", n);
    printf("动作序列: ");
    for (int i = 0; i < n; i++) printf("%d ", out_actions[i]);
    printf("\n");
    #endif
    return n;
}

// 细网格8方向BFS，检查起点(sx,sy)到终点(gx,gy)是否可达，排除炸弹所在粗网格
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

// 判断墙体是否为边界墙
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

// 模拟炸墙（检查推箱子可行性）
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

    int actions[200];
    float car_x = start_x * RESOLUTION;
    float car_y = start_y * RESOLUTION;
    int len = light_sokoban_plan(&temp_state, &temp_map, box_id, car_x, car_y, actions, 200);
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

// 选择最佳墙体（原函数，用于单炸弹场景）
int select_best_wall_to_destroy(GameState* state, GridMap* grid_map,
                                int start_x, int start_y,
                                int box_id,
                                float* out_bomb_target_x, float* out_bomb_target_y) {
    #if DEBUG
    printf("开始选择最佳墙体，起点:(%d,%d), 箱子:%d\n", start_x, start_y, box_id);
    #endif
    int best_wall = -1;
    int best_steps = 1e9;
    int wall_count = 0;

    int bomb_id = -1;
    for (int b = 0; b < state->num_bombs; b++) {
        if (state->bombs[b].active) {
            bomb_id = b;
            break;
        }
    }
    if (bomb_id < 0) {
        #if DEBUG
        printf("没有可用炸弹\n");
        #endif
        return -1;
    }

    float car_wx = start_x * RESOLUTION;
    float car_wy = start_y * RESOLUTION;

    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};

    for (int w = 0; w < state->num_walls; w++) {
        if (is_boundary_wall(&state->walls[w])) continue;
        wall_count++;

        Wall* wall = &state->walls[w];
        #if DEBUG
        printf("  墙体%d: 位置 (%.2f,%.2f)-(%.2f,%.2f)\n", w, wall->x1, wall->y1, wall->x2, wall->y2);
        #endif

        int wall_r = (int)(wall->y1 / CELL_SIZE + 0.5f);
        int wall_c = (int)(wall->x1 / CELL_SIZE + 0.5f);

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

            float bomb_target_x = (nc + 0.5f) * CELL_SIZE;
            float bomb_target_y = (nr + 0.5f) * CELL_SIZE;
            #if DEBUG
            printf("    尝试炸弹目标 (%.2f,%.2f) 对于墙体%d\n", bomb_target_x, bomb_target_y, w);
            #endif

            int actions[100];
            Bomb* bomb = &state->bombs[bomb_id];
            int bomb_start_r = (int)(bomb->y / CELL_SIZE);
            int bomb_start_c = (int)(bomb->x / CELL_SIZE);
            int bomb_target_r = nr;
            int bomb_target_c = nc;
            int car_start_r = (int)(car_wy / CELL_SIZE);
            int car_start_c = (int)(car_wx / CELL_SIZE);

            #if DEBUG
            printf("      炸弹位置: 粗(%d,%d), 车位置: 粗(%d,%d)\n",
                   bomb_start_r, bomb_start_c, car_start_r, car_start_c);
            #endif

            int bomb_ok = light_push_plan(grid_map, bomb_start_r, bomb_start_c, bomb_target_r, bomb_target_c,
                                          car_start_r, car_start_c, actions, 100);
            if (bomb_ok <= 0) {
                #if DEBUG
                printf("      炸弹无法到达\n");
                #endif
                continue;
            }

            int steps;
            if (simulate_wall_destruction(state, grid_map, w, start_x, start_y, box_id, &steps)) {
                #if DEBUG
                printf("      炸弹可达，推箱子步骤:%d\n", steps);
                #endif
                if (steps < best_steps) {
                    best_steps = steps;
                    best_wall = w;
                    *out_bomb_target_x = bomb_target_x;
                    *out_bomb_target_y = bomb_target_y;
                }
            } else {
                #if DEBUG
                printf("      推箱子失败\n");
                #endif
            }
        }
    }
    #if DEBUG
    printf("检查了%d个内部墙体，最佳墙体:%d (步骤:%d)\n", wall_count, best_wall, best_steps);
    #endif
    return best_wall;
}

// 炸弹动作转路径
static int bomb_actions_to_world_path(GameState* state, GridMap* grid_map, int bomb_id,
                                      float start_car_x, float start_car_y,
                                      const int* actions, int action_count,
                                      float* out_x, float* out_y, int max_len) {
    Bomb* bomb = &state->bombs[bomb_id];
    int pr = (int)(start_car_y / CELL_SIZE);
    int pc = (int)(start_car_x / CELL_SIZE);
    int br = (int)(bomb->y / CELL_SIZE);
    int bc = (int)(bomb->x / CELL_SIZE);
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

    ADD_POINT(start_car_x, start_car_y);

    AStarParams params = {5000, 2.0f};
    for (int i = 0; i < action_count; i++) {
        int act = actions[i];
        if (act >= 4) {  // 推动动作
            int d = act - 4;
            int push_r = br - dr4[d];
            int push_c = bc - dc4[d];

            // 小车移动到推动位置的路径
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
                    ADD_POINT(wx, wy);
                }
            } else {
                // A*失败，直接添加推动位置中心
                ADD_POINT((push_c + 0.5f) * CELL_SIZE, (push_r + 0.5f) * CELL_SIZE);
            }

            // 推动后，炸弹移动，小车进入炸弹原位置
            int old_br = br, old_bc = bc;
            br += dr4[d];
            bc += dc4[d];
            pr = old_br;
            pc = old_bc;
            ADD_POINT((pc + 0.5f) * CELL_SIZE, (pr + 0.5f) * CELL_SIZE);
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
                    ADD_POINT(wx, wy);
                }
            } else {
                ADD_POINT((target_c + 0.5f) * CELL_SIZE, (target_r + 0.5f) * CELL_SIZE);
            }
            pr = target_r;
            pc = target_c;
            ADD_POINT((pc + 0.5f) * CELL_SIZE, (pr + 0.5f) * CELL_SIZE);
        }
    }
    #undef ADD_POINT

        int n = (point_count < max_len) ? point_count : max_len;
    for (int i = 0; i < n; i++) {
        float mx, my;
        img_to_motion(points_x[i], points_y[i], &mx, &my);
        out_x[i] = mx;
        out_y[i] = my;
    }
    return n;
}

// 炸弹推动规划
int plan_bomb_to_target(GameState* state, GridMap* grid_map, int bomb_id,
                        float car_x, float car_y,
                        float target_x, float target_y,
                        float* out_path_x, float* out_path_y, int max_path_len) {
    Bomb* bomb = &state->bombs[bomb_id];
    int bomb_start_r = (int)(bomb->y / CELL_SIZE);
    int bomb_start_c = (int)(bomb->x / CELL_SIZE);
    int bomb_target_r = (int)(target_y / CELL_SIZE);
    int bomb_target_c = (int)(target_x / CELL_SIZE);
    int car_start_r = (int)(car_y / CELL_SIZE);
    int car_start_c = (int)(car_x / CELL_SIZE);

    int actions[1000];
    int action_count = light_push_plan(grid_map, bomb_start_r, bomb_start_c, bomb_target_r, bomb_target_c,
                                       car_start_r, car_start_c, actions, 1000);
    #if DEBUG
    printf("light_push_plan 返回动作数: %d\n", action_count);
    for (int i = 0; i < action_count; i++) printf("动作 %d: %d\n", i, actions[i]);
    #endif
    if (action_count <= 0) return -1;

    for (int i = 0; i < action_count; i++) actions[i] += 4;

    return bomb_actions_to_world_path(state, grid_map, bomb_id, car_x, car_y,
                                   actions, action_count,
                                   out_path_x, out_path_y, max_path_len);
}

// ================= 多炸弹并发规划函数 =================

// 计算炸弹优先级（基于到玩家的距离）
static int calculate_bomb_priority(GameState* state, int bomb_id, float player_x, float player_y) {
    Bomb* bomb = &state->bombs[bomb_id];
    float dx = bomb->x - player_x;
    float dy = bomb->y - player_y;
    float dist = sqrtf(dx*dx + dy*dy);
    // 距离越近优先级越高（数值越小）
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
            return 0;  // 冲突
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
    int current_r = (int)(bomb->y / CELL_SIZE);
    int current_c = (int)(bomb->x / CELL_SIZE);
    int time = start_time;

    // 假设每个动作占用1个时间单位（可后续优化）
    for (int i = 0; i < plan->path_len; i++) {
        int action = plan->path_actions[i];
        int dr = 0, dc = 0;

        if (action >= 4) {  // 推动动作
            int d = action - 4;
            const int dr4[4] = {-1,0,1,0}, dc4[4] = {0,1,0,-1};
            dr = dr4[d];
            dc = dc4[d];
        } else {  // 移动动作
            int d = action;
            const int dr4[4] = {-1,0,1,0}, dc4[4] = {0,1,0,-1};
            dr = dr4[d];
            dc = dc4[d];
        }

        int next_r = current_r + dr;
        int next_c = current_c + dc;

        if (next_r < 0 || next_r >= MAP_ROWS || next_c < 0 || next_c >= MAP_COLS) {
            *conflict_time = time;
            return 0;  // 超出边界
        }

        // 检查空间冲突
        if (!check_and_mark_occupancy(next_r, next_c, time, 1)) {
            *conflict_time = time;
            return 0;  // 时间空间冲突
        }

        current_r = next_r;
        current_c = next_c;
        time++;
    }

    return 1;  // 无冲突
}

// 回退策略：降低路径复杂度
static int reduce_path_complexity(BombPlan* plan) {
    // 简化：移除一些中间动作（这里只是示例，实际可以优化路径）
    if (plan->path_len > 2) {
        plan->path_len -= 1;  // 减少一个动作
        return 1;
    }
    return 0;  // 无法进一步简化
}

// ========== 新增：评估墙体对箱子的帮助程度 ==========

// 多炸弹并发规划主函数（带目标选择智能化）
int plan_multiple_bombs(GameState* state, GridMap* grid_map, float player_x, float player_y,
                        BombPlan* plans, int max_plans) {
    #if DEBUG
    printf("开始多炸弹并发规划，可用炸弹数量: %d\n", state->num_bombs);
    #endif

    // 初始化
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

    if (active_count == 0) {
        #if DEBUG
        printf("没有活跃炸弹\n");
        #endif
        return 0;
    }

    #if DEBUG
    printf("活跃炸弹数量: %d\n", active_count);
    for (int i = 0; i < active_count; i++) {
        int bomb_id = active_bombs[i];
        printf("  炸弹%d: 位置(%.2f,%.2f)\n", bomb_id, state->bombs[bomb_id].x, state->bombs[bomb_id].y);
    }
    #endif

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

    // 简单排序（冒泡排序）
    for (int i = 0; i < active_count - 1; i++) {
        for (int j = 0; j < active_count - i - 1; j++) {
            if (priorities[j].priority > priorities[j+1].priority) {
                BombPriority temp = priorities[j];
                priorities[j] = priorities[j+1];
                priorities[j+1] = temp;
            }
        }
    }

    // ========== 目标选择智能化 ==========
    // 收集所有未完成箱子
    int unfinished_boxes[MAX_BOXES];
    int box_count = 0;
    for (int i = 0; i < state->num_boxes; i++) {
        if (state->boxes[i].state == 0) {
            unfinished_boxes[box_count++] = i;
        }
    }

    // 评估所有候选墙体
    WallCandidate all_candidates[MAX_WALLS];
    int total_candidates = 0;
    int car_start_r = (int)(player_y / CELL_SIZE);
    int car_start_c = (int)(player_x / CELL_SIZE);

    for (int i = 0; i < box_count && total_candidates < MAX_WALLS; i++) {
        int box_id = unfinished_boxes[i];
        int added = evaluate_walls_for_box(state, grid_map, box_id,
                                           car_start_r, car_start_c,
                                           all_candidates + total_candidates,
                                           MAX_WALLS - total_candidates);
        total_candidates += added;
    }

    #if DEBUG
    printf("评估出%d个墙体候选\n", total_candidates);
    #endif

    // 按推箱子步数排序（升序）
    for (int i = 0; i < total_candidates - 1; i++) {
        for (int j = i + 1; j < total_candidates; j++) {
            if (all_candidates[j].push_steps < all_candidates[i].push_steps) {
                WallCandidate tmp = all_candidates[i];
                all_candidates[i] = all_candidates[j];
                all_candidates[j] = tmp;
            }
        }
    }

    // 标记哪些候选已被使用
    uint8_t used_candidate[MAX_WALLS] = {0};
    int candidate_index = 0;

    // 分阶段规划
    int current_time = 0;
    for (int i = 0; i < active_count && plan_count < max_plans; i++) {
        int bomb_id = priorities[i].bomb_id;
        Bomb* bomb = &state->bombs[bomb_id];

        // 选择一个未使用的候选墙体（按步数从小到大）
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
            #if DEBUG
            printf("  为炸弹%d选择候选墙体%d，目标(%.2f,%.2f)，步数%d\n",
                   bomb_id, all_candidates[selected_candidate].wall_idx,
                   target_x, target_y, all_candidates[selected_candidate].push_steps);
            #endif
        } else {
            // 无可用候选，回退到默认目标（玩家附近）
            target_x = player_x + (i + 1) * 0.5f;
            target_y = player_y + (i + 1) * 0.5f;
            #if DEBUG
            printf("  无可用候选墙体，为炸弹%d使用默认目标(%.2f,%.2f)\n", bomb_id, target_x, target_y);
            #endif
        }

        // 规划路径
        int actions[MAX_PATH_LENGTH];
        int bomb_start_r = (int)(bomb->y / CELL_SIZE);
        int bomb_start_c = (int)(bomb->x / CELL_SIZE);
        int bomb_target_r = (int)(target_y / CELL_SIZE);
        int bomb_target_c = (int)(target_x / CELL_SIZE);
        int car_start_r = (int)(player_y / CELL_SIZE);
        int car_start_c = (int)(player_x / CELL_SIZE);

        int action_count = light_push_plan(grid_map, bomb_start_r, bomb_start_c,
                                          bomb_target_r, bomb_target_c,
                                          car_start_r, car_start_c, actions, MAX_PATH_LENGTH);

        if (action_count <= 0) {
            #if DEBUG
            printf("炸弹%d规划失败，跳过\n", bomb_id);
            #endif
            continue;
        }

        // 填充计划
        plans[plan_count].bomb_id = bomb_id;
        plans[plan_count].priority = priorities[i].priority;
        plans[plan_count].path_len = action_count;
        plans[plan_count].target_x = target_x;
        plans[plan_count].target_y = target_y;
        plans[plan_count].start_time = current_time;
        memcpy(plans[plan_count].path_actions, actions, action_count * sizeof(int));

        // 检查冲突
        int conflict_time;
        int success = simulate_bomb_path_with_conflict(state, grid_map, &plans[plan_count],
                                                      current_time, &conflict_time);

        if (!success) {
            #if DEBUG
            printf("炸弹%d在时间%d发生冲突，尝试回退\n", bomb_id, conflict_time);
            #endif
            // 回退策略
            if (reduce_path_complexity(&plans[plan_count])) {
                success = simulate_bomb_path_with_conflict(state, grid_map, &plans[plan_count],
                                                          current_time, &conflict_time);
            }
        }

        if (success) {
            plan_count++;
            current_time += action_count;  // 更新时间
            #if DEBUG
            printf("炸弹%d规划成功，路径长度: %d\n", bomb_id, action_count);
            #endif
        } else {
            #if DEBUG
            printf("炸弹%d规划失败，即使回退也无法解决冲突\n", bomb_id);
            #endif
            // 清除已标记的占用（简化实现）
            init_time_occupancy();
            // 重新规划前面的炸弹（简化：这里不重新规划）
        }
    }

    #if DEBUG
    printf("多炸弹规划完成，成功规划%d个炸弹\n", plan_count);
    #endif
    return plan_count;
}