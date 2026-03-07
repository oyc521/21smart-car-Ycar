#include "planner.h"
#include <math.h>
#include <string.h>
#include <rtthread.h>
#include "zf_device_wireless_uart.h"

#define DEBUG_PRINT(...)  { char buf[128]; rt_sprintf(buf, __VA_ARGS__); wireless_uart_send_string(buf); }

// 细网格8方向BFS，检查起点(sx,sy)到终点(gx,gy)是否可达，排除炸弹所在粗网格
static int can_reach_fine_exclude_bomb(GridMap* map, int sx, int sy, int gx, int gy,
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

// 通用轻量级推箱子规划器（用于炸弹）
static int light_push_plan(GridMap* map, int start_r, int start_c, int goal_r, int goal_c,
                           int car_start_r, int car_start_c, int* out_actions, int max_actions) {
    /*#ifdef DEBUG
    DEBUG_PRINT("      light_push_plan: 炸弹从(%d,%d)到(%d,%d), 车从(%d,%d)\n",
           start_r, start_c, goal_r, goal_c, car_start_r, car_start_c);
    #endif*/

    // 构建粗网格障碍地图：墙体 + 所有箱子
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

    static PushNode queue[200];  // 队列大小根据实际需求调整
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
    /*#ifdef DEBUG
    DEBUG_PRINT("light_push_plan 找到目标，路径长度: %d\n", n);
    DEBUG_PRINT("动作序列: ");
    for (int i = 0; i < n; i++) DEBUG_PRINT("%d ", out_actions[i]);
    DEBUG_PRINT("\n");
    #endif*/
    return n;
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
    /*#ifdef DEBUG
    DEBUG_PRINT("  模拟炸墙 %d: ", wall_idx);
    Wall* w = &state->walls[wall_idx];
    DEBUG_PRINT("墙体 (%.2f,%.2f)-(%.2f,%.2f)\n", w->x1, w->y1, w->x2, w->y2);
    #endif*/

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
        /*#ifdef DEBUG
        DEBUG_PRINT("    推箱子成功，步数 %d\n", len);
        #endif*/
        if (out_push_steps) *out_push_steps = len;
        return 1;
    } else {
       /* #ifdef DEBUG
        DEBUG_PRINT("    推箱子失败\n");
        #endif*/
        return 0;
    }
}

// 选择最佳墙体
int select_best_wall_to_destroy(GameState* state, GridMap* grid_map,
                                int start_x, int start_y,
                                int box_id,
                                float* out_bomb_target_x, float* out_bomb_target_y) {
    /*#ifdef DEBUG
    DEBUG_PRINT("开始选择最佳墙体，起点:(%d,%d), 箱子:%d\n", start_x, start_y, box_id);
    #endif*/
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
        /*#ifdef DEBUG
        DEBUG_PRINT("没有可用炸弹\n");
        #endif*/
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
        /*#ifdef DEBUG
        DEBUG_PRINT("  墙体%d: 位置 (%.2f,%.2f)-(%.2f,%.2f)\n", w, wall->x1, wall->y1, wall->x2, wall->y2);
        #endif*/

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
            /*#ifdef DEBUG
            DEBUG_PRINT("    尝试炸弹目标 (%.2f,%.2f) 对于墙体%d\n", bomb_target_x, bomb_target_y, w);
            #endif*/

            int actions[100];
            Bomb* bomb = &state->bombs[bomb_id];
            int bomb_start_r = (int)(bomb->y / CELL_SIZE);
            int bomb_start_c = (int)(bomb->x / CELL_SIZE);
            int bomb_target_r = nr;
            int bomb_target_c = nc;
            int car_start_r = (int)(car_wy / CELL_SIZE);
            int car_start_c = (int)(car_wx / CELL_SIZE);

            /*#ifdef DEBUG
            DEBUG_PRINT("      炸弹位置: 粗(%d,%d), 车位置: 粗(%d,%d)\n",
                   bomb_start_r, bomb_start_c, car_start_r, car_start_c);
            #endif*/

            int bomb_ok = light_push_plan(grid_map, bomb_start_r, bomb_start_c,
                                          bomb_target_r, bomb_target_c,
                                          car_start_r, car_start_c,
                                          actions, 100);
            if (bomb_ok <= 0) {
                /*#ifdef DEBUG
                DEBUG_PRINT("      炸弹无法到达\n");
                #endif*/
                continue;
            }

            int steps;
            if (simulate_wall_destruction(state, grid_map, w, start_x, start_y, box_id, &steps)) {
               /* #ifdef DEBUG
                DEBUG_PRINT("      炸弹可达，推箱子步骤:%d\n", steps);
                #endif*/
                if (steps < best_steps) {
                    best_steps = steps;
                    best_wall = w;
                    *out_bomb_target_x = bomb_target_x;
                    *out_bomb_target_y = bomb_target_y;
                }
            } else {
                /*#ifdef DEBUG
                DEBUG_PRINT("      推箱子失败\n");
                #endif*/
            }
        }
    }
    /*#ifdef DEBUG
    DEBUG_PRINT("检查了%d个内部墙体，最佳墙体:%d (步骤:%d)\n", wall_count, best_wall, best_steps);
    #endif*/
    return best_wall;
}

// 炸弹动作转路径
static int bomb_actions_to_world_path(GameState* state, int bomb_id,
                                      float start_car_x, float start_car_y,
                                      const int* actions, int action_count,
                                      float* out_x, float* out_y, int max_len) {
    Bomb* bomb = &state->bombs[bomb_id];
    int pr = (int)(start_car_y / CELL_SIZE), pc = (int)(start_car_x / CELL_SIZE);
    int br = (int)(bomb->y / CELL_SIZE), bc = (int)(bomb->x / CELL_SIZE);
    const int dr4[4] = {-1,0,1,0}, dc4[4] = {0,1,0,-1};
    static float points_x[MAX_ACT_POINTS];
    static float points_y[MAX_ACT_POINTS];
    int point_count = 0;
    #define ADD_POINT(x, y) do { \
        if (point_count < MAX_ACT_POINTS && \
            (point_count == 0 || fabsf((x)-points_x[point_count-1])>1e-2f || fabsf((y)-points_y[point_count-1])>1e-2f)) { \
            points_x[point_count] = (x); points_y[point_count] = (y); point_count++; \
        } \
    } while(0)

    ADD_POINT(start_car_x, start_car_y);
    for (int i = 0; i < action_count; i++) {
        int act = actions[i];
        if (act >= 4) {
            int d = act - 4;
            int old_br = br, old_bc = bc;
            ADD_POINT((old_bc+0.5f)*CELL_SIZE, (old_br+0.5f)*CELL_SIZE);
            br += dr4[d]; bc += dc4[d];
            pr = old_br; pc = old_bc;
            ADD_POINT((bc+0.5f)*CELL_SIZE, (br+0.5f)*CELL_SIZE);
        } else {
            int d = act;
            pr += dr4[d]; pc += dc4[d];
            ADD_POINT((pc+0.5f)*CELL_SIZE, (pr+0.5f)*CELL_SIZE);
        }
    }
    #undef ADD_POINT

    static float comp_x[MAX_ACT_POINTS];
    static float comp_y[MAX_ACT_POINTS];
    int comp_count = 0;
    if (point_count > 0) { comp_x[0]=points_x[0]; comp_y[0]=points_y[0]; comp_count=1; }
    const float EPS = 1e-4f;
    for (int i = 1; i < point_count-1; i++) {
        float ax=points_x[i-1], ay=points_y[i-1], bx=points_x[i], by=points_y[i], cx=points_x[i+1], cy=points_y[i+1];
        if (fabsf((bx-ax)*(cy-by) - (by-ay)*(cx-bx)) > EPS) {
            if (comp_count < MAX_ACT_POINTS) {
                comp_x[comp_count]=bx; comp_y[comp_count]=by; comp_count++;
            }
        }
    }
    if (point_count > 1 && comp_count < MAX_ACT_POINTS) {
        comp_x[comp_count]=points_x[point_count-1]; comp_y[comp_count]=points_y[point_count-1]; comp_count++;
    }
    int n = (comp_count < max_len) ? comp_count : max_len;
    for (int i = 0; i < n; i++) { out_x[i] = comp_x[i]; out_y[i] = comp_y[i]; }
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
    int action_count = light_push_plan(grid_map, bomb_start_r, bomb_start_c,
                                       bomb_target_r, bomb_target_c,
                                       car_start_r, car_start_c,
                                       actions, 1000);
    /*#ifdef DEBUG
    DEBUG_PRINT("light_push_plan 返回动作数: %d\n", action_count);
    for (int i = 0; i < action_count; i++) DEBUG_PRINT("动作 %d: %d\n", i, actions[i]);
    #endif*/
    if (action_count <= 0) return -1;

    for (int i = 0; i < action_count; i++) actions[i] += 4;

    return bomb_actions_to_world_path(state, bomb_id, car_x, car_y,
                                       actions, action_count,
                                       out_path_x, out_path_y, max_path_len);
}

// 寻找炸弹目标点（墙体旁边的可通行格子）- 可选保留
/*int find_bomb_target_near_wall(GameState* state, GridMap* grid_map, Wall* wall,
                               float* out_x, float* out_y) {
    int wall_r = (int)(wall->y1 / CELL_SIZE);
    int wall_c = (int)(wall->x1 / CELL_SIZE);
    const int dr[] = {-1, 0, 1, 0};
    const int dc[] = {0, 1, 0, -1};
    for (int d = 0; d < 4; d++) {
        int nr = wall_r + dr[d];
        int nc = wall_c + dc[d];
        if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
        int base_x = nc * 4, base_y = nr * 4, occupied = 0;
        for (int dy = 0; dy < 4 && !occupied; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                if (grid_map->occupancy[base_y + dy][base_x + dx] == OCC_WALL ||
                    grid_map->occupancy[base_y + dy][base_x + dx] == OCC_BOX) {
                    occupied = 1;
                    break;
                }
            }
        }
        if (!occupied) {
            *out_x = (nc + 0.5f) * CELL_SIZE;
            *out_y = (nr + 0.5f) * CELL_SIZE;
            return 1;
        }
    }
    return 0;
}*/

// 炸弹子地图生成（调试用）
/*void build_bomb_submap(GameState* state, GridMap* grid_map,
                       int bomb_id, int target_r, int target_c, CoarseMap* submap) {
    // 简单实现，可根据需要填充
    memset(submap->cells, 0, sizeof(submap->cells));
    // ... 略 ...
}*/