#include "planner.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>   // 添加此行
#include <stdio.h>

// ----- RDP 简化相关函数 -----
static float point_line_distance(float x0, float y0, float x1, float y1, float x2, float y2) {
    float vx = x1 - x0, vy = y1 - y0;
    float wx = x2 - x0, wy = y2 - y0;
    float c1 = vx * wx + vy * wy;
    float c2 = vx * vx + vy * vy;
    if (c2 <= 1e-9f) {
        float dx = x2 - x0, dy = y2 - y0;
        return sqrtf(dx*dx + dy*dy);
    }
    float t = c1 / c2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float px = x0 + t * vx;
    float py = y0 + t * vy;
    float dx = x2 - px, dy = y2 - py;
    return sqrtf(dx*dx + dy*dy);
}

static int rdp_recursive(const float* x, const float* y, int n, float eps, int* keep) {
    if (n <= 2) {
        for (int i = 0; i < n; i++) keep[i] = 1;
        return n;
    }
    int idx = -1;
    float maxd = -1.0f;
    for (int i = 1; i < n-1; i++) {
        float d = point_line_distance(x[0], y[0], x[n-1], y[n-1], x[i], y[i]);
        if (d > maxd) { maxd = d; idx = i; }
    }
    if (maxd <= eps) {
        for (int i = 0; i < n; i++) keep[i] = 0;
        keep[0] = 1; keep[n-1] = 1;
        return 2;
    }
    int left_keep[1000];
    int right_keep[1000];
    int left_n = idx + 1;
    int right_n = n - idx;
    rdp_recursive(x, y, left_n, eps, left_keep);
    rdp_recursive(x + idx, y + idx, right_n, eps, right_keep);
    int out_count = 0;
    for (int i = 0; i < left_n; i++) if (left_keep[i]) keep[i] = 1; else keep[i] = 0;
    for (int i = 1; i < right_n; i++) if (right_keep[i]) keep[idx + i] = 1;
    for (int i = 0; i < n; i++) if (keep[i]) out_count++;
    return out_count;
}

static int rdp_simplify(const float* in_x, const float* in_y, int n, float eps, float* out_x, float* out_y) {
    if (n <= 2) {
        for (int i = 0; i < n; i++) { out_x[i] = in_x[i]; out_y[i] = in_y[i]; }
        return n;
    }
    if (n > MAX_KEEP_SIZE) n = MAX_KEEP_SIZE;
    int keep[MAX_KEEP_SIZE] = {0};
    rdp_recursive(in_x, in_y, n, eps, keep);
    int cnt = 0;
    for (int i = 0; i < n; i++) if (keep[i]) { out_x[cnt] = in_x[i]; out_y[cnt] = in_y[i]; cnt++; }
    return cnt;
}

static int line_of_sight(GridMap* map, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (1) {
        if (!(x == x1 && y == y1)) {
            uint8_t occ = map->occupancy[y][x];
            if (occ == OCC_WALL || occ == OCC_BOX) return 0;
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
    return 1;
}

static int connect_with_astar_and_simplify(GridMap* grid_map, int* coarse_r, int* coarse_c, int m,
                                          float* out_x, float* out_y, int max_len, float rdp_eps) {
    if (m <= 0) return 0;
    static float acc_x[MAX_PATH_POINTS];
    static float acc_y[MAX_PATH_POINTS];
    int acc_count = 0;
    AStarParams params = {.max_iterations = 5000, .inflation_radius = 2};
    for (int i = 0; i < m - 1; i++) {
        int cr0 = coarse_r[i], cc0 = coarse_c[i];
        int cr1 = coarse_r[i+1], cc1 = coarse_c[i+1];
        int sx = cc0 * 4 + 2; int sy = cr0 * 4 + 2;
        int gx = cc1 * 4 + 2; int gy = cr1 * 4 + 2;
        if (sx < 0) sx = 0; if (sx >= grid_map->width) sx = grid_map->width-1;
        if (gx < 0) gx = 0; if (gx >= grid_map->width) gx = grid_map->width-1;
        if (sy < 0) sy = 0; if (sy >= grid_map->height) sy = grid_map->height-1;
        if (gy < 0) gy = 0; if (gy >= grid_map->height) gy = grid_map->height-1;

        if (line_of_sight(grid_map, sx, sy, gx, gy)) {
            float wx = (cc0 + 0.5f) * CELL_SIZE;
            float wy = (cr0 + 0.5f) * CELL_SIZE;
            if (acc_count < MAX_PATH_POINTS && (acc_count == 0 || fabsf(wx - acc_x[acc_count-1]) > 1e-6f || fabsf(wy - acc_y[acc_count-1]) > 1e-6f)) {
                acc_x[acc_count] = wx; acc_y[acc_count] = wy; acc_count++;
            }
            continue;
        }

        static int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
        int plen = astar_plan_path(grid_map, sx, sy, gx, gy, path_x, path_y, MAX_PATH_POINTS, &params);
        if (plen <= 0) {
            float wx = (cc0 + 0.5f) * CELL_SIZE;
            float wy = (cr0 + 0.5f) * CELL_SIZE;
            if (acc_count < MAX_PATH_POINTS && (acc_count == 0 || fabsf(wx - acc_x[acc_count-1]) > 1e-6f || fabsf(wy - acc_y[acc_count-1]) > 1e-6f)) {
                acc_x[acc_count] = wx; acc_y[acc_count] = wy; acc_count++;
            }
            continue;
        }
        for (int k = 0; k < plen; k++) {
            int fx = path_x[k], fy = path_y[k];
            float wx, wy;
            grid_to_world(fx, fy, &wx, &wy);
            if (acc_count < MAX_PATH_POINTS && (acc_count == 0 || fabsf(wx - acc_x[acc_count-1]) > 1e-6f || fabsf(wy - acc_y[acc_count-1]) > 1e-6f)) {
                acc_x[acc_count] = wx; acc_y[acc_count] = wy; acc_count++;
            }
        }
        if (acc_count >= MAX_PATH_POINTS) break;
    }
    float last_wx = (coarse_c[m-1] + 0.5f) * CELL_SIZE;
    float last_wy = (coarse_r[m-1] + 0.5f) * CELL_SIZE;
    if (acc_count < MAX_PATH_POINTS && (acc_count == 0 || fabsf(last_wx - acc_x[acc_count-1]) > 1e-6f || fabsf(last_wy - acc_y[acc_count-1]) > 1e-6f)) {
        acc_x[acc_count] = last_wx; acc_y[acc_count] = last_wy; acc_count++;
    }

    static float simp_x[MAX_PATH_POINTS];
    static float simp_y[MAX_PATH_POINTS];
    int simp_n = rdp_simplify(acc_x, acc_y, acc_count, rdp_eps, simp_x, simp_y);
    int out_n = (simp_n < max_len) ? simp_n : max_len;
    for (int i = 0; i < out_n; i++) { out_x[i] = simp_x[i]; out_y[i] = simp_y[i]; }
    return out_n;
}

// 将动作序列转换为世界坐标路径
static int actions_to_world_path(GameState* state, int box_id, float start_car_x, float start_car_y,
                                 const int* actions, int action_count,
                                 float* out_x, float* out_y, int max_len) {
    Box* box = &state->boxes[box_id];
    float box_x = box->x, box_y = box->y;

    int pr = (int)(start_car_y / CELL_SIZE);
    int pc = (int)(start_car_x / CELL_SIZE);
    int br = (int)(box_y / CELL_SIZE);
    int bc = (int)(box_x / CELL_SIZE);

    const int dr4[4] = {-1, 0, 1, 0};
    const int dc4[4] = {0, 1, 0, -1};

    static float points_x[MAX_ACT_POINTS];
    static float points_y[MAX_ACT_POINTS];
    int point_count = 0;

    #define ADD_POINT(x, y) do { \
        if (point_count < MAX_ACT_POINTS && (point_count == 0 || \
            fabsf((x) - points_x[point_count-1]) > 1e-2f || \
            fabsf((y) - points_y[point_count-1]) > 1e-2f)) { \
            points_x[point_count] = (x); \
            points_y[point_count] = (y); \
            point_count++; \
        } \
    } while(0)

    ADD_POINT(start_car_x, start_car_y);

    for (int i = 0; i < action_count; i++) {
        int act = actions[i];
        if (act >= 4) {
            int d = act - 4;
            int old_br = br, old_bc = bc;
            float new_px = (old_bc + 0.5f) * CELL_SIZE;
            float new_py = (old_br + 0.5f) * CELL_SIZE;
            if (point_count == 0 || fabsf(new_px - points_x[point_count-1]) > 1e-6f || fabsf(new_py - points_y[point_count-1]) > 1e-6f) {
                if (point_count < MAX_ACT_POINTS) {
                    points_x[point_count] = new_px;
                    points_y[point_count] = new_py;
                    point_count++;
                }
            }
            br = br + dr4[d];
            bc = bc + dc4[d];
            pr = old_br;
            pc = old_bc;
        } else {
            int d = act;
            pr += dr4[d];
            pc += dc4[d];
            float new_px = (pc + 0.5f) * CELL_SIZE;
            float new_py = (pr + 0.5f) * CELL_SIZE;
            if (point_count == 0 || fabsf(new_px - points_x[point_count-1]) > 1e-6f || fabsf(new_py - points_y[point_count-1]) > 1e-6f) {
                if (point_count < MAX_ACT_POINTS) {
                    points_x[point_count] = new_px;
                    points_y[point_count] = new_py;
                    point_count++;
                }
            }
        }
    }
    #undef ADD_POINT

    static float comp_x[MAX_ACT_POINTS];
    static float comp_y[MAX_ACT_POINTS];
    int comp_count = 0;
    if (point_count > 0) {
        comp_x[comp_count] = points_x[0];
        comp_y[comp_count] = points_y[0];
        comp_count++;
    }
    const float EPS = 1e-4f;
    for (int i = 1; i < point_count - 1; i++) {
        float ax = points_x[i-1], ay = points_y[i-1];
        float bx = points_x[i],   by = points_y[i];
        float cx = points_x[i+1], cy = points_y[i+1];
        float v1x = bx - ax, v1y = by - ay;
        float v2x = cx - bx, v2y = cy - by;
        float cross = fabsf(v1x * v2y - v1y * v2x);
        if (cross > EPS) {
            if (comp_count < MAX_ACT_POINTS) {
                comp_x[comp_count] = bx;
                comp_y[comp_count] = by;
                comp_count++;
            }
        }
    }
    if (point_count > 1 && comp_count < MAX_ACT_POINTS) {
        comp_x[comp_count] = points_x[point_count-1];
        comp_y[comp_count] = points_y[point_count-1];
        comp_count++;
    }

    int n = (comp_count < max_len) ? comp_count : max_len;
    for (int i = 0; i < n; i++) {
        out_x[i] = comp_x[i];
        out_y[i] = comp_y[i];
    }
    return n;
}

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

// 简单角落死锁检测
static int is_deadlock_simple(int br, int bc, int goal_br, int goal_bc,
                              const uint8_t obstacle[MAP_ROWS][MAP_COLS]) {
    if (br == goal_br && bc == goal_bc) return 0;
    const int dir_pairs[4][2] = {{0,1}, {0,3}, {2,1}, {2,3}};
    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};
    for (int p = 0; p < 4; p++) {
        int d1 = dir_pairs[p][0];
        int d2 = dir_pairs[p][1];
        int r1 = br + dr[d1];
        int c1 = bc + dc[d1];
        int r2 = br + dr[d2];
        int c2 = bc + dc[d2];
        int blocked1 = (r1 < 0 || r1 >= MAP_ROWS || c1 < 0 || c1 >= MAP_COLS || obstacle[r1][c1] == 1);
        int blocked2 = (r2 < 0 || r2 >= MAP_ROWS || c2 < 0 || c2 >= MAP_COLS || obstacle[r2][c2] == 1);
        if (blocked1 && blocked2) return 1;
    }
    return 0;
}

// 轻量级推箱子规划器（分解法）
int light_sokoban_plan(GameState* state, GridMap* grid_map, int box_id,
                       float car_x, float car_y,
                       int* out_actions, int max_actions) {
    if (box_id < 0 || box_id >= state->num_boxes) return -1;
    Box* box = &state->boxes[box_id];
    if (box->dest_id < 0) return -1;

    int br = (int)(box->y / CELL_SIZE);
    int bc = (int)(box->x / CELL_SIZE);
    int pr = (int)(car_y / CELL_SIZE);
    int pc = (int)(car_x / CELL_SIZE);
    Destination* dest = &state->destinations[box->dest_id];
    int goal_br = (int)(dest->y / CELL_SIZE);
    int goal_bc = (int)(dest->x / CELL_SIZE);

    uint8_t obstacle[MAP_ROWS][MAP_COLS] = {{0}};
    // 墙体
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            int base_x = c * 4;
            int base_y = r * 4;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    if (grid_map->occupancy[base_y+dy][base_x+dx] == OCC_WALL) {
                        obstacle[r][c] = 1;
                        goto next_cell;
                    }
                }
            }
            next_cell:;
        }
    }
    // 其他未推箱子
    for (int i = 0; i < state->num_boxes; i++) {
        if (i == box_id) continue;
        if (state->boxes[i].state == 0) {
            int or_ = (int)(state->boxes[i].y / CELL_SIZE);
            int oc = (int)(state->boxes[i].x / CELL_SIZE);
            if (or_ >= 0 && or_ < MAP_ROWS && oc >= 0 && oc < MAP_COLS)
                obstacle[or_][oc] = 1;
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
    } LightNode;

    static LightNode queue[200];
    int best_pushes[MAP_ROWS][MAP_COLS];
    memset(best_pushes, -1, sizeof(best_pushes));

    int head = 0, tail = 0;
    queue[tail].br = br; queue[tail].bc = bc;
    queue[tail].pr = pr; queue[tail].pc = pc;
    queue[tail].pushes = 0;
    queue[tail].parent = -1;
    queue[tail].action = -1;
    tail++;
    best_pushes[br][bc] = 0;

    int found_idx = -1;

    while (head < tail) {
        LightNode cur = queue[head];
        if (cur.br == goal_br && cur.bc == goal_bc) {
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

            if (is_deadlock_simple(nbr, nbc, goal_br, goal_bc, obstacle)) continue;

            int sx = cur.pc * 4 + 2;
            int sy = cur.pr * 4 + 2;
            int gx = push_c * 4 + 2;
            int gy = push_r * 4 + 2;
            if (!can_reach_fine(grid_map, sx, sy, gx, gy)) continue;

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
                queue[tail].action = 4 + d;   // 推动动作编码为 4~7
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
    // printf("Light sokoban planner: queue max used %d/200\n", tail);
    return n;
}

// 构建单箱子子地图
void build_single_box_submap(GameState* state, GridMap* grid_map,
                             int box_id, CoarseMap* submap) {
    memset(submap->cells, 0, sizeof(submap->cells));
    // 墙体
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            int base_x = c * 4;
            int base_y = r * 4;
            int has_wall = 0;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    if (grid_map->occupancy[base_y + dy][base_x + dx] == OCC_WALL) {
                        has_wall = 1;
                        break;
                    }
                }
                if (has_wall) break;
            }
            if (has_wall) submap->cells[r][c] = 1;
        }
    }
    // 其他箱子
    for (int i = 0; i < state->num_boxes; i++) {
        if (i == box_id) continue;
        if (state->boxes[i].state == 0) {
            int br = (int)(state->boxes[i].y / CELL_SIZE);
            int bc = (int)(state->boxes[i].x / CELL_SIZE);
            if (br >= 0 && br < MAP_ROWS && bc >= 0 && bc < MAP_COLS)
                submap->cells[br][bc] = 1;
        }
    }
    // 当前箱子
    int br = (int)(state->boxes[box_id].y / CELL_SIZE);
    int bc = (int)(state->boxes[box_id].x / CELL_SIZE);
    if (br >= 0 && br < MAP_ROWS && bc >= 0 && bc < MAP_COLS)
        submap->cells[br][bc] = 2;
    // 目标
    int dest_id = state->boxes[box_id].dest_id;
    if (dest_id >= 0 && dest_id < state->num_destinations) {
        int dr = (int)(state->destinations[dest_id].y / CELL_SIZE);
        int dc = (int)(state->destinations[dest_id].x / CELL_SIZE);
        if (dr >= 0 && dr < MAP_ROWS && dc >= 0 && dc < MAP_COLS)
            submap->cells[dr][dc] = 3;
    }
}

// 顺序规划（简化）
int plan_all_boxes_sequentially(GameState* state, GridMap* grid_map,
                                float car_x, float car_y,
                                float* out_paths[MAX_BOXES], int out_lens[MAX_BOXES],
                                int max_path_len) {
    return 0; // 可根据需要实现
}

// ========== 新增：sokoban_plan_for_box 实现 ==========
int sokoban_plan_for_box(GameState* state, GridMap* grid_map, int box_id,
                         float car_x, float car_y,
                         float* out_path_x, float* out_path_y, int max_path_len) {
    if (box_id < 0 || box_id >= state->num_boxes) return -1;
    Box* box = &state->boxes[box_id];
    if (box->dest_id < 0) return -1;

    int actions[1000];
    int action_count = light_sokoban_plan(state, grid_map, box_id, car_x, car_y, actions, 1000);
    if (action_count <= 0) return -1;

    static float tmp_x[MAX_PATH_POINTS], tmp_y[MAX_PATH_POINTS];
    int tmp_n = actions_to_world_path(state, box_id, car_x, car_y,
                                       actions, action_count,
                                       tmp_x, tmp_y, MAX_PATH_POINTS);
    if (tmp_n <= 0) return -1;

    static int coarse_r[MAX_PATH_POINTS], coarse_c[MAX_PATH_POINTS];
    int coarse_n = 0;
    for (int i = 0; i < tmp_n; i++) {
        int r = (int)(tmp_y[i] / CELL_SIZE);
        int c = (int)(tmp_x[i] / CELL_SIZE);
        if (coarse_n == 0 || coarse_r[coarse_n-1] != r || coarse_c[coarse_n-1] != c) {
            if (coarse_n < MAX_PATH_POINTS) {
                coarse_r[coarse_n] = r;
                coarse_c[coarse_n] = c;
                coarse_n++;
            } else break;
        }
    }

    float rdp_eps = 0.3f;
    int final_n = connect_with_astar_and_simplify(grid_map, coarse_r, coarse_c, coarse_n,
                                                  out_path_x, out_path_y, max_path_len, rdp_eps);
    return final_n;
}