#include "planner.h"
#include <math.h>
#include <rtthread.h>
#include "zf_device_wireless_uart.h"

static int can_reach_fine(GridMap* map, int sx, int sy, int gx, int gy) {
    if (sx == gx && sy == gy) return 1;
    if (map->occupancy[sy][sx] == OCC_WALL || map->occupancy[sy][sx] == OCC_BOX || map->occupancy[sy][sx] == OCC_BOMB ||
        map->occupancy[gy][gx] == OCC_WALL || map->occupancy[gy][gx] == OCC_BOX || map->occupancy[gy][gx] == OCC_BOMB)
        return 0;

    const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    __attribute__((section("OCRAM_CACHE"))) static int qx[FINE_COLS * FINE_ROWS];
    __attribute__((section("OCRAM_CACHE"))) static int qy[FINE_COLS * FINE_ROWS];
    __attribute__((section("OCRAM_CACHE"))) static uint8_t visited[FINE_ROWS][FINE_COLS];
    memset(visited, 0, sizeof(visited));

    int head = 0, tail = 0;
    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sy][sx] = 1;

    while (head < tail) {
        int x = qx[head], y = qy[head];
        head++;
        for (int d = 0; d < 8; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || nx >= map->width || ny < 0 || ny >= map->height) continue;
            if (visited[ny][nx]) continue;
            if (dx[d] != 0 && dy[d] != 0) {
                if (map->occupancy[y][nx] == OCC_WALL || map->occupancy[y][nx] == OCC_BOX || map->occupancy[y][nx] == OCC_BOMB) continue;
                if (map->occupancy[ny][x] == OCC_WALL || map->occupancy[ny][x] == OCC_BOX || map->occupancy[ny][x] == OCC_BOMB) continue;
            }
            if (map->occupancy[ny][nx] == OCC_WALL || map->occupancy[ny][nx] == OCC_BOX || map->occupancy[ny][nx] == OCC_BOMB) continue;
            if (nx == gx && ny == gy) return 1;
            visited[ny][nx] = 1;
            qx[tail] = nx; qy[tail] = ny; tail++;
            if (tail >= FINE_COLS * FINE_ROWS) tail = 0;
        }
    }
    return 0;
}

int can_reach_fine_exclude_bomb(GridMap* map, int sx, int sy, int gx, int gy,
                                int bomb_r, int bomb_c) {
    if (sx == gx && sy == gy) return 1;
    if (map->occupancy[sy][sx] == OCC_WALL || map->occupancy[sy][sx] == OCC_BOX || map->occupancy[sy][sx] == OCC_BOMB ||
        map->occupancy[gy][gx] == OCC_WALL || map->occupancy[gy][gx] == OCC_BOX || map->occupancy[gy][gx] == OCC_BOMB)
        return 0;

    const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    __attribute__((section("OCRAM_CACHE"))) static int qx[FINE_COLS * FINE_ROWS];
    __attribute__((section("OCRAM_CACHE"))) static int qy[FINE_COLS * FINE_ROWS];
    __attribute__((section("OCRAM_CACHE"))) static uint8_t visited[FINE_ROWS][FINE_COLS];
    memset(visited, 0, sizeof(visited));

    int head = 0, tail = 0;
    qx[tail] = sx; qy[tail] = sy; tail++;
    visited[sy][sx] = 1;

    while (head < tail) {
        int x = qx[head], y = qy[head];
        head++;
        for (int d = 0; d < 8; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || nx >= map->width || ny < 0 || ny >= map->height) continue;
            if (visited[ny][nx]) continue;
            if (ny/4 == bomb_r && nx/4 == bomb_c) continue;
            if (dx[d] != 0 && dy[d] != 0) {
                if (map->occupancy[y][nx] == OCC_WALL || map->occupancy[y][nx] == OCC_BOX || map->occupancy[y][nx] == OCC_BOMB) continue;
                if (map->occupancy[ny][x] == OCC_WALL || map->occupancy[ny][x] == OCC_BOX || map->occupancy[ny][x] == OCC_BOMB) continue;
            }
            if (map->occupancy[ny][nx] == OCC_WALL || map->occupancy[ny][nx] == OCC_BOX || map->occupancy[ny][nx] == OCC_BOMB) continue;
            if (nx == gx && ny == gy) return 1;
            visited[ny][nx] = 1;
            qx[tail] = nx; qy[tail] = ny; tail++;
            if (tail >= FINE_COLS * FINE_ROWS) tail = 0;
        }
    }
    return 0;
}

int is_boundary_wall(Wall* w) {
    const float FIELD_WIDTH = 3.2f;
    const float FIELD_HEIGHT = 2.4f;
    const float EPS = 0.01f;
    if (fabsf(w->x1) < EPS || fabsf(w->x2 - FIELD_WIDTH) < EPS ||
        fabsf(w->y1) < EPS || fabsf(w->y2 - FIELD_HEIGHT) < EPS)
        return 1;
    return 0;
}

int simulate_wall_destruction(GameState* state, GridMap* grid_map,
                              int wall_idx, int start_x, int start_y,
                              int box_id, int* out_push_steps) {
    GameState temp_state = *state;
    Wall temp_walls[MAX_WALLS];
    memcpy(temp_walls, state->walls, state->num_walls * sizeof(Wall));

    int kept = 0;
    for (int i = 0; i < temp_state.num_walls; i++) {
        if (i != wall_idx) temp_walls[kept++] = temp_walls[i];
    }
    temp_state.num_walls = kept;
    memcpy(temp_state.walls, temp_walls, kept * sizeof(Wall));

    GridMap temp_map = *grid_map;
    refresh_grid_map(&temp_state, &temp_map);

    float car_img_x = start_x * RESOLUTION;
    float car_img_y = start_y * RESOLUTION;
    int actions[200];
    int len = light_sokoban_plan(&temp_state, &temp_map, box_id, car_img_x, car_img_y, actions, 200);
    if (len > 0) {
        if (out_push_steps) *out_push_steps = len;
        return 1;
    }
    return 0;
}

int select_best_wall_to_destroy(GameState* state, GridMap* grid_map,
                                int start_x, int start_y, int box_id,
                                float* out_bomb_target_x, float* out_bomb_target_y,
                                int* out_push_dir) {
    int car_start_r = start_y / 4;
    int car_start_c = start_x / 4;

    int bomb_id = -1;
    for (int b = 0; b < state->num_bombs; b++) {
        if (state->bombs[b].active) { bomb_id = b; break; }
    }
    if (bomb_id < 0) return -1;
    Bomb* bomb = &state->bombs[bomb_id];

    float bomb_img_x, bomb_img_y;
    motion_to_image(bomb->x, bomb->y, &bomb_img_x, &bomb_img_y);
    int bomb_r = (int)(bomb_img_y / CELL_SIZE);
    int bomb_c = (int)(bomb_img_x / CELL_SIZE);

    int best_wall = -1, best_steps = 1e9, best_dir = -1;
    float best_target_mx = 0, best_target_my = 0;

    for (int w = 0; w < state->num_walls; w++) {
        if (is_boundary_wall(&state->walls[w])) continue;
        Wall* wall = &state->walls[w];

        float wall_center_mx = (wall->x1 + wall->x2) * 0.5f;
        float wall_center_my = (wall->y1 + wall->y2) * 0.5f;

        float dx = wall_center_mx - bomb->x;
        float dy = wall_center_my - bomb->y;
        if (fabsf(dx) < 0.01f && fabsf(dy) < 0.01f) continue;

        int dir = -1;
        if (fabsf(dx) > fabsf(dy)) dir = (dx > 0) ? 2 : 0;
        else dir = (dy > 0) ? 1 : 3;

        int steps;
        if (simulate_wall_destruction(state, grid_map, w, start_x, start_y, box_id, &steps)) {
            if (steps < best_steps) {
                best_steps = steps;
                best_wall = w;
                best_dir = dir;
                best_target_mx = wall_center_mx;
                best_target_my = wall_center_my;
            }
        }
    }

    if (best_wall >= 0) {
        *out_bomb_target_x = best_target_mx;
        *out_bomb_target_y = best_target_my;
        *out_push_dir = best_dir;
    }
    return best_wall;
}

int light_push_plan(GridMap* map, int start_r, int start_c, int goal_r, int goal_c,
                    int car_start_r, int car_start_c, int* out_actions, int max_actions) {
    __attribute__((section("OCRAM_CACHE"))) static uint8_t obstacle[MAP_ROWS][MAP_COLS];
    memset(obstacle, 0, sizeof(obstacle));
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            int base_x = c * 4, base_y = r * 4;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    uint8_t occ = map->occupancy[base_y + dy][base_x + dx];
                    if (occ == OCC_WALL || occ == OCC_BOX || occ == OCC_BOMB) {
                        obstacle[r][c] = 1;
                        break;
                    }
                }
                if (obstacle[r][c]) break;
            }
        }
    }

    int goal_is_wall = 0;
    if (goal_r >= 0 && goal_r < MAP_ROWS && goal_c >= 0 && goal_c < MAP_COLS) {
        int base_x = goal_c * 4, base_y = goal_r * 4;
        for (int dy = 0; dy < 4; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                if (map->occupancy[base_y+dy][base_x+dx] == OCC_WALL ||
                    map->occupancy[base_y+dy][base_x+dx] == OCC_BOMB) {
                    return 0;
                }
            }
            if (goal_is_wall) break;
        }
    }
    if (goal_is_wall) obstacle[goal_r][goal_c] = 0;

    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};

    typedef struct {
        int br, bc;
        int pr, pc;
        int pushes;
        int parent;
        int action;
    } PushNode;

    #define BOMB_QUEUE_SIZE 1000
    __attribute__((section("OCRAM_CACHE"))) static PushNode queue[BOMB_QUEUE_SIZE];
    int best_pushes[MAP_ROWS][MAP_COLS];
    memset(best_pushes, -1, sizeof(best_pushes));

    int head = 0, tail = 0;
    queue[tail].br = start_r; queue[tail].bc = start_c;
    queue[tail].pr = car_start_r; queue[tail].pc = car_start_c;
    queue[tail].pushes = 0;
    queue[tail].parent = -1; queue[tail].action = -1;
    tail++;
    best_pushes[start_r][start_c] = 0;

    int found_idx = -1, expanded = 0;
    #define BFS_MAX_EXPANSIONS_PUSH 3000

    while (head < tail) {
        if (expanded >= BFS_MAX_EXPANSIONS_PUSH) {
            return -1;  // 超时，放弃此方向
        }
        expanded++;

        PushNode cur = queue[head];

        if (cur.br == goal_r && cur.bc == goal_c) { found_idx = head; break; }

        for (int d = 0; d < 4; d++) {
            int push_r = cur.br - dr[d], push_c = cur.bc - dc[d];
            if (push_r < 0 || push_r >= MAP_ROWS || push_c < 0 || push_c >= MAP_COLS) continue;
            if (obstacle[push_r][push_c] == 1) continue;

            int nbr = cur.br + dr[d], nbc = cur.bc + dc[d];
            if (nbr < 0 || nbr >= MAP_ROWS || nbc < 0 || nbc >= MAP_COLS) continue;
            if (obstacle[nbr][nbc] == 1) continue;

            int sx = cur.pc * 4 + 2, sy = cur.pr * 4 + 2;
            int gx = push_c * 4 + 2, gy = push_r * 4 + 2;
            if (!can_reach_fine_exclude_bomb(map, sx, sy, gx, gy, cur.br, cur.bc)) continue;

            int new_pushes = cur.pushes + 1;
            if (best_pushes[nbr][nbc] == -1 || new_pushes < best_pushes[nbr][nbc]) {
                best_pushes[nbr][nbc] = new_pushes;
                if (tail >= BOMB_QUEUE_SIZE) continue;
                queue[tail].br = nbr; queue[tail].bc = nbc;
                queue[tail].pr = cur.br; queue[tail].pc = cur.bc;
                queue[tail].pushes = new_pushes;
                queue[tail].parent = head;
                queue[tail].action = d;
                tail++;
            }
        }
        head++;
    }

    if (found_idx == -1) return -1;

    int actions[200], act_cnt = 0;
    int idx = found_idx;
    while (idx != -1) {
        if (queue[idx].action != -1) actions[act_cnt++] = queue[idx].action;
        idx = queue[idx].parent;
    }
    for (int i = 0; i < act_cnt / 2; i++) {
        int tmp = actions[i];
        actions[i] = actions[act_cnt - 1 - i];
        actions[act_cnt - 1 - i] = tmp;
    }

    int n = (act_cnt < max_actions) ? act_cnt : max_actions;
    for (int i = 0; i < n; i++) out_actions[i] = actions[i];
    return n;
}