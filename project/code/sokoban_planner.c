#include "planner.h"
#include <math.h>
#include "zf_device_wireless_uart.h"
#include <rtthread.h>

static int is_deadlock(int br, int bc, int goal_br, int goal_bc, uint8_t obstacle[MAP_ROWS][MAP_COLS]) {
    if (br == goal_br && bc == goal_bc) return 0;

    int up    = (br == 0 || obstacle[br-1][bc]);
    int down  = (br == MAP_ROWS-1 || obstacle[br+1][bc]);
    int left  = (bc == 0 || obstacle[br][bc-1]);
    int right = (bc == MAP_COLS-1 || obstacle[br][bc+1]);

    int walls = up + down + left + right;
    if (walls < 2) return 0;
    if (walls == 4) return 1;

    // 相邻两墙构成角落死锁（除非在目标上，已提前返回）
    if ((up && left) || (up && right) || (down && left) || (down && right))
        return 1;

    return 0;
}

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

    int max_nodes = 1000;  
    int nodes_visited = 0;

    while (head < tail) {
        nodes_visited++;
        if (nodes_visited > max_nodes) {
            return 0;
        }

        int x = qx[head], y = qy[head];
        head++;
        for (int d = 0; d < 8; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || nx >= map->width || ny < 0 || ny >= map->height) continue;
            if (visited[ny][nx]) continue;

            if (dx[d] != 0 && dy[d] != 0) {
                uint8_t occ1 = map->occupancy[y][nx];
                uint8_t occ2 = map->occupancy[ny][x];
                if ((occ1 == OCC_WALL || occ1 == OCC_BOX || occ1 == OCC_BOMB) &&
                    (occ2 == OCC_WALL || occ2 == OCC_BOX || occ2 == OCC_BOMB))
                    continue;
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
int light_sokoban_plan(GameState* state, GridMap* grid_map, int box_id,
                       float car_x, float car_y,
                       int* out_actions, int max_actions) {
    char buf[128];
    rt_sprintf(buf, "light_sokoban_plan: entry, box_id=%d, car=(%d,%d)\r\n",
               box_id, (int)(car_x * 1000), (int)(car_y * 1000));
    wireless_uart_send_string(buf);

    if (box_id < 0 || box_id >= state->num_boxes) return -1;
    Box* box = &state->boxes[box_id];
    if (box->dest_id < 0) return -1;

    float box_img_x, box_img_y, dest_img_x, dest_img_y;
    motion_to_image(box->x, box->y, &box_img_x, &box_img_y);
    motion_to_image(state->destinations[box->dest_id].x, state->destinations[box->dest_id].y,
                    &dest_img_x, &dest_img_y);
    
    int br = (int)(box_img_y / CELL_SIZE);
    int bc = (int)(box_img_x / CELL_SIZE);
    int pr = (int)(car_y / CELL_SIZE);
    int pc = (int)(car_x / CELL_SIZE);
    int goal_br = (int)(dest_img_y / CELL_SIZE);
    int goal_bc = (int)(dest_img_x / CELL_SIZE);

    if (br < 0) br = 0; if (br >= MAP_ROWS) br = MAP_ROWS - 1;
    if (bc < 0) bc = 0; if (bc >= MAP_COLS) bc = MAP_COLS - 1;
    if (pr < 0) pr = 0; if (pr >= MAP_ROWS) pr = MAP_ROWS - 1;
    if (pc < 0) pc = 0; if (pc >= MAP_COLS) pc = MAP_COLS - 1;
    if (goal_br < 0) goal_br = 0; if (goal_br >= MAP_ROWS) goal_br = MAP_ROWS - 1;
    if (goal_bc < 0) goal_bc = 0; if (goal_bc >= MAP_COLS) goal_bc = MAP_COLS - 1;

    uint8_t obstacle[MAP_ROWS][MAP_COLS] = {{0}};
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            int base_x = c * 4, base_y = r * 4;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    if (grid_map->occupancy[base_y+dy][base_x+dx] == OCC_WALL ||
                        grid_map->occupancy[base_y+dy][base_x+dx] == OCC_BOMB) {
                        obstacle[r][c] = 1;
                        goto next_cell;
                    }
                }
            }
            next_cell:;
        }
    }
    for (int i = 0; i < state->num_boxes; i++) {
        if (i == box_id) continue;
        if (state->boxes[i].state == 0) {
            float other_img_x, other_img_y;
            motion_to_image(state->boxes[i].x, state->boxes[i].y, &other_img_x, &other_img_y);
            int or_ = (int)(other_img_y / CELL_SIZE), oc = (int)(other_img_x / CELL_SIZE);
            //if (or_ == goal_br && oc == goal_bc) continue;
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

    #define BFS_QUEUE_SIZE 2000
    __attribute__((section("OCRAM_CACHE"))) static LightNode queue[BFS_QUEUE_SIZE];
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
    int expanded = 0;
    #define BFS_MAX_EXPANSIONS_SOKOBAN 3000

    while (head < tail) {
        if (expanded >= BFS_MAX_EXPANSIONS_SOKOBAN) {
            return -1;  
        }
        expanded++;

        LightNode cur = queue[head];

        if (cur.br == goal_br && cur.bc == goal_bc) {
            found_idx = head;
            break;
        }

        for (int d = 0; d < 4; d++) {
            int push_r = cur.br - dr[d], push_c = cur.bc - dc[d];
            if (push_r < 0 || push_r >= MAP_ROWS || push_c < 0 || push_c >= MAP_COLS) continue;
            if (obstacle[push_r][push_c] == 1) continue;

            int nbr = cur.br + dr[d], nbc = cur.bc + dc[d];
            if (nbr < 0 || nbr >= MAP_ROWS || nbc < 0 || nbc >= MAP_COLS) continue;
            if (obstacle[nbr][nbc] == 1) continue;
            if (is_deadlock(nbr, nbc, goal_br, goal_bc, obstacle)) continue;

            int sx = cur.pc * 4 + 2, sy = cur.pr * 4 + 2;
            int gx = push_c * 4 + 2, gy = push_r * 4 + 2;
            if (!can_reach_fine(grid_map, sx, sy, gx, gy)) continue;

            int new_pushes = cur.pushes + 1;
            if (best_pushes[nbr][nbc] == -1 || new_pushes < best_pushes[nbr][nbc]) {
                best_pushes[nbr][nbc] = new_pushes;
                if (tail >= BFS_QUEUE_SIZE) continue;
                queue[tail].br = nbr; queue[tail].bc = nbc;
                queue[tail].pr = cur.br; queue[tail].pc = cur.bc;
                queue[tail].pushes = new_pushes;
                queue[tail].parent = head;
                queue[tail].action = 4 + d;
                tail++;
            }
        }
        head++;
    }

    if (found_idx == -1) {
        rt_sprintf(buf, "BFS failed: expanded=%d, head=%d, tail=%d\r\n", expanded, head, tail);
        wireless_uart_send_string(buf);
        return -1;
    }

    int actions[500];
    int act_cnt = 0;
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

    rt_sprintf(buf, "BFS success: actions=%d\r\n", n);
    wireless_uart_send_string(buf);
    return n;
}