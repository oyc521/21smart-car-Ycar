#include "planner.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>   
#include <stdio.h>
#include "zf_device_wireless_uart.h"
#include <rtthread.h>

// 死锁检测：如果箱子不在目标点，且被障碍物包围至少两个方向（角落）则返回1
static int is_deadlock(int br, int bc, int goal_br, int goal_bc, uint8_t obstacle[MAP_ROWS][MAP_COLS]) {
    if (br == goal_br && bc == goal_bc) return 0;  // 目标点不死锁
    
    // 统计四个方向被障碍物（墙/箱子/边界）阻挡的数量
    int walls = 0;
    if (br == 0 || obstacle[br-1][bc]) walls++;
    if (br == MAP_ROWS-1 || obstacle[br+1][bc]) walls++;
    if (bc == 0 || obstacle[br][bc-1]) walls++;
    if (bc == MAP_COLS-1 || obstacle[br][bc+1]) walls++;
    
    // 如果有两个或以上方向被堵，则死锁
    if (walls >= 2) return 1;
    
    // 特例：紧贴墙壁且对面也是墙（例如贴着上墙，上方是墙，左右有一边也是墙）
    // 简化处理，上面已经覆盖了大部分情况
    return 0;
}


int actions_to_world_path(GameState* state, GridMap* grid_map, int box_id,
                          float start_car_x, float start_car_y,
                          const int* actions, int action_count,
                          float* out_x, float* out_y, int max_len) {
    Box* box = &state->boxes[box_id];
    int pr = (int)(start_car_y / CELL_SIZE);
    int pc = (int)(start_car_x / CELL_SIZE);
    int br = (int)(box->y / CELL_SIZE);
    int bc = (int)(box->x / CELL_SIZE);
    const int dr4[4] = {-1, 0, 1, 0};
    const int dc4[4] = {0, 1, 0, -1};

    static float points_x[MAX_ACT_POINTS];
    static float points_y[MAX_ACT_POINTS];
    int point_count = 0;

    #define ADD_POINT(x, y) do { \
        if (point_count < MAX_ACT_POINTS && (point_count == 0 || \
            (int)((x)*1000) != (int)(points_x[point_count-1]*1000) || \
            (int)((y)*1000) != (int)(points_y[point_count-1]*1000))) { \
            points_x[point_count] = (x); \
            points_y[point_count] = (y); \
            point_count++; \
        } \
    } while(0)

    // 添加起点
    ADD_POINT(start_car_x, start_car_y);

    AStarParams params = {5000, 2.0f};
    for (int i = 0; i < action_count; i++) {
        int d = actions[i] - 4;
        if (d < 0 || d >= 4) {
            char dbg[64];
            sprintf(dbg, "Invalid action %d\n", actions[i]);
            wireless_uart_send_string(dbg);
            continue;
        }

        int push_r = br - dr4[d];
        int push_c = bc - dc4[d];
        // 打印动作信息
        char dbg[64];
        sprintf(dbg, "Action %d: d=%d, push(%d,%d)\n", i, d, push_r, push_c);
        wireless_uart_send_string(dbg);

        int sx = pc * 4 + 2;
        int sy = pr * 4 + 2;
        int gx = push_c * 4 + 2;
        int gy = push_r * 4 + 2;
        //sprintf(dbg, "  A* from (%d,%d) to (%d,%d)\n", sx, sy, gx, gy);
        wireless_uart_send_string(dbg);

        int path_x[MAX_PATH_POINTS], path_y[MAX_PATH_POINTS];
        int plen = astar_plan_path(grid_map, sx, sy, gx, gy, path_x, path_y, MAX_PATH_POINTS, &params);
        //sprintf(dbg, "  A* plen=%d\n", plen);
        wireless_uart_send_string(dbg);
        if (plen > 0) {
            for (int k = 0; k < plen; k++) {
                float wx, wy;
                grid_to_world(path_x[k], path_y[k], &wx, &wy);
                ADD_POINT(wx, wy);
            }
        } else {
            // A* 失败，直接添加推动位置
            ADD_POINT((push_c + 0.5f) * CELL_SIZE, (push_r + 0.5f) * CELL_SIZE);
        }

        // 更新箱子位置
        int old_br = br, old_bc = bc;
        br += dr4[d];
        bc += dc4[d];
        pr = old_br;
        pc = old_bc;
        float new_car_x = (pc + 0.5f) * CELL_SIZE;
        float new_car_y = (pr + 0.5f) * CELL_SIZE;
        ADD_POINT(new_car_x, new_car_y);
    }
    #undef ADD_POINT

    // 打印总点数
    char dbg[64];
    sprintf(dbg, "Total points: %d\n", point_count);
    wireless_uart_send_string(dbg);

    int n = (point_count < max_len) ? point_count : max_len;
    for (int i = 0; i < n; i++) {
        out_x[i] = points_x[i];
        out_y[i] = points_y[i];
    }
    return n;
}

int sokoban_plan_for_box(GameState* state, GridMap* grid_map, int box_id,
                         float car_x, float car_y,
                         float* out_path_x, float* out_path_y, int max_path_len) {
    if (box_id < 0 || box_id >= state->num_boxes) return -1;
    Box* box = &state->boxes[box_id];
    if (box->dest_id < 0) return -1;

    int actions[1000];
    int action_count = light_sokoban_plan(state, grid_map, box_id, car_x, car_y, actions, 1000);
    if (action_count <= 0) return -1;

    // 转换为推动动作编码
    for (int i = 0; i < action_count; i++) {
        actions[i] += 4;
    }

    float tmp_x[MAX_PATH_POINTS], tmp_y[MAX_PATH_POINTS];
    int tmp_n = actions_to_world_path(state, grid_map, box_id, car_x, car_y,
                                       actions, action_count,
                                       tmp_x, tmp_y, MAX_PATH_POINTS);
    if (tmp_n <= 0) return -1;

    // 直接输出原始路径点，不进行任何简化
    int n = (tmp_n < max_path_len) ? tmp_n : max_path_len;
    for (int i = 0; i < n; i++) {
        out_path_x[i] = tmp_x[i];
        out_path_y[i] = tmp_y[i];
    }
    return n;
}

void build_single_box_submap(GameState* state, GridMap* grid_map,
                             int box_id, CoarseMap* submap) {
    memset(submap->cells, 0, sizeof(submap->cells));

    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            int base_x = c * 4;
            int base_y = r * 4;
            int has_wall = 0;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    int fx = base_x + dx;
                    int fy = base_y + dy;
                    if (grid_map->occupancy[fy][fx] == OCC_WALL) {
                        has_wall = 1; break;
                    }
                }
                if (has_wall) break;
            }
            if (has_wall) submap->cells[r][c] = 1;
        }
    }

    for (int i = 0; i < state->num_boxes; i++) {
        if (i == box_id) continue;
        if (state->boxes[i].state == 0) {
            int br = (int)(state->boxes[i].y / CELL_SIZE);
            int bc = (int)(state->boxes[i].x / CELL_SIZE);
            if (br >= 0 && br < MAP_ROWS && bc >= 0 && bc < MAP_COLS)
                submap->cells[br][bc] = 1;
        }
    }

    int br = (int)(state->boxes[box_id].y / CELL_SIZE);
    int bc = (int)(state->boxes[box_id].x / CELL_SIZE);
    if (br >= 0 && br < MAP_ROWS && bc >= 0 && bc < MAP_COLS) submap->cells[br][bc] = 2;

    int dest_id = state->boxes[box_id].dest_id;
    if (dest_id >= 0 && dest_id < state->num_destinations) {
        int dr = (int)(state->destinations[dest_id].y / CELL_SIZE);
        int dc = (int)(state->destinations[dest_id].x / CELL_SIZE);
        if (dr >= 0 && dr < MAP_ROWS && dc >= 0 && dc < MAP_COLS) submap->cells[dr][dc] = 3;
    }
}

// ================= 轻量级推箱子规划器（分解法） =================

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

            // 对角线移动：检查两个相邻格子，只要不全被阻隔即可通过
            if (dx[d] != 0 && dy[d] != 0) {
                uint8_t occ1 = map->occupancy[y][nx];
                uint8_t occ2 = map->occupancy[ny][x];
                if ((occ1 == OCC_WALL || occ1 == OCC_BOX) &&
                    (occ2 == OCC_WALL || occ2 == OCC_BOX)) {
                    continue; // 两个方向都被堵，不能斜穿
                }
            }
            // 检查目标格子本身
            if (map->occupancy[ny][nx] == OCC_WALL || map->occupancy[ny][nx] == OCC_BOX) continue;
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
    // 入口打印（使用整数，坐标放大1000倍保留精度）
    rt_sprintf(buf, "light_sokoban_plan: entry, box_id=%d, car=(%d,%d)\r\n",
               box_id, (int)(car_x * 1000), (int)(car_y * 1000));
    wireless_uart_send_string(buf);

    if (box_id < 0 || box_id >= state->num_boxes) return -1;
    Box* box = &state->boxes[box_id];
    if (box->dest_id < 0) return -1;

    // 箱子和目标点运动坐标 -> 图像坐标
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

    // 边界限幅
    if (br < 0) br = 0; if (br >= MAP_ROWS) br = MAP_ROWS - 1;
    if (bc < 0) bc = 0; if (bc >= MAP_COLS) bc = MAP_COLS - 1;
    if (pr < 0) pr = 0; if (pr >= MAP_ROWS) pr = MAP_ROWS - 1;
    if (pc < 0) pc = 0; if (pc >= MAP_COLS) pc = MAP_COLS - 1;
    if (goal_br < 0) goal_br = 0; if (goal_br >= MAP_ROWS) goal_br = MAP_ROWS - 1;
    if (goal_bc < 0) goal_bc = 0; if (goal_bc >= MAP_COLS) goal_bc = MAP_COLS - 1;

    // 新增：打印箱子和目标点的粗网格索引
    rt_sprintf(buf, "light_sokoban_plan: box_grid=(%d,%d) goal_grid=(%d,%d)\r\n",
               br, bc, goal_br, goal_bc);
    wireless_uart_send_string(buf);

    // 构建障碍物地图（粗网格）
    uint8_t obstacle[MAP_ROWS][MAP_COLS] = {{0}};
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
    // 其他箱子（忽略当前箱子）
    for (int i = 0; i < state->num_boxes; i++) {
        if (i == box_id) continue;
        if (state->boxes[i].state == 0) {
            float other_img_x, other_img_y;
            motion_to_image(state->boxes[i].x, state->boxes[i].y, &other_img_x, &other_img_y);
            int or_ = (int)(other_img_y / CELL_SIZE);
            int oc = (int)(other_img_x / CELL_SIZE);
            if (or_ == goal_br && oc == goal_bc) continue;
            if (or_ >= 0 && or_ < MAP_ROWS && oc >= 0 && oc < MAP_COLS)
                obstacle[or_][oc] = 1;
        }
    }

    // 新增：打印箱子周围 3x3 区域的 obstacle 值
    wireless_uart_send_string("Obstacle around box (3x3):\r\n");
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            int r = br + dr, c = bc + dc;
            if (r >= 0 && r < MAP_ROWS && c >= 0 && c < MAP_COLS) {
                rt_sprintf(buf, "%d ", obstacle[r][c]);
            } else {
                rt_sprintf(buf, ". ");
            }
            wireless_uart_send_string(buf);
        }
        wireless_uart_send_string("\r\n");
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

    static LightNode queue[500];
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
        // 新增：找到目标时打印
        if (cur.br == goal_br && cur.bc == goal_bc) {
            wireless_uart_send_string("BFS found goal!\r\n");
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

            // 死锁检测
            if (is_deadlock(nbr, nbc, goal_br, goal_bc, obstacle)) continue;

            int sx = cur.pc * 4 + 2;
            int sy = cur.pr * 4 + 2;
            int gx = push_c * 4 + 2;
            int gy = push_r * 4 + 2;
            if (!can_reach_fine(grid_map, sx, sy, gx, gy)) continue;

            int new_pushes = cur.pushes + 1;
            if (best_pushes[nbr][nbc] == -1 || new_pushes < best_pushes[nbr][nbc]) {
                best_pushes[nbr][nbc] = new_pushes;
                if (tail >= 500) continue;
                queue[tail].br = nbr;
                queue[tail].bc = nbc;
                queue[tail].pr = cur.br;
                queue[tail].pc = cur.bc;
                queue[tail].pushes = new_pushes;
                queue[tail].parent = head;
                queue[tail].action = 4 + d;
                tail++;
            }
        }
        head++;
    }

    // 新增：BFS 结束未找到时打印
    if (found_idx == -1) {
        wireless_uart_send_string("BFS failed to find goal.\r\n");
        return -1;
    }

    // 回溯动作序列
    int actions[500];
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
    rt_sprintf(buf, "light_sokoban_plan: success, actions=%d\r\n", n);
    wireless_uart_send_string(buf);
    return n;
}