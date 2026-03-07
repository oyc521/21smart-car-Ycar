#include "planner.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// 静态数组代替动态分配
static float g_score[FINE_ROWS * FINE_COLS];
static int parent[FINE_ROWS * FINE_COLS];
static bool closed[FINE_ROWS * FINE_COLS];

// 节点优先级队列（最小堆）- 静态实现
typedef struct {
    int x, y;
    float f;
} HeapNode;

static HeapNode heap_nodes[FINE_ROWS * FINE_COLS];
static int heap_size;

static void heap_push(int x, int y, float f) {
    if (heap_size >= FINE_ROWS * FINE_COLS) return;
    int i = heap_size++;
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (heap_nodes[parent].f <= f) break;
        heap_nodes[i] = heap_nodes[parent];
        i = parent;
    }
    heap_nodes[i].x = x;
    heap_nodes[i].y = y;
    heap_nodes[i].f = f;
}

static HeapNode heap_pop(void) {
    HeapNode top = heap_nodes[0];
    heap_size--;
    if (heap_size > 0) {
        HeapNode last = heap_nodes[heap_size];
        int i = 0;
        while (1) {
            int left = 2*i + 1;
            int right = 2*i + 2;
            int smallest = i;
            if (left < heap_size && heap_nodes[left].f < last.f) smallest = left;
            if (right < heap_size && heap_nodes[right].f < heap_nodes[smallest].f) smallest = right;
            if (smallest == i) break;
            heap_nodes[i] = heap_nodes[smallest];
            i = smallest;
        }
        heap_nodes[i] = last;
    }
    return top;
}

// 启发式函数（对角线距离）
static float heuristic(int x1, int y1, int x2, int y2) {
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    return fmaxf(dx, dy) + (sqrtf(2) - 1) * fminf(dx, dy);
}

int astar_plan_path(GridMap* map, int start_x, int start_y, int goal_x, int goal_y,
                    int* out_path_x, int* out_path_y, int max_path_len, AStarParams* params) {
    const int width = map->width;
    const int height = map->height;

    // 检查起点终点有效性
    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
        goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
        return -1;
    }
    if (map->occupancy[start_y][start_x] == OCC_WALL ||
        map->occupancy[goal_y][goal_x] == OCC_WALL) {
        return -1;
    }

    // 初始化数组
    for (int i = 0; i < width * height; i++) {
        g_score[i] = FLT_MAX;
        parent[i] = -1;
        closed[i] = false;
    }

    int start_idx = start_y * width + start_x;
    g_score[start_idx] = 0;
    float f_start = heuristic(start_x, start_y, goal_x, goal_y);

    heap_size = 0;
    heap_push(start_x, start_y, f_start);

    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const float move_cost[8] = {sqrtf(2), 1, sqrtf(2), 1, 1, sqrtf(2), 1, sqrtf(2)};

    int iterations = 0;
    int max_iter = (params) ? params->max_iterations : 5000;
    int found = 0;
    int goal_idx = goal_y * width + goal_x;

    while (heap_size > 0 && iterations < max_iter) {
        iterations++;
        HeapNode current = heap_pop();
        int cx = current.x;
        int cy = current.y;
        int idx = cy * width + cx;

        if (closed[idx]) continue;
        closed[idx] = true;

        if (cx == goal_x && cy == goal_y) {
            found = 1;
            break;
        }

        for (int d = 0; d < 8; d++) {
            int nx = cx + dx[d];
            int ny = cy + dy[d];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            int nidx = ny * width + nx;
            if (closed[nidx]) continue;
            if (map->occupancy[ny][nx] == OCC_WALL) continue;
            if (!(nx == goal_x && ny == goal_y) && map->occupancy[ny][nx] == OCC_BOX) continue;

            float tentative_g = g_score[idx] + move_cost[d] * map->cost_map[ny][nx];
            if (tentative_g < g_score[nidx]) {
                g_score[nidx] = tentative_g;
                parent[nidx] = idx;
                float f = tentative_g + heuristic(nx, ny, goal_x, goal_y);
                heap_push(nx, ny, f);
            }
        }
    }

    int path_len = 0;
    if (found) {
        int cur = goal_idx;
        while (cur != -1) {
            int x = cur % width;
            int y = cur / width;
            if (path_len < max_path_len) {
                out_path_x[path_len] = x;
                out_path_y[path_len] = y;
                path_len++;
            }
            cur = parent[cur];
        }
        for (int i = 0; i < path_len / 2; i++) {
            int j = path_len - 1 - i;
            int tx = out_path_x[i]; out_path_x[i] = out_path_x[j]; out_path_x[j] = tx;
            int ty = out_path_y[i]; out_path_y[i] = out_path_y[j]; out_path_y[j] = ty;
        }
    }

    return found ? path_len : -1;
}