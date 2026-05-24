#include "planner.h"
#include <math.h>
#include <float.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>   // 提供 abs()

// 确保 MAX_PATH_POINTS 有定义（与 hybrid_controller.h 保持一致）
#ifndef MAX_PATH_POINTS
#define MAX_PATH_POINTS 200
#endif

// ===== 将大数组放入 OCRAM，释放 DTCM =====
__attribute__((section("OCRAM_CACHE"))) static float g_score[FINE_ROWS * FINE_COLS];
__attribute__((section("OCRAM_CACHE"))) static int parent[FINE_ROWS * FINE_COLS];
__attribute__((section("OCRAM_CACHE"))) static bool closed[FINE_ROWS * FINE_COLS];

typedef struct {
    int x, y;
    float f;
} HeapNode;

__attribute__((section("OCRAM_CACHE"))) static HeapNode heap_nodes[FINE_ROWS * FINE_COLS];
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

static float heuristic(int x1, int y1, int x2, int y2) {
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    return fmaxf(dx, dy) + (sqrtf(2) - 1) * fminf(dx, dy);
}

// 限制单次最大跳跃步数（细网格），防止长直线剐蹭墙壁
#define MAX_STRAIGHTEN_STEPS 8

static int line_of_sight_grid(GridMap* map, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int x = x0, y = y0;

    // Car body covers 4x4 fine cells (center ±2 in -2..+1 range)
    while (1) {
        if (!(x == x0 && y == y0) && !(x == x1 && y == y1)) {
            for (int dy = -2; dy <= 1; dy++)
                for (int dx = -2; dx <= 1; dx++) {
                    int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < map->width && ny >= 0 && ny < map->height) {
                        uint8_t occ = map->occupancy[ny][nx];
                        if (occ == OCC_WALL || occ == OCC_BOX || occ == OCC_BOMB)
                            return 0;
                    }
                }
        }
        if (x == x1 && y == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
    return 1;
}

static int greedy_straighten(GridMap* map, int* path_x, int* path_y, int len, int max_len) {
    if (len <= 2) return len;

    int kept_x[MAX_PATH_POINTS], kept_y[MAX_PATH_POINTS];
    int kept_len = 0;

    kept_x[kept_len] = path_x[0];
    kept_y[kept_len] = path_y[0];
    kept_len++;

    int last_idx = 0;
    while (last_idx < len - 1) {
        int next_idx = last_idx + 1;
        int max_i = last_idx + MAX_STRAIGHTEN_STEPS;
        if (max_i >= len) max_i = len - 1;
        for (int i = max_i; i > last_idx; i--) {
            if (line_of_sight_grid(map, path_x[last_idx], path_y[last_idx],
                                   path_x[i], path_y[i])) {
                next_idx = i;
                break;
            }
        }
        if (kept_len < max_len) {
            kept_x[kept_len] = path_x[next_idx];
            kept_y[kept_len] = path_y[next_idx];
            kept_len++;
        }
        last_idx = next_idx;
    }

    int out_len = kept_len < max_len ? kept_len : max_len;
    for (int i = 0; i < out_len; i++) {
        path_x[i] = kept_x[i];
        path_y[i] = kept_y[i];
    }
    return out_len;
}

int astar_plan_path(GridMap* map, int start_x, int start_y, int goal_x, int goal_y,
                    int* out_path_x, int* out_path_y, int max_path_len, AStarParams* params) {
    if (map && map->width > 30 && map->height > 2) {
        printf("[A*入口] map->occupancy[2][30]=%d, goal=(%d,%d) occ=%d\n",
               map->occupancy[2][30], goal_x, goal_y, map->occupancy[goal_y][goal_x]);
    }

    const int width = map->width;
    const int height = map->height;

    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
        goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
        return -1;
    }
    if (map->occupancy[start_y][start_x] == OCC_WALL ||
        map->occupancy[start_y][start_x] == OCC_BOMB ||
        map->occupancy[goal_y][goal_x] == OCC_WALL ||
        map->occupancy[goal_y][goal_x] == OCC_BOMB) {
        return -1;
    }

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

            if (map->occupancy[ny][nx] == OCC_WALL || map->occupancy[ny][nx] == OCC_BOMB) continue;
            if (!(nx == goal_x && ny == goal_y) && (map->occupancy[ny][nx] == OCC_BOX)) continue;

            if (dx[d] != 0 && dy[d] != 0) {
                if (map->occupancy[cy][nx] == OCC_WALL || map->occupancy[cy][nx] == OCC_BOX || map->occupancy[cy][nx] == OCC_BOMB)
                    continue;
                if (map->occupancy[ny][cx] == OCC_WALL || map->occupancy[ny][cx] == OCC_BOX || map->occupancy[ny][cx] == OCC_BOMB)
                    continue;
            }

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
				// 1. 先计算路径总长度
				int total_len = 0;
				int cur = goal_idx;
				while (cur != -1) {
						total_len++;
						cur = parent[cur];
				}

				// 2. 决定实际写入的点数
				int write_count = (total_len < max_path_len) ? total_len : max_path_len;
				int skip = (total_len > max_path_len) ? (total_len - max_path_len) : 0;

				// 3. 从 goal 回溯，跳过前面靠近 goal 的多余节点，保留靠近 start 的部分
				cur = goal_idx;
				for (int i = 0; i < skip; i++) {
						cur = parent[cur];
				}

				// 4. 写入 write_count 个节点（逆序：从 start 方向的第 write_count 个点到 start）
				int idx = 0;
				while (idx < write_count && cur != -1) {
						out_path_x[idx] = cur % width;
						out_path_y[idx] = cur / width;
						idx++;
						cur = parent[cur];
				}
				path_len = idx;   // = write_count

				// 5. 反转，使路径从起点开始
				for (int i = 0; i < path_len / 2; i++) {
						int j = path_len - 1 - i;
						int tx = out_path_x[i]; out_path_x[i] = out_path_x[j]; out_path_x[j] = tx;
						int ty = out_path_y[i]; out_path_y[i] = out_path_y[j]; out_path_y[j] = ty;
				}

				// 6. 平滑路径（内部只操作前 path_len 个元素，安全）
				path_len = greedy_straighten(map, out_path_x, out_path_y, path_len, max_path_len);
		}

		return found ? path_len : -1;
}