#include "planner.h"
#include <math.h>
#include "position.h"
#include "encoder.h"
// 计算点到线段的最短距离（内部函数）
static float point_to_segment_distance(float px, float py, float x1, float y1, float x2, float y2) {
    float vx = x2 - x1;
    float vy = y2 - y1;
    float wx = px - x1;
    float wy = py - y1;

    float c1 = vx * wx + vy * wy;
    if (c1 <= 0.0f) {
        return sqrtf((px - x1)*(px - x1) + (py - y1)*(py - y1));
    }

    float c2 = vx * vx + vy * vy;
    if (c2 <= 1e-9f) {
        return sqrtf((px - x1)*(px - x1) + (py - y1)*(py - y1));
    }

    float t = c1 / c2;
    if (t >= 1.0f) {
        return sqrtf((px - x2)*(px - x2) + (py - y2)*(py - y2));
    }

    float proj_x = x1 + t * vx;
    float proj_y = y1 + t * vy;
    return sqrtf((px - proj_x)*(px - proj_x) + (py - proj_y)*(py - proj_y));
}


static float point_to_rectangle_distance(float px, float py, float x1, float y1, float x2, float y2) {
    float dx = 0.0f, dy = 0.0f;
    if (px < x1) dx = x1 - px;
    else if (px > x2) dx = px - x2;
    if (py < y1) dy = y1 - py;
    else if (py > y2) dy = py - y2;
    return sqrtf(dx*dx + dy*dy);
}

static void clear_wall_from_grid(GridMap* map, Wall* wall) {
    int col = (int)(wall->x1 / CELL_SIZE + 0.5f);
    int row = (int)(wall->y1 / CELL_SIZE + 0.5f);
    int base_x = col * 4;
    int base_y = row * 4;
    for (int dy = 0; dy < 4; dy++) {
        for (int dx = 0; dx < 4; dx++) {
            int fx = base_x + dx;
            int fy = base_y + dy;
            if (fx >= 0 && fx < map->width && fy >= 0 && fy < map->height) {
                map->occupancy[fy][fx] = OCC_FREE;
            }
        }
    }
}

// 世界坐标转细网格坐标（四舍五入）
void world_to_grid(float wx, float wy, int* gx, int* gy) {
    *gx = (int)roundf(wx / RESOLUTION);
    *gy = (int)roundf(wy / RESOLUTION);
    if (*gx < 0) *gx = 0;
    if (*gx >= FINE_COLS) *gx = FINE_COLS - 1;
    if (*gy < 0) *gy = 0;
    if (*gy >= FINE_ROWS) *gy = FINE_ROWS - 1;
}

void grid_to_world(int gx, int gy, float* wx, float* wy) {
    *wx = gx * RESOLUTION;
    *wy = gy * RESOLUTION;
}

/**
 * 从文本地图加载
 */
void load_map_from_text(const char* map_text, GridMap* grid_map, GameState* state) {
    // 清空网格
    memset(grid_map->occupancy, OCC_FREE, sizeof(grid_map->occupancy));
    grid_map->width = FINE_COLS;
    grid_map->height = FINE_ROWS;

    state->num_boxes = 0;
    state->num_destinations = 0;
    state->num_bombs = 0;
    state->num_walls = 0;

    const char* p = map_text;
    for (int row = 0; row < MAP_ROWS; row++) {
        for (int col = 0; col < MAP_COLS; col++) {
            while (*p == '\n' || *p == '\r') p++;
            char ch = *p++;

            // 图像坐标（用于标记 occupancy 和生成运动坐标）
            float img_x = (col + 0.5f) * CELL_SIZE;
            float img_y = (row + 0.5f) * CELL_SIZE;
            // 运动坐标（通过标准转换函数获取，现为恒等映射）
            float motion_x, motion_y;
            image_to_motion(img_x, img_y, &motion_x, &motion_y);

            // 细网格范围（基于图像坐标的行列）
            int base_x = col * 4;
            int base_y = row * 4;

            if (ch == '#') {
                // 墙体：存储为运动坐标矩形
                if (state->num_walls < MAX_WALLS) {
                    Wall* w = &state->walls[state->num_walls];
                    // 墙体左上角图像坐标
                    float wall_img_x1 = col * CELL_SIZE;
                    float wall_img_y1 = row * CELL_SIZE;
                    float wall_img_x2 = (col + 1) * CELL_SIZE;
                    float wall_img_y2 = (row + 1) * CELL_SIZE;
                    // 转换为运动坐标存储
                    image_to_motion(wall_img_x1, wall_img_y1, &w->x1, &w->y1);
                    image_to_motion(wall_img_x2, wall_img_y2, &w->x2, &w->y2);
                    state->num_walls++;
                }
                // 标记细网格为墙体（使用图像坐标）
                for (int dy = 0; dy < 4; dy++) {
                    for (int dx = 0; dx < 4; dx++) {
                        int fx = base_x + dx;
                        int fy = base_y + dy;
                        if (fx < FINE_COLS && fy < FINE_ROWS) {
                            grid_map->occupancy[fy][fx] = OCC_WALL;
                        }
                    }
                }
            } else if (ch == '$') {
                // 箱子
                if (state->num_boxes < MAX_BOXES) {
                    Box* b = &state->boxes[state->num_boxes];
                    b->x = motion_x;
                    b->y = motion_y;
                    // 计算 grid 坐标（基于图像坐标）
                    float img_for_grid_x, img_for_grid_y;
                    motion_to_image(b->x, b->y, &img_for_grid_x, &img_for_grid_y);
                    world_to_grid(img_for_grid_x, img_for_grid_y, &b->grid_x, &b->grid_y);
                    b->state = 0;
                    b->dest_id = -1;
                    b->type = BOX_TYPE_UNKNOWN;
                    state->num_boxes++;
                }
                // 标记细网格为箱子（使用图像坐标）
                for (int dy = 0; dy < 4; dy++) {
                    for (int dx = 0; dx < 4; dx++) {
                        int fx = base_x + dx;
                        int fy = base_y + dy;
                        if (fx < FINE_COLS && fy < FINE_ROWS) {
                            grid_map->occupancy[fy][fx] = OCC_BOX;
                        }
                    }
                }
            } else if (ch == '.') {
                // 目的地
                if (state->num_destinations < MAX_DESTINATIONS) {
                    Destination* d = &state->destinations[state->num_destinations];
                    d->x = motion_x;
                    d->y = motion_y;
                    float img_for_grid_x, img_for_grid_y;
                    motion_to_image(d->x, d->y, &img_for_grid_x, &img_for_grid_y);
                    world_to_grid(img_for_grid_x, img_for_grid_y, &d->grid_x, &d->grid_y);
                    d->assigned_box_id = -1;
                    d->required_digit = 0;
                    state->num_destinations++;
                }
                // 标记细网格为目的地（使用图像坐标）
                for (int dy = 0; dy < 4; dy++) {
                    for (int dx = 0; dx < 4; dx++) {
                        int fx = base_x + dx;
                        int fy = base_y + dy;
                        if (fx < FINE_COLS && fy < FINE_ROWS) {
                            grid_map->occupancy[fy][fx] = OCC_DEST;
                        }
                    }
                }
            } else if (ch == '*') {
                // 炸弹
                if (state->num_bombs < MAX_BOMBS) {
                    Bomb* b = &state->bombs[state->num_bombs];
                    b->x = motion_x;
                    b->y = motion_y;
                    float img_for_grid_x, img_for_grid_y;
                    motion_to_image(b->x, b->y, &img_for_grid_x, &img_for_grid_y);
                    world_to_grid(img_for_grid_x, img_for_grid_y, &b->grid_x, &b->grid_y);
                    b->active = 1;
                    b->blast_radius = 0.3f;
                    b->target_id = -1;
                    state->num_bombs++;
                }
                // 标记细网格为炸弹（障碍物）
                for (int dy = 0; dy < 4; dy++)
                    for (int dx = 0; dx < 4; dx++) {
                        int fx = base_x + dx;
                        int fy = base_y + dy;
                        if (fx < FINE_COLS && fy < FINE_ROWS)
                            grid_map->occupancy[fy][fx] = OCC_BOMB;
                    }
            } else if (ch == 'P') {
                // 小车位置
                float img_x_car = (col + 0.5f) * CELL_SIZE;
                float img_y_car = (row + 0.5f) * CELL_SIZE;
                float motion_x_car, motion_y_car;
                image_to_motion(img_x_car, img_y_car, &motion_x_car, &motion_y_car);
                Position_Set(motion_x_car, motion_y_car, 0.0f);
                EncoderReset();
                // P 所在网格为空地，无其他标记
            } else if (ch == '-') {
                // 空地，已经默认为 OCC_FREE，无需额外标记
            }
        }
    }

    // 生成代价地图（使用更新后的 occupancy）
    create_inflated_cost_map(grid_map, state, 2.0f);
}

/**
* 从物体坐标列表加载地图（用于OpenART摄像头数据）
 * 数据格式：
 *   field_width, field_height: 场地尺寸（米）
 *   walls: 数组，每4个float为一组 (x1,y1,x2,y2)
 *   boxes: 数组，每2个float为一组 (x,y)
 *   dests: 数组，每2个float为一组 (x,y)
 *   bombs: 数组，每2个float为一组 (x,y)
 * 所有坐标均为世界坐标，墙体为矩形，其他为中心点。
 */
void load_map_from_objects(GridMap* grid_map, GameState* state,
                           float field_width, float field_height,
                           const float* walls, int num_walls,
                           const float* boxes, int num_boxes,
                           const float* dests, int num_dests,
                           const float* bombs, int num_bombs) {
    // 初始化网格地图（全部设为空地）
    memset(grid_map->occupancy, OCC_FREE, sizeof(grid_map->occupancy));
    grid_map->width = FINE_COLS;
    grid_map->height = FINE_ROWS;

    // 清空游戏状态
    state->num_boxes = 0;
    state->num_destinations = 0;
    state->num_bombs = 0;
    state->num_walls = 0;

    // ========== 1. 处理墙体 ==========
    for (int i = 0; i < num_walls; i++) {
        float x1 = walls[i*4 + 0];
        float y1 = walls[i*4 + 1];
        float x2 = walls[i*4 + 2];
        float y2 = walls[i*4 + 3];
        // 计算墙体覆盖的粗网格范围
        int c_min = (int)(x1 / CELL_SIZE);
        int c_max = (int)((x2 - 1e-4) / CELL_SIZE);
        int r_min = (int)(y1 / CELL_SIZE);
        int r_max = (int)((y2 - 1e-4) / CELL_SIZE);
        if (c_min < 0) c_min = 0;
        if (c_max >= MAP_COLS) c_max = MAP_COLS - 1;
        if (r_min < 0) r_min = 0;
        if (r_max >= MAP_ROWS) r_max = MAP_ROWS - 1;
        for (int r = r_min; r <= r_max; r++) {
            for (int c = c_min; c <= c_max; c++) {
                // 添加墙体记录
                if (state->num_walls < MAX_WALLS) {
                    Wall* w = &state->walls[state->num_walls];
                    w->x1 = c * CELL_SIZE;
                    w->y1 = r * CELL_SIZE;
                    w->x2 = (c + 1) * CELL_SIZE;
                    w->y2 = (r + 1) * CELL_SIZE;
                    state->num_walls++;
                }
                // 标记细网格为墙体
                int base_x = c * 4;
                int base_y = r * 4;
                for (int dy = 0; dy < 4; dy++) {
                    for (int dx = 0; dx < 4; dx++) {
                        int fx = base_x + dx;
                        int fy = base_y + dy;
                        if (fx < FINE_COLS && fy < FINE_ROWS) {
                            grid_map->occupancy[fy][fx] = OCC_WALL;
                        }
                    }
                }
            }
        }
    }

    // ========== 2. 处理箱子 ========== 
    for (int i = 0; i < num_boxes; i++) {
        float x = boxes[i*2 + 0];
        float y = boxes[i*2 + 1];
        int r = (int)(y / CELL_SIZE);
        int c = (int)(x / CELL_SIZE);
        if (r < 0 || r >= MAP_ROWS || c < 0 || c >= MAP_COLS) continue;

        if (state->num_boxes < MAX_BOXES) {
            Box* b = &state->boxes[state->num_boxes];
            b->x = x;
            b->y = y;
            world_to_grid(b->x, b->y, &b->grid_x, &b->grid_y);
            b->state = 0;
            b->dest_id = -1;
            b->type = BOX_TYPE_UNKNOWN;
            state->num_boxes++;
        }

        int base_x = c * 4;
        int base_y = r * 4;
        for (int dy = 0; dy < 4; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                int fx = base_x + dx;
                int fy = base_y + dy;
                if (fx < FINE_COLS && fy < FINE_ROWS) {
                    grid_map->occupancy[fy][fx] = OCC_BOX;
                }
            }
        }
    }

    // ========== 3. 处理目的地，并检查与箱子的重叠 ========== 
    for (int i = 0; i < num_dests; i++) {
        float x = dests[i*2 + 0];
        float y = dests[i*2 + 1];
        int r = (int)(y / CELL_SIZE);
        int c = (int)(x / CELL_SIZE);
        if (r < 0 || r >= MAP_ROWS || c < 0 || c >= MAP_COLS) continue;

        int base_x = c * 4;
        int base_y = r * 4;
        int has_box = 0;
        for (int dy = 0; dy < 4 && !has_box; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                int fx = base_x + dx;
                int fy = base_y + dy;
                if (fx < FINE_COLS && fy < FINE_ROWS &&
                    grid_map->occupancy[fy][fx] == OCC_BOX) {
                    has_box = 1;
                    break;
                }
            }
        }

        if (has_box) {
            for (int j = 0; j < state->num_boxes; j++) {
                int br = (int)(state->boxes[j].y / CELL_SIZE);
                int bc = (int)(state->boxes[j].x / CELL_SIZE);
                if (br == r && bc == c) {
                    state->boxes[j].state = 1;
                    state->boxes[j].dest_id = -1;
                    break;
                }
            }
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    int fx = base_x + dx;
                    int fy = base_y + dy;
                    if (fx < FINE_COLS && fy < FINE_ROWS) {
                        grid_map->occupancy[fy][fx] = OCC_FREE;
                    }
                }
            }
            continue;
        }

        if (state->num_destinations < MAX_DESTINATIONS) {
            Destination* d = &state->destinations[state->num_destinations];
            d->x = x;
            d->y = y;
            world_to_grid(d->x, d->y, &d->grid_x, &d->grid_y);
            d->assigned_box_id = -1;
            d->required_digit = 0;
            state->num_destinations++;
        }

        for (int dy = 0; dy < 4; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                int fx = base_x + dx;
                int fy = base_y + dy;
                if (fx < FINE_COLS && fy < FINE_ROWS) {
                    grid_map->occupancy[fy][fx] = OCC_DEST;
                }
            }
        }
    }

    // ========== 4. 处理炸弹（仅记录状态，不修改占据网格） ========== 
    for (int i = 0; i < num_bombs; i++) {
        float x = bombs[i*2 + 0];
        float y = bombs[i*2 + 1];
        int r = (int)(y / CELL_SIZE);
        int c = (int)(x / CELL_SIZE);
        if (r < 0 || r >= MAP_ROWS || c < 0 || c >= MAP_COLS) continue;
        if (state->num_bombs < MAX_BOMBS) {
            Bomb* b = &state->bombs[state->num_bombs];
            b->x = x;
            b->y = y;
            world_to_grid(b->x, b->y, &b->grid_x, &b->grid_y);
            b->active = 1;
            b->blast_radius = 0.3f;
            b->target_id = -1;
            state->num_bombs++;
        }
    }

    // ========== 5. 生成代价地图 ==========
    create_inflated_cost_map(grid_map, state, 2.0f);
}

/**
 * 创建膨胀代价地图（原有函数）
 */
void create_inflated_cost_map(GridMap* map, GameState* state, float inflation_radius) {
    if (inflation_radius < 3.0f) inflation_radius = 3.0f;
    int radius = (int)inflation_radius;
    for (int y = 0; y < map->height; y++)
        for (int x = 0; x < map->width; x++)
            map->cost_map[y][x] = 1.0f;

    // 障碍物膨胀
    for (int y = 0; y < map->height; y++) {
        for (int x = 0; x < map->width; x++) {
            if (map->occupancy[y][x] == OCC_WALL) {
                for (int dy = -radius; dy <= radius; dy++) {
                    for (int dx = -radius; dx <= radius; dx++) {
                        int ny = y + dy;
                        int nx = x + dx;
                        if (nx >= 0 && nx < map->width && ny >= 0 && ny < map->height) {
                            float dist = sqrtf(dx*dx + dy*dy);
                            if (dist <= radius) {
                                float cost_inc = 3.0f * (1.0f - dist / radius);
                                if (map->cost_map[ny][nx] < 1.0f + cost_inc)
                                    map->cost_map[ny][nx] = 1.0f + cost_inc;
                            }
                        }
                    }
                }
            }
        }
    }

    // 箱子代价略高
    for (int y = 0; y < map->height; y++) {
        for (int x = 0; x < map->width; x++) {
            if (map->occupancy[y][x] == OCC_BOX)
                map->cost_map[y][x] = fmaxf(map->cost_map[y][x], 1.2f);
            else if (map->occupancy[y][x] == OCC_DEST)
                map->cost_map[y][x] = fminf(map->cost_map[y][x], 0.8f);
        }
    }

    // 炸弹区域设置高代价
    for (int i = 0; i < state->num_bombs; i++) {
        Bomb* bomb = &state->bombs[i];
        if (bomb->active) {
            int bx = bomb->grid_x;
            int by = bomb->grid_y;
            int bomb_radius = (int)(bomb->blast_radius / RESOLUTION);
            for (int dy = -bomb_radius; dy <= bomb_radius; dy++) {
                for (int dx = -bomb_radius; dx <= bomb_radius; dx++) {
                    int nx = bx + dx;
                    int ny = by + dy;
                    if (nx >= 0 && nx < map->width && ny >= 0 && ny < map->height) {
                        float dist = sqrtf(dx*dx + dy*dy) * RESOLUTION;
                        if (dist <= bomb->blast_radius) {
                            map->cost_map[ny][nx] = fmaxf(map->cost_map[ny][nx], 10.0f);
                        }
                    }
                }
            }
        }
    }

    // 粗网格中心偏好：使A*偏向通道中央，减少贴墙
    for (int y = 0; y < map->height; y++) {
        for (int x = 0; x < map->width; x++) {
            float dx = (float)(x % 4) - 2.0f;
            float dy = (float)(y % 4) - 2.0f;
            map->cost_map[y][x] += 0.1f * (dx*dx + dy*dy);
        }
    }
}

/**
 * 炸弹爆炸
 */
void explode_bomb(GameState* state, GridMap* map, int bomb_id) {
    if (bomb_id < 0 || bomb_id >= state->num_bombs) return;
    Bomb* bomb = &state->bombs[bomb_id];
    if (!bomb->active) return;

    float cx = bomb->x;
    float cy = bomb->y;
    float r = bomb->blast_radius;

    int kept = 0;
    for (int i = 0; i < state->num_walls; i++) {
        Wall* w = &state->walls[i];
        float dist = point_to_rectangle_distance(cx, cy, w->x1, w->y1, w->x2, w->y2);
        if (dist > r) {
            if (kept != i) {
                state->walls[kept] = state->walls[i];
            }
            kept++;
        } else {
            clear_wall_from_grid(map, w);
        }
    }
    state->num_walls = kept;

    // 清除炸弹自身占据格
    {
        int col = (int)(bomb->x / CELL_SIZE);
        int row = (int)(bomb->y / CELL_SIZE);
        int base_x = col * 4, base_y = row * 4;
        for (int dy = 0; dy < 4; dy++)
            for (int dx = 0; dx < 4; dx++) {
                int fx = base_x + dx, fy = base_y + dy;
                if (fx < map->width && fy < map->height)
                    map->occupancy[fy][fx] = OCC_FREE;
            }
    }
    bomb->active = 0;
    create_inflated_cost_map(map, state, 2.0f);
}

/**
 * 刷新网格地图（根据当前游戏状态重新生成占据网格）
 */
void refresh_grid_map(GameState* state, GridMap* map) {
    // 全部设为空地
    for (int y = 0; y < map->height; y++) {
        for (int x = 0; x < map->width; x++) {
            map->occupancy[y][x] = OCC_FREE;
        }
    }

    // 标记墙体（物体坐标即图像坐标）
    for (int i = 0; i < state->num_walls; i++) {
        Wall* w = &state->walls[i];
        // 直接使用运动坐标计算粗网格行列
        int col = (int)(w->x1 / CELL_SIZE);
        int row = (int)(w->y1 / CELL_SIZE);
        int base_x = col * 4;
        int base_y = row * 4;
        for (int dy = 0; dy < 4; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                int fx = base_x + dx;
                int fy = base_y + dy;
                if (fx >= 0 && fx < map->width && fy >= 0 && fy < map->height) {
                    map->occupancy[fy][fx] = OCC_WALL;
                }
            }
        }
    }

    // 标记未完成的箱子
    for (int i = 0; i < state->num_boxes; i++) {
        if (state->boxes[i].state == 0) {
            int col = (int)(state->boxes[i].x / CELL_SIZE);
            int row = (int)(state->boxes[i].y / CELL_SIZE);
            int base_x = col * 4;
            int base_y = row * 4;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    int fx = base_x + dx;
                    int fy = base_y + dy;
                    if (fx >= 0 && fx < map->width && fy >= 0 && fy < map->height) {
                        map->occupancy[fy][fx] = OCC_BOX;
                    }
                }
            }
        }
    }

    // 标记活跃炸弹
    for (int i = 0; i < state->num_bombs; i++) {
        if (state->bombs[i].active) {
            int col = (int)(state->bombs[i].x / CELL_SIZE);
            int row = (int)(state->bombs[i].y / CELL_SIZE);
            int base_x = col * 4;
            int base_y = row * 4;
            for (int dy = 0; dy < 4; dy++)
                for (int dx = 0; dx < 4; dx++) {
                    int fx = base_x + dx;
                    int fy = base_y + dy;
                    if (fx >= 0 && fx < map->width && fy >= 0 && fy < map->height)
                        map->occupancy[fy][fx] = OCC_BOMB;
                }
        }
    }

    // 标记目的地（跳过已到达的）
    for (int i = 0; i < state->num_destinations; i++) {
        Destination* dest = &state->destinations[i];
        if (dest->assigned_box_id >= 0) {
            Box* box = &state->boxes[dest->assigned_box_id];
            if (box->state == 1) continue;   // 箱子已到达，目的地消失
        }
        int col = (int)(dest->x / CELL_SIZE);
        int row = (int)(dest->y / CELL_SIZE);
        int base_x = col * 4;
        int base_y = row * 4;
        for (int dy = 0; dy < 4; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                int fx = base_x + dx;
                int fy = base_y + dy;
                if (fx >= 0 && fx < map->width && fy >= 0 && fy < map->height) {
                    map->occupancy[fy][fx] = OCC_DEST;
                }
            }
        }
    }

    // 生成代价地图
    create_inflated_cost_map(map, state, 0.12f);  // 建议增大膨胀半径

    // 二次强制标记目的地，防止被其他物体覆盖
    for (int i = 0; i < state->num_destinations; i++) {
        Destination* dest = &state->destinations[i];
        if (dest->assigned_box_id >= 0 && state->boxes[dest->assigned_box_id].state == 1) {
            continue;
        }
        int col = (int)(dest->x / CELL_SIZE);
        int row = (int)(dest->y / CELL_SIZE);
        int base_x = col * 4;
        int base_y = row * 4;
        for (int dy = 0; dy < 4; dy++) {
            for (int dx = 0; dx < 4; dx++) {
                int fx = base_x + dx;
                int fy = base_y + dy;
                if (fx >= 0 && fx < map->width && fy >= 0 && fy < map->height) {
                    map->occupancy[fy][fx] = OCC_DEST;
                }
            }
        }
    }
}
void motion_to_image(float mx, float my, float* wx, float* wy) {
    *wx = mx;   
    *wy = my;   
}

void image_to_motion(float wx, float wy, float* mx, float* my) {
    *mx = wx;   
    *my = wy;   
}