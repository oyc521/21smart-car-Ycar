#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>
#include <stdbool.h>

// 调试输出控制（设为0禁用调试输出）
#ifndef DEBUG
#define DEBUG 0
#endif

// 网格分辨率（细网格）
#define RESOLUTION 0.05f  // 米/格

// 粗网格尺寸（Sokoban使用）
#define CELL_SIZE 0.2f    // 米

// 地图尺寸（基于16x12文本）
#define MAP_COLS 16
#define MAP_ROWS 12

// 细网格尺寸
#define FINE_COLS (MAP_COLS * 4)
#define FINE_ROWS (MAP_ROWS * 4)

// 最大对象数量
#define MAX_BOXES 8
#define MAX_DESTINATIONS 8
#define MAX_BOMBS 8
#define MAX_WALLS 100

// 占据栅格值
#define OCC_FREE     0
#define OCC_WALL     1
#define OCC_BOX      2
#define OCC_DEST     3

// 方向
typedef enum {
    DIR_UP,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_UP_LEFT,
    DIR_UP_RIGHT,
    DIR_DOWN_LEFT,
    DIR_DOWN_RIGHT
} Direction;

// 多炸弹规划数据结构
#define MAX_BOMB_PATH_LENGTH 200
#define MAX_TIME_SLOTS 100
#define MAX_BOMB_PLANS 4
typedef struct {
    int bomb_id;
    int priority;  // 基于距离或重要性
    int path_len;
    int path_actions[MAX_BOMB_PATH_LENGTH];  // 动作序列
    float target_x, target_y;
    int start_time;  // 规划开始时间
} BombPlan;

// 粗网格子地图
typedef struct {
    uint8_t cells[MAP_ROWS][MAP_COLS]; // 0空地，1墙体，2当前箱子，3当前目的地
} CoarseMap;

// 箱子类型枚举（与 OpenART 识别结果对应）
typedef enum {
    BOX_TYPE_UNKNOWN = 0,
    BOX_TYPE_CARTOON_1,   // 葫芦娃
    BOX_TYPE_CARTOON_2,   // 其他卡通形象
} BoxTypeEnum_t;

// 箱子信息
typedef struct {
    float x, y;          // 世界坐标（中心）
    int grid_x, grid_y;  // 细网格坐标
    int state;           // 0未推，1已推
    int dest_id;         // 分配的目的地ID（-1表示未分配）
    BoxTypeEnum_t type;  // 箱子类型
} Box;

// 目的地数字类型
typedef struct {
    float x, y;
    int grid_x, grid_y;
    int assigned_box_id;          // 分配给哪个箱子（-1表示未分配）
    int required_digit;           // 目的地需要的数字
} Destination;

// 炸弹信息
typedef struct {
    float x, y;             // 世界坐标（中心）
    int grid_x, grid_y;     // 细网格坐标
    int active;             // 1=激活（可推动），0=已爆炸/失效
    float blast_radius;     // 爆炸半径（米）
    int target_id;          // 如果炸弹需要被推到某个目标点，存储目标ID（-1表示无）
} Bomb;

// 墙体信息
typedef struct {
    float x1, y1, x2, y2;   // 墙体线段端点（世界坐标）
} Wall;

// 游戏状态
typedef struct {
    Box boxes[MAX_BOXES];
    int num_boxes;
    Destination destinations[MAX_DESTINATIONS];
    int num_destinations;
    Bomb bombs[MAX_BOMBS];
    int num_bombs;
    Wall walls[MAX_WALLS];
    int num_walls;
} GameState;

// 网格地图（用于A*）
typedef struct {
    uint8_t occupancy[FINE_ROWS][FINE_COLS]; // 占据栅格
    float cost_map[FINE_ROWS][FINE_COLS];     // 代价地图
    int width, height;                         // 细网格尺寸
} GridMap;

// A*规划器参数
typedef struct {
    int max_iterations;
    float inflation_radius;
} AStarParams;

// ========== 函数声明 ==========
void load_map_from_text(const char* map_text, GridMap* grid_map, GameState* state);
void load_map_from_objects(GridMap* grid_map, GameState* state,
                           float field_width, float field_height,
                           const float* walls, int num_walls,
                           const float* boxes, int num_boxes,
                           const float* dests, int num_dests,
                           const float* bombs, int num_bombs);
void create_inflated_cost_map(GridMap* map, GameState* state, float inflation_radius);
void explode_bomb(GameState* state, GridMap* map, int bomb_id);
void refresh_grid_map(GameState* state, GridMap* map);
int astar_plan_path(GridMap* map, int start_x, int start_y, int goal_x, int goal_y,
                    int* out_path_x, int* out_path_y, int max_path_len, AStarParams* params);
int sokoban_plan_for_box(GameState* state, GridMap* grid_map, int box_id,
                         float car_x, float car_y,
                         float* out_path_x, float* out_path_y, int max_path_len);
void build_single_box_submap(GameState* state, GridMap* grid_map,
                             int box_id, CoarseMap* submap);
int plan_all_boxes_sequentially(GameState* state, GridMap* grid_map,
                                float car_x, float car_y,
                                float* out_paths[MAX_BOXES], int out_lens[MAX_BOXES],
                                int max_path_len);
void world_to_grid(float wx, float wy, int* gx, int* gy);
void grid_to_world(int gx, int gy, float* wx, float* wy);

// ========== 炸弹规划相关函数声明 ==========
int is_boundary_wall(Wall* w);
int simulate_wall_destruction(GameState* state, GridMap* grid_map,
                              int wall_idx,
                              int start_x, int start_y,
                              int box_id,
                              int* out_push_steps);
void build_bomb_submap(GameState* state, GridMap* grid_map,
                       int bomb_id, int target_r, int target_c, CoarseMap* submap);
int select_best_wall_to_destroy(GameState* state, GridMap* grid_map,
                                int start_x, int start_y,
                                int box_id,
                                float* out_bomb_target_x, float* out_bomb_target_y);
int plan_bomb_to_target(GameState* state, GridMap* grid_map, int bomb_id,
                        float car_x, float car_y,
                        float target_x, float target_y,
                        float* out_path_x, float* out_path_y, int max_path_len);
int find_bomb_target_near_wall(GameState* state, GridMap* grid_map, Wall* wall,
                               float* out_x, float* out_y);
int light_sokoban_plan(GameState* state, GridMap* grid_map, int box_id,
                       float car_x, float car_y,
                       int* out_actions, int max_actions);
int plan_multiple_bombs(GameState* state, GridMap* grid_map, float player_x, float player_y,
                        BombPlan* plans, int max_plans);
int actions_to_world_path(GameState* state, GridMap* grid_map, int box_id,
                          float start_car_x, float start_car_y,
                          const int* actions, int action_count,
                          float* out_x, float* out_y, int max_len);
// 路径规划相关数组最大尺寸
#ifndef MAX_PATH_POINTS
#define MAX_PATH_POINTS  100
#endif

#ifndef MAX_BFS_QUEUE
#define MAX_BFS_QUEUE   150
#endif

#ifndef MAX_KEEP_SIZE
#define MAX_KEEP_SIZE   150
#endif

#ifndef MAX_ACT_POINTS
#define MAX_ACT_POINTS  150
#endif

// 坐标转换函数（运动坐标 <-> 图像坐标）
void motion_to_image(float mx, float my, float* wx, float* wy);
void image_to_motion(float wx, float wy, float* mx, float* my);

#endif // PLANNER_H