#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>
#include <stdbool.h>

// 调试输出控制
#ifndef DEBUG
#define DEBUG 0
#endif

// 网格分辨率
#define RESOLUTION 0.05f
#define CELL_SIZE  0.2f

// 地图尺寸
#define MAP_COLS 16
#define MAP_ROWS 12
#define FINE_COLS (MAP_COLS * 4)
#define FINE_ROWS (MAP_ROWS * 4)

// 最大对象数量
#define MAX_BOXES         8
#define MAX_DESTINATIONS  8
#define MAX_BOMBS         8
#define MAX_WALLS         100

// 占据栅格值
#define OCC_FREE  0
#define OCC_WALL  1
#define OCC_BOX   2
#define OCC_DEST  3
#define OCC_BOMB  4

#ifndef MAX_PATH_POINTS
#define MAX_PATH_POINTS 200
#endif

// 箱子类型枚举（数字映射：0米老鼠 … 9灰太狼）
typedef enum {
    BOX_TYPE_MICKEY = 0,      // 0 米老鼠
    BOX_TYPE_PIKACHU,         // 1 皮卡丘
    BOX_TYPE_SPONGEBOB,       // 2 海绵宝宝
    BOX_TYPE_XIYANGYANG,      // 3 喜羊羊
    BOX_TYPE_DONALD,          // 4 唐老鸭
    BOX_TYPE_NEZHA,           // 5 哪吒
    BOX_TYPE_DATOUEZI,        // 6 大头儿子
    BOX_TYPE_PIGMAN,          // 7 猪猪侠
    BOX_TYPE_HULUWA,          // 8 葫芦娃
    BOX_TYPE_HUITAILANG,      // 9 灰太狼
    BOX_TYPE_UNKNOWN = -1     // 未知类型，置为 -1
} BoxTypeEnum_t;
// 箱子信息
typedef struct {
    float x, y;
    int grid_x, grid_y;
    int state;           // 0未推，1已推
    int dest_id;
    BoxTypeEnum_t type;
    uint8_t recognized;
} Box;

// 目的地信息
typedef struct {
    float x, y;
    int grid_x, grid_y;
    int assigned_box_id;
    int required_digit;
    uint8_t recognized;
} Destination;

// 炸弹信息
typedef struct {
    float x, y;
    int grid_x, grid_y;
    int active;
    float blast_radius;
    int target_id;
} Bomb;

// 墙体信息
typedef struct {
    float x1, y1, x2, y2;
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

// 网格地图
typedef struct {
    uint8_t occupancy[FINE_ROWS][FINE_COLS];
    float cost_map[FINE_ROWS][FINE_COLS];
    int width, height;
} GridMap;

// A*规划器参数
typedef struct {
    int max_iterations;
    float inflation_radius;
} AStarParams;

// 数字-目的地映射表
#define MAX_DIGITS 10
typedef struct {
    int digit;
    int dest_id;
    float x, y;
} DigitMap_t;

extern DigitMap_t g_digit_map[MAX_DIGITS];
extern int g_digit_map_count;

// 识别目标类型
typedef enum {
    RECOG_TARGET_DEST,
    RECOG_TARGET_BOX
} RecognTargetType_t;

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
void world_to_grid(float wx, float wy, int* gx, int* gy);
void grid_to_world(int gx, int gy, float* wx, float* wy);

// 炸弹规划相关（保留实际使用的）
int is_boundary_wall(Wall* w);
int simulate_wall_destruction(GameState* state, GridMap* grid_map,
                              int wall_idx,
                              int start_x, int start_y,
                              int box_id,
                              int* out_push_steps);
int select_best_wall_to_destroy(GameState* state, GridMap* grid_map,
                                int start_x, int start_y,
                                int box_id,
                                float* out_bomb_target_x, float* out_bomb_target_y,
                                int* out_push_dir);

// 推箱子/炸弹规划器
int light_sokoban_plan(GameState* state, GridMap* grid_map, int box_id,
                       float car_x, float car_y,
                       int* out_actions, int max_actions);
int light_push_plan(GridMap* map, int start_r, int start_c, int goal_r, int goal_c,
                    int car_start_r, int car_start_c, int* out_actions, int max_actions);

// 坐标转换
void motion_to_image(float mx, float my, float* wx, float* wy);
void image_to_motion(float wx, float wy, float* mx, float* my);

// 数字-箱子映射
int mapping_get_digit_for_box_type(BoxTypeEnum_t type);
BoxTypeEnum_t mapping_get_box_type_for_digit(int digit);

#endif // PLANNER_H