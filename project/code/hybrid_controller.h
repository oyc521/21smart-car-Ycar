#ifndef HYBRID_CONTROLLER_H
#define HYBRID_CONTROLLER_H

#include "planner.h"

#ifndef MAX_PATH_POINTS
#define MAX_PATH_POINTS 200
#endif

/* 物体类型（用于 EvaluateBestStance） */
#define OBJ_BOX  0
#define OBJ_BOMB 1

/* ========== 可调参数（修改这里即可全局生效） ========== */
#define LOOKAHEAD_DIST      0.15f   // 前视距离 (m)
#define MIN_LOOKAHEAD       0.08f   // 最小前视距离 (m)
#define MAX_LOOKAHEAD       0.50f   // 最大前视距离 (m)
#define PATH_TOLERANCE      0.03f   // 路径到达容差 (m)
#define NAV_SPEED_MAX       0.10f   // 导航最大平移速度 (m/s)
#define PUSH_SPEED_MAX_DFL  0.08f   // 推动最大平移速度默认值 (m/s)
#define MIN_SPEED           0.08f   // 最小平移速度 (m/s)
#define ENABLE_ADAPTIVE_LOOKAHEAD 1 // 是否启用自适应前视
#define STUCK_TIME_MS       2000    // 卡死判定时间窗口 (ms)
#define STUCK_DISP_THRESH   0.02f   // 卡死最小位移 (m)
#define PUSH_INTERP_STEP    0.05f   // 推动路径插值步长 (m)

/* 控制器模式 */
typedef enum {
    CTRL_MODE_IDLE,
    CTRL_MODE_PATH_FOLLOWING,
    CTRL_MODE_RECOGNIZING,
    CTRL_MODE_IMU_MOVE,              // 保留未用，可后续删除
		CTRL_MODE_STOPPING      // 新增
} CtrlMode;

/* 路径用途 */
typedef enum {
    PATH_PURPOSE_NAV_TO_BOX,        // 通用导航（去往某点）
    PATH_PURPOSE_MOVE_ACTION,       // 保留，未用
    PATH_PURPOSE_PUSH_STANCE,       // 推动路径
    PATH_PURPOSE_RECOG_STANCE       // 识别站位路径
} PathPurpose;

/* 完成原因 */
typedef enum {
    CTRL_COMPLETE_SUCCESS,
    CTRL_COMPLETE_FAIL_PLAN,
    CTRL_COMPLETE_FAIL_STUCK,
    CTRL_COMPLETE_FAIL_TIMEOUT
} ControllerCompleteReason_t;

/* 混合控制器结构体 */
typedef struct {
    CtrlMode mode;
    float current_path[MAX_PATH_POINTS][2];
    int path_len;
    int path_following;
    float lookahead_dist;
    float min_lookahead, max_lookahead;
    float max_speed, min_speed;
    float path_tolerance;
    int enable_adaptive_lookahead;

    // 卡死检测（旧版）
    float last_path_pos[2];
    float last_path_angle;
    int path_stuck_counter;
    int path_stuck_threshold;
    uint32_t stuck_start_tick;
    float stuck_start_pos[2];
    float stuck_start_angle;

    int current_box_id;
    int current_bomb_id;
    float bomb_target_pos[2];       // 炸弹目标全局坐标（保留用于导航）
    int is_bomb_path;
		uint8_t axial_tracking;
		int path_target_idx;

    GridMap* grid_map;
    GameState* game_state;

    PathPurpose path_purpose;
    ControllerCompleteReason_t complete_reason;

    // 识别相关
    uint8_t recog_pending;
    RecognTargetType_t recog_target_type;
    int recog_target_id;
    float recog_stand_x;
    float recog_stand_y;
    float recog_align_yaw;
    float recog_original_yaw;

    // 控制器内部参数
    uint8_t use_tangent_heading;    // 0：锁航向模式
    float path_locked_yaw;          // 锁定时目标航向（当前固定为0°）
    float push_speed_max;           // 推动最大速度（保留）
    float push_smoothed_speed;      // 缓启动用，平滑速度（保留）

    // 新增：预评估最优站位结果
    float precomputed_stand_x;
    float precomputed_stand_y;
    int precomputed_actions[200];
    int precomputed_count;
		
		uint32_t stop_start_tick;          // 停稳检测起始时间
    int pending_path_purpose;          // 停稳后要执行的路径用途
} HybridController;

/* ========== 函数声明 ========== */
void HybridController_Init(HybridController* ctrl, GridMap* grid_map, GameState* game_state);
void HybridController_Reset(HybridController* ctrl);

// 路径生成
int HybridController_PlanPathToPoint(HybridController* ctrl,
                                     float start_x, float start_y,
                                     float target_x, float target_y);
int HybridController_PlanPushPath(HybridController* ctrl,
                                  float start_x, float start_y,
                                  int* actions, int action_count);

// 最优站位评估
int EvaluateBestStance(GameState* state, GridMap* grid_map,
                       int obj_id, int obj_type,
                       float bomb_target_x, float bomb_target_y,
                       float* out_stand_x, float* out_stand_y,
                       int* out_actions, int* out_count);

// 路径跟踪
int follow_path(HybridController* ctrl, float car_x, float car_y, float car_angle,
                float* vx, float* vy, float* omega, float* dist_to_end);

// 控制主循环
void HybridController_ComputeControl(HybridController* ctrl,
                                     float car_x, float car_y, float car_angle,
                                     float dt, float current_time,
                                     float* out_vx, float* out_vy, float* out_omega);

// 识别导航
int HybridController_NavigateAndRecognize(HybridController* ctrl,
                                          int target_grid_x, int target_grid_y,
                                          RecognTargetType_t target_type,
                                          int target_id);

// 辅助函数
int find_coarse_adjacent_target(GameState* state, GridMap* grid_map,
                                int box_id, float* out_x, float* out_y,
                                int preferred_dir);

#endif