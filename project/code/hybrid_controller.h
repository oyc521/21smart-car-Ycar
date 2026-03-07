#ifndef HYBRID_CONTROLLER_H
#define HYBRID_CONTROLLER_H

#include "planner.h"

// 路径点最大数量
#ifndef MAX_PATH_POINTS
#define MAX_PATH_POINTS 200
#endif

// Sokoban动作最大数量
#ifndef MAX_SOKOBAN_ACTIONS
#define MAX_SOKOBAN_ACTIONS 500
#endif

// 控制器模式
typedef enum {
    CTRL_MODE_IDLE,                // 空闲
    CTRL_MODE_PATH_FOLLOWING,      // 跟踪A*路径（到箱子附近或炸弹路径）
    CTRL_MODE_SOKOBAN_EXECUTING,    // Sokoban执行模式
		CTRL_MODE_BOMB_EXECUTING,    // 炸弹动作执行模式
		//CTRL_MODE_BOMB_PUSHING,    // 炸弹推动模式
		CTRL_MODE_VISUAL_ALIGNING   	// 视觉对准模式
} CtrlMode;

// 动作编码（与bfs_sokoban输出一致）
#define ACTION_UP     0
#define ACTION_RIGHT  1
#define ACTION_DOWN   2
#define ACTION_LEFT   3
#define ACTION_PUSH_UP    4
#define ACTION_PUSH_RIGHT 5
#define ACTION_PUSH_DOWN  6
#define ACTION_PUSH_LEFT  7

// 混合控制器结构体
typedef struct {
    // 当前模式
    CtrlMode mode;

    // 路径跟踪相关
    float current_path[MAX_PATH_POINTS][2];   // 当前A*路径（世界坐标）
    int path_len;                              // 路径点数
    int path_following;                         // 是否正在跟踪路径
    float lookahead_dist;                       // 前视距离
    float min_lookahead, max_lookahead;
    float max_speed, min_speed;
    float path_tolerance;                        // 到达终点容差（米）
    int enable_adaptive_lookahead;               // 是否启用自适应前视

    // 死锁检测
    float last_path_pos[2];
    float last_path_angle;
    int path_stuck_counter;
    int path_stuck_threshold;                    // 阈值（步数）
	
		// 视觉对准
		float align_target_x, align_target_y;   // 对准目标点
		int visual_align_complete;               // 1表示对准完成

    // 目标标识
    int current_box_id;
    int current_bomb_id;
    float bomb_target_pos[2];                     // 炸弹目标爆炸点（仅用于炸弹路径）
    int is_bomb_path;                              // 1:当前路径为炸弹路径

    // Sokoban 执行相关
    int sokoban_actions[MAX_SOKOBAN_ACTIONS];     // 动作序列
    int sokoban_action_count;
    int sokoban_action_index;
    float sokoban_subpath[MAX_PATH_POINTS][2];    // 当前动作的子路径
    int sokoban_subpath_len;
    int sokoban_subpath_following;                 // 是否正在跟踪子路径
    float sokoban_target_pos[2];                    // 当前动作的目标点（车的位置）

		//bomb-planner执行相关，炸damn···
		int bomb_actions[MAX_SOKOBAN_ACTIONS];     // 炸弹推动动作序列
		int bomb_action_count;
		int bomb_action_index;
		float bomb_subpath[MAX_PATH_POINTS][2];    // 当前动作的子路径
		int bomb_subpath_len;
		int bomb_subpath_following;                 // 是否正在跟踪子路径
		float bomb_action_target[2];                // 当前动作的目标点（车的位置）
		
    // 推动重试
    int push_retry_count;
    int max_push_retries;

    // 外部依赖（由外部传入，不负责管理内存）
    GridMap* grid_map;
    GameState* game_state;

    // 时间管理
    float last_plan_time;
    float plan_interval;                            // 最小规划间隔（秒）

} HybridController;

// 初始化
void HybridController_Init(HybridController* ctrl, GridMap* grid_map, GameState* game_state);

// 规划到箱子旁边（A*路径）
int HybridController_PlanPathToBox(HybridController* ctrl, float start_x, float start_y, int box_id);

// 进入视觉对准模式
void HybridController_StartVisualAlign(HybridController* ctrl, float target_x, float target_y);

// 规划炸弹路径（使用 bomb_planner）
int HybridController_PlanBombPath(HybridController* ctrl, float start_x, float start_y,
                                  int bomb_id, float target_x, float target_y);

// 规划Sokoban动作序列
int HybridController_PlanSokoban(HybridController* ctrl, int box_id, float car_x, float car_y);

// 计算控制量
void HybridController_ComputeControl(HybridController* ctrl,
                                     float car_x, float car_y, float car_angle,
                                     float dt, float current_time,
                                     float* out_vx, float* out_vy, float* out_omega);

// 重置
void HybridController_Reset(HybridController* ctrl);

#endif