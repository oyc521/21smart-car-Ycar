#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <rtthread.h>
#include "planner.h"
#include "hybrid_controller.h"

// 任务状态
typedef enum {
    TASK_STATE_IDLE,                // 空闲，等待开始
    TASK_STATE_RECOGNIZE,           // 识别阶段（请求箱子类型和目的地数字）
    TASK_STATE_MATCH,                // 匹配阶段（分配箱子到目的地）
    TASK_STATE_EXECUTE_BOX,          // 执行箱子任务
    TASK_STATE_EXECUTE_BOMB,         // 执行炸弹任务
    TASK_STATE_PLANNING_BOX,         // 规划箱子路径
    TASK_STATE_PLANNING_BOMB,        // 规划炸弹路径
} TaskState;

// 任务管理器结构体
typedef struct {
    TaskState state;
    rt_event_t event;                 // 用于接收外部事件
    int current_box_id;               // 当前正在执行的箱子ID（-1表示无）
    int current_bomb_id;              // 当前正在执行的炸弹ID（-1表示无）
    int pending_box_ids[MAX_BOXES];   // 待处理的箱子ID列表
    int pending_box_count;
    int recognize_step;               // 识别步骤：0=请求箱子，1=请求目的地
    rt_tick_t last_recog_time;        // 上次发送识别命令的时间
} TaskManager;

// 事件标志
#define TASK_EVENT_RECOG_BOX    0x01   // 箱子识别完成
#define TASK_EVENT_RECOG_DEST   0x02   // 目的地数字识别完成
#define TASK_EVENT_CONTROLLER_IDLE 0x04 // 控制器进入空闲（任务完成）
#define TASK_EVENT_ALL_RECOG    0x08   // 所有识别完成（由匹配状态触发）

// 全局任务管理器实例
extern TaskManager g_task_mgr;

// 初始化任务管理器
void task_manager_init(void);

// 任务管理器线程入口
void task_manager_thread_entry(void *parameter);

// 启动任务管理器（创建线程）
void task_manager_start(void);

// 外部函数：分配箱子到目的地（由识别完成后调用）
void assign_boxes_to_destinations(void);

#endif