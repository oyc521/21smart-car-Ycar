#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <rtthread.h>
#include "planner.h"
#include "hybrid_controller.h"

typedef enum {
    TASK_STATE_IDLE,
    TASK_STATE_PLANNING_BOX,
    TASK_STATE_EXECUTE_BOX,
    TASK_STATE_PLANNING_BOMB,   // 新增
    TASK_STATE_EXECUTE_BOMB     // 新增
} TaskState;

typedef struct {
    TaskState state;
    rt_event_t event;
    int current_box_id;
    int current_bomb_id;          // 新增
    int pending_box_ids[MAX_BOXES];
    int pending_box_count;
		uint8_t retry_count;		//重试次数
} TaskManager;

// 事件标志
#define TASK_EVENT_MAP_READY       0x10   // 新地图已加载
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