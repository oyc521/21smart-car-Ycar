#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <rtthread.h>
#include "planner.h"
#include "hybrid_controller.h"

typedef enum {
    TASK_STATE_IDLE,
    TASK_STATE_RECOGNIZE_DEST,
    TASK_STATE_RECOGNIZE_BOX,
    TASK_STATE_ASSIGN_BOXES,
    TASK_STATE_PLANNING_BOX,
    TASK_STATE_EXECUTE_BOX,       // 统一执行推动（箱/炸弹）
    TASK_STATE_WAIT_MAP,
    TASK_STATE_RETURNING          // 新增：回库
} TaskState;

typedef enum {
    TASK_MODE_STAGE1 = 0,
    TASK_MODE_STAGE2 = 1
} TaskMode_t;

typedef struct {
    TaskState state;
    TaskMode_t mode;
    rt_event_t event;
    int current_box_id;
    int current_bomb_id;
    int pending_box_ids[MAX_BOXES];
    int pending_box_count;
    uint8_t retry_count;
    uint32_t wait_start_tick;

    // 识别相关
    int current_recog_target_id;
    RecognTargetType_t current_recog_type;
    uint8_t all_dest_recognized;
    uint8_t all_box_recognized;

    // 分段推动
    int action_queue[200];
    int action_total;
    int action_index;
} TaskManager;

/* 事件标志 */
#define TASK_EVENT_MAP_READY        0x10
#define TASK_EVENT_RECOG_BOX        0x01
#define TASK_EVENT_RECOG_DEST       0x02
#define TASK_EVENT_CONTROLLER_IDLE  0x04
#define TASK_EVENT_ALL_RECOG        0x08
#define TASK_EVENT_RECOG_DONE       0x20
#define TASK_EVENT_RECOG_FAILED     0x40

extern TaskManager g_task_mgr;

void task_manager_init(void);
void task_manager_thread_entry(void *parameter);
void task_manager_start(void);
void task_manager_set_mode(TaskMode_t mode);
uint8_t IsControllerBusy(void);
void assign_boxes_to_destinations(void);
int find_unrecognized_destination(int *out_idx);
int find_unrecognized_box(int *out_idx);
int find_unpushed_box_for_dest_digit(int dest_digit, int *out_idx);
void start_next_recognition(void);

#endif