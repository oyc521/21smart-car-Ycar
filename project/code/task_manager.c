#include "task_manager.h"
#include "uart2_recognition.h"
#include "hybrid_controller.h"
#include "position.h"
#include "planner.h"
#include <stdio.h>

// 全局变量
TaskManager g_task_mgr;
extern HybridController g_ctrl;
extern GameState g_game_state;
extern Position_t position;

// 静态函数声明
static void task_state_machine(void);
static void send_recognition_commands(void);

void task_manager_init(void)
{
    g_task_mgr.state = TASK_STATE_IDLE;
    g_task_mgr.event = rt_event_create("task", RT_IPC_FLAG_FIFO);
    g_task_mgr.current_box_id = -1;
    g_task_mgr.current_bomb_id = -1;
    g_task_mgr.pending_box_count = 0;
    g_task_mgr.recognize_step = 0;
    g_task_mgr.last_recog_time = 0;
}

void task_manager_start(void)
{
    rt_thread_t tid = rt_thread_create("task_mgr",
                                       task_manager_thread_entry,
                                       RT_NULL,
                                       2048,
                                       RT_THREAD_PRIORITY_MAX / 2 + 2,
                                       20);
    if (tid) rt_thread_startup(tid);
}

void task_manager_thread_entry(void *parameter)
{
    rt_uint32_t recv_set;
    rt_tick_t tick_now;

    while (1) {
        if (rt_event_recv(g_task_mgr.event,
                          TASK_EVENT_RECOG_BOX | TASK_EVENT_RECOG_DEST |
                          TASK_EVENT_CONTROLLER_IDLE | TASK_EVENT_ALL_RECOG,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_TICK_PER_SECOND / 10,
                          &recv_set) == RT_EOK) {
            if (recv_set & TASK_EVENT_RECOG_BOX) {
                // 箱子识别完成
            }
            if (recv_set & TASK_EVENT_RECOG_DEST) {
                // 目的地数字识别完成
            }
            if (recv_set & TASK_EVENT_CONTROLLER_IDLE) {
                if (g_task_mgr.state == TASK_STATE_EXECUTE_BOX ||
                    g_task_mgr.state == TASK_STATE_EXECUTE_BOMB) {
                    g_task_mgr.state = TASK_STATE_PLANNING_BOX;
                }
            }
            if (recv_set & TASK_EVENT_ALL_RECOG) {
                if (g_task_mgr.state == TASK_STATE_RECOGNIZE) {
                    g_task_mgr.state = TASK_STATE_MATCH;
                }
            }
        }

        task_state_machine();

        rt_thread_mdelay(10);
    }
}

static void task_state_machine(void)
{
    int pending, i;
    float car_x, car_y;

    switch (g_task_mgr.state) {
        case TASK_STATE_IDLE:
            pending = 0;
            for (i = 0; i < g_game_state.num_boxes; i++) {
                if (g_game_state.boxes[i].state == 0) pending++;
            }
            if (pending > 0) {
                g_task_mgr.state = TASK_STATE_RECOGNIZE;
                g_task_mgr.recognize_step = 0;
                g_task_mgr.last_recog_time = rt_tick_get();
            }
            break;

        case TASK_STATE_RECOGNIZE:
            send_recognition_commands();
            break;

        case TASK_STATE_MATCH:
            assign_boxes_to_destinations();
            g_task_mgr.pending_box_count = 0;
            for (i = 0; i < g_game_state.num_boxes; i++) {
                if (g_game_state.boxes[i].state == 0 && g_game_state.boxes[i].dest_id >= 0) {
                    g_task_mgr.pending_box_ids[g_task_mgr.pending_box_count++] = i;
                }
            }
            if (g_task_mgr.pending_box_count > 0) {
                g_task_mgr.state = TASK_STATE_PLANNING_BOX;
            } else {
                g_task_mgr.state = TASK_STATE_IDLE;
            }
            break;

        case TASK_STATE_PLANNING_BOX:
            if (g_task_mgr.pending_box_count > 0) {
                int box_id = g_task_mgr.pending_box_ids[0];
                car_x = position.x_m;
                car_y = position.y_m;
                if (HybridController_PlanSokoban(&g_ctrl, box_id, car_x, car_y)) {
                    g_task_mgr.current_box_id = box_id;
                    g_task_mgr.state = TASK_STATE_EXECUTE_BOX;
                } else {
                    g_task_mgr.state = TASK_STATE_PLANNING_BOMB;
                }
            } else {
                g_task_mgr.state = TASK_STATE_IDLE;
            }
            break;

        case TASK_STATE_PLANNING_BOMB:
            // 炸弹规划逻辑（待完善）
            printf("Bomb planning not implemented, skip box %d\n", g_task_mgr.pending_box_ids[0]);
            // 移除当前箱子
            for (i = 1; i < g_task_mgr.pending_box_count; i++) {
                g_task_mgr.pending_box_ids[i-1] = g_task_mgr.pending_box_ids[i];
            }
            g_task_mgr.pending_box_count--;
            g_task_mgr.state = TASK_STATE_PLANNING_BOX;
            break;

        case TASK_STATE_EXECUTE_BOX:
        case TASK_STATE_EXECUTE_BOMB:
            // 等待控制器空闲事件
            break;

        default:
            g_task_mgr.state = TASK_STATE_IDLE;
            break;
    }
}

static void send_recognition_commands(void)
{
    rt_tick_t now = rt_tick_get();
    if (now - g_task_mgr.last_recog_time < RT_TICK_PER_SECOND / 2) {
        return;
    }
    g_task_mgr.last_recog_time = now;

    if (g_task_mgr.recognize_step == 0) {
        request_dest_recognition();
        g_task_mgr.recognize_step = 1;
    } else {
        request_box_recognition();
        g_task_mgr.recognize_step = 0;
    }
}

// 检查是否所有箱子/目的地都已识别（由 uart2_recognition 在更新后调用）
void check_all_recognized(void)
{
    int box_unknown = 0, dest_unknown = 0, i;
    for (i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].type == BOX_TYPE_UNKNOWN && g_game_state.boxes[i].state == 0)
            box_unknown++;
    }
    for (i = 0; i < g_game_state.num_destinations; i++) {
        if (g_game_state.destinations[i].required_digit == 0)
            dest_unknown++;
    }
    if (box_unknown == 0 && dest_unknown == 0) {
        rt_event_send(g_task_mgr.event, TASK_EVENT_ALL_RECOG);
    }
}