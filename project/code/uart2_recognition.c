#include "uart2_recognition.h"
#include "zf_driver_uart.h"
#include "planner.h"          // 包含 g_game_state 的类型定义
#include "mapping.h"          // 映射表接口
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "task_manager.h"
// 声明外部全局游戏状态（由主程序定义）
extern GameState g_game_state;

// 环形缓冲区
typedef struct {
    uint8_t buffer[UART2_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;

static ring_buffer_t g_uart2_rb = { .head = 0, .tail = 0 };

// 全局变量定义
RecognitionResult_t g_recognition_result = { .type = RECOG_TYPE_NONE };
rt_event_t g_recog_event;

// 向环形缓冲区写入一个字节（中断中调用）
static void rb_push(uint8_t data)
{
    uint16_t next = (g_uart2_rb.head + 1) % UART2_RX_BUF_SIZE;
    if (next != g_uart2_rb.tail) {
        g_uart2_rb.buffer[g_uart2_rb.head] = data;
        g_uart2_rb.head = next;
    } else {
        g_uart2_rb.tail = (g_uart2_rb.tail + 1) % UART2_RX_BUF_SIZE;
        g_uart2_rb.buffer[g_uart2_rb.head] = data;
        g_uart2_rb.head = next;
    }
}

// 从环形缓冲区读取一个字节（线程中调用）
static uint8_t rb_pop(uint8_t *data)
{
    if (g_uart2_rb.head == g_uart2_rb.tail) return 0;
    *data = g_uart2_rb.buffer[g_uart2_rb.tail];
    g_uart2_rb.tail = (g_uart2_rb.tail + 1) % UART2_RX_BUF_SIZE;
    return 1;
}

// 自定义中断处理函数
void UART2_IRQ_Handler(void)
{
    uint8_t data;
    if (LPUART_GetStatusFlags(LPUART2) & kLPUART_RxDataRegFullFlag) {
        data = LPUART_ReadByte(LPUART2);
        rb_push(data);
    }
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);
}

// ---------- 新增辅助函数：根据坐标查找最近对象 ----------

// 查找距离 (x,y) 最近的未识别箱子（type == BOX_TYPE_UNKNOWN 且未推）
static int find_nearest_box(float x, float y)
{
    int best = -1;
    float min_dist_sq = 1e9f;
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].type == BOX_TYPE_UNKNOWN &&
            g_game_state.boxes[i].state == 0) {
            float dx = g_game_state.boxes[i].x - x;
            float dy = g_game_state.boxes[i].y - y;
            float dist_sq = dx*dx + dy*dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best = i;
            }
        }
    }
    // 可选：增加距离阈值，例如如果最小距离 > 0.5m 则返回 -1
    // if (min_dist_sq > 0.25f) return -1;  // 0.5^2
    return best;
}

// 查找距离 (x,y) 最近的未分配数字的目的地（required_digit == 0）
static int find_nearest_dest(float x, float y)
{
    int best = -1;
    float min_dist_sq = 1e9f;
    for (int i = 0; i < g_game_state.num_destinations; i++) {
        if (g_game_state.destinations[i].required_digit == 0) {
            float dx = g_game_state.destinations[i].x - x;
            float dy = g_game_state.destinations[i].y - y;
            float dist_sq = dx*dx + dy*dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best = i;
            }
        }
    }
    // 可选：增加距离阈值
    // if (min_dist_sq > 0.25f) return -1;
    return best;
}

// ---------- 分配函数（贪心算法） ----------
/*void assign_boxes_to_destinations(void)
{
    // 收集未分配的箱子（state == 0 且 dest_id == -1）
    int box_ids[MAX_BOXES];
    int box_count = 0;
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].state == 0 && g_game_state.boxes[i].dest_id == -1) {
            box_ids[box_count++] = i;
        }
    }

    // 收集未分配的目的地（assigned_box_id == -1）
    int dest_ids[MAX_DESTINATIONS];
    int dest_count = 0;
    for (int i = 0; i < g_game_state.num_destinations; i++) {
        if (g_game_state.destinations[i].assigned_box_id == -1) {
            dest_ids[dest_count++] = i;
        }
    }

    // 贪心：为每个箱子找最近且数字匹配的目的地
    for (int i = 0; i < box_count; i++) {
        int box_id = box_ids[i];
        BoxTypeEnum_t type = g_game_state.boxes[box_id].type;
        int required_digit = mapping_get_digit_for_box_type(type);
        if (required_digit < 0) continue;   // 无映射，跳过

        int best_dest = -1;
        float min_dist_sq = 1e9f;
        for (int j = 0; j < dest_count; j++) {
            int dest_id = dest_ids[j];
            if (g_game_state.destinations[dest_id].required_digit != required_digit) continue;
            float dx = g_game_state.boxes[box_id].x - g_game_state.destinations[dest_id].x;
            float dy = g_game_state.boxes[box_id].y - g_game_state.destinations[dest_id].y;
            float dist_sq = dx*dx + dy*dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_dest = dest_id;
            }
        }
        if (best_dest >= 0) {
            g_game_state.boxes[box_id].dest_id = best_dest;
            g_game_state.destinations[best_dest].assigned_box_id = box_id;
            // 从待分配目的地列表中移除
            for (int j = 0; j < dest_count; j++) {
                if (dest_ids[j] == best_dest) {
                    dest_ids[j] = dest_ids[dest_count - 1];
                    dest_count--;
                    break;
                }
            }
        }
    }

    printf("Assignment completed.\n");
    // 可在此处触发任务管理器开始执行第一个箱子任务
    // 例如：if (box_count > 0) HybridController_PlanPathToBox(&g_ctrl, ...);
}
*/
// ---------- 检查是否所有对象都已识别 ----------
static void check_all_recognized(void)
{
    int box_unknown = 0, dest_unknown = 0;
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].type == BOX_TYPE_UNKNOWN && g_game_state.boxes[i].state == 0)
            box_unknown++;
    }
    for (int i = 0; i < g_game_state.num_destinations; i++) {
        if (g_game_state.destinations[i].required_digit == 0)
            dest_unknown++;
    }
    if (box_unknown == 0 && dest_unknown == 0) {
        assign_boxes_to_destinations();
    }
}

// ---------- 修改后的解析函数 ----------
static void parse_line(char *line)
{
    char type_str[32];
    float x, y;
    int digit;

    // 箱子识别: box,type,x,y
    if (sscanf(line, "box,%[^,],%f,%f", type_str, &x, &y) == 3) {
        int box_id = find_nearest_box(x, y);
        if (box_id >= 0) {
            if (strcmp(type_str, "huluwa") == 0)
                g_game_state.boxes[box_id].type = BOX_TYPE_CARTOON_1;
            else if (strcmp(type_str, "other") == 0)
                g_game_state.boxes[box_id].type = BOX_TYPE_CARTOON_2;
            else
                g_game_state.boxes[box_id].type = BOX_TYPE_UNKNOWN;
            // 发送事件（可选）
            // rt_event_send(g_recog_event, 0x01);
        }
        check_all_recognized();
        return;
    }

    // 目的地数字识别: dest,digit,x,y
    if (sscanf(line, "dest,%d,%f,%f", &digit, &x, &y) == 3) {
        int dest_id = find_nearest_dest(x, y);
        if (dest_id >= 0) {
            g_game_state.destinations[dest_id].required_digit = digit;
            // rt_event_send(g_recog_event, 0x02);
        }
        check_all_recognized();
        return;
    }

    // 映射规则: map,box_type,digit
    if (sscanf(line, "map,%[^,],%d", type_str, &digit) == 2) {
        BoxTypeEnum_t type;
        if (strcmp(type_str, "huluwa") == 0)
            type = BOX_TYPE_CARTOON_1;
        else if (strcmp(type_str, "other") == 0)
            type = BOX_TYPE_CARTOON_2;
        else
            return;
        mapping_set(type, digit);
        printf("Mapping set: %s -> %d\n", type_str, digit);
        return;
    }
}

// ---------- 初始化UART2 ----------
void uart2_recognition_init(void)
{
    uart_init(UART_2, 115200, UART2_TX_B18, UART2_RX_B19);
    NVIC_SetVector(LPUART2_IRQn, (uint32_t)UART2_IRQ_Handler);
    uart_rx_interrupt(UART_2, ZF_ENABLE);
    g_recog_event = rt_event_create("recog", RT_IPC_FLAG_FIFO);
}

// ---------- UART2解析线程入口 ----------
void uart2_recognition_thread_entry(void *parameter)
{
    static uint8_t line_buf[LINE_BUF_SIZE];
    uint16_t idx = 0;
    uint8_t byte;

    while (1) {
        if (rb_pop(&byte)) {
            if (idx < LINE_BUF_SIZE - 1) {
                line_buf[idx++] = byte;
                if (byte == '\n') {
                    line_buf[idx] = '\0';
                    parse_line((char*)line_buf);
                    idx = 0;
                }
            } else {
                idx = 0;  // 行过长，丢弃
            }
        } else {
            rt_thread_mdelay(1);
        }
    }
}

// ---------- 发送识别命令（保留原接口） ----------
void request_box_recognition(void)
{
    uart_write_string(UART_2, "box\n");
}

void request_dest_recognition(void)
{
    uart_write_string(UART_2, "dest\n");
}