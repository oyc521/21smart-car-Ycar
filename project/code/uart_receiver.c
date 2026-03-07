#include "uart_receiver.h"
#include "zf_driver_uart.h"
#include "planner.h"
#include "hybrid_controller.h"
#include "position.h"
#include <string.h>
#include <stdio.h>

/* 外部全局变量 */
extern GridMap g_grid_map;
extern GameState g_game_state;
extern HybridController g_ctrl;
extern Position_t position;

/* 环形缓冲区实例 */
ring_buffer_t g_uart_rb = { .head = 0, .tail = 0 };

/* 向环形缓冲区写入一个字节（由中断调用） */
static void rb_push(uint8_t data)
{
    uint16_t next = (g_uart_rb.head + 1) % UART_RX_BUF_SIZE;
    if (next != g_uart_rb.tail) {
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    } else {
        /* 缓冲区满，丢弃最旧的数据 */
        g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    }
}

/* 从环形缓冲区读取一个字节（由线程调用），成功返回1，失败返回0 */
static uint8_t rb_pop(uint8_t *data)
{
    if (g_uart_rb.head == g_uart_rb.tail) {
        return 0;
    }
    *data = g_uart_rb.buffer[g_uart_rb.tail];
    g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}

/* UART1 中断服务程序 */
/*void LPUART1_IRQHandler(void)
{
    uint8_t data;
    if (LPUART_GetStatusFlags(LPUART1) & kLPUART_RxDataRegFullFlag) {
        data = LPUART_ReadByte(LPUART1);
        rb_push(data);
    }
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
}*/
/* 自定义的 UART1 中断处理函数（改名） */
void UART1_IRQ_Handler(void)
{
    uint8_t data;
    if (LPUART_GetStatusFlags(LPUART1) & kLPUART_RxDataRegFullFlag) {
        data = LPUART_ReadByte(LPUART1);
        rb_push(data);
    }
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);
}

/* 初始化 UART1 并开启接收中断 */
void uart_receive_init(void)
{
    /* 使用逐飞库初始化 UART1，波特率 115200，根据实际接线修改引脚 */
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
		/* 重定向 UART1 中断向量到自定义处理函数 */
		NVIC_SetVector(LPUART1_IRQn, (uint32_t)UART1_IRQ_Handler);
    /* 开启接收中断 */
    uart_rx_interrupt(UART_1, 1);
    /* 可设置中断优先级（根据需要） */
    // NVIC_SetPriority(LPUART1_IRQn, 2);
}

/* 解析完整的一帧数据，调用 load_map_from_objects 更新地图 */
static void parse_complete_frame(const uint8_t *data, uint32_t len)
{
    float field_width, field_height;
    uint16_t num_walls, num_boxes, num_dests, num_bombs;

    /* 使用 memcpy 避免字节对齐问题 */
    memcpy(&field_width,  data + 2, 4);
    memcpy(&field_height, data + 6, 4);
    memcpy(&num_walls,    data + 10, 2);
    memcpy(&num_boxes,    data + 12, 2);
    memcpy(&num_dests,    data + 14, 2);
    memcpy(&num_bombs,    data + 16, 2);

    const uint8_t *walls_ptr = data + 18;
    const uint8_t *boxes_ptr = walls_ptr + num_walls * 16;
    const uint8_t *dests_ptr = boxes_ptr + num_boxes * 8;
    const uint8_t *bombs_ptr = dests_ptr + num_dests * 8;

    load_map_from_objects(&g_grid_map, &g_game_state,
                          field_width, field_height,
                          (const float*)walls_ptr, num_walls,
                          (const float*)boxes_ptr, num_boxes,
                          (const float*)dests_ptr, num_dests,
                          (const float*)bombs_ptr, num_bombs);

    printf("Map updated: walls=%d, boxes=%d, dests=%d, bombs=%d\n",
           num_walls, num_boxes, num_dests, num_bombs);
}

/* 自动规划触发函数（选择最近箱子） */
static void trigger_planning(void)
{
    int nearest = -1;
    float min_dist = 1e9f;
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        if (g_game_state.boxes[i].state == 0) {
            float dx = g_game_state.boxes[i].x - position.x_m;
            float dy = g_game_state.boxes[i].y - position.y_m;
            float d = dx*dx + dy*dy;
            if (d < min_dist) {
                min_dist = d;
                nearest = i;
            }
        }
    }
    if (nearest >= 0) {
        if (HybridController_PlanPathToBox(&g_ctrl, position.x_m, position.y_m, nearest)) {
            printf("Planning to box %d\n", nearest);
        } else {
            printf("Plan to box %d failed\n", nearest);
        }
    } else {
        printf("No available box\n");
    }
}

/* 尝试从解析缓冲区中提取完整的一帧，成功返回帧长度，否则返回0 */
static uint32_t try_parse_frame(uint8_t *buf, uint16_t buf_len)
{
    if (buf_len < 18) return 0;

    uint16_t magic;
    memcpy(&magic, buf, 2);
    if (magic != 0xAA55) return 0;

    uint16_t num_walls, num_boxes, num_dests, num_bombs;
    memcpy(&num_walls, buf + 10, 2);
    memcpy(&num_boxes, buf + 12, 2);
    memcpy(&num_dests, buf + 14, 2);
    memcpy(&num_bombs, buf + 16, 2);

    /* 数量合理性检查（可选） */
    if (num_walls > MAX_WALLS || num_boxes > MAX_BOXES ||
        num_dests > MAX_DESTINATIONS || num_bombs > MAX_BOMBS) {
        return 0;
    }

    uint32_t expected_len = 18 + num_walls * 16 + num_boxes * 8 +
                            num_dests * 8 + num_bombs * 8;
    if (buf_len < expected_len) return 0;

    parse_complete_frame(buf, expected_len);
    return expected_len;
}

/* 解析线程入口函数 */
void parse_uart_data_thread_entry(void *parameter)
{
    static uint8_t parse_buf[PARSE_BUF_SIZE] __attribute__((aligned(4)));
    uint16_t parse_len = 0;
    uint8_t byte;

    while (1) {
        if (rb_pop(&byte)) {
            parse_buf[parse_len++] = byte;
            if (parse_len >= PARSE_BUF_SIZE) {
                parse_len = 0;   /* 溢出则丢弃所有数据，重新同步 */
                continue;
            }

            /* 在缓冲区中查找帧头 0xAA55 */
            for (uint16_t i = 0; i + 1 < parse_len; i++) {
                uint16_t magic;
                memcpy(&magic, parse_buf + i, 2);
                if (magic == 0xAA55) {
                    uint32_t frame_len = try_parse_frame(parse_buf + i, parse_len - i);
                    if (frame_len > 0) {
                        uint16_t remain = parse_len - (i + frame_len);
                        if (remain > 0) {
                            memmove(parse_buf, parse_buf + i + frame_len, remain);
                        }
                        parse_len = remain;
                        trigger_planning();   /* 地图已更新，触发规划 */
                        break;
                    }
                }
            }
        } else {
            rt_thread_mdelay(1);   /* 缓冲区空，延时等待 */
        }
    }
}