#include "uart_receiver.h"
#include "zf_driver_uart.h"
#include "planner.h"
#include "hybrid_controller.h"
#include "position.h"
#include "encoder.h"
#include "task_manager.h"
#include <string.h>
#include <stdio.h>

/* 外部全局变量 */
extern GridMap g_grid_map;
extern GameState g_game_state;
extern HybridController g_ctrl;
extern Position_t position;
extern uint8_t waiting_map;
extern uint8_t need_map_update;

/* 环形缓冲区实例 */
ring_buffer_t g_uart_rb = { .head = 0, .tail = 0 };

/* 标志定义 */
uint8_t g_map_updated = 0;

/* 向环形缓冲区写入一个字节（由中断调用） */
static void rb_push(uint8_t data) {
    uint16_t next = (g_uart_rb.head + 1) % UART_RX_BUF_SIZE;
    if (next != g_uart_rb.tail) {
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    } else {
        g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    }
}

/* 从环形缓冲区读取一个字节（由线程调用） */
static uint8_t rb_pop(uint8_t *data) {
    if (g_uart_rb.head == g_uart_rb.tail) return 0;
    *data = g_uart_rb.buffer[g_uart_rb.tail];
    g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}

/* ========== UART1 中断回调（由 isr.c 调用） ========== */
void uart1_rx_callback(void) {
    uint8_t data;
    while (uart_query_byte(UART_1, &data)) {
        rb_push(data);
    }
}

/* 初始化 UART1 并开启接收中断 */
void uart_receive_init(void) {
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    uart_rx_interrupt(UART_1, 1);
    NVIC_SetPriority(LPUART1_IRQn, 2);
}

/* 将图像坐标转换为运动坐标（规划器内部使用） */
static void image_to_motion_coords(float img_x, float img_y, float *motion_x, float *motion_y) {
    // 图像坐标：X 0~3.2 (列), Y 0~2.4 (行)，原点左上角
    // 运动坐标：X 0~2.4 (行), Y 0~3.2 (列)，原点左下角
    *motion_x = img_y;   // 图像 Y -> 运动 X
    *motion_y = img_x;   // 图像 X -> 运动 Y
}

/* ========== 解析地图帧（类型 0x01） ========== */
static void parse_map_frame(const uint8_t *data, uint32_t len) {
    if (len != 192) {
        wireless_uart_send_string("Map frame data length error!\r\n");
        return;
    }
    char map_text[12*16 + 12];
    int idx = 0;
    for (int r = 0; r < 12; r++) {
        memcpy(map_text + idx, data + r*16, 16);
        idx += 16;
        map_text[idx++] = '\n';
    }
    map_text[idx] = '\0';

    wireless_uart_send_string("Raw map text received:\r\n");
    wireless_uart_send_string(map_text);
    wireless_uart_send_string("\r\n");

    // 加载地图到网格和游戏状态（load_map_from_text 内部将图像坐标转换为运动坐标）
    load_map_from_text(map_text, &g_grid_map, &g_game_state);
    wireless_uart_send_string("Map loaded from text.\r\n");

    // 可选：从地图文本中解析小车位置作为备份（当视觉位置帧丢失时使用）
    // 但主要位置源还是位置帧，这里可以保留注释，不启用
    /*
    int car_row = -1, car_col = -1;
    const char* p = map_text;
    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            if (*p == 'P') {
                car_row = r;
                car_col = c;
                break;
            }
            p++;
        }
        if (car_row != -1) break;
        p++;
    }
    if (car_row >= 0 && car_col >= 0) {
        position.x_m = (car_col + 0.5f) * CELL_SIZE;
        position.y_m = (car_row + 0.5f) * CELL_SIZE;
        position.x_dis = position.x_m * 100.0f;
        position.y_dis = position.y_m * 100.0f;
        EncoderReset();
        wireless_uart_send_string("Car position backup from map.\r\n");
    }
    */

    // ========== 恢复二次坐标转换（确保箱子、目的地坐标正确） ==========
    // 转换箱子坐标
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        float img_x = g_game_state.boxes[i].x;  // 图像 X
        float img_y = g_game_state.boxes[i].y;  // 图像 Y
        float motion_x, motion_y;
        image_to_motion_coords(img_x, img_y, &motion_x, &motion_y);
        g_game_state.boxes[i].x = motion_x;
        g_game_state.boxes[i].y = motion_y;
        world_to_grid(motion_x, motion_y, &g_game_state.boxes[i].grid_x, &g_game_state.boxes[i].grid_y);
    }
    // 转换目的地坐标
    for (int i = 0; i < g_game_state.num_destinations; i++) {
        float img_x = g_game_state.destinations[i].x;
        float img_y = g_game_state.destinations[i].y;
        float motion_x, motion_y;
        image_to_motion_coords(img_x, img_y, &motion_x, &motion_y);
        g_game_state.destinations[i].x = motion_x;
        g_game_state.destinations[i].y = motion_y;
        world_to_grid(motion_x, motion_y, &g_game_state.destinations[i].grid_x, &g_game_state.destinations[i].grid_y);
    }
    // 转换炸弹坐标（如果有）
    for (int i = 0; i < g_game_state.num_bombs; i++) {
        float img_x = g_game_state.bombs[i].x;
        float img_y = g_game_state.bombs[i].y;
        float motion_x, motion_y;
        image_to_motion_coords(img_x, img_y, &motion_x, &motion_y);
        g_game_state.bombs[i].x = motion_x;
        g_game_state.bombs[i].y = motion_y;
        world_to_grid(motion_x, motion_y, &g_game_state.bombs[i].grid_x, &g_game_state.bombs[i].grid_y);
    }

    // 刷新网格地图（基于转换后的运动坐标）
    refresh_grid_map(&g_game_state, &g_grid_map);

    // 分配箱子到最近的目的地
    assign_boxes_to_destinations();

    // 调试打印箱子的运动坐标（整数）
    char dbg[128];
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        rt_sprintf(dbg, "Box %d motion: (%d,%d)\r\n", i,
                   (int)(g_game_state.boxes[i].x * 1000), (int)(g_game_state.boxes[i].y * 1000));
        wireless_uart_send_string(dbg);
    }

    waiting_map = 0;
    rt_event_send(g_task_mgr.event, TASK_EVENT_MAP_READY);
}

/* ========== 解析位置帧（类型 0x02） ========== */
static void parse_position_frame(const uint8_t *data, uint32_t len) {
    if (len != 8) return;
    float img_x, img_y;
    memcpy(&img_x, data, 4);
    memcpy(&img_y, data + 4, 4);
    // 图像坐标 -> 运动坐标
    float motion_x, motion_y;
    image_to_motion_coords(img_x, img_y, &motion_x, &motion_y);
    position.x_m = motion_x;
    position.y_m = motion_y;
    position.x_dis = motion_x * 100.0f;
    position.y_dis = motion_y * 100.0f;
    EncoderReset();
    wireless_uart_send_string("Position updated from vision, encoder reset.\r\n");
}

/* ========== 帧解析分发 ========== */
static uint32_t try_parse_frame(uint8_t *buf, uint16_t buf_len) {
    if (buf_len < 5) return 0;
    uint16_t magic;
    memcpy(&magic, buf, 2);
    if (magic != 0xAA55) return 0;

    uint8_t type;
    memcpy(&type, buf + 2, 1);
    uint16_t data_len;
    memcpy(&data_len, buf + 3, 2);

    // 合理性检查
    if ((type == 0x01 && data_len != 192) || (type == 0x02 && data_len != 8)) {
        return 0;
    }

    uint32_t expected_len = 5 + data_len;
    if (buf_len < expected_len) return 0;

    if (type == 0x01) {
        parse_map_frame(buf + 5, data_len);
        g_map_updated = 1;
    } else if (type == 0x02) {
        parse_position_frame(buf + 5, data_len);   // 恢复位置帧解析
    } else {
        return 0;
    }
    return expected_len;
}

/* ========== 解析线程入口函数 ========== */
void parse_uart_data_thread_entry(void *parameter) {
    static uint8_t parse_buf[PARSE_BUF_SIZE] __attribute__((aligned(4)));
    uint16_t parse_len = 0;
    uint8_t byte;

    while (1) {
        if (rb_pop(&byte)) {
            parse_buf[parse_len++] = byte;
            if (parse_len >= PARSE_BUF_SIZE) {
                parse_len = 0;
                wireless_uart_send_string("Parse buffer overflow, reset\r\n");
                continue;
            }

            // 在缓冲区中查找帧头 0xAA55
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
                        break;
                    }
                }
            }
        } else {
            rt_thread_mdelay(1);
        }
    }
}