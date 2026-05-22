#include "uart_receiver.h"
#include "zf_driver_uart.h"
#include "planner.h"
#include "hybrid_controller.h"
#include "position.h"
#include "encoder.h"
#include "task_manager.h"
#include "motor.h"          // 为了获取 car_ctrl.current_vx/vy
#include <string.h>
#include <stdio.h>
#include <rtthread.h>

/* 外部全局变量 */
extern GridMap g_grid_map;
extern GameState g_game_state;
extern HybridController g_ctrl;
extern Position_t position;
extern uint8_t waiting_map;
extern uint8_t need_map_update;
extern CarController_t car_ctrl;   // 用于获取当前速度
extern rt_mutex_t g_map_mutex;

/* 环形缓冲区实例 */
ring_buffer_t g_uart_rb = { .head = 0, .tail = 0 };

/* 地图更新标志 */
uint8_t g_map_updated = 0;

/* 视觉位置校正相关变量 */
// 原 static uint8_t g_vision_valid = 0; 改为全局或提供函数
static uint8_t g_vision_valid = 0;   // 仍可静态，通过函数访问
static float g_vision_pos_x = 0.0f;
static float g_vision_pos_y = 0.0f;

// 获取视觉位置
void get_vision_position(float *x, float *y) {
    *x = g_vision_pos_x;
    *y = g_vision_pos_y;
}

// 获取视觉有效标志
uint8_t is_vision_valid(void) {
    return g_vision_valid;
}
void clear_vision_valid(void) {
    g_vision_valid = 0;
}
// 在 parse_map_frame 中找到 P 点时，设置 g_vision_valid = 1
// （代码已有，无需修改，但需确保 g_vision_valid 在提取到 P 时被置 1）
/* 环形缓冲区写入一个字节（中断调用） */
static void rb_push(uint8_t data) {
    uint16_t next = (g_uart_rb.head + 1) % UART_RX_BUF_SIZE;
    if (next != g_uart_rb.tail) {
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    } else {
        // 缓冲区满，覆盖最旧的数据
        g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    }
}

/* 从环形缓冲区读取一个字节（线程调用） */
static uint8_t rb_pop(uint8_t *data) {
    if (g_uart_rb.head == g_uart_rb.tail) return 0;
    *data = g_uart_rb.buffer[g_uart_rb.tail];
    g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}

/* ========== UART1 中断回调（在 isr.c 中调用） ========== */
void uart1_rx_callback(void) {
    uint8_t data;
    while (uart_query_byte(UART_1, &data)) {
        rb_push(data);
    }
}

/* 初始化 UART1 及环形缓冲区中断 */
void uart_receive_init(void) {
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    uart_rx_interrupt(UART_1, 1);
    NVIC_SetPriority(LPUART1_IRQn, 2);
}


/* ========== 解析地图帧（类型 0x01） ========== */
static void parse_map_frame(const uint8_t *data, uint32_t len) {
    if (len != 192) {
        wireless_uart_send_string("Map frame data length error!\r\n");
        return;
    }

    // 将原始地图数据转换为文本（12行，每行16个字符 + 换行符）
    char map_text[12 * 17 + 1];  // 每行16字符+换行，末尾'\0'
    int idx = 0;
    for (int r = 0; r < 12; r++) {
        memcpy(map_text + idx, data + r * 16, 16);
        idx += 16;
        map_text[idx++] = '\n';
    }
    map_text[idx] = '\0';

    wireless_uart_send_string("Raw map text received:\r\n");
    wireless_uart_send_string(map_text);
    wireless_uart_send_string("\r\n");

    // 从地图文本提取小车位置（视觉校正用），只处理第一个找到的
    int found_p = 0;
    for (int r = 0; r < 12; r++) {
        for (int c = 0; c < 16; c++) {
            if (map_text[r * 17 + c] == 'P') {  // 每行实际占17字节（16字符+换行）
                float img_x = (c + 0.5f) * CELL_SIZE;
                float img_y = (r + 0.5f) * CELL_SIZE;
                g_vision_pos_x = img_x;
                g_vision_pos_y = img_y;
                g_vision_valid = 1;
                found_p = 1;

                char buf[64];
                int pos_x_mm = (int)(img_x * 1000.0f);
                int pos_y_mm = (int)(img_y * 1000.0f);
                rt_sprintf(buf, "[Vision] P at grid (%d,%d) -> (%d, %d) mm\r\n",
                           r, c, pos_x_mm, pos_y_mm);
                wireless_uart_send_string(buf);
                break;
            }
        }
        if (found_p) break;
    }

    // 在互斥锁保护下加载地图、刷新、分配箱子
    if (g_map_mutex) {
        rt_mutex_take(g_map_mutex, RT_WAITING_FOREVER);
    }

    load_map_from_text(map_text, &g_grid_map, &g_game_state);
    refresh_grid_map(&g_game_state, &g_grid_map);
    assign_boxes_to_destinations();

    if (g_map_mutex) {
        rt_mutex_release(g_map_mutex);
    }

    // 调试打印箱子坐标（仅测试用）
    char dbg[128];
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        rt_sprintf(dbg, "Box %d motion: (%d,%d)\r\n", i,
                   (int)(g_game_state.boxes[i].x * 1000), (int)(g_game_state.boxes[i].y * 1000));
        wireless_uart_send_string(dbg);
    }

    // waiting_map 清零和 MAP_READY 事件由 main.c 控制循环统一处理
}


/* ========== 视觉校正（由主循环调用） ========== */
/*void try_vision_reset(void) {
    // 必须同时满足：允许校正 && 有有效视觉位置
    if (!g_allow_vision_reset || !g_vision_valid) return;

    // 控制器在非空闲模式时禁止校正
    if (g_ctrl.mode != CTRL_MODE_IDLE) return;

    // 要求静止：连续5次检测速度<0.02m/s
    static uint8_t still_count = 0;
    float speed = sqrtf(car_ctrl.current_vx * car_ctrl.current_vx + 
                        car_ctrl.current_vy * car_ctrl.current_vy);
    if (speed < 0.02f) {
        still_count++;
    } else {
        still_count = 0;
    }
    if (still_count < 5) return;
    still_count = 0;

    // 执行校正（Position_Set 内部会重置位移低通滤波器）
    Position_Set(g_vision_pos_x, g_vision_pos_y, position.yaw_rad);
    EncoderReset();

    g_allow_vision_reset = 0;
    g_vision_valid = 0;

    // 打印校正后的位置
    char buf[64];
    int pos_x_mm = (int)(g_vision_pos_x * 1000.0f);
    int pos_y_mm = (int)(g_vision_pos_y * 1000.0f);
    rt_sprintf(buf, "[Vision] Reset to (%d, %d)\r\n", pos_x_mm, pos_y_mm);
    wireless_uart_send_string(buf);
}*/
/* ========== 帧解析器 ========== */
static uint32_t try_parse_frame(uint8_t *buf, uint16_t buf_len) {
    if (buf_len < 5) return 0;
    uint16_t magic;
    memcpy(&magic, buf, 2);
    if (magic != 0xAA55) return 0;

    uint8_t type;
    memcpy(&type, buf + 2, 1);
    uint16_t data_len;
    memcpy(&data_len, buf + 3, 2);

    // 仅处理地图帧
    if (type == 0x01 && data_len == 192) {
        uint32_t expected_len = 5 + data_len;
        if (buf_len >= expected_len) {
            parse_map_frame(buf + 5, data_len);
            g_map_updated = 1;
            return expected_len;
        }
    }
    // 其他类型帧暂不处理
    return 0;
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