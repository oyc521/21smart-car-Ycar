/**
 * @file uart_receiver.c
 * @brief UART1 数据接收与解析模块
 * @details 从 OpenArt 上位机接收地图帧、视觉定位等信息，并更新全局地图和状态。
 */

#include "uart_receiver.h"
#include "zf_driver_uart.h"
#include "planner.h"
#include "hybrid_controller.h"
#include "position.h"
#include "encoder.h"
#include "task_manager.h"
#include "motor.h"          // 用于获取 car_ctrl.current_vx/vy（速度检测）
#include <string.h>
#include <stdio.h>
#include <rtthread.h>

/* 外部全局变量声明 */
extern GridMap g_grid_map;
extern GameState g_game_state;
extern HybridController g_ctrl;
extern Position_t position;
extern volatile uint8_t waiting_map;
extern volatile uint8_t need_map_update;
extern CarController_t car_ctrl;   // 用于获取当前速度
extern rt_mutex_t g_map_mutex;

/* 环形缓冲区（用于存储 UART 接收数据） */
ring_buffer_t g_uart_rb = { .head = 0, .tail = 0 };

/* 地图更新标志（由解析线程设置，主循环读取） */
uint8_t g_map_updated = 0;

/* 视觉定位数据（来自地图中的 'P' 字符） */
static uint8_t g_vision_valid = 0;   // 视觉定位数据是否有效
static float g_vision_pos_x = 0.0f;
static float g_vision_pos_y = 0.0f;

/**
 * @brief 获取最近一次视觉定位的位置
 * @param x, y 输出位置（图像坐标系，像素或毫米）
 */
void get_vision_position(float *x, float *y) {
    *x = g_vision_pos_x;
    *y = g_vision_pos_y;
}

/**
 * @brief 检查视觉定位数据是否有效
 * @return 1有效，0无效
 */
uint8_t is_vision_valid(void) {
    return g_vision_valid;
}

/**
 * @brief 清除视觉定位有效标志
 */
void clear_vision_valid(void) {
    g_vision_valid = 0;
}

/* 注意：parse_map_frame 中解析到 'P' 时会设置 g_vision_valid = 1，
 * 因此视觉定位仅在接收到地图帧且包含 'P' 时有效。
 */

/**
 * @brief 向环形缓冲区压入一个字节
 * @param data 待压入的数据
 * @note 若缓冲区已满，则丢弃最旧的数据（覆盖）
 */
static void rb_push(uint8_t data) {
    uint16_t next = (g_uart_rb.head + 1) % UART_RX_BUF_SIZE;
    if (next != g_uart_rb.tail) {
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    } else {
        // 缓冲区满，覆盖旧数据（丢弃最旧的一个）
        g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    }
}

/**
 * @brief 从环形缓冲区弹出一个字节
 * @param data 输出指针
 * @return 1成功（有数据），0缓冲区为空
 */
static uint8_t rb_pop(uint8_t *data) {
    if (g_uart_rb.head == g_uart_rb.tail) return 0;
    *data = g_uart_rb.buffer[g_uart_rb.tail];
    g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}

/* ========== UART1 接收中断回调（由 isr.c 调用） ========== */
void uart1_rx_callback(void) {
    uint8_t data;
    while (uart_query_byte(UART_1, &data)) {
        rb_push(data);
    }
}

/**
 * @brief 初始化 UART1 接收（115200，引脚 B12/B13，开启接收中断）
 */
void uart_receive_init(void) {
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    uart_rx_interrupt(UART_1, 1);
    NVIC_SetPriority(LPUART1_IRQn, 2);
}

/* ========== 解析地图帧（类型 0x01，长度 192 字节） ========== */
/**
 * @brief 解析地图帧数据
 * @param data 指向192字节地图文本的指针（每行16字符，共12行）
 * @param len 数据长度，必须为192
 * @details 将接收到的字符地图转换为内部 GridMap 和 GameState，
 *          同时检测 'P' 字符作为小车视觉定位。
 */
static void parse_map_frame(const uint8_t *data, uint32_t len) {
    if (len != 192) {
        wireless_uart_send_string("Map frame data length error!\r\n");
        return;
    }

    // 构建带换行的文本地图（12行×16列，每行末尾加 '\n'）
    char map_text[12 * 17 + 1];  // 16列 + 换行符 + 终止符
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

    // 在文本中查找 'P'（小车位置），并记录视觉定位
    int found_p = 0;
    for (int r = 0; r < 12; r++) {
        for (int c = 0; c < 16; c++) {
            if (map_text[r * 17 + c] == 'P') {  // 每行长度17（16字符+换行）
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

    // 加锁，更新地图和游戏状态
    if (g_map_mutex) {
        rt_mutex_take(g_map_mutex, RT_WAITING_FOREVER);
    }

    load_map_from_text(map_text, &g_grid_map, &g_game_state);
    refresh_grid_map(&g_game_state, &g_grid_map);
    assign_boxes_to_destinations();   // 将箱子与目标点匹配

    if (g_map_mutex) {
        rt_mutex_release(g_map_mutex);
    }

    // 调试输出箱子位置（运动学坐标）
    char dbg[128];
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        rt_sprintf(dbg, "Box %d motion: (%d,%d)\r\n", i,
                   (int)(g_game_state.boxes[i].x * 1000), (int)(g_game_state.boxes[i].y * 1000));
        wireless_uart_send_string(dbg);
    }

    // 注意：waiting_map 标志和 MAP_READY 事件由 main.c 中的控制循环处理
}

/* ========== 尝试解析完整帧（从缓冲区中） ========== */
/**
 * @brief 尝试解析一帧数据
 * @param buf 输入缓冲区
 * @param buf_len 缓冲区已有数据长度
 * @return 成功解析的帧长度（字节数），0表示未解析到完整帧
 */
static uint32_t try_parse_frame(uint8_t *buf, uint16_t buf_len) {
    if (buf_len < 5) return 0;
    uint16_t magic;
    memcpy(&magic, buf, 2);
    if (magic != 0xAA55) return 0;

    uint8_t type;
    memcpy(&type, buf + 2, 1);
    uint16_t data_len;
    memcpy(&data_len, buf + 3, 2);

    // 只处理地图帧（类型0x01，数据长度192）
    if (type == 0x01 && data_len == 192) {
        uint32_t expected_len = 5 + data_len;
        if (buf_len >= expected_len) {
            parse_map_frame(buf + 5, data_len);
            g_map_updated = 1;        // 通知主循环地图已更新
            return expected_len;
        }
    }
    // 其他帧类型暂不处理
    return 0;
}

/* ========== UART 解析线程入口 ========== */
/**
 * @brief 串口数据解析线程（由 RT-Thread 调度）
 * @param parameter 线程入口参数（未使用）
 * @details 不断从环形缓冲区取数据，尝试匹配帧头 0xAA55，
 *          若找到则调用 try_parse_frame 解析完整帧。
 */
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

            // 在缓冲区中搜索帧头 0xAA55
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
                        break;  // 重新开始搜索
                    }
                }
            }
        } else {
            rt_thread_mdelay(1);
        }
    }
}

/* ========== 视觉复位函数（供参考） ========== */
/*
void try_vision_reset(void) {
    // 条件：允许视觉复位 && 视觉数据有效
    if (!g_allow_vision_reset || !g_vision_valid) return;

    // 仅在控制器空闲时执行
    if (g_ctrl.mode != CTRL_MODE_IDLE) return;

    // 检查小车是否静止（连续5个周期速度<0.02m/s）
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

    // 使用视觉位置重置里程计（保留当前航向）
    Position_Set(g_vision_pos_x, g_vision_pos_y, position.yaw_rad);
    EncoderReset();

    g_allow_vision_reset = 0;
    g_vision_valid = 0;

    // 打印复位信息
    char buf[64];
    int pos_x_mm = (int)(g_vision_pos_x * 1000.0f);
    int pos_y_mm = (int)(g_vision_pos_y * 1000.0f);
    rt_sprintf(buf, "[Vision] Reset to (%d, %d)\r\n", pos_x_mm, pos_y_mm);
    wireless_uart_send_string(buf);
}
*/