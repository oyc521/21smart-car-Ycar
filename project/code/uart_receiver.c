#include "uart_receiver.h"
#include "zf_driver_uart.h"
#include "planner.h"
#include "hybrid_controller.h"
#include "position.h"
#include "encoder.h"
#include "task_manager.h"
#include "motor.h"          // Ϊ�˻�ȡ car_ctrl.current_vx/vy
#include <string.h>
#include <stdio.h>
#include <rtthread.h>

/* �ⲿȫ�ֱ��� */
extern GridMap g_grid_map;
extern GameState g_game_state;
extern HybridController g_ctrl;
extern Position_t position;
extern volatile uint8_t waiting_map;
extern volatile uint8_t need_map_update;
extern CarController_t car_ctrl;   // ���ڻ�ȡ��ǰ�ٶ�
extern rt_mutex_t g_map_mutex;

/* ���λ�����ʵ�� */
ring_buffer_t g_uart_rb = { .head = 0, .tail = 0 };

/* ��ͼ���±�־ */
uint8_t g_map_updated = 0;

/* �Ӿ�λ��У����ر��� */
// ԭ static uint8_t g_vision_valid = 0; ��Ϊȫ�ֻ��ṩ����
static uint8_t g_vision_valid = 0;   // �Կɾ�̬��ͨ����������
static float g_vision_pos_x = 0.0f;
static float g_vision_pos_y = 0.0f;

// ��ȡ�Ӿ�λ��
void get_vision_position(float *x, float *y) {
    *x = g_vision_pos_x;
    *y = g_vision_pos_y;
}

// ��ȡ�Ӿ���Ч��־
uint8_t is_vision_valid(void) {
    return g_vision_valid;
}
void clear_vision_valid(void) {
    g_vision_valid = 0;
}
// �� parse_map_frame ���ҵ� P ��ʱ������ g_vision_valid = 1
// ���������У������޸ģ�����ȷ�� g_vision_valid ����ȡ�� P ʱ���� 1��
/* ���λ�����д��һ���ֽڣ��жϵ��ã� */
static void rb_push(uint8_t data) {
    uint16_t next = (g_uart_rb.head + 1) % UART_RX_BUF_SIZE;
    if (next != g_uart_rb.tail) {
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    } else {
        // ����������������ɵ�����
        g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
        g_uart_rb.buffer[g_uart_rb.head] = data;
        g_uart_rb.head = next;
    }
}

/* �ӻ��λ�������ȡһ���ֽڣ��̵߳��ã� */
static uint8_t rb_pop(uint8_t *data) {
    if (g_uart_rb.head == g_uart_rb.tail) return 0;
    *data = g_uart_rb.buffer[g_uart_rb.tail];
    g_uart_rb.tail = (g_uart_rb.tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}

/* ========== UART1 �жϻص����� isr.c �е��ã� ========== */
void uart1_rx_callback(void) {
    uint8_t data;
    while (uart_query_byte(UART_1, &data)) {
        rb_push(data);
    }
}

/* ��ʼ�� UART1 �����λ������ж� */
void uart_receive_init(void) {
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    uart_rx_interrupt(UART_1, 1);
    NVIC_SetPriority(LPUART1_IRQn, 2);
}


/* ========== ������ͼ֡������ 0x01�� ========== */
static void parse_map_frame(const uint8_t *data, uint32_t len) {
    if (len != 192) {
        wireless_uart_send_string("Map frame data length error!\r\n");
        return;
    }

    // ��ԭʼ��ͼ����ת��Ϊ�ı���12�У�ÿ��16���ַ� + ���з���
    char map_text[12 * 17 + 1];  // ÿ��16�ַ�+���У�ĩβ'\0'
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

    // �ӵ�ͼ�ı���ȡС��λ�ã��Ӿ�У���ã���ֻ������һ���ҵ���
    int found_p = 0;
    for (int r = 0; r < 12; r++) {
        for (int c = 0; c < 16; c++) {
            if (map_text[r * 17 + c] == 'P') {  // ÿ��ʵ��ռ17�ֽڣ�16�ַ�+���У�
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

    // �ڻ����������¼��ص�ͼ��ˢ�¡���������
    if (g_map_mutex) {
        rt_mutex_take(g_map_mutex, RT_WAITING_FOREVER);
    }

    load_map_from_text(map_text, &g_grid_map, &g_game_state);
    refresh_grid_map(&g_game_state, &g_grid_map);
    assign_boxes_to_destinations();

    if (g_map_mutex) {
        rt_mutex_release(g_map_mutex);
    }

    // ���Դ�ӡ�������꣨�������ã�
    char dbg[128];
    for (int i = 0; i < g_game_state.num_boxes; i++) {
        rt_sprintf(dbg, "Box %d motion: (%d,%d)\r\n", i,
                   (int)(g_game_state.boxes[i].x * 1000), (int)(g_game_state.boxes[i].y * 1000));
        wireless_uart_send_string(dbg);
    }

    // waiting_map ����� MAP_READY �¼��� main.c ����ѭ��ͳһ����
}


/* ========== �Ӿ�У��������ѭ�����ã� ========== */
/*void try_vision_reset(void) {
    // ����ͬʱ���㣺����У�� && ����Ч�Ӿ�λ��
    if (!g_allow_vision_reset || !g_vision_valid) return;

    // �������ڷǿ���ģʽʱ��ֹУ��
    if (g_ctrl.mode != CTRL_MODE_IDLE) return;

    // Ҫ��ֹ������5�μ���ٶ�<0.02m/s
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

    // ִ��У����Position_Set �ڲ�������λ�Ƶ�ͨ�˲�����
    Position_Set(g_vision_pos_x, g_vision_pos_y, position.yaw_rad);
    EncoderReset();

    g_allow_vision_reset = 0;
    g_vision_valid = 0;

    // ��ӡУ�����λ��
    char buf[64];
    int pos_x_mm = (int)(g_vision_pos_x * 1000.0f);
    int pos_y_mm = (int)(g_vision_pos_y * 1000.0f);
    rt_sprintf(buf, "[Vision] Reset to (%d, %d)\r\n", pos_x_mm, pos_y_mm);
    wireless_uart_send_string(buf);
}*/
/* ========== ֡������ ========== */
static uint32_t try_parse_frame(uint8_t *buf, uint16_t buf_len) {
    if (buf_len < 5) return 0;
    uint16_t magic;
    memcpy(&magic, buf, 2);
    if (magic != 0xAA55) return 0;

    uint8_t type;
    memcpy(&type, buf + 2, 1);
    uint16_t data_len;
    memcpy(&data_len, buf + 3, 2);

    // ��������ͼ֡
    if (type == 0x01 && data_len == 192) {
        uint32_t expected_len = 5 + data_len;
        if (buf_len >= expected_len) {
            parse_map_frame(buf + 5, data_len);
            g_map_updated = 1;
            return expected_len;
        }
    }
    // ��������֡�ݲ�����
    return 0;
}

/* ========== �����߳���ں��� ========== */
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

            // �ڻ������в���֡ͷ 0xAA55
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