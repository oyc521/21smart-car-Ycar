#include "uart4_recognition.h"
#include "zf_driver_uart.h"
#include "zf_device_wireless_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "mapping.h"

// ---------- 环形缓冲区 ----------
typedef struct {
    uint8_t buffer[UART4_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;
static ring_buffer_t g_uart4_rb = { .head = 0, .tail = 0 };

// ---------- 中断回调 ----------
void uart4_rx_callback(void)
{
    uint8_t data;
    while (LPUART_GetStatusFlags(LPUART4) & kLPUART_RxDataRegFullFlag) {
        data = LPUART_ReadByte(LPUART4);
        uint16_t next = (g_uart4_rb.head + 1) % UART4_RX_BUF_SIZE;
        if (next != g_uart4_rb.tail) {
            g_uart4_rb.buffer[g_uart4_rb.head] = data;
            g_uart4_rb.head = next;
        }
    }
}

// ---------- 读取一行（修复超时计算，避免除零） ----------
static int read_line(char *line_buf, int max_len, uint32_t timeout_ms)
{
    uint32_t start = rt_tick_get();
    int idx = 0;
    while (1) {
        uint8_t ch;
        if (g_uart4_rb.head != g_uart4_rb.tail) {
            ch = g_uart4_rb.buffer[g_uart4_rb.tail];
            g_uart4_rb.tail = (g_uart4_rb.tail + 1) % UART4_RX_BUF_SIZE;
            if (idx < max_len - 1) {
                line_buf[idx++] = ch;
                if (ch == '\n') {
                    line_buf[idx] = '\0';
                    return 1;
                }
            } else {
                idx = 0;  // 行过长，丢弃并重新开始
            }
        } else {
            // 修复：避免 1000/RT_TICK_PER_SECOND 整数除零或向下取整问题
            if (rt_tick_get() - start > (timeout_ms * RT_TICK_PER_SECOND) / 1000) {
                return 0;
            }
            rt_thread_mdelay(5);
        }
    }
}

// ---------- 发送命令并等待应答（带内部重试） ----------
static int send_command_and_wait(const char *cmd, int *out_digit, BoxTypeEnum_t *out_type)
{
    char line[LINE_BUF_SIZE];
    const int MAX_RETRIES = 3;      // 总共尝试3次（首次 + 2次重试）
    const uint32_t LINE_TIMEOUT = 3000;  // 适当延长超时，给视觉识别更多时间

    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        // 清空缓冲区，发送命令
        g_uart4_rb.head = g_uart4_rb.tail = 0;
        uart_write_string(UART_4, cmd);

        if (read_line(line, sizeof(line), LINE_TIMEOUT)) {
            wireless_uart_send_string("[RAW] ");
            wireless_uart_send_string(line);

            char buf[32];

            // ---------- 数字识别 ----------
            if (sscanf(line, "digit:%31s", buf) == 1) {
                // OpenArt 识别失败时返回 "digit:-1"
                if (strcmp(buf, "-1") == 0) {
                    return 0;
                }

                int digit = -1;
                // 处理 "numXX" 格式（如 "num05"）
                if (strncmp(buf, "num", 3) == 0) {
                    int d1 = buf[3] - '0';
                    int d2 = buf[4] - '0';
                    if (d1 >= 0 && d1 <= 9 && d2 >= 0 && d2 <= 9) {
                        digit = d2;   // 取最后一位数字
                    }
                } else {
                    // 直接解析整数
                    sscanf(buf, "%d", &digit);
                }

                if (digit >= 0 && digit <= 9) {
                    if (out_digit) *out_digit = digit;
                    return 1;
                }
                return 0;
            }

            // ---------- 箱子类型识别 ----------
            if (sscanf(line, "box:%31s", buf) == 1) {
                wireless_uart_send_string("[PARSED] ");
                wireless_uart_send_string(buf);

                int len = strlen(buf);
                if (len > 0 && buf[len-1] >= '0' && buf[len-1] <= '9') {
                    int digit = buf[len-1] - '0';
                    BoxTypeEnum_t type = mapping_get_box_type_for_digit(digit);
                    if (out_type) *out_type = type;
                    return 1;
                }
            }
        }

        // 如果不是最后一次尝试，短暂延迟后重试
        if (retry < MAX_RETRIES - 1) {
            rt_thread_mdelay(50);
        }
    }

    return 0;   // 所有重试均失败
}

int uart4_request_digit(int *out_digit)
{
    return send_command_and_wait("DETECT_DEST\n", out_digit, NULL);
}

int uart4_request_box_type(BoxTypeEnum_t *out_type)
{
    return send_command_and_wait("DETECT_BOX\n", NULL, out_type);
}

// ---------- 初始化 ----------
void uart4_recognition_init(void)
{
    uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);
    uart_rx_interrupt(UART_4, ZF_ENABLE);
}