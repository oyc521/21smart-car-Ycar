#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H

#include <rtthread.h>
#include <stdint.h>

/* 环形缓冲区大小，必须大于最大一帧数据长度 */
#define UART_RX_BUF_SIZE    2048

/* 解析缓冲区大小，必须大于最大帧长度 */
#define PARSE_BUF_SIZE      2048

/* 环形缓冲区结构体 */
typedef struct {
    uint8_t buffer[UART_RX_BUF_SIZE];
    volatile uint16_t head;     /* 写入索引（中断中更新） */
    volatile uint16_t tail;     /* 读取索引（线程中更新） */
} ring_buffer_t;

/* 全局环形缓冲区实例 */
extern ring_buffer_t g_uart_rb;

/* 地图更新标志（由解析线程设置，主线程查询） */
extern uint8_t g_map_updated;
/* 需要请求地图标志（由混合控制器设置，主线程发送请求） */
extern uint8_t need_map_update;

/* 初始化串口接收（配置 UART1，开启中断） */
void uart_receive_init(void);

/* 解析线程入口函数 */
void parse_uart_data_thread_entry(void *parameter);

/* 外部触发规划函数（在 uart_receiver.c 中定义） */
void trigger_planning(void);

// uart_receiver.h 中添加以下函数声明（在文件末尾）
void get_vision_position(float *x, float *y);
uint8_t is_vision_valid(void);
void clear_vision_valid(void);

#endif