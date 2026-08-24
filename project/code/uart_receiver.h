#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H

#include <rtthread.h>
#include <stdint.h>

/**
 * @file uart_receiver.h
 * @brief UART1 数据接收与解析模块 —— 头文件
 * @details 定义环形缓冲区结构、外部全局变量和函数接口。
 */

/* 环形缓冲区大小（必须足够大，以容纳完整的一帧数据） */
#define UART_RX_BUF_SIZE    2048

/* 解析缓冲区大小（用于暂存从环形缓冲区取出的数据，进行帧解析） */
#define PARSE_BUF_SIZE      2048

/**
 * @brief 环形缓冲区结构
 */
typedef struct {
    uint8_t buffer[UART_RX_BUF_SIZE];
    volatile uint16_t head;     /* 写指针（由中断服务程序更新） */
    volatile uint16_t tail;     /* 读指针（由解析线程更新） */
} ring_buffer_t;

/* 全局环形缓冲区实例（在 uart_receiver.c 中定义） */
extern ring_buffer_t g_uart_rb;

/* 地图更新标志（由解析线程设置，主循环查询） */
extern uint8_t g_map_updated;

/* 需要请求地图标志（由混合控制器设置，主循环发送请求） */
extern volatile uint8_t need_map_update;

/**
 * @brief 初始化 UART1 接收（配置波特率、引脚、开启接收中断）
 */
void uart_receive_init(void);

/**
 * @brief UART 解析线程入口函数（由 RT-Thread 调度）
 * @param parameter 线程入口参数（未使用）
 */
void parse_uart_data_thread_entry(void *parameter);

/**
 * @brief 触发路径规划（外部函数，由规划器实现）
 * @note 该函数在 uart_receiver.c 中未实现，由其他模块（如 planner）提供
 */
void trigger_planning(void);

/**
 * @brief 获取最近一次视觉定位的位置（图像坐标系）
 * @param x, y 输出位置指针
 */
void get_vision_position(float *x, float *y);

/**
 * @brief 检查视觉定位数据是否有效
 * @return 1有效，0无效
 */
uint8_t is_vision_valid(void);

/**
 * @brief 清除视觉定位有效标志
 */
void clear_vision_valid(void);

#endif /* UART_RECEIVER_H */