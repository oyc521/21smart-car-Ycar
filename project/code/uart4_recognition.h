#ifndef UART4_RECOGNITION_H
#define UART4_RECOGNITION_H

#include <rtthread.h>
#include <stdint.h>
#include "planner.h"

#define UART4_RX_BUF_SIZE    256
#define LINE_BUF_SIZE        128

// 初始化 UART4 识别模块（中断接收）
void uart4_recognition_init(void);

// 发送命令并等待识别结果（阻塞调用，超时返回0）
int uart4_request_digit(int *out_digit);
int uart4_request_box_type(BoxTypeEnum_t *out_type);

// 可选的解析线程入口（若需后台连续解析，可创建线程）
void uart4_recognition_thread_entry(void *parameter);

#endif