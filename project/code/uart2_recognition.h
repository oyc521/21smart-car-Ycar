#ifndef UART2_RECOGNITION_H
#define UART2_RECOGNITION_H

#include <rtthread.h>
#include <stdint.h>
#include "planner.h"   // 包含箱子类型枚举

#define UART2_RX_BUF_SIZE    256
#define LINE_BUF_SIZE        128

// 识别结果类型
typedef enum {
    RECOG_TYPE_NONE = 0,
    RECOG_TYPE_BOX,      // 箱子类型识别结果
    RECOG_TYPE_DEST      // 目的地数字识别结果
} RecogType_t;

// 识别结果数据结构
typedef struct {
    RecogType_t type;
    union {
        BoxTypeEnum_t box_type;   // 箱子类型
        int digit;                // 目的地数字
    } data;
} RecognitionResult_t;

// 全局识别结果和事件
extern RecognitionResult_t g_recognition_result;
extern rt_event_t g_recog_event;

// 初始化UART2
void uart2_recognition_init(void);

// UART2解析线程入口
void uart2_recognition_thread_entry(void *parameter);

// 发送识别命令
void request_box_recognition(void);
void request_dest_recognition(void);
void assign_boxes_to_destinations(void);

#endif