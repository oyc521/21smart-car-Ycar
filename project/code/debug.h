#ifndef _MY_DEBUG_H_
#define _MY_DEBUG_H_

#include <rtthread.h>
#include <stdint.h>

/* 调试功能总开关：1-启用，0-完全禁用 */
#define DEBUG_ENABLE 1

/* 每组可调参数通道数（与SeekFree上位机通道数一致） */
#define DEBUG_CHANNEL_COUNT 8
#define DEBUG_GROUP_COUNT   3
#define DEBUG_CMD_QUEUE_SIZE 8
#define DEBUG_CMD_LINE_MAX   128

/* 参数组枚举 */
typedef enum {
    DEBUG_GROUP_SPEED = 0,     /* 速度环/电机底层参数 */
    DEBUG_GROUP_ANGLE,         /* 角度环PID参数 */
    DEBUG_GROUP_TRACKING,      /* 路径跟踪参数 */
} DebugGroup_t;

/* 单个参数通道描述 */
typedef struct {
    const char *name;          /* 参数名称（用于状态显示） */
    float *param_ptr;          /* 指向实际变量的指针 */
    float default_value;       /* 默认值（预留扩展） */
    float min_val;             /* 最小值约束 */
    float max_val;             /* 最大值约束 */
} DebugChannel_t;

/* 函数声明（无条件，源文件根据 DEBUG_ENABLE 提供实现） */
void debug_module_init(void);
void debug_process_commands(void);
void debug_apply_parameter(uint8_t channel, float value);
void debug_seekfree_loop(void);
void debug_thread_entry(void *parameter);

#endif