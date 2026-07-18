#ifndef _MY_DEBUG_H_
#define _MY_DEBUG_H_

#include <rtthread.h>
#include <stdint.h>

#define DEBUG_ENABLE 1

#if DEBUG_ENABLE

#define DEBUG_CHANNEL_COUNT 8
#define DEBUG_GROUP_COUNT   4
#define DEBUG_CMD_QUEUE_SIZE 8
#define DEBUG_CMD_LINE_MAX   128

typedef enum {
    DEBUG_GROUP_SPEED = 0,
    DEBUG_GROUP_ANGLE,
    DEBUG_GROUP_TRACKING,
    DEBUG_GROUP_LATERAL
} DebugGroup_t;

typedef struct {
    const char *name;
    float *param_ptr;
    float default_value;
    float min_val;
    float max_val;
} DebugChannel_t;

void debug_module_init(void);
void debug_process_commands(void);
void debug_apply_parameter(uint8_t channel, float value);
void debug_seekfree_loop(void);
void debug_thread_entry(void *parameter);

#else

static inline void debug_module_init(void) {}
static inline void debug_process_commands(void) {}
static inline void debug_apply_parameter(uint8_t channel, float value) {}
static inline void debug_seekfree_loop(void) {}
static inline void debug_thread_entry(void *parameter) {}

#endif

#endif
