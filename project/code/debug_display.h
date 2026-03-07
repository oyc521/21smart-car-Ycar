#ifndef DEBUG_DISPLAY_H
#define DEBUG_DISPLAY_H

#include <stdint.h>

// 调试开关，由外部定义（可在编译选项或配置文件中设置）
#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE 0   // 默认关闭，调试时可改为0关闭显示线程，改为1打开地图显示线程
#endif

// 初始化debug显示模块（创建显示线程）
void debug_display_init(void);

// 逐飞助手调试接口（预留，后续实现）
void debug_send_to_assistant(const uint8_t *data, uint32_t len);

#endif