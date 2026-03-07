#ifndef MAPPING_H
#define MAPPING_H

#include "planner.h"

// 初始化映射表（将所有映射设为-1）
void mapping_init(void);

// 设置箱子类型对应的数字
void mapping_set(BoxTypeEnum_t type, int digit);

// 根据箱子类型获取所需数字，返回-1表示无映射
int mapping_get_digit_for_box_type(BoxTypeEnum_t type);

#endif