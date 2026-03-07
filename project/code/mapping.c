#include "mapping.h"

// 假设最大枚举值为 BOX_TYPE_CARTOON_2，根据实际扩展
#define BOX_TYPE_MAX  (BOX_TYPE_CARTOON_2 + 1)

static int s_box_to_digit[BOX_TYPE_MAX];

void mapping_init(void)
{
    for (int i = 0; i < BOX_TYPE_MAX; i++) {
        s_box_to_digit[i] = -1;   // 初始无映射
    }
}

void mapping_set(BoxTypeEnum_t type, int digit)
{
    if (type >= 0 && type < BOX_TYPE_MAX) {
        s_box_to_digit[type] = digit;
    }
}

int mapping_get_digit_for_box_type(BoxTypeEnum_t type)
{
    if (type < 0 || type >= BOX_TYPE_MAX) return -1;
    return s_box_to_digit[type];
}