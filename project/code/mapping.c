#include "mapping.h"

// 箱子类型到数字的映射（0米老鼠,1皮卡丘,2海绵宝宝,3喜羊羊,4唐老鸭,5哪吒,6大头儿子,7猪猪侠,8葫芦娃,9灰太狼）
int mapping_get_digit_for_box_type(BoxTypeEnum_t type)
{
    switch (type) {
        case BOX_TYPE_MICKEY:      return 0;
        case BOX_TYPE_PIKACHU:     return 1;
        case BOX_TYPE_SPONGEBOB:   return 2;
        case BOX_TYPE_XIYANGYANG:  return 3;
        case BOX_TYPE_DONALD:      return 4;
        case BOX_TYPE_NEZHA:       return 5;
        case BOX_TYPE_DATOUEZI:    return 6;
        case BOX_TYPE_PIGMAN:      return 7;
        case BOX_TYPE_HULUWA:      return 8;
        case BOX_TYPE_HUITAILANG:  return 9;
        default: return -1;
    }
}

// 数字到箱子类型的反向映射
BoxTypeEnum_t mapping_get_box_type_for_digit(int digit)
{
    switch (digit) {
        case 0: return BOX_TYPE_MICKEY;
        case 1: return BOX_TYPE_PIKACHU;
        case 2: return BOX_TYPE_SPONGEBOB;
        case 3: return BOX_TYPE_XIYANGYANG;
        case 4: return BOX_TYPE_DONALD;
        case 5: return BOX_TYPE_NEZHA;
        case 6: return BOX_TYPE_DATOUEZI;
        case 7: return BOX_TYPE_PIGMAN;
        case 8: return BOX_TYPE_HULUWA;
        case 9: return BOX_TYPE_HUITAILANG;
        default: return BOX_TYPE_UNKNOWN;
    }
}