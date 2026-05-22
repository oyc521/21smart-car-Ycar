#ifndef MAPPING_H
#define MAPPING_H

#include "planner.h"

int mapping_get_digit_for_box_type(BoxTypeEnum_t type);
BoxTypeEnum_t mapping_get_box_type_for_digit(int digit);

#endif