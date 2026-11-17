#ifndef __ENCODER_H
#define __ENCODER_H

#include "headfile.h"

extern int temp_EX;//实例变量（文档教学）

typedef enum
{
    ENCODER1,
	ENCODER2
}encoder_index_enum;

void encoder_init(encoder_index_enum encoder_n);
int32 encoder_get_count(encoder_index_enum encoder_n,uint8 reversal);

#endif
