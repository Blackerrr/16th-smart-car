#ifndef __ADC_H
#define __ADC_H
#include "headfile.h"

#define  NM     5                     // 滑动平均滤波队列长度

uint8 adc_caiji(void);
void return_to_one(uint8 i);
void return_to_one_xianfu(uint8 i);
void adc_config(void);
#endif
