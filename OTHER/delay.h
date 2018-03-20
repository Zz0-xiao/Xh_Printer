#ifndef _DELAY_H_
#define _DELAY_H_

#include "stm32f0xx.h"
void Delay_init(void);
void Delay_us(int32_t nus);
void Delay_ms(int16_t nms);
void HAL_Delayms(int ms);
#endif
