#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f0xx.h"
extern  uint16_t time3Usart1ms;//串口超时计时标志
//extern uint16_t time3Debouncet1ms;//按键消抖
extern volatile int delayCounter;

extern uint32_t time2Counter1ms;//外设不占用延时函数执行间隔

//<o>TIM14、TIM16 PWM基数周期设定(Hz)
//<100=>100Hz
//<1000=>1KHz
//<1000000=>1MHz
#define basePeroid 10

//#define TIMEEND   0


void TIM3_Initial(void); //每1 Ms发生一次更新事件(进入中断服务程序).

void TIM16_Initial(uint16_t periodHz);

void TIM14_Initial(uint16_t periodHz);

#endif
