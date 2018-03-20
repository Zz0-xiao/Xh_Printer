#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f0xx.h"
extern  uint16_t time3Usart1ms;//���ڳ�ʱ��ʱ��־
//extern uint16_t time3Debouncet1ms;//��������
extern volatile int delayCounter;

extern uint32_t time2Counter1ms;//���費ռ����ʱ����ִ�м��

//<o>TIM14��TIM16 PWM���������趨(Hz)
//<100=>100Hz
//<1000=>1KHz
//<1000000=>1MHz
#define basePeroid 10

//#define TIMEEND   0


void TIM3_Initial(void); //ÿ1 Ms����һ�θ����¼�(�����жϷ������).

void TIM16_Initial(uint16_t periodHz);

void TIM14_Initial(uint16_t periodHz);

#endif
