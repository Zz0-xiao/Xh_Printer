#include "delay.h"
#include "timer.h"
static int8_t  fac_us = 0; //us
static int16_t fac_ms = 0; //ms

void Delay_init()
{
//    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//ѡ���ⲿʱ��  HCLK/8
	 RCC_HSEConfig(RCC_HSE_OFF);
    RCC_LSEConfig(RCC_LSE_OFF);
	
    fac_us = SystemCoreClock / 8000000;	//Ϊϵͳʱ�ӵ�1/8
    fac_ms = (int16_t)fac_us * 1000; //ÿ��ms��Ҫ��systickʱ����
}
//��ʱNus
void Delay_us(int32_t nus)
{
    int32_t temp;
    SysTick->LOAD = nus * fac_us; //ʱ�����
    SysTick->VAL = 0x00;      //��ռ�����
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;        //��ʼ����
    do
    {
        temp = SysTick->CTRL;
    }
    while(temp & 0x01 && !(temp & (1 << 16))); //�ȴ�ʱ�䵽��
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;     //�رռ�����
    SysTick->VAL = 0X00;      //��ռ�����
}
//��ʱNms

void Delay_ms(int16_t nms)
{
    int32_t temp;
    SysTick->LOAD = (int32_t)nms * fac_ms; //ʱ�����(SysTick->LOADΪ24bit)
    SysTick->VAL = 0x00;          //��ռ�����
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;        //��ʼ����
    do
    {
        temp = SysTick->CTRL;
    }
    while(temp & 0x01 && !(temp & (1 << 16))); //�ȴ�ʱ�䵽��
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;     //�رռ�����
    SysTick->VAL = 0X00;      //��ռ�����
}


/*******************************
���ƣ�HAL_Delayms(int ms);
���ܣ���ȷ1ms��ʱ��
������
���أ���
*******************************/

void HAL_Delayms(int ms)
{
	delayCounter = ms;
	while(delayCounter>1)
	{
		IWDG_ReloadCounter(); 
	}		
}

