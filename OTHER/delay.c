#include "delay.h"
#include "timer.h"
static int8_t  fac_us = 0; //us
static int16_t fac_ms = 0; //ms

void Delay_init()
{
//    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//选择外部时钟  HCLK/8
	 RCC_HSEConfig(RCC_HSE_OFF);
    RCC_LSEConfig(RCC_LSE_OFF);
	
    fac_us = SystemCoreClock / 8000000;	//为系统时钟的1/8
    fac_ms = (int16_t)fac_us * 1000; //每个ms需要的systick时钟数
}
//延时Nus
void Delay_us(int32_t nus)
{
    int32_t temp;
    SysTick->LOAD = nus * fac_us; //时间加载
    SysTick->VAL = 0x00;      //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;        //开始倒数
    do
    {
        temp = SysTick->CTRL;
    }
    while(temp & 0x01 && !(temp & (1 << 16))); //等待时间到达
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;     //关闭计数器
    SysTick->VAL = 0X00;      //清空计数器
}
//延时Nms

void Delay_ms(int16_t nms)
{
    int32_t temp;
    SysTick->LOAD = (int32_t)nms * fac_ms; //时间加载(SysTick->LOAD为24bit)
    SysTick->VAL = 0x00;          //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;        //开始倒数
    do
    {
        temp = SysTick->CTRL;
    }
    while(temp & 0x01 && !(temp & (1 << 16))); //等待时间到达
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;     //关闭计数器
    SysTick->VAL = 0X00;      //清空计数器
}

//void Delay_10ms(uint32_t ms)
//{
//    int i = 0;
//    while(ms--)
//    {
//        i = 1680;
//        while(i--)
//        {
//            IWDG_ReloadCounter();
//        }
//    }
//}

//void HAL_Delay(uint32_t time1ms)
//{
//	int i = 0;
//	while(time1ms--)
//	{
//		i=1680;
//		while(i--)
//		{
//			IWDG_ReloadCounter();
//		}
//	}
//}

/*******************************
名称：HAL_Delayms(int ms);
功能：精确1ms定时器
参数：
返回：无
*******************************/

void HAL_Delayms(int ms)
{
	delayCounter = ms;
	while(delayCounter>1)
	{
		IWDG_ReloadCounter(); 
	}		
}

