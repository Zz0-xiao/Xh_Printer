#include "timer.h"




//将系统时钟24分频，计数频率为48MHz/24 = 2MHz，则计一个数的时间为0.5us。计满200个数的时间为100us。这就是TIM3的定时周期。
void TIM3_Initial(void) //每100us发生一次更新事件(进入中断服务程序).
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_DeInit(TIM3);                        //重新将Timer设置为缺省值
    TIM_InternalClockConfig(TIM3);           //采用内部时钟给TIM3提供时钟源
    TIM_ARRPreloadConfig(TIM3, DISABLE);     //禁止ARR预装载缓冲器 预装载寄存器的内容被立即传送到影子寄存器

    TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1; //预分频系数为24-1，这样计数器计数频率为48MHz/24 = 2MHz
//    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;//1us = 1 000 000Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //设置时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //设置计数器模式为向上计数模式
    TIM_TimeBaseStructure.TIM_Period = 1999;                     //设置计数溢出大小，每200个数就产生一个更新事件
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);             //将配置应用到TIM3中

    TIM_ClearFlag(TIM3, TIM_FLAG_Update); //清除溢出中断标志
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //使能TIM3的更新中断
    TIM_Cmd(TIM3, ENABLE); //开启定时器3

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPriority = 1; //先占优先级1级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);                //初始化NVIC寄存器
}


/*******************************
名称：TIM16_Initial();
功能：TIM16初始化
参数: 分频值
返回：无
*******************************/

//*Tout= ((自动重装载+1)*(时钟频率除数的预分频值+1))/Tclk；

void TIM16_Initial(uint16_t periodHz)
{
//    periodHz=5000;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 , ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 10 - 1;                       														//预分频系数为10-1，这样计数器计数频率为48MHz/10 = 2MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;																	//TIM向上计数模式
    TIM_TimeBaseStructure.TIM_Period = (uint16_t) (SystemCoreClock / (periodHz * basePeroid)) - 1; //设置计数溢出大小，
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;																								//设置时钟分割:TDTS = Tck_tim

    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;																		//选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;												//比较输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = (uint16_t)(SystemCoreClock / (periodHz * basePeroid)) / 2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM16, &TIM_OCInitStructure);

//    TIM_Cmd(TIM16, ENABLE);

    TIM_CtrlPWMOutputs(TIM16, ENABLE);
}


/*******************************
名称：TIM14_Initial();
功能：TIM14初始化
参数: 分频值
返回：无
*******************************/

//*Tout= ((自动重装载+1)*(时钟频率除数的预分频值+1))/Tclk；

void TIM14_Initial(uint16_t periodHz)
{
//    periodHz=2500;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_4);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);

    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 10-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = (uint16_t) (SystemCoreClock / (periodHz * basePeroid)) - 1;
//    TIM_TimeBaseStructure.TIM_Period = 48000-1;//(uint16_t) (100000 / (periodHz*basePeroid)) - 1;//TimerPeriod;
//    TIM_TimeBaseStructure.TIM_Period = (uint16_t) (100000 / (periodHz * basePeroid)) - 1; //TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
//    TIM_OCInitStructure.TIM_Pulse =24000;//(uint16_t) (100000 / (periodHz*basePeroid))/2;
//    TIM_OCInitStructure.TIM_Pulse = (uint16_t) (100000 / (periodHz * basePeroid)) / 2;
    TIM_OCInitStructure.TIM_Pulse = (uint16_t)(SystemCoreClock / (periodHz * basePeroid)) / 2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM14, &TIM_OCInitStructure);

//    TIM_Cmd(TIM14, ENABLE);

    TIM_CtrlPWMOutputs(TIM14, ENABLE);
}

uint16_t time3Usart1ms = 1; //串口超时计时标志
//uint16_t time3Debouncet1ms = 1; //按键消抖
//extern volatile uint32_t timer3_100us = 0;
volatile int delayCounter = 0;//仅用于HAL_Delay()精确定时器函数，勿改变
uint32_t time2Counter1ms = 1;//外设不占用延时函数执行间隔

void TIM3_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //?? TIM3 ????????
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        time3Usart1ms++;

        if(time2Counter1ms < 10000)time2Counter1ms++;
        if(delayCounter)delayCounter--;

//	if(sensevi == 6)sensevi = 0;
//	if(sensehi == 2)sensehi = 0;
//	if(GPIO_ReadInputDataBit(VSENSOR[sensevi].GPIOx,VSENSOR[sensevi].GPIO_Pin) == Bit_RESET)
//	{
//		motorCurrentPosition[MOTORV] = sensevi+1;
//		if(timeOut[0])timeOut[0] = TIMESTART;
//	}
//	if(GPIO_ReadInputDataBit(HSENSOR[sensehi].GPIOx,HSENSOR[sensehi].GPIO_Pin) == Bit_RESET)
//	{
//		motorCurrentPosition[MOTORH] = sensehi+1;
//		if(timeOut[1]) timeOut[1] = TIMESTART;
//	}
//	sensevi++;
//	sensehi++;
    }
}








//* 文件名：void TIMX_Configuration(u16 arr,u16 psc)
//* 描  述: 定时器3初始化配置函数
//* 功  能：刷新
//* 作  者：
//* 版本号：1.0.1(2015.03.03)
//*******************************************************************************/
///*计算中断时间,计算公式如下：
//*Tout= ((arr+1)*(psc+1))/Tclk；
//*其中：
//*Tclk：TIM3 的输入时钟频率（单位为 Mhz）72MHz
//*Tout：TIM3 溢出时间（单位为 us）
//*/
//void TIM3_Configuration(u16 arr,u16 psc)//通用定时器（2，3，4，5）的配置函数
//{
//        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);//使能定时器3,APB1PeriphClock

//        //定时器 TIM3 初始化
//        TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载寄存器周期的值
//        TIM_TimeBaseStructure.TIM_Prescaler = psc;//设置时钟频率除数的预分频值
//        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割
//        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数
//        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //②初始化 TIM3
//        TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //③允许更新中断
//        TIM_Cmd(TIM3, ENABLE); //⑤使能 TIM3
//
//}



