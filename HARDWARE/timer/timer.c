#include "timer.h"




//��ϵͳʱ��24��Ƶ������Ƶ��Ϊ48MHz/24 = 2MHz�����һ������ʱ��Ϊ0.5us������200������ʱ��Ϊ100us�������TIM3�Ķ�ʱ���ڡ�
void TIM3_Initial(void) //ÿ100us����һ�θ����¼�(�����жϷ������).
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_DeInit(TIM3);                        //���½�Timer����Ϊȱʡֵ
    TIM_InternalClockConfig(TIM3);           //�����ڲ�ʱ�Ӹ�TIM3�ṩʱ��Դ
    TIM_ARRPreloadConfig(TIM3, DISABLE);     //��ֹARRԤװ�ػ����� Ԥװ�ؼĴ��������ݱ��������͵�Ӱ�ӼĴ���

    TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1; //Ԥ��Ƶϵ��Ϊ24-1����������������Ƶ��Ϊ48MHz/24 = 2MHz
//    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;//1us = 1 000 000Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //����ʱ�ӷָ�
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ü�����ģʽΪ���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = 1999;                     //���ü��������С��ÿ200�����Ͳ���һ�������¼�
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);             //������Ӧ�õ�TIM3��

    TIM_ClearFlag(TIM3, TIM_FLAG_Update); //�������жϱ�־
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //ʹ��TIM3�ĸ����ж�
    TIM_Cmd(TIM3, ENABLE); //������ʱ��3

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPriority = 1; //��ռ���ȼ�1��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);                //��ʼ��NVIC�Ĵ���
}


/*******************************
���ƣ�TIM16_Initial();
���ܣ�TIM16��ʼ��
����: ��Ƶֵ
���أ���
*******************************/

//*Tout= ((�Զ���װ��+1)*(ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ+1))/Tclk��

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
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;											//�����������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 , ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 10 - 1;                       														//Ԥ��Ƶϵ��Ϊ10-1����������������Ƶ��Ϊ48MHz/10 = 2MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;																	//TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = (uint16_t) (SystemCoreClock / (periodHz * basePeroid)) - 1; //���ü��������С��
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;																								//����ʱ�ӷָ�:TDTS = Tck_tim

    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;																		//ѡ��ʱ��ģʽ:TIM������ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;												//�Ƚ����ʹ��
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
���ƣ�TIM14_Initial();
���ܣ�TIM14��ʼ��
����: ��Ƶֵ
���أ���
*******************************/

//*Tout= ((�Զ���װ��+1)*(ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ+1))/Tclk��

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

uint16_t time3Usart1ms = 1; //���ڳ�ʱ��ʱ��־
//uint16_t time3Debouncet1ms = 1; //��������
//extern volatile uint32_t timer3_100us = 0;
volatile int delayCounter = 0;//������HAL_Delay()��ȷ��ʱ����������ı�
uint32_t time2Counter1ms = 1;//���費ռ����ʱ����ִ�м��

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








//* �ļ�����void TIMX_Configuration(u16 arr,u16 psc)
//* ��  ��: ��ʱ��3��ʼ�����ú���
//* ��  �ܣ�ˢ��
//* ��  �ߣ�
//* �汾�ţ�1.0.1(2015.03.03)
//*******************************************************************************/
///*�����ж�ʱ��,���㹫ʽ���£�
//*Tout= ((arr+1)*(psc+1))/Tclk��
//*���У�
//*Tclk��TIM3 ������ʱ��Ƶ�ʣ���λΪ Mhz��72MHz
//*Tout��TIM3 ���ʱ�䣨��λΪ us��
//*/
//void TIM3_Configuration(u16 arr,u16 psc)//ͨ�ö�ʱ����2��3��4��5�������ú���
//{
//        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);//ʹ�ܶ�ʱ��3,APB1PeriphClock

//        //��ʱ�� TIM3 ��ʼ��
//        TIM_TimeBaseStructure.TIM_Period = arr; //�����Զ���װ�ؼĴ������ڵ�ֵ
//        TIM_TimeBaseStructure.TIM_Prescaler = psc;//����ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�
//        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM ���ϼ���
//        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //�ڳ�ʼ�� TIM3
//        TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //�����������ж�
//        TIM_Cmd(TIM3, ENABLE); //��ʹ�� TIM3
//
//}


