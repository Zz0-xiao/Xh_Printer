#include "motor.h"
#include "RS232.h"
#include "delay.h"


//�������״̬��־��ֻ�ܸ�ֵMOTORV,MOTORH,MOTOR_OKû���ϣ�Ԥ��
//�˱�־��λ����ֹͣ������Ӧ�κε������ָ��
uint8_t MotorErrorFlag = MOTOR_OK;
//��MOTORһһ��Ӧ ��ʱ��
TIM_TypeDef* MOTORTIM[] = {TIM14, TIM16};

uint8_t motorState[2] = {MOTOR_STOP, MOTOR_STOP}; //��¼���״̬�����ת��ı�ʱ��ͻ��

//С���ת����һ��־��ʱ
//volatile uint32_t timeOut[] = {TIMEEND, TIMEEND};

void Motor_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //ģʽ�����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //������ͣ��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //ģʽ�����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //������ͣ��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*******************************
���ƣ�MotorDrive57();
���ܣ�57��������������ϡ�ǰ���¡��� ������
������motor_number �������ֱ���MOTORV��ˮƽ���MOTORH˳���ܵߵ�
			motor_mode   ������������MOTOR_STATE
���أ����������communication.h
*******************************/
HAL_StatusTypeDef MotorDrive57(MOTOR_STATE motor_number, MOTOR_STATE motor_mode)
{
    if(MotorErrorFlag != MOTOR_OK)
    {
        TIM_Cmd(MOTORTIM[motor_number], DISABLE);
        return HAL_MOTORERROR;
    }

    if(motorState[motor_number] != motor_mode)
    {
//		if(motor_number == MOTORV)
//			indexKHz = 0;
//		timePeriod[MOTORV] = 47999;
//		timePeriod[MOTORH] = 23999;
//		TIM_SetAutoreload(MOTORTIM[motor_number],timePeriod[motor_number]-1);//��ʼ����ʱ������ֵ
//		TIM_SetCompare1(MOTORTIM[motor_number],(timePeriod[motor_number]-1)/2);

        if((motor_mode != MOTOR_STOP) && (motorState[motor_number] != MOTOR_STOP))
        {
            TIM_Cmd(MOTORTIM[motor_number], DISABLE); //ֹͣ���
            HAL_Delayms(500);
        }

        motorState[motor_number] = motor_mode;//���ĵ��״̬

//        if(timeOut[motor_number] == TIMEEND)
//            timeOut[motor_number] = TIMESTART; //���û�п��������ʱ������

        switch(motor_mode)
        {
        case MOTOR_STOP:
//            timeOut[motor_number] = TIMEEND;
            TIM_Cmd(MOTORTIM[motor_number], DISABLE); //ֹͣ���
            break;
        case MOTOR_MOVE_BD:
            if(motor_number == MOTORV)
                M1DIR_L();
            else
                M2DIR_L();
//				GPIO_WriteBit(MOTORDIR[motor_number].GPIOx,MOTORDIR[motor_number].GPIO_Pin,Bit_RESET);
            TIM_Cmd(MOTORTIM[motor_number], ENABLE);
            break;
        case MOTOR_MOVE_FU:
            if(motor_number == MOTORV)
                M1DIR_H();
            else
                M2DIR_H();
//				GPIO_WriteBit(MOTORDIR[motor_number].GPIOx,MOTORDIR[motor_number].GPIO_Pin,Bit_SET);
            TIM_Cmd(MOTORTIM[motor_number], ENABLE);
            break;
        default:
            break;
        }
    }
    return HAL_OK;
}

/*******************************
���ƣ�Introduction();
���ܣ�ֱ��������ƣ��Ƴ�ֽ��
������STOP��RUN
���أ���
*******************************/

void Put(uint8_t state)
{
    if(state == RUN)
    {
        PUT_H();
    }
    else
        PUT_L();
}




