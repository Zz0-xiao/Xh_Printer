#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f0xx.h"
#include "RS232.h"

#define M1EN_L()	 	GPIO_ResetBits(GPIOA, GPIO_Pin_8)//��ֱ���Ϊ1
#define M1EN_H()  		GPIO_SetBits(GPIOA, GPIO_Pin_8)

#define M2EN_L()		GPIO_ResetBits(GPIOA, GPIO_Pin_12)//ˮƽ���Ϊ2
#define M2EN_H()		GPIO_SetBits(GPIOA, GPIO_Pin_12)

#define M1DIR_L()		GPIO_ResetBits(GPIOC, GPIO_Pin_9)
#define M1DIR_H()		GPIO_SetBits(GPIOC, GPIO_Pin_9)

#define M2DIR_L()		GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define M2DIR_H()		GPIO_SetBits(GPIOA, GPIO_Pin_11)

#define PUT_L()		GPIO_ResetBits(GPIOA, GPIO_Pin_2)
#define PUT_H()		GPIO_SetBits(GPIOA, GPIO_Pin_2)

//#define TIMEEND   0
//#define TIMESTART 1

#define RUN   0
#define STOP   1

//uint8_t motorState[2]; 
extern uint8_t motorState[2]; 
extern TIM_TypeDef* MOTORTIM[];
typedef enum
{
    MOTORV = 0,//��ֱ�������
    MOTORH,//ˮƽ�������  ����λ���ܸı�
//	MOTOR_ENABLE,
//	MOTOR_DISABLE,
    MOTOR_MOVE_FU,//�����ת
    MOTOR_MOVE_BD,//�����ת
//	MOTOR_START,
    MOTOR_STOP,
//	MOTOR_HOLD,
//	MOTORV_INI,
//	MOTORH_INI,
    MOTOR_OK,//����������
//	MOTOR_BUSY,
} MOTOR_STATE;

//uint8_t motorState[2]; //��¼���״̬�����ת��ı�ʱ��ͻ��

void Motor_Init(void);

/*******************************
���ƣ�MotorDrive57();
���ܣ�57��������������ϡ�ǰ���¡��� ������
������motor_number �������ֱ���MOTORV��ˮƽ���MOTORH˳���ܵߵ�
			motor_mode   ������������MOTOR_STATE
���أ����������communication.h
*******************************/
HAL_StatusTypeDef MotorDrive57(MOTOR_STATE motor_number, MOTOR_STATE motor_mode);

void Put(uint8_t state);

#endif


//#define Set(n)    GPIO_SetBits(GPIOA,n) //����Ӧ�ܽ�����ߵ�ƽ
//#define Reset(n)  GPIO_ResetBits(GPIOA,n)//����͵�ƽ

