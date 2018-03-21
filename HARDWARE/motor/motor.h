#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f0xx.h"
#include "RS232.h"

#define M1EN_L()	 	GPIO_ResetBits(GPIOA, GPIO_Pin_8)//竖直电机为1
#define M1EN_H()  		GPIO_SetBits(GPIOA, GPIO_Pin_8)

#define M2EN_L()		GPIO_ResetBits(GPIOA, GPIO_Pin_12)//水平电机为2
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
    MOTORV = 0,//竖直电机索引
    MOTORH,//水平电机索引  此两位不能改变
//	MOTOR_ENABLE,
//	MOTOR_DISABLE,
    MOTOR_MOVE_FU,//电机正转
    MOTOR_MOVE_BD,//电机反转
//	MOTOR_START,
    MOTOR_STOP,
//	MOTOR_HOLD,
//	MOTORV_INI,
//	MOTORH_INI,
    MOTOR_OK,//允许电机工作
//	MOTOR_BUSY,
} MOTOR_STATE;

//uint8_t motorState[2]; //记录电机状态，电机转向改变时不突变

void Motor_Init(void);

/*******************************
名称：MotorDrive57();
功能：57电机驱动，包括上、前，下、后 不声明
参数：motor_number 电机号竖直电机MOTORV、水平电机MOTORH顺序不能颠倒
			motor_mode   电机驱动方向见MOTOR_STATE
返回：操作结果见communication.h
*******************************/
HAL_StatusTypeDef MotorDrive57(MOTOR_STATE motor_number, MOTOR_STATE motor_mode);

void Put(uint8_t state);

#endif


//#define Set(n)    GPIO_SetBits(GPIOA,n) //将对应管脚输出高电平
//#define Reset(n)  GPIO_ResetBits(GPIOA,n)//输出低电平

