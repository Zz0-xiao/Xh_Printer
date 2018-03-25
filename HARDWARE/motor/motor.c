#include "motor.h"
#include "RS232.h"
#include "delay.h"


//电机错误状态标志，只能赋值MOTORV,MOTORH,MOTOR_OK没用上，预留
//此标志置位则电机停止并不响应任何电机驱动指令
uint8_t MotorErrorFlag = MOTOR_OK;
//与MOTOR一一对应 定时器
TIM_TypeDef* MOTORTIM[] = {TIM14, TIM16};

uint8_t motorState[2] = {MOTOR_STOP, MOTOR_STOP}; //记录电机状态，电机转向改变时不突变

//小电机转到下一标志计时
//volatile uint32_t timeOut[] = {TIMEEND, TIMEEND};

void Motor_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //模式：输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //输出类型，推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //模式：输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //输出类型，推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*******************************
名称：MotorDrive57();
功能：57电机驱动，包括上、前，下、后 不声明
参数：motor_number 电机号竖直电机MOTORV、水平电机MOTORH顺序不能颠倒
			motor_mode   电机驱动方向见MOTOR_STATE
返回：操作结果见communication.h
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
//		TIM_SetAutoreload(MOTORTIM[motor_number],timePeriod[motor_number]-1);//初始化定时器计数值
//		TIM_SetCompare1(MOTORTIM[motor_number],(timePeriod[motor_number]-1)/2);

        if((motor_mode != MOTOR_STOP) && (motorState[motor_number] != MOTOR_STOP))
        {
            TIM_Cmd(MOTORTIM[motor_number], DISABLE); //停止电机
            HAL_Delayms(500);
        }

        motorState[motor_number] = motor_mode;//更改电机状态

//        if(timeOut[motor_number] == TIMEEND)
//            timeOut[motor_number] = TIMESTART; //如果没有开启电机超时，则开启

        switch(motor_mode)
        {
        case MOTOR_STOP:
//            timeOut[motor_number] = TIMEEND;
            TIM_Cmd(MOTORTIM[motor_number], DISABLE); //停止电机
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
名称：Introduction();
功能：直流电机控制，推出纸张
参数：STOP，RUN
返回：无
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




