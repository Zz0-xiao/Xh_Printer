#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f0xx.h"

//////////////////////////////////////////////////////////////////////////////////


#define LSENSOR	  GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_6)//低的位置传感器竖直方向
#define HSENSOR	  GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_7)

#define LINFRARE	  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)//低的红外传感器检测有无物体掉落
#define HINFRARE	  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)
#define RINFRARE	  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)//水平复位用

#define LSENSOR_RESPONSE	1//传感器位置低
#define HSENSOR_RESPONSE	2//传感器位置高

//#define LINFRARE_RESPONSE	1//检测有纸张
//#define HINFRARE_RESPONSE	2//检测纸张被拿走

//#define RINFRARE_RESPONSE	1//复位传感器位置托盘横向

void SENSOR_Init(void); //IO初始化

uint8_t SENSOR_Scan(void);
//uint8_t INFRARE_Scan(void);

//功能：检测水平位置是否复位
//uint8_t RNFRARE_Scan(void);
#endif
