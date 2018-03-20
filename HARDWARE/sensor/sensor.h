#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f0xx.h"

//////////////////////////////////////////////////////////////////////////////////


#define LSENSOR	  GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_6)//�͵�λ�ô�������ֱ����
#define HSENSOR	  GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_7)

#define LINFRARE	  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)//�͵ĺ��⴫������������������ s1
#define HINFRARE	  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)
#define RINFRARE	  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)//ˮƽ��λ��

#define LSENSOR_RESPONSE	1//������λ�õ�
#define HSENSOR_RESPONSE	2//������λ�ø�

//#define LINFRARE_RESPONSE	1//�����ֽ��
//#define HINFRARE_RESPONSE	2//���ֽ�ű�����

//#define RINFRARE_RESPONSE	1//��λ������λ�����̺���

void SENSOR_Init(void); //IO��ʼ��

uint8_t SENSOR_Scan(void);
//uint8_t INFRARE_Scan(void);

//���ܣ����ˮƽλ���Ƿ�λ
//uint8_t RNFRARE_Scan(void);
#endif
