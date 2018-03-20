#ifndef __RS232_H__
#define __RS232_H__

#include "stm32f0xx.h"


#define MAXREVSIZE 512		//�����������ֽ��� 512

//extern uint16_t UART1RXDataLenth��//UART1�������ݳ���
//HAL_StatusTypeDef UARTFaultStatus; //���ڱ���ʱ

extern uint8_t UART1RevData[MAXREVSIZE];
//extern uint8_t UART2RevData[MAXREVSIZE];

extern uint16_t UART1RXDataLenth;//UART1�������ݳ���


typedef enum
{
    HAL_INI = 0,
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT,

    HAL_UART1TIMEOUT,//����1���ճ�ʱ
    HAL_UART2TIMEOUT,//����2���ճ�ʱ
    HAL_UART1RXFULL,//����1װ
    HAL_UART2RXFULL,

    HAL_NULL,//û�н��յ�����
    HAL_HEADERROR,//�յ�ͷ����
    HAL_ENDERROR, //ͷ��ȷ��β����
    HAL_XORERROR,//У��λ����
    HAL_CRCERROR,

    HAL_VMOTOROK,
    HAL_HMOTOROK,
    HAL_DCMOTOROK,
    HAL_MOTORERROR,//�������
    HAL_DCTIMEOUT,
} HAL_StatusTypeDef;

typedef enum
{
    SENDNOPROTOCOL = 0,//���ͺ���ר�ã����������Ƿ����ͷβ���Լ�Э��
//	SENDPROTOCOL,
//	SENDNOHEAD,
//	SENDNOEND,
//
//	UARTSETNOPARAMETER,//UART��ʼ��ר�ã��Ƿ��������ʼ��
//	UARTSETPARAMETER
} PARAMETER;


HAL_StatusTypeDef	UART_Initial(USART_TypeDef* huart, int buad);//���ڳ�ʼ��
HAL_StatusTypeDef CheckProtocol(USART_TypeDef *uart, uint8_t* pbuff);//���Э�飬��������
HAL_StatusTypeDef CheckCrc(void);
//���ڷ������ݣ��Զ����SUNRISE�Լ�0X0D 0X0A,�����ⲿ������SendBuff����
HAL_StatusTypeDef UART_TransmitData_API(USART_TypeDef* huart, const void* data, uint16_t datasize, PARAMETER sendmode);
//���ڷ������ݣ���������˼���Զ�����SDsEs,crc
HAL_StatusTypeDef UART_TransmitData_SDSES(USART_TypeDef* huart, uint32_t len, uint16_t cmdr, uint8_t state, const void* data);

#endif


