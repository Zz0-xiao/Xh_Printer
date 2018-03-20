#ifndef __RS232_H__
#define __RS232_H__

#include "stm32f0xx.h"


#define MAXREVSIZE 512		//定义最大接收字节数 512

//extern uint16_t UART1RXDataLenth；//UART1接受数据长度
//HAL_StatusTypeDef UARTFaultStatus; //串口报超时

extern uint8_t UART1RevData[MAXREVSIZE];
//extern uint8_t UART2RevData[MAXREVSIZE];

extern uint16_t UART1RXDataLenth;//UART1接受数据长度


typedef enum
{
    HAL_INI = 0,
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT,

    HAL_UART1TIMEOUT,//串口1接收超时
    HAL_UART2TIMEOUT,//串口2接收超时
    HAL_UART1RXFULL,//串口1装
    HAL_UART2RXFULL,

    HAL_NULL,//没有接收到数据
    HAL_HEADERROR,//收到头错误
    HAL_ENDERROR, //头正确，尾错误
    HAL_XORERROR,//校验位错误
    HAL_CRCERROR,

    HAL_VMOTOROK,
    HAL_HMOTOROK,
    HAL_DCMOTOROK,
    HAL_MOTORERROR,//电机错误
    HAL_DCTIMEOUT,
} HAL_StatusTypeDef;

typedef enum
{
    SENDNOPROTOCOL = 0,//发送函数专用，发送数据是否添加头尾，以及协议
//	SENDPROTOCOL,
//	SENDNOHEAD,
//	SENDNOEND,
//
//	UARTSETNOPARAMETER,//UART初始化专用，是否带参数初始化
//	UARTSETPARAMETER
} PARAMETER;


HAL_StatusTypeDef	UART_Initial(USART_TypeDef* huart, int buad);//串口初始化
HAL_StatusTypeDef CheckProtocol(USART_TypeDef *uart, uint8_t* pbuff);//检查协议，并不声明
HAL_StatusTypeDef CheckCrc(void);
//串口发送数据，自动添加SUNRISE以及0X0D 0X0A,利用外部缓冲区SendBuff过度
HAL_StatusTypeDef UART_TransmitData_API(USART_TypeDef* huart, const void* data, uint16_t datasize, PARAMETER sendmode);
//串口发送数据，适用于神思，自动加入SDsEs,crc
HAL_StatusTypeDef UART_TransmitData_SDSES(USART_TypeDef* huart, uint32_t len, uint16_t cmdr, uint8_t state, const void* data);

#endif


