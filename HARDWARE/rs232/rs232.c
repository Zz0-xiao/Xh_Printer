#include "RS232.h"
#include "utility.h"
#include "timer.h"
#include "string.h"

uint8_t UART1RevData[MAXREVSIZE];
uint8_t UART2RevData[MAXREVSIZE];

uint16_t UART1RXDataLenth = 0;//UART1�������ݳ���
HAL_StatusTypeDef UARTFaultStatus = HAL_OK;//���ڽ��ձ�ը

uint8_t HEAD[] =  "SDsEs";//{'S','D','s','E','s'};
uint8_t END[]  =  "ETX";
uint8_t HEADSIZE = sizeof(HEAD) / sizeof(uint8_t) - 1;
uint8_t ENDSIZE = sizeof(END) / sizeof(uint8_t) - 1;

uint8_t SendBuff[512];//�����ܷ����ݴ�

/*******************************
���ƣ�HAL_StatusTypeDef	UART_Initial(USART_TypeDef* huart,int buad)
���ܣ����ڳ�ʼ��
������UART_HandleTypeDef *huart���ڽṹ��
			int buad ������
���أ�HAL_StatusTypeDef
			���communication.h
*******************************/

HAL_StatusTypeDef	UART_Initial(USART_TypeDef* huart, int buad)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);					//ʹ��USART1��GPIOAʱ��

    if(huart == USART1)
    {
        /* Enable USART clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        /* Connect pin to Periph */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

        /* Configure pins as AF pushpull */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    }
    else
    {
        /* Enable USART clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        /* Connect pin to Periph */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

        /* Configure pins as AF pushpull */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;

        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    }

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 IRQ Channel configuration */

    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;		//��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);										//����ָ���Ĳ�����ʼ��VIC�Ĵ���


    USART_InitStructure.USART_BaudRate = buad;								//���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

    USART_Init(huart, &USART_InitStructure);			//��ʼ������
    USART_ITConfig(huart, USART_IT_RXNE, ENABLE);//�������ڽ����ж�


    /* Enable the 8xUSARTs */
    USART_Cmd(huart, ENABLE);						//ʹ�ܴ���
    return HAL_OK;
}

/*******************************
���ƣ�CheckProtocol(uint8_t* pbuff)
���ܣ����Э�飬��������
������USART_TypeDef *uart ���ں� uint8_t* pbuff ����ȫ�����ݣ���ƫ��
���أ�HAL_StatusTypeDef communication.h
*******************************/
HAL_StatusTypeDef CheckProtocol(USART_TypeDef *uart, uint8_t* pbuff)
{
    int i;
    uint16_t lenth;
    /*	if(UART1RXDataLenth<HEADSIZE) return HAL_HEADERROR;*/
    for(i = 0; i < HEADSIZE; i++)
    {
        if(pbuff[i] != HEAD[i])
            return HAL_HEADERROR;
    }
    lenth = (UART1RevData[HEADSIZE] << 24) + (UART1RevData[HEADSIZE + 1] << 26) + (UART1RevData[HEADSIZE + 2] << 8) + UART1RevData[HEADSIZE + 3];

    if(UART1RXDataLenth - 9 < lenth)
        return HAL_BUSY;
    if(CheckCrc() != HAL_OK)
        return HAL_CRCERROR;
    return HAL_OK;
}

HAL_StatusTypeDef CheckCrc(void)
{
    uint16_t crc = 0;
    uint8_t  crc_out[2] = {0, 0};
    uint32_t len;
    uint8_t  value;
    uint16_t i;
    uint8_t  tmp;

    len = (UART1RevData[HEADSIZE] << 24) + (UART1RevData[HEADSIZE + 1] << 26) + (UART1RevData[HEADSIZE + 2] << 8) + UART1RevData[HEADSIZE + 3] + 2;
    i = 0;
    while (len--)
    {
        crc = crc16one(UART1RevData[i + 6], crc);
        i++;
    }
    crc_out[0] = crc / 256;
    crc_out[1] = crc % 256;
    value = UART1RevData[i + 6];
    tmp = UART1RevData[i + 1 + 6];
    if((crc_out[0] == value) && (crc_out[1] == tmp))
    {
        return HAL_OK;
    }
    return HAL_CRCERROR;
}

/*******************************
���ƣ�UART_TransmitData_API();;
���ܣ����ڷ������ݣ��Զ����SUNRISE�Լ�0X0D 0X0A,�����ⲿ������SendBuff����
������UART_HandleTypeDef *huart���ڽṹ��
			const void* data�ⲿ��������ָ��
			uint16_t datasize�������ݴ�С����ֹ���
			PARAMETER sendmode ����ģʽ������������ṹ��
���أ�HAL_StatusTypeDef communication.h
*******************************/
HAL_StatusTypeDef UART_TransmitData_API(USART_TypeDef* huart, const void* data, uint16_t datasize, PARAMETER sendmode)
{
    uint16_t i, timeout, sendLen;
//	uint8_t Xor;
    uint8_t* pdata = (uint8_t*) data;
    if(datasize == 0)
        datasize = strlen((char*)pdata);

    if(sendmode == SENDNOPROTOCOL)
    {
        sendLen = datasize;
        memcpy(SendBuff, pdata, sendLen);
    }
    else
    {
        sendLen = sizeof(HEAD) / sizeof(uint8_t);
        memcpy(SendBuff, HEAD, sendLen);

        memcpy(SendBuff + sendLen, pdata, datasize);
        sendLen += datasize;

        memcpy(SendBuff + sendLen, END, sizeof(END) / sizeof(uint8_t));
        sendLen += sizeof(END) / sizeof(uint8_t);

//			Xor = CrcCheck(SendBuff,sendLen);
//			SendBuff[sendLen] = Xor;
//			sendLen++;
    }
    for(i = 0; i < sendLen; i++)
    {
        USART_SendData(huart, SendBuff[i]);
        timeout = 0;
        while(USART_GetFlagStatus(huart, USART_FLAG_TXE) == RESET)
        {
            timeout++;
            if(timeout == SystemCoreClock / 1000) //���ͳ�ʱ1ms
                return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/*******************************
���ƣ�UART_TransmitData_SDSES()
���ܣ����ڷ������ݣ���������˼���Զ�����SDsEs,crc
������UART_HandleTypeDef *huart���ڽṹ��
			uint32_t ���ȣ�ֻ��data����,�������Զ���5
			cmdr  ����
			data  ��������,�������κ�Э�����ݣ�ֻ��data���ݣ�������
���أ�HAL_StatusTypeDef communication.h
*******************************/
HAL_StatusTypeDef UART_TransmitData_SDSES(USART_TypeDef* huart, uint32_t len, uint16_t cmdr, uint8_t state, const void* data)
{
    uint16_t i, timeout = 0, crc16 = 0, sendLen = 0;
    uint8_t lenth[4];
    uint8_t cmd[2];
    uint8_t crc[2];
    uint8_t* pdata = (uint8_t*) data;
    if(len == 0)
        len = strlen((char*)pdata);
    //ͷ SDsEs 5
    memcpy(SendBuff, HEAD, HEADSIZE);
    sendLen += HEADSIZE;
    //���� 4�ֽ�
    len += 5;            //�����Ѽ�5
    for(i = 0; i < 4; i++)
    {
        lenth[i] = (len >> ((3 - i) * 8)) & 0xff;
    }
    memcpy(SendBuff + sendLen, lenth, 4);
    sendLen += 4;

    cmd[0] = cmdr >> 8;
    cmd[1] = cmdr & 0xff;
    memcpy(SendBuff + sendLen, cmd, 2);
    sendLen += 2;
    //state 1�ֽ�
    SendBuff[sendLen] = state;
    sendLen++;
    //data len�ֽ�
    memcpy(SendBuff + sendLen, pdata, len - 5);
    sendLen += len - 5;
    //crc 2�ֽ�
    for(i = 0; i < sendLen - 5; i++)
    {
        crc16 = crc16one(SendBuff[i + 5], crc16);
    }
    crc[0] = crc16 >> 8;
    crc[1] = crc16 & 0xFF;
    memcpy(SendBuff + sendLen, crc, 2);
    sendLen += 2;

    for(i = 0; i < sendLen; i++)
    {
        USART_SendData(huart, SendBuff[i]);
        timeout = 0;
        while(USART_GetFlagStatus(huart, USART_FLAG_TXE) == RESET)
        {
            timeout++;
            if(timeout == SystemCoreClock / 1000) //���ͳ�ʱ1ms
                return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

void USART1_IRQHandler(void)
{
    uint8_t tempdata;
    USART_ClearITPendingBit(USART1, USART_IT_ORE); //ORE�ж���� �����������ʱ���������
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        tempdata = USART_ReceiveData(USART1);
        UART1RevData[UART1RXDataLenth] = tempdata;
        UART1RXDataLenth++;
        if(UART1RXDataLenth >= MAXREVSIZE)//���ձ�ը��
        {
            UART1RXDataLenth = UART1RXDataLenth - 1;
            UARTFaultStatus = HAL_UART1RXFULL;
            time3Usart1ms = 0;//ֹͣ��ʱ
        }
        else
            time3Usart1ms = 1;//������ʱ
    }
}

