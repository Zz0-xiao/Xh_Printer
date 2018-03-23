#include "delay.h"
#include "timer.h"
#include "rs232.h"
#include "utility.h"
#include "motor.h"
#include "sensor.h"

// TIM14,TIM16 PWM Ƶ���趨
#define INIHz 100


HAL_StatusTypeDef Protocol_Process(uint8_t* pbuff);
void ResultSend(uint8_t* pbuff, HAL_StatusTypeDef result);
void Main_Process(void);
void Reset(void);

HAL_StatusTypeDef processResult = HAL_INI;

uint8_t PaperDetection = 0; //����ֽ�ű�־
uint8_t etc = 0;//��λ�ȴ���־ 1Ϊ����λ��

static void IWDG_Config(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(2000 * 40 / 256); //2s
    IWDG_ReloadCounter();
    IWDG_Enable();
}


int main(void)
{
    Delay_init();

    TIM3_Initial();
    TIM14_Initial(1000);
    TIM16_Initial(10000);
    UART_Initial(USART1, 9600);
    UART_TransmitData_API(USART1, "XH-Wardrobe-V2.5", 0, SENDNOPROTOCOL);
//    IWDG_Config();

    SENSOR_Init();
    Motor_Init();

    while (1)
    {
//        HAL_StatusTypeDef processResult = HAL_INI;

//        if(time3Usart1ms > 5)
//        {
//            time3Usart1ms = 0;
//            processResult = CheckProtocol(USART1, UART1RevData);
//            if(processResult == HAL_OK)
//            {
//                processResult = Protocol_Process(UART1RevData);//Э�鴦����
//            }
//            UART1RXDataLenth = 0;
//            ResultSend(UART1RevData, processResult);
//            BuffReset_API(UART1RevData, MAXREVSIZE);
//        }
        if(time2Counter1ms > 99)
        {
            Main_Process();
        }
        IWDG_ReloadCounter();
    }
}

/*******************************
���ƣ�Main_Process();
���ܣ���ѭ��������main�н���
������null
���أ�null
*******************************/
void Main_Process(void)
{
    //�����λ��־Ϊ0����û��ֽ�ž͸�λ
    Reset();

    if((PaperDetection == 0) && (LINFRARE == Bit_SET) && (SENSOR_Scan() == LSENSOR_RESPONSE)) //��ֽ�ŵ��䡣���͵Ĵ����������壬������λ�õ�
    {
        HAL_Delayms(500);
        processResult = MotorDrive57(MOTORH, MOTOR_MOVE_FU);//FU��ʾ���̻�ԭλ
        etc = 0;
        PaperDetection = 1;
    }
    else if(RINFRARE == Bit_SET)
        processResult = MotorDrive57(MOTORH, MOTOR_STOP);//ˮƽ���ֹͣ


    if((PaperDetection == 1) && (RINFRARE == Bit_SET) && (SENSOR_Scan() != HSENSOR_RESPONSE )) //ˮƽ��λ������..��ֽ�ţ�������ԭλ��
    {
//        processResult = MotorDrive57(MOTORH, MOTOR_STOP);//ˮƽ���ֹͣ
        processResult = MotorDrive57(MOTORV, MOTOR_MOVE_BD);//����
    }
    else if(SENSOR_Scan() == HSENSOR_RESPONSE)
    {
        processResult = MotorDrive57(MOTORV, MOTOR_STOP);//����ֹͣ
        Put(RUN);
//			  PaperDetection = 0;
    }

    if((HINFRARE == Bit_RESET)  && (PaperDetection == 1))
    {
        Put(STOP);
        PaperDetection = 0;
    }

    /*   if((PaperDetection == 1) && (SENSOR_Scan() == HSENSOR_RESPONSE))
        {
              processResult = MotorDrive57(MOTORV, MOTOR_STOP);//����ֹͣ
            Put(RUN);
            UART_TransmitData_API(USART1, "Put_paper", 0, SENDNOPROTOCOL);
        }

    ///////////////
        if(HINFRARE == 0)
        {
            PaperDetection = 0;
            Put(STOP);
            UART_TransmitData_API(USART1, "Reset_wait ", 0, SENDNOPROTOCOL);
            Reset();
        }
    */
}

/*******************************
���ƣ�Reset();
���ܣ���λ
������null
���أ�null
*******************************/
void Reset(void)
{
    //˵���ϴζ���δȡ��
//    if(LINFRARE == Bit_SET)
//    {
//        PaperDetection = 1;//����ж���
//        //return ???;
//    }
    //ˮƽ��λ
    if((RINFRARE == Bit_RESET)  && (etc == 0)) ///�����о����Բ���
    {
        processResult = MotorDrive57(MOTORH, MOTOR_MOVE_FU);//FU��ʾ���̻�ԭλ
    }
    else if(RINFRARE == Bit_SET)
    {
        processResult = MotorDrive57(MOTORH, MOTOR_STOP);
    }
    //��ֱ��λ
    if((PaperDetection == 0) && (SENSOR_Scan() != LSENSOR_RESPONSE) && (RINFRARE == Bit_SET) && (etc == 0))
    {
        processResult = MotorDrive57(MOTORV, MOTOR_MOVE_FU);  //FU��ʾ����
    }
    else if((SENSOR_Scan() == LSENSOR_RESPONSE) && (PaperDetection == 0) && (etc == 0))
    {
        processResult = MotorDrive57(MOTORV, MOTOR_STOP);
        //ˮƽλ���������
        processResult = MotorDrive57(MOTORH, MOTOR_MOVE_BD);
        HAL_Delayms(3000);
        etc = 1;
        processResult = MotorDrive57(MOTORH, MOTOR_STOP);
        UART_TransmitData_API(USART1, "Reset_End", 0, SENDNOPROTOCOL);
    }


    /*    //ˮƽ��λ
        if(RINFRARE == Bit_RESET )///�����о����Բ���
        {
            processResult = MotorDrive57(MOTORH, MOTOR_MOVE_FU);//FU��ʾ���̻�ԭλ
            while(RINFRARE == Bit_RESET)
            {
                IWDG_ReloadCounter();
            }
            processResult = MotorDrive57(MOTORH, MOTOR_STOP);
        }


        if(SENSOR_Scan() != LSENSOR_RESPONSE)
    //    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == Bit_RESET)
        {
            processResult = MotorDrive57(MOTORV, MOTOR_MOVE_FU);  //FU��ʾ����
            while(SENSOR_Scan() != LSENSOR_RESPONSE)
            {
                IWDG_ReloadCounter();
            }
            processResult = MotorDrive57(MOTORV, MOTOR_STOP);
        }


        //ˮƽ��λ
        if(SENSOR_Scan() != LSENSOR_RESPONSE)
    //    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == Bit_RESET)
        {
            processResult = MotorDrive57(MOTORV, MOTOR_MOVE_FU);  //FU��ʾ����
            while(SENSOR_Scan() != LSENSOR_RESPONSE)
            {
                IWDG_ReloadCounter();
            }
            processResult = MotorDrive57(MOTORV, MOTOR_STOP);
        }

        //��ֱ��λ
    //    while(SENSOR_Scan() != LSENSOR_RESPONSE)
    //    {
    //        processResult = MotorDrive57(MOTORV, MOTOR_MOVE_FU);
    //        IWDG_ReloadCounter();
    //    }

        //ˮƽλ���������
        processResult = MotorDrive57(MOTORH, MOTOR_MOVE_BD);
        HAL_Delayms(3000);

        processResult = MotorDrive57(MOTORH, MOTOR_STOP);

        UART_TransmitData_API(USART1, "Reset_End", 0, SENDNOPROTOCOL);
    		*/
}


/*******************************
���ƣ�Protocol_Process(uint8_t* pbuff,);
���ܣ�Э�鴦����
������Э�����ݻ�����pbuff
���أ���������������communication.h�����
*******************************/
const uint32_t DEV_ID = 0xffffffff;//��ʼ����Ϊffffffff,����д�벻�ɹ�

HAL_StatusTypeDef Protocol_Process(uint8_t* pbuff)
{
//	HAL_StatusTypeDef processResult;
//	uint16_t cmdr;
//	uint8_t senddata;
//	uint32_t setID;
//	uint8_t sensorStatus[11];
//	int i = 0;
//	cmdr = ((uint16_t)pbuff[9]<<8)+pbuff[10];
//	indexKHz = 0;
//	cV = 0;
//	switch(cmdr)
//	{
//		case RSENDOUT_SDSES:
//			processResult = SendOut(&pbuff[11]);
//			break;
//		case RMOTOR_SDSES:
//			processResult = Debug_Process(&pbuff[11]);
//			break;
//		case RSENSOR_SDSES:
//			for(i=0;i<6;i++)
//			{
//				if(GPIO_ReadInputDataBit(VSENSOR[i].GPIOx,VSENSOR[i].GPIO_Pin)==Bit_RESET)
//					sensorStatus[i]='1'+i;
//				else
//					sensorStatus[i]='0';
//			}
//			for(i=6;i<8;i++)
//			{
//				if(GPIO_ReadInputDataBit(HSENSOR[i-6].GPIOx,HSENSOR[i-6].GPIO_Pin)==Bit_RESET)
//					sensorStatus[i]='1'+i-6;
//				else
//					sensorStatus[i]='0';
//			}
//			for(i=8;i<11;i++)
//			{
//				if(GPIO_ReadInputDataBit(INFRARE[i-8].GPIOx,INFRARE[i-8].GPIO_Pin)==Bit_SET)
//					sensorStatus[i]='1'+i-8;
//				else
//					sensorStatus[i]='0';
//			}
//			UART_TransmitData_API(USART1,"\r\nVERTICAL:",0,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,sensorStatus,6,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,"\r\nHORIZENTAL:",0,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,sensorStatus+6,2,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,"\r\nINFRARE:",0,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,sensorStatus+8,3,SENDNOPROTOCOL);
//			break;
//		case 0x2011:
//			senddata = motorCurrentPosition[MOTORV]+'0';
//			UART_TransmitData_API(USART1,"MOTORV POSITION",0,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,&senddata,1,SENDNOPROTOCOL);
//			UART_TransmitData_API(USART1,"MOTORV END",0,SENDNOPROTOCOL);
//		  motorErrorFlag = MOTOR_OK;
//			processResult = HAL_OK;
//			break;
//		case 0x2018:
//			UART_TransmitData_API(USART1,"XH-Wardrobe-V2.5",0,SENDNOPROTOCOL);
//			break;
//		default:
//			break;
//	}
//	return processResult;
    return HAL_OK;
}


/*******************************
���ƣ�ResultSend(uint8_t* pbuff, uint16_t buffSize)
���ܣ����̽���ϴ�
������pbuff ���ջ����������ڷ���˼Э���ϴ��ظ����result ���
���أ�null
*******************************/
void ResultSend(uint8_t* pbuff, HAL_StatusTypeDef result)
{
//		uint8_t dataSDSES[10];
//	  if(motorErrorFlag != MOTOR_OK)
//		{
//			UART_TransmitData_API(USART1,"57MOTOR ERROR",0,SENDNOPROTOCOL);
//			MotorDrive57(MOTORV,MOTOR_STOP);
//			MotorDrive57(MOTORH,MOTOR_STOP);
//			return;
//		}
//		if(result == HAL_ERROR)
//		{
//			UART_TransmitData_SDSES(USART1,0,0xA100,OPFAILED,"");
//		}
//		if(result == HAL_OK)
//		{
//			UART_TransmitData_SDSES(USART1,0,0xA100,OPSUCCESS,"");
//		}
//		if(result == HAL_TIMEOUT)//���������ʱ
//		{
//			motorErrorFlag = MOTOR_OK;
//			MotorDrive57(MOTORV,MOTOR_STOP);
//			MotorDrive57(MOTORH,MOTOR_STOP);
//			dataSDSES[0]=0x02;
//			UART_TransmitData_SDSES(USART1,1,0xA100,OPFAILED,dataSDSES);
//		}
//		if(result == HAL_DCTIMEOUT)
//		{
//			dataSDSES[0] = 0x01;//�޻�
//			UART_TransmitData_SDSES(USART1,1,0xA100,OPFAILED,dataSDSES);
//		}
//		if(result == HAL_CRCERROR)
//			UART_TransmitData_API(USART1,"CRC ERROR",0,SENDNOPROTOCOL);
//		if(result == HAL_BUSY)
//			UART_TransmitData_API(USART1,"ERROR BUSY",0,SENDNOPROTOCOL);
//
//		cV = 0;
}


