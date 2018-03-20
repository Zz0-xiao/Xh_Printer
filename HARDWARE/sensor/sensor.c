#include "sensor.h"
#include "timer.h"
/*******************************
名称：SENSOR_Init();
功能：传感器初始化
参数: 无
返回：无
*******************************/
void SENSOR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOF | RCC_AHBPeriph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/*******************************
名称：SENSOR_Scan();
功能：检测托盘位置
参数: 无
返回：托盘位置
*******************************/
uint8_t SENSOR_Scan(void)
{
    if(LSENSOR == 0 || HSENSOR == 0)
    {
//        if((time3Debouncet1ms > 20) && (LSENSOR == 0))
        if(LSENSOR == 0)
        {
//            time3Debouncet1ms = 0;
            return LSENSOR_RESPONSE;
        }
//        else if((time3Debouncet1ms > 20) && (HSENSOR == 0))
        if(HSENSOR == 0)
        {
//            time3Debouncet1ms = 0;
            return HSENSOR_RESPONSE;
        }
//        else time3Debouncet1ms = 0;
    }
    return 0;
}

///*******************************
//名称：SENSOR_Scan();
//功能：检测水平托盘位置
//参数: 无
//返回：托盘位置
//*******************************/
//uint8_t HSENSOR_Scan(void)
//{
//    if(LSENSOR == 0 || HSENSOR == 0)
//    {
//        if((time3Debouncet1ms > 20) && (LSENSOR == 0))
//        {
//            time3Debouncet1ms = 0;
//            return LSENSOR_RESPONSE;
//        }
//        else if((time3Debouncet1ms > 20) && (HSENSOR == 0))
//        {
//            time3Debouncet1ms = 0;
//            return HSENSOR_RESPONSE;
//        }
//        else time3Debouncet1ms = 0;
//    }
//    return 0;
//}

///*******************************
//名称：SENSOR_Scan();
//功能：检测有没有纸张
//参数: 无
//返回：是否有纸张
//*******************************/
//uint8_t INFRARE_Scan(void)
//{
//    if(LINFRARE == 1 || HINFRARE == 1)
//    {
//        if((time3Debouncet1ms > 20) && (LINFRARE == 0))
//        {
//            time3Debouncet1ms = 0;
//            return LINFRARE_RESPONSE;
//        }
//        else if((time3Debouncet1ms > 20) && (HINFRARE == 0))
//        {
//            time3Debouncet1ms = 0;
//            return HINFRARE_RESPONSE;
//        }
//        else time3Debouncet1ms = 0;
//    }
//    return 0;
//}


/*******************************
名称：SENSOR_Scan();
功能：检测水平位置是否复位
参数: 无
返回：是否复位
*******************************/
//uint8_t RNFRARE_Scan(void)
//{
//    if(RINFRARE == 1)
//    {
//        return RINFRARE_RESPONSE;
////        if((time3Debouncet1ms > 20) && (RINFRARE == 0))
////        {
////            time3Debouncet1ms = 0;
////            return RINFRARE_RESPONSE;
////        }
////        else time3Debouncet1ms = 0;
//    }
//    return 0;
//}


// u8 KEY_Scan(u8 mode)
// {
// static u8 key_up=1;//
// if(mode)key_up=1;  //
// if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==1))
// {
// delay_ms(10);//
// key_up=0;
// if(KEY0==0)return KEY0_PRES;
// else if(KEY1==0)return KEY1_PRES;
// else if(KEY2==0)return KEY2_PRES;
// else if(WK_UP==1)return WKUP_PRES;
// }else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1;
// return 0;//
// }



