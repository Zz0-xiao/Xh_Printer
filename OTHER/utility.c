#include "utility.h"

/*******************************
名称：CheckCrc(uint8_t* pbuff) & crc16one()
功能：CCITT校验，并不声明
参数：null
返回：null
*******************************/
uint16_t Crc16CCITT_Table[16]= { /* CRC 16bit余式表 */
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

uint16_t crc16one(uint8_t input,uint16_t incrc)
{
    uint8_t temp;
    uint16_t outcrc;

    outcrc = incrc;

    temp = ((uint8_t)(outcrc>>8))>>4;
    outcrc <<= 4;
    outcrc ^= Crc16CCITT_Table[temp^input/16];
    temp = ((uint8_t)(outcrc>>8))>>4;
    outcrc <<= 4;
    outcrc ^= Crc16CCITT_Table[(temp^input)&0x0f];
    return outcrc;
}

/*******************************
名称：XorCheck();
功能：异或校验以及计算
参数：uint8_t *buff 缓冲区地址
			uint16_t buffsize 缓冲区大小
			uint8_t* crc crc值
返回：异或值 1Byte
*******************************/

uint8_t CrcCheck(uint8_t *buff,uint16_t buffsize,uint8_t* crc)
{
    uint8_t Xor=0;
    int i;
    for(i=0; i<buffsize; i++)
        Xor ^= buff[i];
    *crc = Xor;
    return Xor;
}

/*******************************
名称：BuffReset_API();
功能：数组清空，赋0
参数：uint8_t *buff 缓冲区地址
			uint16_t size 缓冲区大小
返回：null
*******************************/
void BuffReset_API(uint8_t *pdata,uint16_t size)
{
    uint16_t i = 0;
    for(i=0; i<size-1; i++)
    {
        *pdata++=0;
    }
    *pdata = 0;
}

/*******************************
名称：ASCII_TO_INT;
功能：ascii码转换成8进制
参数：uint8_t* pdata所有数据的首地址，转换个数
返回：协议中DATA部分的长度
*******************************/
uint32_t ASCII_TO_INT(uint8_t *pdata,uint16_t NUM)
{
    uint32_t result=0,i;
    for(i=0; i<NUM; i++)
    {
        result=result*10+((pdata[i])-'0');
    }
    return result;
}

