/******************** (C) COPYRIGHT 2013 YunMiao ********************
 * File Name          : main.c
 * Author             : YunMiao
 * Version            : V2.0.1
 * Date               : 08/01/20013
 * Description        : IIC basic function
 ********************************************************************************
 ********************************************************************************
 *******************************aircraft****************************************/
//#include "i2c.h"
#include "all_define.h"
#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

/******************************************************************************
 * 函数名称: I2c_delay
 * 函数功能: I2c 延时函数
 * 入口参数: 无
 ******************************************************************************/
#define I2c_delay()  {\
    volatile unsigned char i = 1;\
    while (i)\
        i--;\
}


int8_t IIC_Read_One_Byte(uint8_t addr,uint8_t reg)
{
	uint8_t recive = 0;

	return recive;
}

/*****************************************************************************
 *函数名称:	i2cWrite
 *函数功能:	写入指定设备 指定寄存器一个字节
 *入口参数： addr 目标设备地址
 *		     reg   寄存器地址
 *		     data 读出的数据将要存放的地址
 *******************************************************************************/
int8_t IIC_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{

  return SUCCESS;
}

int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{

    return SUCCESS;
}
