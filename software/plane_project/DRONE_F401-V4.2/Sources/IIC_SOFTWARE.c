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
 * ��������: I2c_delay
 * ��������: I2c ��ʱ����
 * ��ڲ���: ��
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
 *��������:	i2cWrite
 *��������:	д��ָ���豸 ָ���Ĵ���һ���ֽ�
 *��ڲ����� addr Ŀ���豸��ַ
 *		     reg   �Ĵ�����ַ
 *		     data ���������ݽ�Ҫ��ŵĵ�ַ
 *******************************************************************************/
int8_t IIC_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{

  return SUCCESS;
}

int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{

    return SUCCESS;
}
