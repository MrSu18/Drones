//========================================================================
//  爱好者电子工作室-淘宝 https://devotee.taobao.com/
//  STM32四轴爱好者QQ群: 810149456
//  作者：小刘
//  电话:13728698082
//  邮箱:1042763631@qq.com
//  日期：2018.05.17
//  版本：V1.0
//========================================================================
//套件购买地址：https://devotee.taobao.com/
//                 爱好者电子工作室
//特此声明：
//
//         此程序只能用作学习，如用商业用途。必追究责任！
//          
//
//
#include "ALL_DATA.h"
#include "mpu6050.h"
//#include "I2C.h"
#include "filter.h"
#include <string.h>
//#include "LED.h"
#include "myMath.h"
#include "kalman.h"
#include "main.h"

int16_t MpuOffset[6] = {0};

static volatile int16_t *pMpu = (int16_t *)&MPU6050;
extern I2C_HandleTypeDef hi2c1;

int8_t IIC_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{ 
  if(HAL_I2C_Mem_Write(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,&data,1,2)!=HAL_OK)
    return ERROR;
  else
    return SUCCESS;
}

uint8_t IIC_Read_One_Byte(uint8_t addr,uint8_t reg)
{ uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,&data,1,2);
  return data;
}
/****************************************************************************************
*@brief  
*@brief   
*@param[in]
*****************************************************************************************/
int8_t mpu6050_rest(void)
{ if(IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80) == ERROR)
    return ERROR;  //复位  
  //delay_ms(20);
  return SUCCESS;
}
/****************************************************************************************
*@brief   
*@brief  
*@param[in]
*****************************************************************************************/
int8_t MpuInit(void) //初始化
{
  static uint8_t date = SUCCESS;
  do
  {
  date = IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);  //复位
  HAL_Delay(30);//delay_ms(30);
  date += IIC_Write_One_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x02); //陀螺仪采样率，0x00(500Hz)
  date += IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x03);  //设置设备时钟源，陀螺仪Z轴
  date += IIC_Write_One_Byte(MPU6050_ADDRESS, CONFIGL, 0x03);   //低通滤波频率，0x03(42Hz)
  date += IIC_Write_One_Byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);//+-2000deg/s
  date += IIC_Write_One_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);//+-4G
  //date += IIC_Write_One_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x08);//+-2G
  }
  while(date != SUCCESS);
  date = IIC_Read_One_Byte(MPU6050_ADDRESS, 0x75);
  if(date!= MPU6050_PRODUCT_ID)
    return ERROR;
  else 
    MpuGetOffset();
    return SUCCESS;
}
/****************************************************************************************
*@brief    
*@brief   
*@param[in]
*****************************************************************************************/

#define  Gyro_Read() HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS, 0X3B,I2C_MEMADD_SIZE_8BIT,buffer,6,10)//IIC_read_Bytes(MPU6050_ADDRESS, 0X3B,buffer,6)
#define  Acc_Read() HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS, 0X43,I2C_MEMADD_SIZE_8BIT,&buffer[6],6,10)//IIC_read_Bytes(MPU6050_ADDRESS, 0x43,&buffer[6],6)

void MpuGetData(void) //读取陀螺仪数据加滤波
{
    uint8_t i;
    static uint8_t buffer[12];

    Gyro_Read();
    Acc_Read();

    for(i=0;i<6;i++)
    {
      pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1])-MpuOffset[i];    
      if(i < 3)
      {
        {
          static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};  
          kalman_1(&ekf[i],(float)pMpu[i]);  //一维卡尔曼
          pMpu[i] = (int16_t)ekf[i].out;
        }
    }
    if(i > 2)
    {  
      uint8_t k=i-3;
      const float factor = 0.15f;  //滤波因素      
      static float tBuff[3];    

      pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;                
    }
  }
}

/****************************************************************************************
*@brief   get mpu offset
*@brief   initial and cmd call this
*@param[in]
*****************************************************************************************/
void MpuGetOffset(void) //校准
{
  int32_t buffer[6]={0};
  int16_t i;  
  uint8_t k=30;
  const int8_t MAX_GYRO_QUIET = 5;
  const int8_t MIN_GYRO_QUIET = -5;  
/*           wait for calm down                                                                */
  int16_t LastGyro[3] = {0};
  int16_t ErrorGyro[3];  
  /*           set offset initial to zero        */
  
  memset(MpuOffset,0,12);
  MpuOffset[2] = 8192;   //set offset from the 8192  
    
  while(k--)
  {
    do
    { 
      HAL_Delay(10);  //delay_ms(10);
      MpuGetData();
      for(i=0;i<3;i++)
      {
        ErrorGyro[i] = pMpu[i+3] - LastGyro[i];
        LastGyro[i] = pMpu[i+3];  
      }      
    }while ((ErrorGyro[0] >  MAX_GYRO_QUIET )|| (ErrorGyro[0] < MIN_GYRO_QUIET)
          ||(ErrorGyro[1] > MAX_GYRO_QUIET )|| (ErrorGyro[1] < MIN_GYRO_QUIET)
          ||(ErrorGyro[2] > MAX_GYRO_QUIET )|| (ErrorGyro[2] < MIN_GYRO_QUIET)
            );
  }  

/*           throw first 100  group data and make 256 group average as offset                    */  
  for(i=0;i<356;i++)
  {    
    MpuGetData();
    if(100 <= i)
    {
      uint8_t k;
      for(k=0;k<6;k++)
      {
        buffer[k] += pMpu[k];
      }
    }
  }

  for(i=0;i<6;i++)
  {
    MpuOffset[i] = buffer[i]>>8;
  }

}
/**************************************END OF FILE*************************************/

