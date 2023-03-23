#ifndef __MPU6050_H
#define __MPU6050_H


//#include "sys.h"
//#include "I2C.h"
#include "stdint.h"

extern int16_t MpuOffset[6];

extern int8_t MpuInit(void);
extern void MpuGetData(void);
extern void MpuGetOffset(void);

#define  SMPLRT_DIV    0x19  //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define  CONFIGL       0x1A  //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define  GYRO_CONFIG   0x1B  //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define  ACCEL_CONFIG  0x1C  //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define  ACCEL_ADDRESS 0x3B
#define  ACCEL_XOUT_H  0x3B
#define  ACCEL_XOUT_L  0x3C
#define  ACCEL_YOUT_H  0x3D
#define  ACCEL_YOUT_L  0x3E
#define  ACCEL_ZOUT_H  0x3F
#define  ACCEL_ZOUT_L  0x40

#define  TEMP_OUT_H    0x41
#define  TEMP_OUT_L    0x42

#define  GYRO_ADDRESS  0x43
#define  GYRO_XOUT_H   0x43
#define  GYRO_XOUT_L   0x44  
#define  GYRO_YOUT_H   0x45
#define  GYRO_YOUT_L   0x46
#define  GYRO_ZOUT_H   0x47
#define  GYRO_ZOUT_L   0x48

#define  PWR_MGMT_1    0x6B  //��Դ��������ֵ��0x00(��������)
#define  WHO_AM_I      0x75  //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define MPU6050_PRODUCT_ID 0x68
#define MPU6052C_PRODUCT_ID 0x72

//#define   MPU6050_is_DRY()      GPIO_ReadOutBit(HT_GPIOC, GPIO_PIN_0)//IRQ������������
  #ifdef    USE_I2C_HARDWARE
    
    #define MPU6050_ADDRESS 0x68
  #else
    #define  MPU6050_ADDRESS 0xD0   //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
  #endif
  
#endif // __MPU6050_H__








