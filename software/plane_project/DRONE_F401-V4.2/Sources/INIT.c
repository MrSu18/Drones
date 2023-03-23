//========================================================================
//  �����ߵ��ӹ�����-�Ա� https://devotee.taobao.com/
//  STM32���ᰮ����QQȺ: 810149456
//  ���ߣ�С��
//  �绰:13728698082
//  ����:1042763631@qq.com
//  ���ڣ�2018.05.17
//  �汾��V1.0
//========================================================================
//�׼������ַ��https://devotee.taobao.com/
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ�� ����ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include "ALL_DEFINE.h"


volatile uint32_t SysTick_count; //ϵͳʱ�����
_st_Mpu MPU6050;   //MPU6050ԭʼ����
_st_Mag AK8975;
_st_AngE Angle;    //��ǰ�Ƕ���ֵ̬
_st_Remote Remote; //ң��ͨ��ֵ


_st_ALL_flag ALL_flag; //ϵͳ��־λ������������־λ��


PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;
//�߶Ȼ�PID����
PidObject pidHeightRate;
PidObject pidHeightHigh;
PidObject pidHeightAcc;//���ٶ�

void pid_param_Init(void); //PID���Ʋ�����ʼ������дPID�����ᱣ�����ݣ��������ɺ�ֱ���ڳ�������� ����¼���ɿ�


void ALL_Init(void)
{


  //IIC_Init();             //I2C��ʼ��
  
  pid_param_Init();       //PID������ʼ��

  //LEDInit();              //LED���Ƴ�ʼ��

  MpuInit();              //MPU6050��ʼ��
  
  //USART3_Config();        //��λ�����ڳ�ʼ��

  //NRF24L01_init();        //2.4Gң��ͨ�ų�ʼ��
  
  //TIM2_PWM_Config();      //4·PWM��ʼ��
  
  //TIM3_Config();          //ϵͳ�������ڳ�ʼ�� 
}



void pid_param_Init(void)
{
  
  pidRateX.kp = 2.0f;//2.0
  pidRateY.kp = 2.0f;//2.0
  pidRateZ.kp = 4.0f;//2.0
  
  pidRateX.ki = 0.0f;
  pidRateY.ki = 0.0f;
  pidRateZ.ki = 0.0f;  
  
  pidRateX.kd = 0.08f;//0.08
  pidRateY.kd = 0.08f;//0.08
  pidRateZ.kd = 0.5f;//0.05
  
  pidPitch.kp = 7.0f;
  pidRoll.kp = 7.0f;
  pidYaw.kp = 7.0f;  
	
	
}










