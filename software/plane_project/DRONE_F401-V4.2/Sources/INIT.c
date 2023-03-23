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
//         此程序只能 用作学习，如用商业用途。必追究责任！
//          
//
//
#include "ALL_DEFINE.h"


volatile uint32_t SysTick_count; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_Mag AK8975;
_st_AngE Angle;    //当前角度姿态值
_st_Remote Remote; //遥控通道值


_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等


PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
PidObject pidRoll;
PidObject pidYaw;
//高度环PID数据
PidObject pidHeightRate;
PidObject pidHeightHigh;
PidObject pidHeightAcc;//加速度

void pid_param_Init(void); //PID控制参数初始化，改写PID并不会保存数据，请调试完成后直接在程序里更改 再烧录到飞控


void ALL_Init(void)
{


  //IIC_Init();             //I2C初始化
  
  pid_param_Init();       //PID参数初始化

  //LEDInit();              //LED闪灯初始化

  MpuInit();              //MPU6050初始化
  
  //USART3_Config();        //上位机串口初始化

  //NRF24L01_init();        //2.4G遥控通信初始化
  
  //TIM2_PWM_Config();      //4路PWM初始化
  
  //TIM3_Config();          //系统工作周期初始化 
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










