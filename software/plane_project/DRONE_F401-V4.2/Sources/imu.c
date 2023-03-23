/******************************************************************************************************
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 *******************************************************************************************************/  
#include "imu.h"
#include "myMath.h"
#include <math.h>


static float NormAcc;

  
//float ex_int = 0, ey_int = 0, ez_int = 0;   //X、Y、Z轴的比例误差
//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    //定义四元素
//float his_q0 = 1, his_q1 = 0, his_q2 = 0, his_q3 = 0; 
//float q0_yaw = 1, q1_yaw = 0, q2_yaw = 0, q3_yaw = 0;    //弥补Mahony算法在无地磁情况解算Yaw轴满足不了大扰动要求的现象
//float his_q0_yaw = 1, his_q1_yaw = 0, his_q2_yaw = 0, his_q3_yaw = 0;

//void GetAngle(const stMpu *pMpu,void *pAngE, float dt) 
//{
//  static const float KpDef = 0.85f ;
//  static const float KiDef = 0.0035f;
//  const float Gyro_Gr = 0.0005326f;     //面计算度每秒,转换弧度每秒则 0.03051756   * 0.0174533f = 0.0005326
//  float HalfTime = 0.5 * dt;
//  float gx = pMpu->gyroX  *  Gyro_Gr;
//  float gy = pMpu->gyroY  *  Gyro_Gr;
//  float gz = pMpu->gyroZ  *  Gyro_Gr;
//  float ax =  pMpu->accX;
//  float  ay =  pMpu->accY;
//  float az =  pMpu->accZ;
//  
//  float vx, vy, vz;
//  float ex, ey, ez;
//    
//  static float his_q0q0;
//  static float his_q0q1;
//  static float his_q0q2;
//  static float his_q1q1;
//  static float his_q1q3;
//  static float his_q2q2;
//  static float his_q2q3;
//  static float his_q3q3;
//  
//  float q0q0;
//  float q0q1;
//  float q0q2;
//  float q1q1;
//  float q1q3;
//  float q2q2;
//  float q2q3;
//  float q3q3;    
//  
//  float  q0_yawq0_yaw;
//  float  q1_yawq1_yaw;
//  float  q2_yawq2_yaw;
//  float  q3_yawq3_yaw;
//  float  q1_yawq2_yaw;
//  float  q0_yawq3_yaw;
//  
////**************************Yaw轴计算******************************
//  
//  //Yaw轴四元素的微分方程
//  q0_yaw = his_q0_yaw + (-his_q1_yaw * gx - his_q2_yaw * gy - his_q3_yaw * gz) * HalfTime;
//  q1_yaw = his_q1_yaw + ( his_q0_yaw * gx + his_q2_yaw * gz - his_q3_yaw * gy) * HalfTime;
//  q2_yaw = his_q2_yaw + ( his_q0_yaw * gy - his_q1_yaw * gz + his_q3_yaw * gx) * HalfTime;
//  q3_yaw = his_q3_yaw + ( his_q0_yaw * gz + his_q1_yaw * gy - his_q2_yaw * gx) * HalfTime;
//  
//  q0_yawq0_yaw = q0_yaw * q0_yaw;
//  q1_yawq1_yaw = q1_yaw * q1_yaw;
//  q2_yawq2_yaw = q2_yaw * q2_yaw;
//  q3_yawq3_yaw = q3_yaw * q3_yaw;
//  q1_yawq2_yaw = q1_yaw * q2_yaw;
//  q0_yawq3_yaw = q0_yaw * q3_yaw;
//  
//  //规范化Yaw轴四元数
//  norm = Q_rsqrt(q0_yawq0_yaw + q1_yawq1_yaw + q2_yawq2_yaw + q3_yawq3_yaw);
//  q0_yaw = q0_yaw * norm;
//  q1_yaw = q1_yaw * norm;
//  q2_yaw = q2_yaw * norm;
//  q3_yaw = q3_yaw * norm;  
//  
//  //if(ax * ay * az  == 0)
//  //return ;
//  
//  //归一化加速度计值
//  norm = Q_rsqrt(ax * ax + ay * ay + az * az); 
//  ax = ax * norm;
//  ay = ay * norm;
//  az = az * norm;
//  
//  //由姿态阵估计重力方向和流量/变迁
//  vx = 2 * (his_q1q3 - his_q0q2);                      
//  vy = 2 * (his_q0q1 + his_q2q3);
//  vz = his_q0q0 - his_q1q1 - his_q2q2 + his_q3q3;
//  
//  //向量外积再相减得到差分就是误差 两个单位向量的差积即为误差向量
//  ex = (ay * vz - az * vy) ;      
//  ey = (az * vx - ax * vz) ;
//  ez = (ax * vy - ay * vx) ;

//  //对误差进行PI计算
//  ex_int = ex_int + ex * KiDef;      
//  ey_int = ey_int + ey * KiDef;
//  ez_int = ez_int + ez * KiDef;

//  //校正陀螺仪
//  gx = gx + KpDef * ex + ex_int;          
//  gy = gy + KpDef * ey + ey_int;
//  gz = gz + KpDef * ez + ez_int;      
//      
//  //四元素的微分方程
//  q0 = his_q0 + (-his_q1 * gx - his_q2 * gy - his_q3 *  gz)  *  HalfTime;
//  q1 = his_q1 + ( his_q0 * gx + his_q2 * gz - his_q3 *  gy)  *  HalfTime;
//  q2 = his_q2 + ( his_q0 * gy - his_q1 * gz + his_q3 *  gx)  *  HalfTime;
//  q3 = his_q3 + ( his_q0 * gz + his_q1 * gy - his_q2 *  gx)  *  HalfTime;

//  q0q0 = q0 * q0;
//  q0q1 = q0 * q1;
//  q0q2 = q0 * q2;
//  q1q1 = q1 * q1;
//  q1q3 = q1 * q3;
//  q2q2 = q2  * q2;
//  q2q3 = q2  *  q3;
//  q3q3 = q3  *  q3;  

//  //规范化Pitch、Roll轴四元数
//  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
//  q0 = q0 * norm;
//  q1 = q1 * norm;
//  q2 = q2 * norm;
//  q3 = q3 * norm;
//  
//  //求解欧拉角
//  *((float *)pAngE+2) = atan2f(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1) * RtA;  //ROLL
//  *((float *)pAngE+1) = asin(2 * q0q2 -2 * q1q3) * 57.3;  //PITCH
//  *(float *)pAngE = atan2f(2 * q1_yawq2_yaw + 2 * q0_yawq3_yaw, -2 * q2_yawq2_yaw - 2 * q3_yawq3_yaw + 1)  * RtA;  //YAW
//  
//  
//  //存储更替相应的四元数
//  his_q0_yaw = q0_yaw;
//  his_q1_yaw = q1_yaw;
//  his_q2_yaw = q2_yaw;
//  his_q3_yaw = q3_yaw;

//  his_q0 = q0;
//  his_q1 = q1;
//  his_q2 = q2;
//  his_q3 = q3;
//  
//  his_q0q0 = q0q0;
//  his_q0q1 = q0q1;
//  his_q0q2 = q0q2;
//  his_q1q1 = q1q1;
//  his_q1q3 = q1q3;
//  his_q2q2 = q2q2;
//  his_q2q3 = q2q3;
//  his_q3q3 = q3q3;  
//}








typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;


void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt) 
{    
  volatile struct V{
        float x;
        float y;
        float z;
        } Gravity,Acc,Gyro,AccGravity;

  static struct V GyroIntegError = {0};
  static  float KpDef = 0.8f ;
  static  float KiDef = 0.0003f;
  static Quaternion NumQ = {1, 0, 0, 0};
  float q0_t,q1_t,q2_t,q3_t;
  //float NormAcc;  
  float NormQuat; 
  float HalfTime = dt * 0.5f;

  

  // 提取等效旋转矩阵中的重力分量 
  Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);                
  Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);              
  Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);  
  // 加速度归一化
  NormAcc = Q_rsqrt(squa(MPU6050.accX)+ squa(MPU6050.accY) +squa(MPU6050.accZ));
  
  Acc.x = pMpu->accX * NormAcc;
  Acc.y = pMpu->accY * NormAcc;
  Acc.z = pMpu->accZ * NormAcc;  
   //向量差乘得出的值
  AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
  AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
  AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
  //再做加速度积分补偿角速度的补偿值
  GyroIntegError.x += AccGravity.x * KiDef;
  GyroIntegError.y += AccGravity.y * KiDef;
  GyroIntegError.z += AccGravity.z * KiDef;
  //角速度融合加速度积分补偿值
  Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
  Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
  Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;    
  // 一阶龙格库塔法, 更新四元数

  q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
  q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
  q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
  q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
  
  NumQ.q0 += q0_t;
  NumQ.q1 += q1_t;
  NumQ.q2 += q2_t;
  NumQ.q3 += q3_t;
  // 四元数归一化
  NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
  NumQ.q0 *= NormQuat;
  NumQ.q1 *= NormQuat;
  NumQ.q2 *= NormQuat;
  NumQ.q3 *= NormQuat;  
  // 四元数转欧拉角
  {
    #ifdef  YAW_GYRO
    *(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
    #else
      float yaw_G = pMpu->gyroZ * Gyro_G;
      if((yaw_G > 3.0f) || (yaw_G < -3.0f)) //数据太小可以认为是干扰，不是偏航动作
      {
        pAngE->yaw  += yaw_G * dt;      
      }
    #endif
    pAngE->pitch  =  asin(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;            
    pAngE->roll  = atan2(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;  //PITCH       
  }
}


/***************************************************END OF FILE***************************************************/




