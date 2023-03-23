#include "stdint.h"

#ifndef _ALL_USER_DATA_H_
#define _ALL_USER_DATA_H_

typedef struct{
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
}_st_Mpu;


typedef struct{
  int16_t magX;
  int16_t magY;
  int16_t magZ;
}_st_Mag;


typedef struct{
  float rate;
  float height;
}High;


typedef struct{
  float roll;
  float pitch;
  float yaw;
}_st_AngE;



typedef struct
{
  uint16_t roll;
  uint16_t pitch;
  uint16_t thr;
  uint16_t yaw;
  uint16_t AUX1;
  uint16_t AUX2;
  uint16_t AUX3;
  uint16_t AUX4;
  uint16_t AUX5;
  uint16_t AUX6;  
}_st_Remote;



typedef volatile struct
{
  float desired;     //< set point
  float offset;      //
  float prevError;    //< previous error
  float integ;        //< integral
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float IntegLimitHigh;       //< integral limit
  float IntegLimitLow;
  float measured;
  float out;
  float OutLimitHigh;
  float OutLimitLow;
}PidObject;


typedef volatile struct
{
  uint8_t unlock;
  

}_st_ALL_flag;


extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_Mag AK8975; //¡À??????¨¨??????????
extern _st_AngE Angle;


extern _st_ALL_flag ALL_flag;


extern  PidObject pidRateX;
extern  PidObject pidRateY;
extern  PidObject pidRateZ;

extern  PidObject pidPitch;
extern  PidObject pidRoll;
extern  PidObject pidYaw;

extern  PidObject pidHeightRate;
extern  PidObject pidHeightHigh;
extern  PidObject pidHeightAcc;//¼ÓËÙ¶È

#endif
