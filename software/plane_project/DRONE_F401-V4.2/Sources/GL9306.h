#ifndef __OPTICAFLOW_IIC_H_
#define __OPTICAFLOW_IIC_H_

#include "main.h"

#define Debug 0
#if Debug
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#endif
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) ) *( (in) - (out) ))
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )

#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }
typedef struct
{
  float kp;      
  float ki;      
  float kd;        
  float k_pre_d; 
  float inc_hz;  
  float k_inc_d_norm;    
}_PID_arg_st;

typedef struct
{
  float err;
  float err_old;
  float feedback_old;
  float feedback_d;
  float err_d;
  float err_d_lpf;
  float err_i;
  float pre_d;

}_PID_val_st;

extern uint8_t b_updata_uret;//光流数据更新
#if Debug
extern int16_t test1,test2;//test3,test4;
#endif 
extern float sum_flow_x, sum_flow_y;
extern float pos_pid_out_x, pos_pid_out_y;

extern float filter_x,filter_y;
extern float speed_x,speed_y;

extern float OpticalFlowOut[2];

extern void PID_para_init(void);
extern void OPTICAL2Read(void);
extern void flow_decode(void);
void LPF2pSetCutoffFreq(float sample_freq, float cutoff_freq);
float LPF2pApply(float sample);
int32_t MedianFilter(int32_t newSonarReading);
#endif  






