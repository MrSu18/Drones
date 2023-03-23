#include <stdlib.h>  //abs();
#include <stdio.h>   //printf
#include <math.h>  //sqrt();
#include <myMath.h>
//#include "system.h"
//#include "XN297L.h"
#include "imu.h"
//#include "fast_maths.h"
//#include "stabilizer.h"
#include "GL9306.h"
//#include "ekf.h"
#include "SPL06_001.h"

#define UartRxOpticalFlow uart2_rxbuf
#define alt_to_Gnd 50
#define M_PI_FLOAT 3.14159265
extern uint8_t uart2_rxbuf[];
extern UART_HandleTypeDef huart1;

/******************************************/
#if Debug
int16_t test1,test2;//test3,test4;
#endif 
_PID_arg_st pos_pid_para;
_PID_arg_st vec_pid_para;

_PID_val_st pos_pid_value_x;
_PID_val_st pos_pid_value_y;
_PID_val_st vec_pid_value_x;
_PID_val_st vec_pid_value_y;
float sum_flow_x, sum_flow_y;
float pos_pid_out_x, pos_pid_out_y;
/******************************************/

uint8_t b_updata_uret;//¹âÁ÷Êý¾Ý¸üÐÂ
uint8_t pos_flag;
float filter_x,filter_y;
float speed_x,speed_y;
float OpticalFlowOut[2];

/******************************************/
static float PID_calculate(float T, float in_ff, float expect, float feedback, _PID_arg_st *pid_arg, _PID_val_st *pid_val, float inte_lim);
static void PID_Value_reset(_PID_val_st *pid_val);
void flow_decode(void);
/*****************************************/


#define KKP 0.1f//0.28 0.34//¿¹·çÊÔ×ÅÔö´óÕâ¸öÏµÊý£¡
#define KP 12.0f//7.2
#define KD 1.0f//0.7

void PID_para_init(void)
{  
  pos_pid_para.kp = KKP;
  pos_pid_para.ki = 0.0f;
  pos_pid_para.kd = 0.0f;

  vec_pid_para.kp = KP;//15
  vec_pid_para.ki = 0.0f;//0.05
  vec_pid_para.kd = KD;//1.0
  
  LPF2pSetCutoffFreq(500,5);
}

//uint8_t count;int16_t sum_accx,sum_accy;
//uint8_t dot = 1,tt = 0,pt =0;
//The evil capitalism
void OPTICAL2Read(void)
{    
  uint16_t height = alt_to_Gnd;
  float vx,vy,scale = 1.0f;
  //static int16_t angle_x_old,angle_y_old;//Ê¹ÓÃ½Ç¶È²¹³¥¹âÁ÷µÄÖµ
  if(b_updata_uret) //if ÊÕµ½ÁËÐÂµÄ¹âÁ÷Êý¾Ý°ü
  {  
    b_updata_uret = 0;
    
    if(height <= 110)    
      scale = 1.0f;
    else if(height <= 170)  
      scale = 1.1f;
    else if(height <= 220)  
      scale = 1.2f;
    else if(height <= 270)  
      scale = 1.4f;
    else if(height <= 370)  
      scale = 1.5f;
    else if(height <= 420)  
      scale = 1.6f;
    else           
      scale = 1.7f;
    vec_pid_para.kp = KP*scale;    
    //vec_pid_para.kd = KD*scale;  
    //pos_pid_para.kp = KKP;
    
    flow_decode();    
    if((abs(Remote.roll-1500) <= 30) && (abs(Remote.pitch-1500) <= 30))//×óÒ¡¸ËÔÚÖÐÐÄÎ»ÖÃÊ±£¬Ê¹ÄÜ¹âÁ÷
    {  
//      vy = constrainf(speed_y,-10.0f,10.0f);
//      vx = constrainf(speed_x,-10.0f,10.0f); 
  
//      if(dot == 1)
//      {      
//        if(++tt > 150)
//        {
//          dot = 0;tt = 0;pt = 1;
//        }
//      }      
//      if(pt == 1)
//      {
//        vec_pid_para.kp = KP*scale*1.8f;
//        pos_pid_para.kp = KKP*0.33f;
//      }
      
      vx = speed_x;
      vy = speed_y;
      sum_flow_x += vx*0.05f;
      sum_flow_y += vy*0.05f;
      sum_flow_x = LIMIT(sum_flow_x,-50.0f,50.0f);  
      sum_flow_y = LIMIT(sum_flow_y,-50.0f,50.0f);//¿¹·çÔö¼Ó´ËÏî   25.0f
      
      pos_flag++;
      if(pos_flag == 2)
      {
        pos_flag = 0;
        pos_pid_out_x = PID_calculate(0.05f,0.0f,0.0f,sum_flow_x,&pos_pid_para,&pos_pid_value_x,50.0f);    
        pos_pid_out_y = PID_calculate(0.05f,0.0f,0.0f,sum_flow_y,&pos_pid_para,&pos_pid_value_y,50.0f);
      }        
        
      vx = speed_x;// - (angle_x_old -  EulerRoll)*0.25f*scale;
      vy = speed_y;// + (angle_y_old - EulerPitch)*0.25f*scale;
//      vy = constrainf(speed_y,-14.0f,14.0f);// + (angle_y_old - EulerPitch)*0.2f*scale;//ÀûÓÃÅ·À­½Ç±ä»»²¹³¥¹âÁ÷Êä³öÖµ
//      vx = constrainf(speed_x,-14.0f,14.0f);// - (angle_x_old -  EulerRoll)*0.2f*scale;//·½Ïò!?
      
//      //feedforword
//      sum_accx = constrain((sum_accx/count),-80,80);
//      sum_accy = constrain((sum_accy/count + 20),-80,80);
//      if(abs(sum_accx) < 5) sum_accx = 0;
//      if(abs(sum_accy) < 5) sum_accy = 0;

      filter_x = PID_calculate(0.05f,0,pos_pid_out_x,vx,&vec_pid_para,&vec_pid_value_x,100.0f);  
      filter_y = PID_calculate(0.05f,0,pos_pid_out_y,vy,&vec_pid_para,&vec_pid_value_y,100.0f);
//      OpticalFlowOut[PITCH0] = filter_y;
//      OpticalFlowOut[ROLL1]  = filter_x;
      LPF_1_(2.5f,0.05f,filter_x,OpticalFlowOut[1]);
      LPF_1_(2.5f,0.05f,filter_y,OpticalFlowOut[0]);
//      OpticalFlowOut[PITCH0] = constrain(OpticalFlowOut[PITCH0],-110,110);
//      OpticalFlowOut[ROLL1]  = constrain(OpticalFlowOut[ROLL1],-110,110);
      //angle_y_old = Angle.pitch;//EulerPitch;
      //angle_x_old = Angle.roll;//EulerRoll;
//      count = 0;sum_accx = 0;sum_accy = 0;
    }    
    else 
    {  //dot = 1;pt = 0;
      OpticalFlowOut[0] = 0;
      OpticalFlowOut[1] = 0;
      pos_pid_out_x = 0;
      pos_pid_out_y = 0;
      sum_flow_x = 0;       
      sum_flow_y = 0;    
      PID_Value_reset(&pos_pid_value_x);
      PID_Value_reset(&pos_pid_value_y);
      PID_Value_reset(&vec_pid_value_x);
      PID_Value_reset(&vec_pid_value_y);
    }
  }
//  else
//  {
//    count++;
//    sum_accx += accelMpu.x;//Í¬Ò»·½Ïòò
//    sum_accy -= accelMpu.y;
//  }
}

void flow_decode(void)
{
  uint8_t Check_sum = 0;
  static int16_t flow_x,flow_y;
  static uint8_t flow_cnt=0;
  if(UartRxOpticalFlow[0]==0xfe)//Ð£ÑéÍ·
  {
    Check_sum = (uint8_t)(UartRxOpticalFlow[2]+UartRxOpticalFlow[3]+UartRxOpticalFlow[4]+UartRxOpticalFlow[5]);
    if(Check_sum == UartRxOpticalFlow[6])//Ð£ÑéºÍÕýÈ·
    {    
      flow_x = UartRxOpticalFlow[2] + (UartRxOpticalFlow[3] << 8);
      flow_y = -(UartRxOpticalFlow[4] + (UartRxOpticalFlow[5] << 8)); //
        
      speed_x = -flow_x;
      speed_y = -flow_y;
      //test1 = speed_x;test2 = speed_y;
      flow_cnt++;
      if(flow_cnt>=5)
      { flow_cnt=0;
				printf("nx=%4d,\t ny=%4d,\t px=%6.2f,\t py=%6.2f\r\n",flow_x,flow_y,speed_x,speed_y);
      }
      
    }
  }
}

#define FF 0.45f
static float PID_calculate(float T, float in_ff, float expect, float feedback, _PID_arg_st *pid_arg, _PID_val_st *pid_val, float inte_lim)
{
  float out,differential;
  
  pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);
  
  pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
  
  pid_val->err =  (expect - feedback);  
  
  pid_val->err_d = LIMIT((pid_val->err - pid_val->err_old),-8.0f,8.0f) *safe_div(1.0f,T,0);//5.0f
  
  differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
  
  if((int16_t)(100 *pid_arg->inc_hz)!=0)  LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf);
    else  pid_val->err_d_lpf = 0;
  
  pid_val->err_i += (pid_val->err + pid_arg->k_pre_d *pid_val->feedback_d)*T;
  pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
  
  out =   FF *in_ff + \
      pid_arg->kp *pid_val->err + \
        pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential + \
        pid_arg->ki *pid_val->err_i;

  pid_val->feedback_old = feedback;
  pid_val->err_old = pid_val->err;
  return (out);
}

static void PID_Value_reset(_PID_val_st *pid_val)
{
  pid_val->err = 0;
  pid_val->err_old = 0;
  pid_val->feedback_old = 0;
  pid_val->feedback_d = 0;
  pid_val->err_d = 0;
  pid_val->err_d_lpf = 0;
  pid_val->err_i = 0;
  pid_val->pre_d = 0;
}

static float  _a1;
static float  _a2;
static float  _b0;
static float  _b1;
static float  _b2;
static float  _delay_element_1;// buffered sample -1
static float  _delay_element_2;// buffered sample -2

//¼ÆËãIIRÂË²¨Æ÷²ÎÊý
//sample_freq : ²ÉÑùÆµÂÊ
//cutoff_freq : ½ØÖ¹ÆµÂÊ
void LPF2pSetCutoffFreq(float sample_freq, float cutoff_freq)
{
    float fr =0, ohm =0,c =0;

    fr = sample_freq/cutoff_freq;
    ohm = tanf(M_PI_FLOAT/fr);
    c = 1.0f+2.0f*cosf(M_PI_FLOAT/4.0f)*ohm + ohm*ohm;

  _b0 = ohm*ohm/c;
  _b1 = 2.0f*_b0;
  _b2 = _b0;
  _a1 = 2.0f*(ohm*ohm-1.0f)/c;
  _a2 = (1.0f-2.0f*cosf(M_PI_FLOAT/4.0f)*ohm+ohm*ohm)/c;
}

//¶þ½×IIRÂË²¨Æ÷
float LPF2pApply(float sample)
{
    float delay_element_0 = 0, output=0;

  delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
  // don't allow bad values to propogate via the filter
  if(isnan(delay_element_0) || isinf(delay_element_0))   
    delay_element_0 = sample;
  
  output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

  _delay_element_2 = _delay_element_1;
  _delay_element_1 = delay_element_0;

  return output; 
}

/*
static int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    QMF_COPY(p, v, 3);
    QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]) ;
    return p[1];
}*/
/*
int32_t quickMedianFilter5(int32_t * v)
{
    int32_t p[5];
    QMF_COPY(p, v, 5);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[2]); QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]); 
    return p[2];
}

#define DISTANCE_SAMPLES_MEDIAN 5

int32_t MedianFilter(int32_t newSonarReading)
{
    static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static uint8_t medianFilterReady = 0;
    int nextSampleIndex;
 
    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = 1;
    }

    sonarFilterSamples[currentFilterSampleIndex] = newSonarReading;
    currentFilterSampleIndex = nextSampleIndex;
    
    if (medianFilterReady)
        return quickMedianFilter5(sonarFilterSamples);
    else
        return newSonarReading;
}
*/
