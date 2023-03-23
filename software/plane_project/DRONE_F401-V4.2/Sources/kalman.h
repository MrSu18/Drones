#ifndef _KALMAN_H
#define _KALMAN_H




struct _1_ekf_filter
{
  float LastP;
  float  Now_P;
  float out;
  float Kg;
  float Q;
  float R;  
};
typedef struct{
  float x;
  float p;
  float A;
  float H;
  float q;
  float r;
  float gain;
}kalman1_state;
#pragma pack(1)
typedef struct{
  float x[2];
  float p[2][2];
  float A[2][2];
  float H[2];
  float q[2];
  float r;
  float gain[2];
}kalman2_state;
#pragma pack()
//void ekf_1(struct EKF *ekf,void *input);  //Ò»Î¬¿¨¶ûÂü
extern void kalman_1(struct _1_ekf_filter *ekf,float input);  //Ò»Î¬¿¨¶ûÂü
extern void kalman2_init(kalman2_state *state,float dt);
extern float kalman2_filter(kalman2_state *state,float z_measure,float acc_measure,float dt);
#endif


