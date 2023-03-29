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
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT 0
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //�ṹ�����飬��ÿһ�������һ��pid�ṹ�壬�����Ϳ���������������PID��������  �������ʱ������λpid�������ݣ�����������仰�����þͿ�����
    ,&pidHeightRate
    ,&pidHeightHigh
};
/**************************************************************
 *  flight control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void FlightPidControl(float dt)
{
  volatile static uint8_t status=WAITING_1;

  switch(status)
  {    
    case WAITING_1: //�ȴ�����
      if(ALL_flag.unlock)
      {
        status = READY_11;  
      }      
      break;
    case READY_11:  //׼���������
      pidRest(pPidObject,8); //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���

      Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //����ƫ����

      status = PROCESS_31;

      break; 
    case PROCESS_31: //��ʽ�������
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
      pidRateY.measured = MPU6050.gyroY * Gyro_G;
      pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
    
      pidPitch.measured = Angle.pitch; //�⻷����ֵ ��λ���Ƕ�
      pidRoll.measured = Angle.roll;
      pidYaw.measured = Angle.yaw;
    
      pidUpdate(&pidRoll,dt);    //����PID�������������⻷  �����PID    
      pidRateX.desired = pidRoll.out; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
      pidUpdate(&pidRateX,dt);  //�ٵ����ڻ�

      pidUpdate(&pidPitch,dt);    //����PID�������������⻷  ������PID  
      pidRateY.desired = pidPitch.out;  
      pidUpdate(&pidRateY,dt); //�ٵ����ڻ�

      CascadePID(&pidRateZ,&pidYaw,dt);  //Ҳ����ֱ�ӵ��ô���PID����������
      //������ͣ�ж�
      if (Remote.AUX5==1)
      {
        //�߶Ȼ�
        pidUpdate(&pidHeightHigh,10*dt);    
        pidHeightRate.desired=pidHeightHigh.out;
        //�ٶȻ�
        pidUpdate(&pidHeightRate,10*dt);
        pidHeightAcc.desired=pidHeightRate.out;
        //���ٶȻ�
        pidHeightAcc.measured=MPU6050.accZ;
        pidUpdate(&pidHeightAcc,dt);
      }
      break;
    case EXIT_255:  //�˳�����
      pidRest(pPidObject,8);
      status = WAITING_1;//���صȴ�����
      break;
    default:
      status = EXIT_255;
      break;
  }
  if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
    status = EXIT_255;
}


int16_t motor[4];
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3] 

void MotorControl(void)
{  
  volatile static uint8_t status=WAITING_1;
  static uint16_t take_off_time=0;//�ɻ���ɼ��������ŵ���
  
  if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
    status = EXIT_255;  
  switch(status)
  {    
    case WAITING_1: //�ȴ�����  
      MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
      if(ALL_flag.unlock)
      {
        status = WAITING_2;
      }
    case WAITING_2: //������ɺ��ж�ʹ�����Ƿ�ʼ����ң�˽��з��п���
      if(Remote.thr>MIN_THR)
      {
        status = PROCESS_31;
      }
      break;
    case PROCESS_31:
      {
        int16_t temp;
        if(Remote.AUX5==1)//��ͣ�ж�
        {
          temp=pidHeightAcc.out -1000 + pidHeightRate.out;//�߶Ȼ����ڲ�����
        }
        else
        {
          if(Remote.AUX6==1 && take_off_time<(TAKE_OFF_THR/TAKE_OFF_K))//һ������ж�
          {
            Remote.thr=take_off_time*TAKE_OFF_K;
            take_off_time++;
            LIMIT(Remote.thr,0,TAKE_OFF_THR);
          }
          else if(Remote.AUX6==0 && take_off_time>0)//һ������
          {
            Remote.thr=take_off_time*TAKE_OFF_K;
            take_off_time--;
            LIMIT(Remote.thr,0,TAKE_OFF_THR);
          }
          // printf("Remote.thr:%d,Remote.AUX6:%d,Remote.AUX5:%d \r\n",Remote.thr,Remote.AUX6,Remote.AUX5);
          temp = Remote.thr -1000 + pidHeightRate.out; //����+�������ֵ
          if(Remote.thr<MIN_THR)    //����̫���ˣ����������  ��Ȼ�ɻ���ת                        
          {
            MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4=0;
            break;
          }
        }
        MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(temp,0,900); //��100����̬����

        MOTOR1 +=    + pidRateX.out + pidRateY.out + pidRateZ.out ;//��̬����������������Ŀ�����;
        MOTOR2 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
        MOTOR3 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
        MOTOR4 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;//; 
      }  
      break;
    case EXIT_255:
      MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
      status = WAITING_1;  
      break;
    default:
      break;
  }
  
  TIM3->CCR1 = LIMIT(MOTOR1,0,1000);  //����PWM
  TIM3->CCR2 = LIMIT(MOTOR2,0,1000);
  TIM3->CCR3 = LIMIT(MOTOR3,0,1000);
  TIM3->CCR4 = LIMIT(MOTOR4,0,1000);
} 

/************************************END OF FILE********************************************/ 
