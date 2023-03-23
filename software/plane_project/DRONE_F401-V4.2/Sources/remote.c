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
#include "NRF24L01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "ANO_DT.h"
#include <stdio.h>
extern UART_HandleTypeDef huart1;


//#define SUCCESS 0
//#undef FAILED
//#define FAILED  1
/*****************************************************************************************
 *  ͨ�����ݴ���
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/  
uint16_t nrf_cnt=0;
uint8_t RC_rxData[32];
extern UART_HandleTypeDef huart6;

void remote_unlock(void);  
void RC_Analy(void)  
{ 
  uint8_t rxlen;
  static uint8_t hover_flag=0,take_off_status=0;//��ͣ��־λ
  static _st_Remote last_remote;//remote����һ�ε�״̬�������ڼ��״̬����
  uint16_t temp=0;
  //Receive  and check RC data 
  rxlen=NRF24L01_RxPacket(RC_rxData);
  // send_char_array(&huart1,RC_rxData,rxlen);
  if(rxlen>0)
  { 
    uint8_t i;
    uint8_t CheckSum=0;
    //send_char_array(&huart6,RC_rxData,rxlen);
    nrf_cnt = 0;
    for(i=0;i<rxlen-1;i++)
    {
      CheckSum +=  RC_rxData[i];
    }
    if(RC_rxData[rxlen-1]==CheckSum)
    { 
      if (RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //������յ���ң������֡ͷ��ȷ
      {
        switch(RC_rxData[2])
        {
          case 0x03: //ң�����������ź���̬�ǵĿ�������
              temp=((uint16_t)RC_rxData[4] <<8) | RC_rxData[5];
              if (take_off_status==0)
              {
                Remote.thr  =((uint16_t)RC_rxData[4] <<8) | RC_rxData[5] ;  //ͨ��1
                last_remote.thr=Remote.thr;
              }
              Remote.yaw  =((uint16_t)RC_rxData[6] <<8) | RC_rxData[7] ;  //ͨ��2
              Remote.roll =((uint16_t)RC_rxData[8] <<8) | RC_rxData[9] ;  //ͨ��3
              Remote.pitch=((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];  //ͨ��4
              Remote.AUX1 =((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];  //ͨ��5  ���Ͻǰ���������ͨ��5  
              Remote.AUX2 =((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];  //ͨ��6  ���Ͻǰ���������ͨ��6 
              Remote.AUX3 =((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];  //ͨ��7  ���±߰���������ͨ��7 
              Remote.AUX4 =((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];  //ͨ��8  ���±߰���������ͨ��8  
              Remote.AUX5 =((uint16_t)RC_rxData[20]<<8) | RC_rxData[21];  //ͨ��8  ��ͣ
              last_remote.AUX6=Remote.AUX6;
              Remote.AUX6 =((uint16_t)RC_rxData[22]<<8) | RC_rxData[23];  //ͨ��8  һ����ɣ�һ������
              if (last_remote.AUX6==0 && Remote.AUX6==1)//״̬���䣬����һ����ɶ���
              {
                last_remote.thr=1400;
                take_off_status=1;
              }
              else if(last_remote.AUX6==1 && Remote.AUX6==0)
              {
                last_remote.thr=900;
                take_off_status=2;
              }
              else if(take_off_status==1 && temp>last_remote.thr)
              {
                take_off_status=0;
              }
              else if(take_off_status==2 && temp<last_remote.thr)
              {
                take_off_status=0;
              }
              
              LIMIT(Remote.thr  ,1000,2000);
              LIMIT(Remote.yaw  ,1000,2000);
              LIMIT(Remote.roll ,1000,2000);
              LIMIT(Remote.pitch,1000,2000);
              LIMIT(Remote.AUX1 ,1000,2000);
              LIMIT(Remote.AUX2 ,1000,2000);
              LIMIT(Remote.AUX3 ,1000,2000);
              LIMIT(Remote.AUX4 ,1000,2000); 
              // LIMIT(Remote.AUX5 ,1000,2000);
              // LIMIT(Remote.AUX6 ,1000,2000); 
              
              //�ж���ͣ
              if(hover_flag==0 && Remote.AUX5==1)//�������ͣ����ô��̬���Ƶ�����Ϊ0
              {
                pidHeightHigh.desired=pidHeightHigh.measured;//�õ�ǰ�Ĳ���ֵ��Ϊ����ֵ
                pidPitch.desired=0;pidRoll.desired=0;pidYaw.desired=0;
                LED.status=AlwaysOff;
                hover_flag=1;//����ͣ��־������1�������ظ��趨�߶�����
              }
              else if(hover_flag==1 && Remote.AUX5==0)
              {
                LED.status=AlwaysOn;
                hover_flag=0;//ȡ����ͣ

                const float roll_pitch_ratio = 0.04f;
                //const float yaw_ratio =  0.0015f;    
            
                pidPitch.desired =(1500-Remote.pitch)*roll_pitch_ratio;   //��ң��ֵ��Ϊ���нǶȵ�����ֵ
                pidRoll.desired = (1500-Remote.roll)*roll_pitch_ratio;
                if(Remote.yaw>1820)
                {
                  pidYaw.desired += 0.75f;  
                }
                else if(Remote.yaw <1180)
                {
                  pidYaw.desired -= 0.75f;  
                }            
              }
              remote_unlock();
              break;
          case 0x02: //��λ��ͨ��ң����ת���Ķ�ȡPID��������
              ANO_Recive(RC_rxData);//�ѷ���PID���ݵ���λ���ı�־������Ϊ1
              LED.status=WARNING;
              break;
          case 0x10: //��λ��ͨ��ң����ת��������PID��������
              ANO_Recive(RC_rxData);
              LED.status=DANGEROURS;
              break;
          case 0x11: //��λ��ͨ��ң����ת��������PID��������
              ANO_Recive(RC_rxData);
              LED.status=DANGEROURS;
              break;
          case 0x12: //��λ��ͨ��ң����ת��������PID��������
              ANO_Recive(RC_rxData);
              LED.status=DANGEROURS;
              break;
        }
      } 
    }
  }
}

/*****************************************************************************************
 *  �����ж�
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/  
void remote_unlock(void)
{
  static uint8_t status=WAITING_1;
  static uint16_t cnt=0;

  if(Remote.thr<900 && Remote.yaw<1200)                         //����ң�����½�����
  {
    status = EXIT_255;
  }
  
  switch(status)
  {
    case WAITING_1://�ȴ�����
      if(Remote.thr<900)           //���������࣬�������->�������->������� ����LED�Ʋ����� ����ɽ���
      {       
          //if(cnt++ > 20)                                     // ����ң�˴������30S�Զ�����
          //{ //cnt=0;               
            status = WAITING_2;
          //}            
      }    
      break;
    case WAITING_2:
      if(Remote.thr>1800)          
      {    
        status = WAITING_3;
      }      
      break;
    case WAITING_3:
      if(Remote.thr<900)          
      {       
        status = WAITING_4;         
      }      
      break;      
    case WAITING_4:  //����ǰ׼��                 
        ALL_flag.unlock = 1;
        status = PROCESS_31;
        LED.status = AlwaysOn;                  
        break;    
    case PROCESS_31:  //�������״̬
        if(Remote.thr<900)
        {
          if(cnt++ > 1500)    // ����ң�˴������30S�Զ�����
          { 
            cnt=0;               
            status = EXIT_255;                
          }
        }
        else if(!ALL_flag.unlock)                           //Other conditions lock 
        {
          status = EXIT_255;        
        }
        else          
          cnt = 0;
      break;
    case EXIT_255: //��������
      LED.status = AllFlashLight;                                   //exit
      cnt = 0;
      LED.FlashTime = 100; //100*3ms    
      ALL_flag.unlock = 0;
      status = WAITING_1;
      break;
    default:
      status = EXIT_255;
      break;
  }
}
/***********************END OF FILE*************************************/







