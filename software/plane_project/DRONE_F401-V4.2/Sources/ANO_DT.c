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
#include <stdlib.h>
#include <string.h>
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "UART.h"
#include "main.h"
/******************************************************************/
//--------------------------
static uint8_t RatePID[18];//����PIDʱ����ݴ����
static uint8_t AnglePID[18];
static uint8_t HighPID[18];
extern uint8_t nrf2401_txbuf[];
extern uint8_t txbuf_pos;

extern UART_HandleTypeDef huart1;
extern uint32_t baro_height;

static struct{
  uint8_t PID1 :1; //���ܵ���λ��PID��1
  uint8_t PID2 :1; //���ܵ���λ��PID��2
  uint8_t PID3 :1; //���ܵ���λ��PID��3
  uint8_t PID4 :1; //���ܵ���λ��PID��4
  uint8_t PID5 :1; //���ܵ���λ��PID��5
  uint8_t PID6 :1; //���ܵ���λ��PID��6  
  uint8_t CMD2_READ_PID:1; //���ܵ���λ����ȡPID������
}ANTO_Recived_flag;


/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/  
void ANO_Recive(uint8_t *pt)                   //���յ���λ��������
{
  switch(pt[2])
  {
    case ANTO_RATE_PID:
      ANTO_Recived_flag.PID1 = 1;             //���յ���λ��������PID����
      memcpy(RatePID,&pt[4],18);              //�Ȱѽ��յ����������������ֹ����һ��PID���ݸ��ǣ������PID�Ǹ��ٶȻ��õ�
      break;
    case ANTO_ANGLE_PID:                      //�����PID�Ǹ��ǶȻ��õ�
      memcpy(AnglePID,&pt[4],18);
      ANTO_Recived_flag.PID2 = 1;
      break;
    case ANTO_HEIGHT_PID:                     //�����PID�Ǹ��߶Ȼ��õ�
      memcpy(HighPID,&pt[4],18);
      ANTO_Recived_flag.PID3 = 1;
      break;
    case ANTO_PID4:
      break;
    case ANTO_PID5:   
      break;
    case ANTO_PID6:
      break;
    case 0x01:                                //��λ��������CMD1 ��������У׼

      break;
    case 0x02:                                //��λ��������CMD2 ���������ȡPID��
      { 
        // ANTO_Recived_flag.CMD2_READ_PID = 1;//test
        enum                                  //��λ����ɿ�����
        {
          READ_PID = 0X01,                    //��ȡ�ɿص�PID����
          READ_MODE = 0x02,                   //��ȡ����ģʽ
          READ_ROUTE = 0x21,                  //��ȡ������Ϣ
          READ_VERSION = 0XA0,                //��ȡ�ɿذ汾
          RETURN_DEFAULT_PID = 0xA1           //�ָ�Ĭ��PID
         };  //CMD2;/

        switch(*(uint8_t*)&pt[4])             //�ж���λ������CMD������
        {
          case READ_PID:                      //��λ�������ȡ�ɿ�PID����
            ANTO_Recived_flag.CMD2_READ_PID = 1;
            break;
          case READ_MODE: 
            break;
          case READ_ROUTE: 
            break;          
          case READ_VERSION:  
            break;
          case RETURN_DEFAULT_PID:  
            break;          
          default: 
            break;          
        }
      }
      break;
    case ANTO_RCDATA: //Immediately deal with 
      break;
    default:
      break;      
  }
  return;
}
/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
static void ANTO_Send(const enum ANTO_SEND FUNCTION) //�������ݵ���λ��
{
  uint8_t i;
  uint8_t len=2;
  int16_t Anto[12];
  int8_t *pt = (int8_t*)(Anto);
  PidObject *pidX=0;
  PidObject *pidY=0;
  PidObject *pidZ=0;

  switch(FUNCTION)
  {
    case ANTO_RATE_PID:      //PID1
         pidX = &pidRateX;
         pidY = &pidRateY;
         pidZ = &pidRateZ;
         goto send_pid;    
    case ANTO_ANGLE_PID:      //PID2
         pidX = &pidRoll;
         pidY = &pidPitch;
         pidZ = &pidYaw;
         goto send_pid;        
    case ANTO_HEIGHT_PID:     //PID3
         pidX = &pidHeightRate;
         pidY = &pidHeightHigh;
         goto send_pid;              
    case ANTO_PID4:           //PID4
    case ANTO_PID5:           //PID5
    case ANTO_PID6:           //PID6

send_pid:
      if(pidX!=NULL)
      {
        Anto[2] = (int16_t)(pidX->kp *1000);
        Anto[3] = (int16_t)(pidX->ki *1000);
        Anto[4] = (int16_t)(pidX->kd *1000);
      }
      if(pidY!=NULL)
      {
        Anto[5] = (int16_t)(pidY->kp *1000);
        Anto[6] = (int16_t)(pidY->ki *1000);
        Anto[7] = (int16_t)(pidY->kd *1000);
      }
      if(pidZ!=NULL)
      {
        Anto[8] = (int16_t)(pidZ->kp *1000);
        Anto[9] = (int16_t)(pidZ->ki *1000);
        Anto[10] = (int16_t)(pidZ->kd *1000);
      }
      len = 18;
      break;
    case ANTO_MOTOR:    //send motor
      len = 8;
      break;  
    case ANTO_RCDATA: //send RC data
      break;
    case ANTO_MPU_MAGIC:     //����MPU6050�ʹ����Ƶ�����
      memcpy(&Anto[2],(int8_t*)&MPU6050,sizeof(_st_Mpu));
      AK8975.magZ=(int16_t)pidHeightHigh.measured;
      AK8975.magY=(int16_t)baro_height;
      memcpy(&Anto[8],(int8_t*)&AK8975,sizeof(_st_Mag));
      len = 18;
      break;
    case ANTO_SENSER2:
      break;
    case ANTO_STATUS:     //send angle
        Anto[2] = (int16_t)(-Angle.roll*100);
        Anto[3] = (int16_t)(Angle.pitch*100);
        Anto[4] = (int16_t)(-Angle.yaw*100);
        Anto[5] = baro_height>>16;
        Anto[6] = baro_height;
        Anto[7] = (0x01<<8)|(ALL_flag.unlock);
        len = 12;
      break;
    case ANTO_POWER:
        break;
    default:
      break;      
  }
  
  Anto[0] = 0XAAAA;
  Anto[1] = len | FUNCTION<<8;//12|0x01<<8
  pt[len+4] = (int8_t)(0xAA+0xAA);
  for(i=2;i<len+4;i+=2)    //a swap with b;
  {
    pt[i] ^= pt[i+1];
    pt[i+1] ^= pt[i];
    pt[i] ^= pt[i+1];
    pt[len+4] += pt[i] + pt[i+1];
  }
  
  send_char_array(&huart1,(uint8_t *)pt,len+5);
  if(FUNCTION==ANTO_STATUS)
  { 
	  //NRF24L01_Write_Buf(0xa8, (uint8_t *)pt, len+5);
    txbuf_pos=len+5;
    for(i=0;i<txbuf_pos;i++)
      nrf2401_txbuf[i]=pt[i];
  }
  else if(FUNCTION==ANTO_RATE_PID || FUNCTION==ANTO_ANGLE_PID || FUNCTION==ANTO_HEIGHT_PID)
  {
    //��������ģ�黺����
    txbuf_pos=len+5;
    for(i=0;i<txbuf_pos;i++)
      nrf2401_txbuf[i]=pt[i];
    //�ô���1��������
    // send_char_array(&huart1,(uint8_t *)pt,len+5);
  }
  else if(FUNCTION==ANTO_MPU_MAGIC)
  {
    //��������ģ�黺����
    txbuf_pos=len+5;
    for(i=0;i<txbuf_pos;i++)
      nrf2401_txbuf[i]=pt[i];
  }
}
/***********************************************************************
 * polling  work.
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void ANTO_polling(void) //��ѯɨ����λ���˿�
{
  volatile static uint8_t status = 0;
  switch(status)
  {
    case 0:
      status = 1;
      break;
    case 1:
      ANTO_Send(ANTO_MPU_MAGIC);
      HAL_Delay(1);
      ANTO_Send(ANTO_STATUS);
      HAL_Delay(3);
      if(*(uint8_t*)&ANTO_Recived_flag != 0) //һ�����յ���λ�������ݣ�����ͣ�������ݵ���λ����ת��ȥ�ж���λ��Ҫ��ɿ���ʲô��
      {
        status = 2;
      }
        break;
    case 2:
      if(*(uint8_t*)&ANTO_Recived_flag == 0)//��λ���ķ����������ݶ��������ˣ��򷵻�ר�ĵķ������ݵ���λ��
      {
        status = 1;
      }
  
      if(ANTO_Recived_flag.CMD2_READ_PID) //�ж���λ���Ƿ����󷢷��ͷɿ�PID���ݵ���λ��
      {    
        ANTO_Send(ANTO_RATE_PID);
        HAL_Delay(10);
        ANTO_Send(ANTO_ANGLE_PID);
        HAL_Delay(10);
        ANTO_Send(ANTO_HEIGHT_PID);
        HAL_Delay(10);
        ANTO_Recived_flag.CMD2_READ_PID = 0;
        // txbuf_pos=0;
      }
      
      if(*(uint8_t*)&ANTO_Recived_flag & 0x3f) //���յ���λ��������PID����
      {
          PidObject *pidX=0;
          PidObject *pidY=0;
          PidObject *pidZ=0;
          uint8_t *P;
        
          if(ANTO_Recived_flag.PID1)
          {
            pidX = &pidRateX;
            pidY = &pidRateY;
            pidZ = &pidRateZ;
            P = RatePID;
            ANTO_Recived_flag.PID1 = 0;
          }
          else if(ANTO_Recived_flag.PID2)
          {
            pidX = &pidRoll;
            pidY = &pidPitch;
            pidZ = &pidYaw;
            P = AnglePID;  
            ANTO_Recived_flag.PID2 = 0;                             
          }
          else if(ANTO_Recived_flag.PID3)
          {
            pidX = &pidHeightRate;
            pidY = &pidHeightHigh;
            P = HighPID;  
            ANTO_Recived_flag.PID3 = 0;                 
          }
          else
          {
            break;
          }
          {
              union 
              {
                uint16_t _16;
                uint8_t _u8[2];
              }data;
              
              if(pidX!=NULL)
              {
                data._u8[1] = P[0]; 
                data._u8[0] = P[1];
                pidX->kp =  data._16 /1000.0f;
                data._u8[1] = P[2]; 
                data._u8[0] = P[3];
                pidX->ki =  data._16 /1000.0f;
                data._u8[1] = P[4]; 
                data._u8[0] = P[5];
                pidX->kd =  data._16 /1000.0f;                
              }
              if(pidY!=NULL)
              {
                data._u8[1] = P[6]; 
                data._u8[0] = P[7];
                pidY->kp =  data._16 /1000.0f;
                data._u8[1] = P[8]; 
                data._u8[0] = P[9];
                pidY->ki =  data._16 /1000.0f;
                data._u8[1] = P[10]; 
                data._u8[0] = P[11];
                pidY->kd =  data._16 /1000.0f;    
              }
              if(pidZ!=NULL)
              {
                data._u8[1] = P[12]; 
                data._u8[0] = P[13];
                pidZ->kp =  data._16 /1000.0f;
                data._u8[1] = P[14]; 
                data._u8[0] = P[15];
                pidZ->ki =  data._16 /1000.0f;
                data._u8[1] = P[16]; 
                data._u8[0] = P[17];
                pidZ->kd =  data._16 /1000.0f;    
              }        
          }        
      }
      break;
    default:
      break;
  }

}

/************************END OF FILE********************/
