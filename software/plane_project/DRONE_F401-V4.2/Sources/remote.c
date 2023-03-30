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
//         此程序只能用作学习，如用商业用途。必追究责任！
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
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/  
uint16_t nrf_cnt=0;
uint8_t RC_rxData[32];
extern UART_HandleTypeDef huart6;

uint8_t CheckSum=0;

extern uint8_t nrf2401_txbuf[];
extern uint8_t txbuf_pos;

/*对于地面站写入PID参数的命令返回确认帧*/
static void Check(uint8_t num)
{
  uint8_t check_buff[7];
  check_buff[0]=0xaa;
  check_buff[1]=0xaa;
  check_buff[2]=0xef;
  check_buff[3]=0x02; //LEN
  check_buff[4]=num;  //PID分组序号
  /*CHECK_SUM*/
  check_buff[5]=(uint8_t)CheckSum;
  /*SUM*/
  int sum=0;
  for (int i = 0; i < 6; i++)
  {
    sum+=check_buff[i];
  }
  check_buff[6]=(uint8_t)sum;
  /*存入缓冲区*/
  for (int i = 0; i < 7; i++)
  {
    nrf2401_txbuf[i]=check_buff[i];
  }
  txbuf_pos=7;
}

void remote_unlock(void);  
void RC_Analy(void)  
{ 
  uint8_t rxlen;
  static uint8_t hover_flag=0,take_off_status=0;//悬停标志位
  static _st_Remote last_remote;//remote的上一次的状态量，用于检测状态跳变
  uint16_t temp=0;
  //Receive  and check RC data 
  rxlen=NRF24L01_RxPacket(RC_rxData);
  // send_char_array(&huart1,RC_rxData,rxlen);
  if(rxlen>0)
  { 
    uint8_t i;
    // uint8_t CheckSum=0;  //修改为全局变量用于返回确认帧
    CheckSum=0;
    //send_char_array(&huart6,RC_rxData,rxlen);
    nrf_cnt = 0;
    for(i=0;i<rxlen-1;i++)
    {
      CheckSum +=  RC_rxData[i];
    }
    if(RC_rxData[rxlen-1]==CheckSum)
    { 
      if (RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //如果接收到的遥控数据帧头正确
      {
        switch(RC_rxData[2])
        {
          case 0x03: //遥控器发的油门和姿态角的控制命令
              temp=((uint16_t)RC_rxData[4] <<8) | RC_rxData[5];
              if (take_off_status==0)
              {
                Remote.thr  =((uint16_t)RC_rxData[4] <<8) | RC_rxData[5] ;  //通道1
                last_remote.thr=Remote.thr;
              }
              Remote.yaw  =((uint16_t)RC_rxData[6] <<8) | RC_rxData[7] ;  //通道2
              Remote.roll =((uint16_t)RC_rxData[8] <<8) | RC_rxData[9] ;  //通道3
              Remote.pitch=((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];  //通道4
              Remote.AUX1 =((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];  //通道5  左上角按键都属于通道5  
              Remote.AUX2 =((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];  //通道6  右上角按键都属于通道6 
              Remote.AUX3 =((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];  //通道7  左下边按键都属于通道7 
              Remote.AUX4 =((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];  //通道8  右下边按键都属于通道8  
              Remote.AUX5 =((uint16_t)RC_rxData[20]<<8) | RC_rxData[21];  //通道8  悬停
              last_remote.AUX6=Remote.AUX6;//保留上一次的值用于状态脉冲的测量
              Remote.AUX6 =((uint16_t)RC_rxData[22]<<8) | RC_rxData[23];  //通道8  一键起飞，一键降落
              if (last_remote.AUX6==0 && Remote.AUX6==1)//状态跳变，触发一键起飞动作
              {
                last_remote.thr=TAKE_OFF_THR;
                take_off_status=1;
              }
              else if(last_remote.AUX6==1 && Remote.AUX6==0)
              {
                last_remote.thr=MIN_THR;
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
              
              //判断悬停
              if(hover_flag==0 && Remote.AUX5==1)//如果是悬停，那么姿态控制的期望为0
              {
                pidHeightHigh.desired=pidHeightHigh.measured;//用当前的测量值作为期望值
                pidPitch.desired=0;pidRoll.desired=0;pidYaw.desired=0;
                LED.status=AlwaysOff;
                hover_flag=1;//把悬停标志变量置1，避免重复设定高度期望
              }
              else if(hover_flag==1 && Remote.AUX5==0)
              {
                LED.status=AlwaysOn;
                hover_flag=0;//取消悬停

                const float roll_pitch_ratio = 0.04f;
                //const float yaw_ratio =  0.0015f;    
            
                pidPitch.desired =(1500-Remote.pitch)*roll_pitch_ratio;   //将遥杆值作为飞行角度的期望值
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
          case 0x02: //上位机通过遥控器转发的读取PID参数命令
              ANO_Recive(RC_rxData);//把发送PID数据到上位机的标志变量置为1
              LED.status=WARNING;
              break;
          case 0x10: //上位机通过遥控器转发的设置PID参数命令
              ANO_Recive(RC_rxData);
              Check(0x10);
              LED.status=DANGEROURS;
              break;
          case 0x11: //上位机通过遥控器转发的设置PID参数命令
              ANO_Recive(RC_rxData);
              Check(0x11);
              LED.status=DANGEROURS;
              break;
          case 0x12: //上位机通过遥控器转发的设置PID参数命令
              ANO_Recive(RC_rxData);
              Check(0x12);
              LED.status=DANGEROURS;
              break;
          case 0x13: //设置PID4
              ANO_Recive(RC_rxData);
              Check(0x13);
              break;
          case 0x14: //设置PID5
              ANO_Recive(RC_rxData);
              Check(0x14);
              break;
          case 0x15: //设置PID6
              ANO_Recive(RC_rxData);
              Check(0x15);
              break;
          default:break;
        }
      } 
    }
  }
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/  
void remote_unlock(void)
{
  static uint8_t status=WAITING_1;
  static uint16_t cnt=0;

  if(Remote.thr<900 && Remote.yaw<1200)                         //油门遥杆左下角锁定
  {
    status = EXIT_255;
  }
  
  switch(status)
  {
    case WAITING_1://等待解锁
      if(Remote.thr<MIN_THR)           //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
      {       
          //if(cnt++ > 20)                                     // 油门遥杆处于最低30S自动上锁
          //{ //cnt=0;               
            status = WAITING_2;
          //}            
      }    
      break;
    case WAITING_2:
      if(Remote.thr>MAX_THR)          
      {    
        status = WAITING_3;
      }      
      break;
    case WAITING_3:
      if(Remote.thr<MIN_THR)          
      {       
        status = WAITING_4;         
      }      
      break;      
    case WAITING_4:  //解锁前准备                 
        ALL_flag.unlock = 1;
        status = PROCESS_31;
        LED.status = AlwaysOn;                  
        break;    
    case PROCESS_31:  //进入解锁状态
        if(Remote.thr<MIN_THR)
        {
          if(cnt++ > 1500)    // 油门遥杆处于最低30S自动上锁
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
    case EXIT_255: //进入锁定
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







