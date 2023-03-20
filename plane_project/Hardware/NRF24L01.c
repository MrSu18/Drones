#include "NRF24L01.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////   
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//NRF24L01 驱动函数     
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/16 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////     
#define EN_DYNAMIC_DATA_LENGTH 1  //1:动态数据长度;0:固定长度(32字节)

//const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
//const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //接收地址
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]= {0x11,0x22,0x33,0x44,0x55};  //本地地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]= {0x11,0x22,0x33,0x44,0x55};  //接收地址RX_ADDR_P0 == RX_ADDR
extern SPI_HandleTypeDef hspi1; 

uint8_t SPIx_ReadWriteByte(uint8_t TxData)
{ uint8_t RxData;
  HAL_SPI_TransmitReceive(&hspi1,&TxData,&RxData,1,10);
  return RxData;
}
//检测24L01是否存在
//返回值:0，成功;1，失败  
uint8_t NRF24L01_Check(void)
{
  uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5},buf2[5];
  uint8_t i;
  NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.  
  NRF24L01_Read_Buf(TX_ADDR,buf2,5); //读出写入的地址  
  for(i=0;i<5;i++)if(buf2[i]!=0XA5)break;                    
  if(i!=5)return 1;//检测24L01错误  
  return 0;     //检测到24L01
}      
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
  uint8_t status;  
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);                 //使能SPI传输
  status =SPIx_ReadWriteByte(reg);//发送寄存器号 
  SPIx_ReadWriteByte(value);      //写入寄存器的值
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);                 //禁止SPI传输     
  return(status);             //返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
  uint8_t reg_val;      
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);          //使能SPI传输    
  SPIx_ReadWriteByte(reg);   //发送寄存器号
  reg_val=SPIx_ReadWriteByte(0XFF);//读取寄存器内容
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);          //禁止SPI传输        
  return(reg_val);           //返回状态值
}  
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
  uint8_t status,u8_ctr;         
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);           //使能SPI传输
  status=SPIx_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值        
  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
    pBuf[u8_ctr]=SPIx_ReadWriteByte(0XFF);//读出数据
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);       //关闭SPI传输
  return status;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status,u8_ctr;      
  
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);//NRF24L01_CSN = 0;//使能SPI传输
  status = SPIx_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  for(u8_ctr=0; u8_ctr<len; u8_ctr++)
    SPIx_ReadWriteByte(*pBuf++); //写入数据   
    
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET); //NRF24L01_CSN = 1;//关闭SPI传输
  return status;          //返回读到的状态值
}           
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf,uint8_t len)
{
  uint8_t sta;
  uint16_t cnt=0;  
  //SPIx_SetSpeed(SPI_SPEED_8);//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
#if EN_DYNAMIC_DATA_LENGTH//if使能动态数据长度
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,len);//写数据到TX BUF  最多32个字节
#else
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  最多32个字节  
#endif
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//准备发送
  //while((NRF24L01_IRQ!=0)&&cnt<10000)cnt++;//等待发送完成
  while(cnt<1000) cnt++; //CE变高10us后才开始发送
  sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
  while( ((sta&TX_OK)==0) && (cnt<20000) ) //发送未完成，则等待
  { 
    sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
    cnt++;
  }    
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,sta&0x70); //清除TX_DS或MAX_RT中断标志
  if(sta&TX_OK)//发送完成
  {
    return TX_OK;
  }
  if(sta&MAX_TX)//达到最大重发次数
  {
    NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
    return MAX_TX; 
  }
  return 0xff;//其他原因发送失败
}
//启动NRF24L01接收一次数据
//rxbuf:存储接收到的数据，首地址
//返回值:n，收到的字节数；0，没收到数据
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
  uint8_t sta,len;                         
  //SPIx_SetSpeed(SPI_SPEED_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
  sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值       
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,sta); //清除RX_DR等中断标志
  if(sta&RX_OK)//接收到数据
  {
  #if EN_DYNAMIC_DATA_LENGTH//if使能动态数据长度
    NRF24L01_Read_Buf(R_RX_PL_WID,&len,1);//读取有效数据长度,存在len中
    NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,len);//读取len个字节数据
    NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
    NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,RX_OK); //清除中断标志
    return len; 
  #else
    NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
    NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
    return RX_PLOAD_WIDTH;     
  #endif
  }     
  return 0;//没收到任何数据
}              
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了       
void RX_Mode(void)
{
  NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
  NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);    
  
  //NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址     
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,99);       //设置RF通信频率      
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度,动态数据长度模式下不需配置       
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);//设置RX参数,增益0x0f:7db,0x0b:0db,2Mbps,低噪声增益开启   
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0e);//设置RX参数,4db增益,2Mbps,低噪声增益开启   
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x3f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开放接收中断，屏蔽发送中断和发送最大次数中断 
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //清除RX_DR、TX_DS、MAX_RT中断标志

#if EN_DYNAMIC_DATA_LENGTH//if使能动态数据长度
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x01);  //DPL_P0
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04);//EN_DPL
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04+0x02);//EN_DPL+使能ACK带数据返回确认帧
#endif  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CE为高,进入接收模式 
  //如果不加后面这三句，则飞控板要先发送成功数据才能接收，很奇怪。 加了这三句后，上电启动就能接收
  //STM32F401核心板接无线模块，不用加这三句就能上电即可接收。
  HAL_Delay(10);
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  //HAL_Delay(1);
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CE为高,进入接收模式
}             
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了       
//CE为高大于10us,则启动发送.   
void TX_Mode(void)
{                             
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  
  NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK    

  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);     //禁用通道0-5的自动应答    
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x12);//设置自动重发间隔时间:500us + 86us;自动重发次数:2次
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,99);       //设置RF频率
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,7db增益,2Mbps,低噪声增益开启   
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0B);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x7e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发送模式,屏蔽所有中断
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //清除RX_DR、TX_DS、MAX_RT中断标志
#if EN_DYNAMIC_DATA_LENGTH//if使能动态数据长度
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x01);  //DPL_P0
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04);//EN_DPL
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04+0x02);//EN_DPL+使能ACK带数据返回确认帧
#endif  
  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CE为高,10us后启动发送
} 

void RX2TX(void)
{
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x0e); //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发送模式,开启所有中断
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x7e); //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发送模式,屏蔽所有中断
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //清除RX_DR、TX_DS、MAX_RT中断标志
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CE为高,10us后启动发送  
}
void TX2RX(void)
{
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x3f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,屏蔽发送中断和发送最大次数中断 
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //清除RX_DR、TX_DS、MAX_RT中断标志  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CE为高,进入接收模式 

}

