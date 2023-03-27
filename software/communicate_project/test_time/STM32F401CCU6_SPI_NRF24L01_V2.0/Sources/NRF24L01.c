#include "NRF24L01.h"
#include "main.h"
#include "stdio.h"
//////////////////////////////////////////////////////////////////////////////////   
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//////////////////////////////////////////////////////////////////////////////////     
#define EN_DYNAMIC_DATA_LENGTH 1  //1:��̬���ݳ���;0:�̶�����(32�ֽ�)
#define NRF24L01_FREQUENCY  99  //ͨ��Ƶ��

//const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
//const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���յ�ַ
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x11,0x22,0x33,0x44,0x55}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x11,0x22,0x33,0x44,0x55}; //���յ�ַ
extern SPI_HandleTypeDef hspi1; 
uint8_t status_si2401=0;
uint8_t SPIx_ReadWriteByte(uint8_t TxData)
{ uint8_t RxData;
  HAL_SPI_TransmitReceive(&hspi1,&TxData,&RxData,1,10);
  return RxData;
}
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��  
uint8_t NRF24L01_Check(void)
{
  uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5},buf2[5];
  uint8_t i;
  NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.  
  NRF24L01_Read_Buf(TX_ADDR,buf2,5); //����д��ĵ�ַ  
  for(i=0;i<5;i++)if(buf2[i]!=0XA5)break;                    
  if(i!=5)return 1;//���24L01����  
  return 0;     //��⵽24L01
}      
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
//����״̬�Ĵ�����ֵ
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
  uint8_t status;  
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);                 //ʹ��SPI����
  status =SPIx_ReadWriteByte(reg);//���ͼĴ����� 
  SPIx_ReadWriteByte(value);      //д��Ĵ�����ֵ
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);                 //��ֹSPI����     
  return(status);             //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{ //static uint8_t status;
  static uint8_t reg_val;      
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);          //ʹ��SPI����    
  SPIx_ReadWriteByte(reg);   //���ͼĴ�����
  reg_val=SPIx_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);          //��ֹSPI����        
  return(reg_val);           //����״ֵ̬
}  
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
  uint8_t status,u8_ctr;         
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);           //ʹ��SPI����
  status=SPIx_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬        
  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
    pBuf[u8_ctr]=SPIx_ReadWriteByte(0XFF);//��������
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);       //�ر�SPI����
  return status;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status,u8_ctr;      
  
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);//NRF24L01_CSN = 0;//ʹ��SPI����
  status = SPIx_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(u8_ctr=0; u8_ctr<len; u8_ctr++)
    SPIx_ReadWriteByte(*pBuf++); //д������   
    
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET); //NRF24L01_CSN = 1;//�ر�SPI����
  return status;          //���ض�����״ֵ̬
}           
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *txbuf,uint8_t len)
{
  uint8_t sta;
  uint16_t cnt=0;  
  //SPIx_SetSpeed(SPI_SPEED_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  status_si2401=NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,len);//д���ݵ�TX BUF  ���32���ֽ�
#else
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  ���32���ֽ�  
#endif
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//��������
  //while((NRF24L01_IRQ!=0)&&cnt<10000)cnt++;//�ȴ��������
  while(cnt<1000)cnt++; //�ȴ�������ɡ�  ��֣����������һ��䣬���ݾ������Ͳ��ɹ���
  sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
  while( ((sta&TX_OK)==0) && (cnt<20000) ) //����δ��ɣ���ȴ�
  { 
    sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
    cnt++;
  }   
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
  if(sta&MAX_TX)//�ﵽ����ط�����
  {
    status_si2401=NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
    return MAX_TX; 
  }
  if(sta&TX_OK)//�������
  {
    return TX_OK;
  }
  return 0xff;//����ԭ����ʧ��
}
//����NRF24L01����һ������
//rxbuf:�洢���յ������ݣ��׵�ַ
//����ֵ:n���յ����ֽ�����0��û�յ�����
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
  uint8_t sta,len;                         
  //SPIx_SetSpeed(SPI_SPEED_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
  sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ       
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,sta); //���RX_DR���жϱ�־
  if(sta&RX_OK)//���յ�����
  {
  #if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
    status_si2401=NRF24L01_Read_Buf(R_RX_PL_WID,&len,1);//��ȡ��Ч���ݳ���,����len��
    status_si2401=NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,len);//��ȡlen���ֽ�����
    status_si2401=NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
    return len; 
  #else
    NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
    NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
    return RX_PLOAD_WIDTH;     
  #endif
  }     
  return 0;//û�յ��κ�����
}              
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������       
void RX_Mode(void)
{
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);    
  
  status_si2401=NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);     //����ͨ��0-5���Զ�Ӧ��    
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ     
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,NRF24L01_FREQUENCY);       //����RFͨ��Ƶ��      
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��       
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_AW,0X03); //���õ�ַ���Ϊ 5bytes
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x2f);//����TX�������,0db����,2Mbps,���������濪��   
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־
#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04);//EN_DPL
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x01);  //DPL_P0
#endif  
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x0F);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���η����жϺͷ����������ж� 
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CEΪ��,�������ģʽ 
  //������Ӻ��������䣬��ɿذ�Ҫ�ȷ��ͳɹ����ݲ��ܽ��գ�����֡� ������������ϵ��������ܽ���
  //STM32F401���İ������ģ�飬���ü�����������ϵ缴�ɽ��ա�
  HAL_Delay(1);
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CEΪ��,�������ģʽ
}             
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������       
//CEΪ�ߴ���10us,����������.   
void TX_Mode(void)
{                             
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  
  status_si2401=NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  status_si2401=NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK    

  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x00);     //����ͨ��0-5���Զ�Ӧ��    
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,NRF24L01_FREQUENCY);       //����RFͨ��Ϊ40����Χ0-125
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,7db����,2Mbps,���������濪��   
  //status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x0B);  //����TX�������,0db����,2Mbps,���������濪��   
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x4e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ�����������ж�
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־
#if EN_DYNAMIC_DATA_LENGTH//ifʹ�ܶ�̬���ݳ���
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+DYNPD,0x01);  //DPL_P0
  status_si2401=NRF24L01_Write_Reg(NRF24L01_WRITE_REG+FEATURE,0x04);//EN_DPL
#endif  
  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CEΪ��,10us����������
  HAL_Delay(1);
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  if(time_flag==0)
  {
    time1=micros();
    time_flag=1;
  }
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CEΪ��,�������ģʽ
} 

void RX2TX(void)
{
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  //NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x0e); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x7e); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);//CEΪ��,10us����������  
}
void TX2RX(void)
{
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET);
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x3f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���η����жϺͷ����������ж� 
  NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS,0x70); //���RX_DR��TX_DS��MAX_RT�жϱ�־  
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET); //CEΪ��,�������ģʽ 

}

