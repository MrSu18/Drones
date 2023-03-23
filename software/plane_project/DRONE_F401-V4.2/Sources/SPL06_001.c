#ifndef SPL06_01_C
#define SPL06_01_C
#include "SPL06_001.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#undef SUCCESS 
#define SUCCESS 0
#undef FAILED 
#define FAILED 1
#undef ENABLE 
#define ENABLE 1
#undef DISABLE 
#define DISABLE 0

//�Ĵ�������
#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG_REG 0x06 //��ѹ������������
#define TMP_CFG_REG 0x07 //�¶Ȳ����ٶ�����
#define MEAS_CFG_REG 0x08 //���������봫��������
#define CFG_REG 0x09 //�ж�/FIFO/SPI����������
#define INT_STS_REG 0X0A //�ж�״̬��־λ
#define FIFO_STS_REG 0X0B //FIFO״̬
#define RESET_REG 0X0C
#define ID_REG 0x0D
#define COEF_REG 0x10

//#define HW_ADR 0x77 //SDO HIGH OR NC//��SDO���߻��߸���ʹ�õ�I2C��ַ
//#define HW_ADR 0x76 //SDO LOW//��SDO����ʹ�õ�I2C��ַ

//#define SPL06_ADDR  0xEE //(0x77<<1)
#define SPL06_ADDR  0xEC //(0x76<<1)

static struct  {  //�ڲ�����У׼����
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
}SPL06_calib_param;


struct  {  
    uint8_t chip_id; /**<chip id*/  
    int32_t i32rawPressure;//ԭʼ��ѹ����
    int32_t i32rawTemperature;//ԭʼ�¶�����
    int32_t i32kP;    //��ѹ��������
    int32_t i32kT;//�¶Ȳ�������
}SPL06;



//I2C����
//#define SPL06_001_write(ADDR,REG,DATA)  IIC_Write_1Byte(&hi2c1,ADDR,REG,DATA)
void SPL06_001_write(uint8_t hwadr, uint8_t regadr, uint8_t wdata)
{
  HAL_I2C_Mem_Write(&hi2c1,hwadr,regadr,I2C_MEMADD_SIZE_8BIT,&wdata,1,10);
}

uint8_t SPL06_001_read(uint8_t hwadr, uint8_t regadr)
{
  uint8_t reg_data;
  
  //IIC_Read_1Byte(&hi2c1,hwadr,regadr,&reg_data);
  HAL_I2C_Mem_Read(&hi2c1,hwadr,regadr,I2C_MEMADD_SIZE_8BIT,&reg_data,1,10);
  return reg_data;
}

//ʲô�ǹ����������ǲ�������ȡһ�����ݣ������¶Ȳ���Ƶ��10HZ�����ù�����Ϊ8������ÿ���¶ȶ�ȡ����8��ȡƽ�����ڲ�ʵ������80HZ�����ݲɼ�����̨ģʽ�޹�����
/***********************************************************************
 * ��ѹ����������
 * @param[in] ��̨ģʽbackground_rate����Ƶ��N��/ÿ�루������̨ģʽʹ��,����ģʽ����Ƶ�����û��Լ���������С���ȡ���ڹ���������ʱ��,��ͬ�Ĺ�����ʱ������в�ͬ�Ĺ��ģ���oversamply����������
 * @param[out] 
 * @return     
 **********************************************************************/
#define PRESSURE_RATE_1_TIMES 0 //������ 
#define PRESSURE_RATE_2_TIMES 1
#define PRESSURE_RATE_4_TIMES 2
#define PRESSURE_RATE_8_TIMES 3
#define PRESSURE_RATE_16_TIMES 4
#define PRESSURE_RATE_32_TIMES 5
#define PRESSURE_RATE_64_TIMES 6
#define PRESSURE_RATE_128_TIMES 7

void SPL06_pressure_rate_config(uint8_t background_rate,uint8_t oversamply)
{
    uint8_t data;
  
    data = (background_rate<<4)|oversamply;
    if(oversamply>PRESSURE_RATE_8_TIMES)//��������������EMPERATURE_RATE_8_TIMES��Ӧ���������ݱ��µ����ݸ��ǣ��ڲ�ӵ����ѹ���¶ȹ�32����FIFO���ڴ���8�Σ�Ҳ���Ǵ��ڻ����16�ι���������ʱ����Ҫ���µ����ݸ��ǣ��������ݾͻᶪʧ
    {
        uint8_t data;
        data = SPL06_001_read(SPL06_ADDR, CFG_REG);//��ȡԭ�Ĵ���ֵ
        data |= 0X04;//P-SHIFTλ��1
        SPL06_001_write(SPL06_ADDR, CFG_REG, data);//����д�ؼĴ���          
    }    
    switch(oversamply)
    {
        case PRESSURE_RATE_2_TIMES:
          SPL06.i32kP = 1572864;
            break;
        case PRESSURE_RATE_4_TIMES:
            SPL06.i32kP  = 3670016;
            break;
        case PRESSURE_RATE_8_TIMES:
            SPL06.i32kP  = 7864320;
            break;
        case PRESSURE_RATE_16_TIMES:
            SPL06.i32kP = 253952;
            break;
        case PRESSURE_RATE_32_TIMES:
            SPL06.i32kP = 516096;       
            break;
        case PRESSURE_RATE_64_TIMES:
            SPL06.i32kP = 1040384;           
            break;
        case PRESSURE_RATE_128_TIMES:
            SPL06.i32kP = 2088960;
            break;
        case PRESSURE_RATE_1_TIMES:
        default:
            SPL06.i32kP = 524288;
            break;
    }    
    SPL06_001_write(SPL06_ADDR, PRS_CFG_REG, data);//д������
}

/***********************************************************************
 * �¶Ȳ���������
 * @param[in] background_rate����Ƶ��N��/ÿ�루������̨ģʽʹ��,����ģʽ����Ƶ�����û��Լ���������С���ȡ���ڹ���������ʱ�䣩��oversamply���������� ext�¶ȼ�ѡ��
 * @param[out] 
 * @return     
 **********************************************************************/
#define TEMPERATURE_RATE_1_TIMES 0 //������
#define TEMPERATURE_RATE_2_TIMES 1
#define TEMPERATURE_RATE_4_TIMES 2
#define TEMPERATURE_RATE_8_TIMES 3
#define TEMPERATURE_RATE_16_TIMES 4
#define TEMPERATURE_RATE_32_TIMES 5
#define TEMPERATURE_RATE_64_TIMES 6
#define TEMPERATURE_RATE_128_TIMES 7
#define TEMPERATURE_RATE_TMP_EXT_INTERNAL 0  //���ɵ�·�ϵ��¶ȼ�
#define TEMPERATURE_RATE_TMP_EXT_EXTERNAL 1  //������MEMS��ѹоƬ���¶ȼ�
void SPL06_temperature_rate_config(uint8_t background_rate,uint8_t oversamply,uint8_t ext)
{
    uint8_t data;
  
      data = (ext<<7)|(background_rate<<4)|oversamply;
      if(oversamply>TEMPERATURE_RATE_8_TIMES)//��������������EMPERATURE_RATE_8_TIMES��Ӧ���������ݱ��µ����ݸ���
      {
          uint8_t data;
          data = SPL06_001_read(SPL06_ADDR, CFG_REG);//��ȡԭ�Ĵ���ֵ
          data |= 0X08;//T-SHIFTλ��1
          SPL06_001_write(SPL06_ADDR, CFG_REG, data);  //����д�ؼĴ���          
      }      
    switch(oversamply)
    {
        case TEMPERATURE_RATE_2_TIMES:
          SPL06.i32kT = 1572864;
            break;
        case TEMPERATURE_RATE_4_TIMES:
            SPL06.i32kT  = 3670016;
            break;
        case TEMPERATURE_RATE_8_TIMES:
            SPL06.i32kT  = 7864320;
            break;
        case TEMPERATURE_RATE_16_TIMES:
            SPL06.i32kT = 253952;
            break;
        case TEMPERATURE_RATE_32_TIMES:
            SPL06.i32kT = 516096;       
            break;
        case TEMPERATURE_RATE_64_TIMES:
            SPL06.i32kT = 1040384;           
            break;
        case TEMPERATURE_RATE_128_TIMES:
            SPL06.i32kT = 2088960;
            break;
        case TEMPERATURE_RATE_1_TIMES:
        default:
            SPL06.i32kT = 524288;
            break;
    }    
    SPL06_001_write(SPL06_ADDR, TMP_CFG_REG, data);//д������
}
/***********************************************************************
 * ����������ģʽ������״̬��ȡ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
#define MEAS_CFG_COEF_RDY 0X80 // �������ڲ�У׼ֵ�ɶ�����������
#define MEAS_CFG_SENSOR_RDY 0X40 // �������ѳ�ʼ����ɣ���������
#define MEAS_CFG_TMP_RDY 0x20 //�¶�ֵ�Ѿ�׼�����������Խ��ж�ȡ���ñ�־λ��ȡ���Զ���0
#define MEAS_CFG_PRS_RDY 0x10 //��ѹֵ�Ѿ�׼�����������Խ��ж�ȡ���ñ�־λ
#define MEAS_CFG_MEAS_CTR_STANDBY 0 //ģʽ���� ����ģʽ
#define MEAS_CFG_MEAS_CTR_COMMAND_PRS 0x01 //ģʽ���� ����ģʽ��������ѹ�ɼ�
#define MEAS_CFG_MEAS_CTR_COMMAND_TMP 0x02 //ģʽ���� ����ģʽ�������¶Ȳɼ�
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PRS 0x05 //ģʽ���� ��̨ģʽֻ��ȡ��ѹֵ
#define MEAS_CFG_MEAS_CTR_BACKGROUND_TMP 0X06 //ģʽ���� ��̨ģʽֻ��ȡ�¶�ֵ
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP 0X07 //ģʽ���� ��̨ģʽͬʱ��ȡ�¶�ֵ����ѹֵ
//��ȡ���������ݾ�λ״̬//����������״̬
uint8_t SPL06_get_measure_status(void)
{
  return SPL06_001_read(SPL06_ADDR, MEAS_CFG_REG);
}
//���ö�ȡģʽ+��ȡ��ʽ
void SPL06_set_measure_mode(uint8_t mode)  //����Ϊģʽֵ
{
   SPL06_001_write(SPL06_ADDR, MEAS_CFG_REG,mode);
}
//��������ģʽ��ȡ�¶�ֵ
void SPL06_start_temperature(void)
{
    SPL06_001_write(SPL06_ADDR, MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_COMMAND_TMP);
}
//��������ģʽ��ȡ��ѹֵ
void SPL06_start_pressure(void)
{
    SPL06_001_write(SPL06_ADDR, MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_COMMAND_PRS);
}
//�������ģʽ�������ֹͣ�ɼ�����ֱ���ٴ��л�ģʽ
void SPL06_enter_standby(void)
{
    SPL06_001_write(SPL06_ADDR, MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_STANDBY);
}

/***********************************************************************
 * �ж���FIFO���á�SPIģʽ����
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
#define CFG_INT_LEVEL_ACTIVE_LOW 0//�жϵ͵�ƽ��Ч
#define CFG_INT_LEVEL_ACTIVE_HIGH 1//�жϸߵ�ƽ��Ч
#define CFG_INT_FIFO 0X40    //��FIFO��ʹ���ж� 
#define CFG_INT_PRS 0X20    //����ѹ�ƶ�ȡ���ʹ���ж� 
#define CFG_INT_TMP 0X10    //���¶ȶ�ȡ���ʹ���ж� 
#define CFG_T_SHIFT 0X08    //�������ݱ����ǣ����Խ�����һ�ʲɼ�
#define CFG_P_SHIFT 0X04    //�������ݱ����ǣ����Խ�����һ�ʲɼ�
#define CFG_FIF 0X02    //ʹ��FIFO
#define CFG_SPI_3_WIRE 1    //3��SPI
#define CFG_SPI_4_WIRE 0    //4��SPI

void SPL06_set_interrupt(uint8_t interrupt,uint8_t type)//�����ж�ʹ��
{
  uint8_t data;
  data = SPL06_001_read(SPL06_ADDR, CFG_REG);
  if(type!=ENABLE )
    data &= ~interrupt;
  else
    data |= interrupt;  
  SPL06_001_write(SPL06_ADDR, CFG_REG,data);
}

void SPL06_set_spi_wire(uint8_t wire)//����SPI���� //3��/4��SPI��ȡ����
{
  uint8_t data;
  data = SPL06_001_read(SPL06_ADDR, CFG_REG);
  data &= 0xf7;//SPI����������λ��0
  data |= wire;
  SPL06_001_write(SPL06_ADDR, CFG_REG,data);
}

void SPL06_set_intrupt_level(uint8_t level)//�����ж���Ч��ƽ//INT�ߵ�ƽ��Ч���ߵ͵�ƽ��Ч//levelΪ0��͵�ƽ��Ч,Ϊ1��ߵ�ƽ��Ч
{
  uint8_t data;
  data = SPL06_001_read(SPL06_ADDR, CFG_REG);
  data &= 0x7f;//�жϵ�ƽ��Чλ��0
  data |= level<<7;
  SPL06_001_write(SPL06_ADDR, CFG_REG,data);
}

/***********************************************************************
 * �ж�״̬��ȡ����Ӳ����1����������������жϣ���ȡ�üĴ����Զ���0
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
#define INT_STS_FIFO_FULL  0X04 //FIFO���ж�״̬
#define INT_STS_FIFO_TMP   0X02  //�¶Ȳ�����ɱ�־λ
#define INT_STS_FIFO_PRS  0X01  //��ѹ������ɱ�־λ

uint8_t SPL06_get_int_status(void)//��ȡ������״̬
{
  return SPL06_001_read(SPL06_ADDR, INT_STS_REG);
}

/***********************************************************************
 * FIFO״̬��ȡ��
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
#define FIFO_STS_FULL  0X02 //FIFO��
#define FIFO_STS_EMPTY   0X01 //FIFO����
uint8_t SPL06_get_fifo_status(void)
{
  return SPL06_001_read(SPL06_ADDR, FIFO_STS_REG);
}
/***********************************************************************
 *��λ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
#define RESET_FIFO_FLUSH 0X80 //FIFO��0
#define RESET_SOFT 0X09//�����λ

void SPL06_soft_reset(void)//�����λ
{
   SPL06_001_write(SPL06_ADDR,RESET_REG,RESET_SOFT);
}
void SPL06_reset_fifo(void)//�����FIFO
{
  SPL06_001_write(SPL06_ADDR,RESET_REG,RESET_FIFO_FLUSH);
}
/***********************************************************************
 *ID
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
#define PRODUCT_ID 0X10//��ƷID
uint8_t SPL06_get_chip_id(void)//��ȡ��ƷID//��ȡ��Ʒ�汾//���ڰ汾�ڲ�ͬ�Ĵ������в�ͬ��������ֻ�ж�ID��ʶ��SPL06
{
  return SPL06_001_read(SPL06_ADDR, ID_REG);
}

/***********************************************************************
 * ��ȡ��ѹ���ڲ���У׼����
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void SPL06_001_get_calib_param(void)//�ڲ�У׼ֵ//��ѹ�ƽ����Լ��²�ʹ��//���ڲ������趨
{
    unsigned long h;
    unsigned long m;
    unsigned long l;
    h =  SPL06_001_read(SPL06_ADDR, 0x10);
    l  =  SPL06_001_read(SPL06_ADDR, 0x11);
    SPL06_calib_param.c0 = (int16_t)h<<4 | l>>4;
    SPL06_calib_param.c0 = (SPL06_calib_param.c0&0x0800)?(0xF000|SPL06_calib_param.c0):SPL06_calib_param.c0;
    h =  SPL06_001_read(SPL06_ADDR, 0x11);
    l  =  SPL06_001_read(SPL06_ADDR, 0x12);
    SPL06_calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
    SPL06_calib_param.c1 = (SPL06_calib_param.c1&0x0800)?(0xF000|SPL06_calib_param.c1):SPL06_calib_param.c1;
    h =  SPL06_001_read(SPL06_ADDR, 0x13);
    m =  SPL06_001_read(SPL06_ADDR, 0x14);
    l =  SPL06_001_read(SPL06_ADDR, 0x15);
    SPL06_calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    SPL06_calib_param.c00 = (SPL06_calib_param.c00&0x080000)?(0xFFF00000|SPL06_calib_param.c00):SPL06_calib_param.c00;
    h =  SPL06_001_read(SPL06_ADDR, 0x15);
    m =  SPL06_001_read(SPL06_ADDR, 0x16);
    l =  SPL06_001_read(SPL06_ADDR, 0x17);
    SPL06_calib_param.c10 = (int32_t)h<<16 | (int32_t)m<<8 | l;
    SPL06_calib_param.c10 = (SPL06_calib_param.c10&0x080000)?(0xFFF00000|SPL06_calib_param.c10):SPL06_calib_param.c10;
    h =  SPL06_001_read(SPL06_ADDR, 0x18);
    l  =  SPL06_001_read(SPL06_ADDR, 0x19);
    SPL06_calib_param.c01 = (int16_t)h<<8 | l;
    h =  SPL06_001_read(SPL06_ADDR, 0x1A);
    l  =  SPL06_001_read(SPL06_ADDR, 0x1B);
    SPL06_calib_param.c11 = (int16_t)h<<8 | l;
    h =  SPL06_001_read(SPL06_ADDR, 0x1C);
    l  =  SPL06_001_read(SPL06_ADDR, 0x1D);
    SPL06_calib_param.c20 = (int16_t)h<<8 | l;
    h =  SPL06_001_read(SPL06_ADDR, 0x1E);
    l  =  SPL06_001_read(SPL06_ADDR, 0x1F);
    SPL06_calib_param.c21 = (int16_t)h<<8 | l;
    h =  SPL06_001_read(SPL06_ADDR, 0x20);
    l  =  SPL06_001_read(SPL06_ADDR, 0x21);
    SPL06_calib_param.c30 = (int16_t)h<<8 | l;
}
/***********************************************************************
 * ��ʼ��
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
uint8_t SPL06_001_init(void)
{
    uint8_t SPL06_start_status;
    //�ȴ��ڲ�У׼���ݿ���, ��ʱ�˳�
    uint8_t tickstart=HAL_GetTick();
    do
    { SPL06_start_status = SPL06_get_measure_status();//��ȡ��ѹ������״̬
      if(HAL_GetTick()-tickstart>200)
        return ERROR;
    }
    while((SPL06_start_status&MEAS_CFG_COEF_RDY)!=MEAS_CFG_COEF_RDY);
    //��ȡ�ڲ�У׼ֵ  
    SPL06_001_get_calib_param();
    
    //�ȴ��������ڲ���ʼ�����, ��ʱ�˳�
    tickstart=HAL_GetTick();
    do
    { SPL06_start_status = SPL06_get_measure_status();//��ȡ��ѹ������״̬
      if(HAL_GetTick()-tickstart>200)
        return ERROR;
    }   
    while((SPL06_start_status&MEAS_CFG_SENSOR_RDY)!=MEAS_CFG_SENSOR_RDY);
    //��ȡCHIP ID
    SPL06.chip_id = SPL06_get_chip_id();
    //�ж϶�ȡ��ID�Ƿ���ȷ������ֻ�жϸ�4λ��ID�����жϵ�4λ�İ汾��
    if((SPL06.chip_id&0xf0)!=PRODUCT_ID)
      return FAILED;//���ID��ȡʧ�ܣ��򷵻�ʧ��
    //��̨���ݲ�������128HZ ��������32��
    SPL06_pressure_rate_config(PRESSURE_RATE_128_TIMES,PRESSURE_RATE_64_TIMES);
    //��̨���ݲ�������32HZ ��������8��//���ô������ϵ��¶ȼ���Ϊ�¶Ȳɼ�
    SPL06_temperature_rate_config(TEMPERATURE_RATE_32_TIMES,TEMPERATURE_RATE_8_TIMES,TEMPERATURE_RATE_TMP_EXT_EXTERNAL);
    //������̨��ȡ����
    SPL06_set_measure_mode(MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP);
    return SUCCESS;//��ʼ���ɹ�
}


/***********************************************************************
 * ��ȡԭʼ�¶�ֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void SPL06_001_get_raw_temp(void)
{
    uint8_t h[3] = {0};
    
    h[0] = SPL06_001_read(SPL06_ADDR, 0x03);
    h[1] = SPL06_001_read(SPL06_ADDR, 0x04);
    h[2] = SPL06_001_read(SPL06_ADDR, 0x05);

    SPL06.i32rawTemperature = (int32_t)h[0]<<16 | (int32_t)h[1]<<8 | (int32_t)h[2];
    SPL06.i32rawTemperature= (SPL06.i32rawTemperature&0x800000) ? (0xFF000000|SPL06.i32rawTemperature) : SPL06.i32rawTemperature;
}

/***********************************************************************
 * ��ȡԭʼ��ѹֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void SPL06_001_get_raw_pressure(void)
{
    uint8_t h[3];
    
    h[0] = SPL06_001_read(SPL06_ADDR, 0x00);
    h[1] = SPL06_001_read(SPL06_ADDR, 0x01);
    h[2] = SPL06_001_read(SPL06_ADDR, 0x02);
    
    SPL06.i32rawPressure = (int32_t)h[0]<<16 | (int32_t)h[1]<<8 | (int32_t)h[2];
    SPL06.i32rawPressure= (SPL06.i32rawPressure&0x800000) ? (0xFF000000|SPL06.i32rawPressure) : SPL06.i32rawPressure;
}


/***********************************************************************
 * �¶Ƚ���ֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
float SPL06_001_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = SPL06.i32rawTemperature / (float)SPL06.i32kT;
    fTCompensate =  SPL06_calib_param.c0 * 0.5 + SPL06_calib_param.c1 * fTsc;
    return fTCompensate;
}

/***********************************************************************
 * ��ѹ���㲢�����¶Ȳ���
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
float SPL06_001_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = SPL06.i32rawTemperature / (float)SPL06.i32kT;
    fPsc = SPL06.i32rawPressure / (float)SPL06.i32kP;
    qua2 = SPL06_calib_param.c10 + fPsc * (SPL06_calib_param.c20 + fPsc* SPL06_calib_param.c30);
    qua3 = fTsc * fPsc * (SPL06_calib_param.c11 + fPsc * SPL06_calib_param.c21);
    //qua3 = 0.9f *fTsc * fPsc * (SPL06_calib_param.c11 + fPsc * SPL06_calib_param.c21);
  
    fPCompensate = SPL06_calib_param.c00 + fPsc * qua2 + fTsc * SPL06_calib_param.c01 + qua3;
    //fPCompensate = SPL06_calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * SPL06_calib_param.c01 + qua3;
    return fPCompensate;
}



/***********************************************************************
 * ��ȡ�¶�ֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
float user_SPL06_001_get_temperature()
{
    SPL06_001_get_raw_temp();//��ȡ�¶�ԭʼֵ
    return SPL06_001_get_temperature();//�¶Ƚ�����ֵ
}
/***********************************************************************
 * ��ȡ��ѹ���¶Ȳ���ֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
float user_SPL06_001_get_presure()
{
    SPL06_001_get_raw_pressure();//��ȡ��ѹֵԭʼֵ
    return SPL06_001_get_pressure();  //��ѹ���㲢�����¶Ȳ��������ѹֵ
}

#endif
