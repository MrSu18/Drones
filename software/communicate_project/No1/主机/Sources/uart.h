#include "main.h"
void send_char(UART_HandleTypeDef *huart,uint8_t c); //���ڷ���һ���ֽ�����
void send_char_array(UART_HandleTypeDef *huart,uint8_t ch[],uint8_t n);//���ڷ���n���ֽ�����
void send_str(UART_HandleTypeDef *huart,uint8_t *s);  //���ڷ����ַ�������0����
void send_int(UART_HandleTypeDef *huart,int32_t num); //���ڷ���һ��������������Χ-2147483647��+2147483647
void send_float(UART_HandleTypeDef *huart,float num,uint8_t dotnum);  //���ڷ���һ��������
