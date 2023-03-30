#include "main.h"
void send_char(UART_HandleTypeDef *huart,uint8_t c); //串口发送一个字节数据
void send_char_array(UART_HandleTypeDef *huart,uint8_t ch[],uint8_t n);//串口发送n个字节数据
void send_str(UART_HandleTypeDef *huart,uint8_t *s);  //串口发送字符串，遇0结束
void send_int(UART_HandleTypeDef *huart,int32_t num); //串口发送一个整数，整数范围-2147483647到+2147483647
void send_float(UART_HandleTypeDef *huart,float num,uint8_t dotnum);  //串口发送一个浮点数
