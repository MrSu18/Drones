#include "usart.h"
#include "stdio.h"

//�����ض���
int fputc(int ch, FILE *f)
{
  uint8_t temp[1] = {ch};
  HAL_UART_Transmit(&huart1, temp, 1, 2);//huart1��Ҫ������������޸�
  return ch;
}
