#include "main.h"
#include "uart.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;
int fputc(int ch,FILE *f)
{
  send_char(&huart1,ch);
  return 0;
}

void send_char(UART_HandleTypeDef *huart,uint8_t c)  //���ڷ���һ���ֽ�����
{
  while((huart->Instance->SR & USART_SR_TXE)==0);//��ѯ�ϴ������Ƿ�����ɣ�û������ȴ��������������������ִ�С������������͵�����£���������ʡ��
  huart->Instance->DR=c;   //����c
} 

void send_char_array(UART_HandleTypeDef *huart,uint8_t ca[],uint8_t n) //���ڷ���n���ֽ�����
{ uint8_t i;
  for(i=0;i<n;i++)
    send_char(huart,ca[i]);
}

void send_str(UART_HandleTypeDef *huart,uint8_t *s)  //���ڷ����ַ�������0����
{ uint8_t *str=s;
  while(*str!=0)
    send_char(huart,*str++);
}

void send_int(UART_HandleTypeDef *huart,int32_t num)  //���ڷ���һ��������������Χ-2147483647��+2147483647
{ uint8_t str[12],i=0;
  str[11]=0;
  if(num<0)
  { num=-num;
    str[0]='-';
  }
  else
    str[0]=' ';
  
  do
  { str[10-i]=num%10+'0';
    num=num/10;
    i++;
  }while(num!=0);
  
  if(str[0]=='-')
    str[10-i++]='-';
  send_str(huart,str+11-i);
}
//*
void send_float(UART_HandleTypeDef *huart,float num,uint8_t dotnum)  //���ڷ���һ��������
{ uint8_t i;
  int32_t int_part;
  float fract_part;
  int_part=num;
  fract_part=num-int_part;
  
  send_int(huart,int_part);
  send_char(huart,'.');
  
  if(fract_part<0)
    fract_part=-fract_part;

  for(i=0;i<dotnum;i++)
  { fract_part=fract_part*10;
    if((fract_part<1)&&(i<dotnum-1))
      send_char(huart,'0');
  }
  send_int(huart,fract_part);
} //*/


