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
//#include "stm32f10x.h"
#include "stm32f4xx_hal.h"
#include "LED.h"
#include "ALL_DATA.h"
#include "main.h"
//-------------------------------------------------
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */
sLED LED = {300,AllFlashLight};  //LED initial statue is off;
                             //default 300ms flash the status
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/  
void PilotLED() //flash 300MS interval
{
  static uint32_t LastTime = 0;
  uint32_t SysTick_count=HAL_GetTick();

  if(SysTick_count - LastTime < LED.FlashTime)
  {
    return;
  }
  else
    LastTime = SysTick_count;
  switch(LED.status)
  {
    case AlwaysOff:      //常暗   
      LED1_H();
      LED2_H();
      LED3_H();
      LED4_H();
      break;
    case AllFlashLight:          //全部同时闪烁
      LED1_Toggle();    
      LED2_Toggle();
      LED3_Toggle();      
      LED4_Toggle();      
      break;
    case AlwaysOn:  //常亮
      
      LED1_L();
      LED2_L();
      LED3_L();
      LED4_L();
      break;
    case AlternateFlash: //交替闪烁
      LED1_H();
      LED2_H();
      LED3_L();
      LED4_L();
      LED.status = AllFlashLight;
      break;
    case WARNING:
      LED1_Toggle();
      LED2_Toggle();
      LED3_L();
      LED4_L();
      LED.FlashTime = 100;
      break;
    case DANGEROURS:
      LED1_L();
      LED2_L();
      LED3_Toggle();
      LED4_Toggle();
      LED.FlashTime = 70;
      break;
    default:
      LED.status = AlwaysOff;
      break;
  }
}

/**************************END OF FILE*********************************/



