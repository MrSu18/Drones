#ifndef __LED_H
#define __LED_H

#include "main.h"
//#include "GPIO.H"

//机身前灯   LED1右前灯，LED2左前灯    
#define LED1_H()       HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_L()       HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_Toggle()  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)

#define LED2_H()       HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_L()       HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_Toggle()  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)

//-------------------------------------------------
//机身后灯  LED3左后灯，LED4右后灯     
#define LED3_H()       HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_L()       HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED3_Toggle()  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)

#define LED4_H()       HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET)
#define LED4_L()       HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET)
#define LED4_Toggle()  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin)



typedef struct
{
  uint16_t FlashTime;
  enum
  {
    AlwaysOn, 
    AlwaysOff, 
    AllFlashLight,
    AlternateFlash, 
    WARNING,
    DANGEROURS,
    GET_OFFSET  
  }status; 
}sLED;  

extern sLED LED;
extern void LEDInit(void);
extern void LEDtest(void);
extern void PilotLED(void);

#define LED_TAKE_OFF_ENTER      LED.status = WARNING  
#define LED_TAKE_OFF_EXIT       LED.status = AllFlashLight  
#define LED_HEIGHT_LOCK_ENTER   LED.FlashTime=50;LED.status = AlternateFlash
#define LED_HEIGHT_LOCK_EXIT    LED.FlashTime=100;LED.status = AllFlashLight
#define LED_3D_ROLL_ENTER        LED.status = WARNING 
#define LED_3D_ROLL_EXIT        LED.status = AllFlashLight  
#define LED_SAFTY_TAKE_DOWN_ENTER  LED.status = DANGEROURS 
#define LED_SAFTY_TAKE_DOWN_EXIT   LED.status = AlwaysOn
#define LED_GET_MPU_OFFSET_ENTER   LED.status = GET_OFFSET
#define LED_GO_HOME_ENTER       LED.status = WARNING    
#define LED_GO_HOME_EXIT        LED.status = AllFlashLight  

#endif 


