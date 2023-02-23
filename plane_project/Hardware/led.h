#ifndef _SU_LED_H
#define _SU_LED_H

#include "main.h"
#include "gpio.h"

#define ON	0
#define OFF	1

#define LED(N) HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,N?OFF:ON);
#define LED_TOGGLE  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

#endif
