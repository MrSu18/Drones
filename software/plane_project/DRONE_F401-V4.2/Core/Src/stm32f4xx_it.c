/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "init.h"
#include "ALL_DATA.H"
#include "imu.h"
#include "control.h"
#include "nrf24L01.h"
#include "uart.h"
#include "remote.h"
#include "ano_dt.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern _st_Mpu MPU6050;   //MPU6050原始数据
//extern _st_Mag AK8975;
extern _st_AngE Angle;    //当前角度姿态值
extern _st_Remote Remote; //遥控通道值
uint16_t cnt_500ms=0;
extern uint8_t nrf2401_tx_flag;
extern uint8_t nrf2401_txbuf[];
extern uint8_t txbuf_pos;
uint8_t cnt=0;
//uint8_t uart1_st=0; //串口1状态,0空闲，1正在接收，2接收完成
uint32_t tick_start=0;

extern uint8_t uart1_rxbuf[100];
extern uint8_t uart1_buf_pos;
extern uint8_t uart1_flag;
extern uint32_t uart1_tick_start;

extern uint8_t uart2_rxbuf[100];
extern uint8_t uart2_buf_pos;
extern uint8_t uart2_flag;
extern uint32_t uart2_tick_start;
extern uint8_t b_updata_uret;//光流数据更新
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  if((tick_start>0)&&(HAL_GetTick()-tick_start>10)) //超时,接收完成
  { 
    //uart1_st=2; 
    nrf2401_tx_flag=1;
    tick_start=0;
  }
  if((uart1_tick_start>0)&&(HAL_GetTick()-uart1_tick_start>1)) //uart1接收超时,接收完成
  { 
    uart1_flag=1;
    uart1_tick_start=0;
  }

  if((uart2_tick_start>0)&&(HAL_GetTick()-uart2_tick_start>1)) //uart2接收超时,接收完成
  { 
    uart2_flag=1;
    uart2_tick_start=0;
  }
  //*
  cnt_500ms++;

  if(cnt_500ms==500)
  { 
    cnt_500ms=0;
    //HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    /*
    send_str(&huart6,(uint8_t *)"pitch=");
    send_float(&huart6,Angle.pitch,2);
    send_str(&huart6,(uint8_t *)"  roll=");
    send_float(&huart6,Angle.roll,2);
    send_str(&huart6,(uint8_t *)"  yaw=");
    send_float(&huart6,Angle.yaw,2);
    send_str(&huart6,(uint8_t *)"\r\n");
  // */
  }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  uint8_t revdata;
  uint32_t isrflags = USART1->SR;//isrflags = huart1.Instance->SR;
  if((isrflags & USART_SR_RXNE) != 0) //如果是接收中断，说明串口1收到了一个字节的新数据
  { 
    revdata=USART1->DR;//revdata=huart1.Instance->DR;  读出新收到数据给revdata
    tick_start=HAL_GetTick(); //开始计时
    
    if(txbuf_pos<TX_PLOAD_WIDTH)
    { 
      nrf2401_txbuf[txbuf_pos++]=revdata;
      if(txbuf_pos==TX_PLOAD_WIDTH)
      { 
        nrf2401_tx_flag=1;
        tick_start=0;
      }
    }
  }  

  #if 0  //把下面CubeMX自动生成的函数HAL_UART_IRQHandler(&huart1)注释掉
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  #endif
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  uint32_t isrflags = USART2->SR;//isrflags = huart1.Instance->SR;
  if((isrflags & USART_SR_IDLE) != 0U)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    b_updata_uret=1;
		uart2_flag=1;
    HAL_UART_Receive_DMA(&huart2,(uint8_t *)uart2_rxbuf,9);
  } 

  #if 0  
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  #endif 
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(NRF24L01_IRQ_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
