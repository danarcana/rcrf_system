/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart_cb.h"
#include "nb_bc.h"
#include "ring_buffer.h"
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
volatile uint8_t usart2_dr = 0;
//extern volatile uint8_t enable_tim;
extern __IO uint8_t      usart3_is_msg;
extern __IO uint8_t   usart3_rx_len;
extern __IO uint8_t usart3_buff_idx;

extern __IO uint32_t usart1_rx_buffer_idx;
extern __IO uint8_t usart_rx_dma_buffer[250];
extern __IO uint8_t usart1_idle_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
	systick_counter++;
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
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

    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART1) && LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1);        /* Clear IDLE line flag */
//        usart_rx_check();                       /* Check for data to process */
        __NOP();

        ringbuffer_Write(&nb_bc_rx_ring_buffer, nb_bc_start_of_msg, nb_bc_recv_len);
        nb_bc_start_of_msg = 0;
        usart1_idle_flag = 1;

//        LL_TIM_DisableCounter(TIMER_M95_EOF);
//        LL_TIM_SetCounter(TIMER_M95_EOF, 0);
//        LL_TIM_ClearFlag_UPDATE(TIMER_M95_EOF);

    }

    __IO uint8_t  c;

    if(LL_USART_IsActiveFlag_RXNE(NB_BC_UART))
    {
      c = LL_USART_ReceiveData8(NB_BC_UART);
//      LL_TIM_SetCounter(TIMER_M95_EOF, 0);

      if((nb_bc_recv_buf_p > NB_BC_UART_RX_MAX_ADDR))
      {
        nb_bc_recv_buf_p = NB_BC_UART_RX_BASE_ADDR;
      }

      if( 0 == nb_bc_start_of_msg )
      {

        nb_bc_start_of_msg = (uint32_t)nb_bc_recv_buf_p;
        nb_bc_recv_len = 0;
//        LL_TIM_EnableCounter(TIMER_M95_EOF);
      }

      if((0x0a != c) && (0x0d != c))
      {
        *nb_bc_recv_buf_p++ = c;
        nb_bc_recv_len++;
      }
//      		  usart_rx_dma_buffer[usart1_rx_buffer_idx]= LL_USART_ReceiveData8(USART1);
		  usart_rx_dma_buffer[usart1_rx_buffer_idx]= c;
      		  usart1_rx_buffer_idx ++;
    }

//	  if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
//	  {
//	    /* RXNE flag will be cleared by reading of DR register (done in call) */
//	    /* Call function in charge of handling Character reception */
//		  usart_rx_dma_buffer[usart1_rx_buffer_idx]= LL_USART_ReceiveData8(USART1);
//		  usart1_rx_buffer_idx ++;
//	  }
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	 /* Check RXNE flag value in SR register */
	  if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
	  {
	    /* RXNE flag will be cleared by reading of DR register (done in call) */
	    /* Call function in charge of handling Character reception */
//	    USART_CharReception_Callback();
		  usart2_dr = LL_USART_ReceiveData8(USART2);
		  LL_USART_TransmitData8(USART3, usart2_dr);
	  }

	  if(LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2))
	  {
	    /* TXE flag will be automatically cleared when writing new data in DR register */

	    /* Call function in charge of handling empty DR => will lead to transmission of next character */
//	    USART_TXEmpty_Callback();
	    LL_USART_ClearFlag_TC(USART2);
//	    USART_ITConfig (USART1, USART_IT_TXE, DISABLE);
//	    LL_USART_EnableIT_TXE(USART2);
	  }

	  if(LL_USART_IsEnabledIT_TC(USART2) && LL_USART_IsActiveFlag_TC(USART2))
	  {
	    /* Clear TC flag */
		  USART_TXEmpty_Callback();
	    LL_USART_ClearFlag_TC(USART2);
	    /* Call function in charge of handling end of transmission of sent character
	       and prepare next character transmission */
//	    USART_CharTransmitComplete_Callback();
	  }

	  if(LL_USART_IsEnabledIT_ERROR(USART2) && LL_USART_IsActiveFlag_NE(USART2))
	  {
	    /* Call Error function */
//	    USART_TransferError_Callback();
	  }

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

	  if(LL_USART_IsActiveFlag_IDLE(USART3))
	  {
	    LL_USART_ClearFlag_IDLE(USART3);
	    if (usart3_buff_idx > 0)
	    {
	    	usart3_is_msg = 1;
	    }
	  }

	 /* Check RXNE flag value in SR register */
	  if(LL_USART_IsActiveFlag_RXNE(USART3) && LL_USART_IsEnabledIT_RXNE(USART3))
	  {
	    /* RXNE flag will be cleared by reading of DR register (done in call) */
	    /* Call function in charge of handling Character reception */
	    USART_CharReception_Callback();
//	    HAL_TIM_Base_Stop_IT(&htim14);
//	    TIM14->CNT = 0;
	    // Start timer
//	    HAL_TIM_Base_Start_IT(&htim14);
//	    HAL_TIM_Base_MspInit(&htim14);
//	    enable_tim =1;
	  }

	  if(LL_USART_IsEnabledIT_TXE(USART3) && LL_USART_IsActiveFlag_TXE(USART3))
	  {
	    /* TXE flag will be automatically cleared when writing new data in DR register */

	    /* Call function in charge of handling empty DR => will lead to transmission of next character */
//	    USART_TXEmpty_Callback();
	  }

	  if(LL_USART_IsEnabledIT_TC(USART3) && LL_USART_IsActiveFlag_TC(USART3))
	  {
	    /* Clear TC flag */
	    LL_USART_ClearFlag_TC(USART3);
	    /* Call function in charge of handling end of transmission of sent character
	       and prepare next character transmission */
//	    USART_CharTransmitComplete_Callback();
	  }

	  if(LL_USART_IsEnabledIT_ERROR(USART3) && LL_USART_IsActiveFlag_NE(USART3))
	  {
	    /* Call Error function */
//	    USART_TransferError_Callback();
	  }


  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
