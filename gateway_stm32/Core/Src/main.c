/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc11xx.h"
#include "nb_bc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#define USART3_RX_BUFFER_SIZE      250

extern __IO uint8_t   usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
extern __IO uint8_t   usart3_rx_len;
extern __IO uint8_t   usart3_is_msg;
extern __IO uint8_t   usart3_buff_idx;

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/**
 * \brief           USART RX buffer for DMA to transfer every received byte
 * \note            Contains raw data that are about to be processed by different events
 */
__IO uint8_t usart_rx_dma_buffer[250];
__IO uint32_t usart1_rx_buffer_idx = 0;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//#define F_CPU 16000000UL
//#ifndef F_CPU
//#warning "F_CPU not defined, using 2MHz by default"
//#define F_CPU 2000000UL
//#endif
//
////#include <stdint.h>
//void delay_ms(uint32_t ms) {
//    uint32_t i;
//    for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
//        _asm("nop");
//    }
//}

void usart_transmit_data(const void* data, uint16_t len) {
    const uint8_t* b = data;
    while (len--) {
        LL_USART_TransmitData8(USART2, *b++);

        while (!LL_USART_IsActiveFlag_TXE(USART2)){
          uint32_t transmit_empty_register=LL_USART_IsActiveFlag_TXE(USART2);
          uint32_t  transfer_complete=LL_USART_IsActiveFlag_TC(USART2);
          //LL_USART_ClearFlag_PE(USART1);
        }
        }
 }

//void dma_configure_nb(void)
//{
//  /* TX */
//  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_7,
//                         (uint32_t)m95_send_buf,
//                         LL_USART_DMA_GetRegAddr(M95_UART),
//                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//
//  /* tx channel ***************************/
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
//
//  /* Enable DMA TX Interrupt */
//  LL_USART_EnableDMAReq_TX(M95_UART);
//}



/**
 * \brief           Check if DMA is active and if not try to send data
 * \return          `1` if transfer just started, `0` if on-going or no data to transmit
 */
uint8_t
usart_start_tx_dma_transfer(void) {
    uint32_t primask;
    uint8_t started = 0;

    /*
     * First check if transfer is currently in-active,
     * by examining the value of usart_tx_dma_current_len variable.
     *
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     *
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call,
     *         indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active
     *         and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access to this function w/o
     * exclusive access protection (mutex) configured,
     * or if application calls this function from multiple interrupts.
     *
     * This example assumes worst use case scenario,
     * hence interrupts are disabled prior every check
     */
//    primask = __get_PRIMASK();
//    __disable_irq();
//    if (usart_tx_dma_current_len == 0
//            && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_rb)) > 0) {
        /* Disable channel if enabled */
//        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

        /* Clear all flags */
        LL_DMA_ClearFlag_TC3(DMA1);
        LL_DMA_ClearFlag_HT3(DMA1);
        LL_DMA_ClearFlag_GI3(DMA1);
        LL_DMA_ClearFlag_TE3(DMA1);

        /* Prepare DMA data and length */
//        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, usart_tx_dma_current_len);
//        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_rb));

        /* Start transfer */
//        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
        started = 1;
//    }
//    __set_PRIMASK(primask);
    return started;
}


void dma_configure_nb(void)
{
	/* TX */
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
	                       (uint32_t)nb_bc_send_buf,
	                       LL_USART_DMA_GetRegAddr(NB_BC_UART),
	                       LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  /* TX */
//  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_7,
//                         (uint32_t)m95_send_buf,
//                         LL_USART_DMA_GetRegAddr(M95_UART),
//                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  /* tx channel ***************************/
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);

//	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)usart_rx_dma_buffer);
//	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ARRAY_LEN(usart_rx_dma_buffer));

    /* Enable HT & TC interrupts */
//    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
//    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
	    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

    /* DMA1_Channel5_IRQn interrupt configuration */
//    NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
//    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	  NVIC_EnableIRQ(DMA1_Channel4_IRQn);


//    LL_USART_EnableDMAReq_RX(USART1);
	  LL_USART_EnableDMAReq_TX(USART1);


	  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

    /* USART interrupt */
//    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
//    NVIC_EnableIRQ(USART1_IRQn);


  /* Enable DMA TX Interrupt */
//  LL_USART_EnableDMAReq_TX(M95_UART);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_Init1msTick(48000000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  TIM14->CNT = 0;
//  htim14.Instance->CR1 &= (~((uint32_t)TIM_CR1_CEN));
//  dma_configure_nb();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  // Start timer
//  HAL_TIM_Base_Start_IT(&htim14);

  HAL_Delay(30);
  uint8_t Tx_data_Usart2[10] = {0};
  Tx_data_Usart2[0] = 'I';
  Tx_data_Usart2[1] = 'N';
  Tx_data_Usart2[2] = 'I';
  Tx_data_Usart2[3] = 'T';
  usart_transmit_data(Tx_data_Usart2, 4);

  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_RXNE(USART3);
  LL_USART_EnableIT_IDLE(USART3);
  LL_USART_EnableIT_TC(USART2);

  rc_hw_reset();
  rc_exit_config();
  HAL_Delay(300);
  rc_enter_config();

  rc_read_config();
  rc_configure();
  usart3_is_msg = 0;
  usart3_buff_idx =0;
  LL_SYSTICK_EnableIT();
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_IDLE(USART1);
//  LL_USART_EnableIT_TC(USART1);
  nb_bc_reboot();

//  rc_factory_reset();
  while(1)
  {
	  HAL_Delay(250);
	  //forward USART3 receive data on USART2
	  if (usart3_is_msg==1)
	  {
		  usart3_rx_len = usart3_buff_idx;
		  usart3_buff_idx = 0;
		  LL_USART_TransmitData8(USART2, usart3_rx_buffer[usart3_buff_idx]);
		  usart3_buff_idx ++;
		  usart3_is_msg = 0;
	  }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_2;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  /**USART3 GPIO Configuration
  PC10   ------> USART3_TX
  PC11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  LL_GPIO_AF_RemapPartial_USART3();

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

//  /* Set DMA transfer addresses of source and destination */
//  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, LL_USART_DMA_GetRegAddr(USART3),
//                         (uint32_t)&usart3_rx_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//
//  /* Set DMA transfer size */
//  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, USART3_RX_BUFFER_SIZE);
//
//  /* Enable DMA transfer interruption: transfer complete */
//  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_3);
//  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
//
//  /* Enable DMA transfer interruption: transfer error */
//  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
//
//  /*## Activation of DMA #####################################################*/
//  /* Enable the DMA transfer */
//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
//
//  LL_USART_EnableDMAReq_RX(USART3);

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6|OUT_NB_STAT_Pin|OUT_NB_RST_Pin|OUT_NB_PWRKEY_Pin);

  /**/
  LL_GPIO_SetOutputPin(OUT_NB_PSM_EINT_GPIO_Port, OUT_NB_PSM_EINT_Pin);

  /**/
  LL_GPIO_SetOutputPin(RC11XX_CFG_PIN_GPIO_Port, RC11XX_CFG_PIN_Pin);

  /**/
  LL_GPIO_SetOutputPin(RC11XX_RESET_PIN_GPIO_Port, RC11XX_RESET_PIN_Pin);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|OUT_NB_STAT_Pin|OUT_NB_RST_Pin|OUT_NB_PWRKEY_Pin
                          |OUT_NB_PSM_EINT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RC11XX_CFG_PIN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(RC11XX_CFG_PIN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RC11XX_RESET_PIN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(RC11XX_RESET_PIN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
