/**
 * @file    uart2.c
 * @brief   Controlling USART2
 * @date    12 kwi 2014
 * @author  Michal Ksiezopolski
 * 
 * @verbatim
 * Copyright (c) 2014 Michal Ksiezopolski.
 * All rights reserved. This program and the 
 * accompanying materials are made available 
 * under the terms of the GNU Public License 
 * v3.0 which accompanies this distribution, 
 * and is available at 
 * http://www.gnu.org/licenses/gpl.html
 * @endverbatim
 */

#include <stm32f7xx_hal.h>
#include <uart6.h>

/**
 * @addtogroup UART6
 * @{
 */

/* Definition for USARTx clock resources */
#define USARTx                           USART6
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART6_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART6_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART6_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOC
#define USARTx_TX_AF                     GPIO_AF8_USART6
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's NVIC: used for receiving data over Rx pin */
#define USARTx_IRQn                      USART6_IRQn
#define USARTx_IRQHandler                USART6_IRQHandler

static void  (*rxCallback)(uint8_t);   ///< Callback function for receiving data
static int   (*txCallback)(uint8_t*);  ///< Callback function for transmitting data (fills up buffer with data to send)
static UART_HandleTypeDef uartHandle; ///< Handle for UART peripheral

static volatile int isSendingData; ///< Flag saying if UART is currently sending any data

/**
 * @brief Checks if UART is currently sending any data
 * @details If so the IRQ will automatically get new data from the FIFO. If not we
 * have to explicitly call HAL_UART_SendData to enable the TX IRQ.
 * @retval 1 UART is sending data
 * @retval 0 UART is not sending data
 */
int HAL_UART_IsSendingData(void) {
  return isSendingData;
}

/**
 * @brief Sends data using the UART IRQ
 * @details This function is called automatically when the TX IRQ is running.
 * However if the IRQ is not running this function has to be called manually to
 * enable the IRQ.
 */
void HAL_UART_SendData(void) {

  // has to be static to serve as a buffer for UART
  static uint8_t buf[2048];

  // if no function set do nothing
  if (txCallback == NULL) {
    return;
  }

  // get the data
  int ret = txCallback(buf);

  // if there is any data in the FIFO
  if (ret != 0) {
    // send it to PC
    HAL_UART_Transmit_IT(&uartHandle, buf, ret);
    isSendingData = 1;
  } else {
    isSendingData = 0;
  }

}

/**
 * @brief Transfer completed callback (called whenever IRQ sends the whole buffer)
 * @param huart UART handle
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_UART_SendData();
}

/**
 * @brief Initialize USART2
 * @param baud
 * @param rxCb
 * @param txCb
 */
void UART6_Init(uint32_t baud, void(*rxCb)(uint8_t), int(*txCb)(uint8_t*) ) {

  txCallback = txCb;

  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit) : BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  uartHandle.Instance        = USARTx;

  uartHandle.Init.BaudRate   = baud;
  uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  uartHandle.Init.StopBits   = UART_STOPBITS_1;
  uartHandle.Init.Parity     = UART_PARITY_NONE;
  uartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  uartHandle.Init.Mode       = UART_MODE_TX_RX;

  if (HAL_UART_Init(&uartHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef  GPIO_InitStruct;

  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();

  /* Select SysClk as source of USART1 clocks */
  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  RCC_PeriphClkInit.Usart6ClockSelection = RCC_USART6CLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  /* Enable USARTx clock */
  USARTx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC for UART ########################################*/
  HAL_NVIC_SetPriority(USARTx_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);

}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

  HAL_NVIC_DisableIRQ(USARTx_IRQn);
}

/**
 * @brief  This function handles UART interrupt request.
 */
void USARTx_IRQHandler(void) {
  HAL_UART_IRQHandler(&uartHandle);
}

/**
 * @}
 */
