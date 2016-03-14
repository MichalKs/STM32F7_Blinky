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
 * @addtogroup USART2
 * @{
 */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

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
#define USARTx_TX_AF                     GPIO_AF8_USART6 //???
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_AF                     GPIO_AF8_USART6 //???

void    (*rxCallback)(uint8_t);   ///< Callback function for receiving data
uint8_t (*txCallback)(uint8_t*);  ///< Callback function for transmitting data

UART_HandleTypeDef UartHandle;

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();

  /* Select SysClk as source of USART1 clocks */
  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  RCC_PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
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
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
//PUTCHAR_PROTOTYPE
//{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

//  return ch;
//}

/**
 * @brief Initialize USART2
 * @param baud
 * @param rxCb
 * @param txCb
 */
void UART6_Init(uint32_t baud, void(*rxCb)(uint8_t), uint8_t(*txCb)(uint8_t*) ) {

  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit) : BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

//  // assign the callbacks
//  rxCallback = rxCb;
//  txCallback = txCb;
//
//  GPIO_InitTypeDef  GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//
//  // Enable clocks for peripherals
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);
//
//  // USART2 TX on PA2
//  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//  // USART2 RX on PA3
//  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//  // Connect USART2 TX pin to AF2
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
//
//  // USART initialization (standard 8n1)
//  USART_InitStructure.USART_BaudRate            = baud;
//  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
//  USART_InitStructure.USART_Parity              = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
//  USART_Init(USART2, &USART_InitStructure);
//
//  // Enable USART2
//  USART_Cmd(USART2, ENABLE);
//
//  // Enable RXNE interrupt
//  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//  // Disable TXE interrupt - we enable it only when there is
//  // data to send
//  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
//
//  // Enable USART2 global interrupt
//  NVIC_EnableIRQ(USART2_IRQn);

}
/**
 * @brief Enable transmitter.
 * @details This function has to be called by the higher layer
 * in order to start the transmitter.
 */
void UART6_TxEnable(void) {
//  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

/**
 * @brief IRQ handler for USART2
 */
void USART6_IRQHandler(void) {
//
//  // If transmit buffer empty interrupt
//  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
//
//    uint8_t c;
//
//    if (txCallback) { // if not NULL
//      // get data from higher layer using callback
//      if (txCallback(&c)) {
//        USART_SendData(USART2, c); // Send data
//      } else { // if no more data to send disable the transmitter
//        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
//      }
//    }
//  }
//
//  // If RX buffer not empty interrupt
//  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
//
//    uint8_t c = USART_ReceiveData(USART2); // Get data from UART
//
//    if (rxCallback) { // if not NULL
//      rxCallback(c); // send received data to higher layer
//    }
//  }
}

/**
 * @}
 */
