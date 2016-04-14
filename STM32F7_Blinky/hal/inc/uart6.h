/**
 * @file    uart2.h
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

#ifndef UART_H_
#define UART_H_

#include <inttypes.h>
#include <stm32f7xx_hal.h>

/**
 * @defgroup  USART2 USART2
 * @brief     USART2 low level functions
 */

/**
 * @addtogroup USART2
 * @{
 */

void    UART6_Init(uint32_t baud, void(*rxCb)(uint8_t), int(*txCb)(uint8_t*));
void    UART6_TxEnable(void);
int     HAL_UART_IsSendingData(void);
void    HAL_UART_SendData(void);

// HAL functions for use in higher level
#define COMM_HAL_Init         UART6_Init
#define COMM_HAL_TxEnable()   UART6_TxEnable
#define COMM_HAL_IrqEnable()  HAL_NVIC_EnableIRQ(USART6_IRQn);
#define COMM_HAL_IrqDisable() HAL_NVIC_DisableIRQ(USART6_IRQn);

/**
 * @}
 */

#endif /* UART_H_ */
