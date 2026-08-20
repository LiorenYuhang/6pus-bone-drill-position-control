/**
  ******************************************************************************
  * @file    usart.h
  * @brief   USART1 configuration for printf / debug output.
  *          PA9 - TX, PA10 - RX, 115200 baud, 8N1.
  ******************************************************************************
  */

#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
