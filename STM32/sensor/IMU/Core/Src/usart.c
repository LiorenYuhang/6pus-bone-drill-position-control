/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART1 initialization for STM32H743VIT6.
  *          PA9 - TX (AF7), PA10 - RX (AF7), 115200 baud, 8N1.
  ******************************************************************************
  */

#include "usart.h"
#include "main.h"

UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  Retarget printf() to USART1 via HAL_UART_Transmit.
  *         Called automatically by newlib-nano syscalls.
  */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* UART Extended callbacks (weak, required for linking) */
void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void HAL_UARTEx_TxFifoEmptyCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void HAL_UARTEx_RxFifoFullCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}
