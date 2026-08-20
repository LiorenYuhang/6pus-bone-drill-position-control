/**
 ******************************************************************************
 * @file    wifi_uart.h
 * @brief   ATK-MW8266D WiFi module UART interface layer.
 *
 *          Provides UART initialization, printf-style transmit, and
 *          interrupt-driven frame receive with IDLE-line detection.
 *
 *          Default pin mapping (USART2, AF7):
 *            PD5 — TX   (USART2_TX)
 *            PD6 — RX   (USART2_RX)
 *            PD7 — RST  (GPIO output, active-low)
 *
 *          STM32H743VIT6  |  ATK-MW8266D V1.4
 *          ----------------+---------------------
 *          PD5  (TX)       →  RXD (module UART RX)
 *          PD6  (RX)       →  TXD (module UART TX)
 *          PD7  (RST)      →  RST (module reset)
 *          3.3V             →  VCC
 *          GND              →  GND
 *
 *          ATK-MW8266D factory defaults: 115200 baud, 8N1, no flow control.
 ******************************************************************************
 */

#ifndef __WIFI_UART_H
#define __WIFI_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdarg.h>

/* ========================== Pin Configuration ============================== */

/* ----- USART2: module communication ----- */
#define WIFI_UART_TX_PORT               GPIOD
#define WIFI_UART_TX_PIN                GPIO_PIN_5
#define WIFI_UART_TX_AF                 GPIO_AF7_USART2

#define WIFI_UART_RX_PORT               GPIOD
#define WIFI_UART_RX_PIN                GPIO_PIN_6
#define WIFI_UART_RX_AF                 GPIO_AF7_USART2

#define WIFI_UART_INSTANCE              USART2
#define WIFI_UART_IRQn                  USART2_IRQn
#define WIFI_UART_IRQHandler            USART2_IRQHandler

/* ----- GPIO: hardware reset (active-low) ----- */
#define WIFI_RST_PORT                   GPIOD
#define WIFI_RST_PIN                    GPIO_PIN_7

/* =========================== Buffer Sizes ================================== */

#define WIFI_UART_RX_BUF_SIZE           128
#define WIFI_UART_TX_BUF_SIZE           256

/* =========================== Public API ==================================== */

/**
 * @brief  Initialize the WiFi module UART with given baud rate.
 *         Calls HAL_UART_Init() which in turn invokes HAL_UART_MspInit()
 *         to configure GPIO pins and clocks.
 *
 * @param  baudrate : communication speed (typically 115200).
 */
void wifi_uart_init(uint32_t baudrate);

/**
 * @brief  Transmit a formatted string to the WiFi module (like printf).
 * @param  fmt : format string.
 * @param  ... : variadic arguments.
 */
void wifi_uart_printf(const char *fmt, ...);

/**
 * @brief  Restart / clear the RX frame buffer for a new capture.
 */
void wifi_uart_rx_restart(void);

/**
 * @brief  Get a pointer to the most recently completed RX frame.
 *         A "frame" is complete when the UART IDLE-line flag fires
 *         (gap between bytes >= 1 character time).
 *
 * @return Pointer to null-terminated frame buffer, or NULL if no
 *         complete frame has been received since the last restart.
 */
uint8_t *wifi_uart_rx_get_frame(void);

/**
 * @brief  Get the length of the most recently completed RX frame.
 * @return Length in bytes, or 0 if no frame available.
 */
uint16_t wifi_uart_rx_get_frame_len(void);

/**
 * @brief  De-initialize the WiFi UART peripheral.
 */
void wifi_uart_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_UART_H */
