/**
 ******************************************************************************
 * @file    wifi_uart.c
 * @brief   ATK-MW8266D UART interface — initialization, TX, interrupt RX
 *          with IDLE-line frame detection.
 *
 *          USART2, PD5=TX, PD6=RX, PD7=RST.
 *          Target: STM32H743VIT6 @ 400 MHz, HAL library.
 ******************************************************************************
 */

#include "wifi_uart.h"
#include <stdio.h>
#include <string.h>

/* ========================== Static Variables =============================== */

static UART_HandleTypeDef g_wifi_uart = {0};

/* Frame receive buffer and status */
static struct
{
    uint8_t buf[WIFI_UART_RX_BUF_SIZE];   /* ring / linear buffer           */
    struct
    {
        uint16_t len   : 15;              /* received byte count [14:0]     */
        uint16_t finsh : 1;               /* frame-complete flag [15]       */
    } sta;
} g_rx_frame = {0};

/* Transmit buffer — large enough for printf formatting */
static char g_tx_buf[WIFI_UART_TX_BUF_SIZE];

/* ====================== Hardware Pin / Clock Helpers ======================= */

/**
 * @brief  Enable clocks for the WiFi UART and GPIO ports.
 */
static void wifi_uart_clk_enable(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
}

/**
 * @brief  Configure GPIO pins for USART2: PD5=TX(AF7), PD6=RX(AF7).
 */
static void wifi_uart_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* TX pin: PD5, Alternate Function Push-Pull, AF7 */
    gpio.Pin       = WIFI_UART_TX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = WIFI_UART_TX_AF;
    HAL_GPIO_Init(WIFI_UART_TX_PORT, &gpio);

    /* RX pin: PD6, Alternate Function Push-Pull, AF7 */
    gpio.Pin       = WIFI_UART_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = WIFI_UART_RX_AF;
    HAL_GPIO_Init(WIFI_UART_RX_PORT, &gpio);

    /* RST pin: PD7, push-pull output, default HIGH (module not reset) */
    gpio.Pin   = WIFI_RST_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WIFI_RST_PORT, &gpio);
    HAL_GPIO_WritePin(WIFI_RST_PORT, WIFI_RST_PIN, GPIO_PIN_SET);
}

/* ======================== Public API ======================================= */

/**
 * @brief  Initialize the WiFi UART peripheral with the given baud rate.
 *
 *         Configures GPIO pins, clock, NVIC, and starts the UART in
 *         interrupt-receive mode.  Users should call this once at boot
 *         after the system clock is set up.
 */
void wifi_uart_init(uint32_t baudrate)
{
    /* --- GPIO and clock --- */
    wifi_uart_clk_enable();
    wifi_uart_gpio_init();

    /* --- UART handle --- */
    g_wifi_uart.Instance          = WIFI_UART_INSTANCE;
    g_wifi_uart.Init.BaudRate     = baudrate;
    g_wifi_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    g_wifi_uart.Init.StopBits     = UART_STOPBITS_1;
    g_wifi_uart.Init.Parity       = UART_PARITY_NONE;
    g_wifi_uart.Init.Mode         = UART_MODE_TX_RX;
    g_wifi_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    g_wifi_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&g_wifi_uart);

    /* --- Enable UART interrupts --- */
    __HAL_UART_ENABLE_IT(&g_wifi_uart, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&g_wifi_uart, UART_IT_IDLE);

    HAL_NVIC_SetPriority(WIFI_UART_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(WIFI_UART_IRQn);
}

/**
 * @brief  De-initialize the WiFi UART (disable clock for power saving).
 */
void wifi_uart_deinit(void)
{
    HAL_NVIC_DisableIRQ(WIFI_UART_IRQn);
    HAL_UART_DeInit(&g_wifi_uart);
}

/**
 * @brief  Send a formatted string to the WiFi module.
 *
 *         Uses vsnprintf into an internal buffer then transmits via
 *         blocking HAL_UART_Transmit.  Suitable for AT command use;
 *         not recommended for high-throughput data streaming.
 */
void wifi_uart_printf(const char *fmt, ...)
{
    va_list ap;
    int len;

    va_start(ap, fmt);
    len = vsnprintf(g_tx_buf, WIFI_UART_TX_BUF_SIZE, fmt, ap);
    va_end(ap);

    if (len > 0)
    {
        HAL_UART_Transmit(&g_wifi_uart, (uint8_t *)g_tx_buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}

/**
 * @brief  Clear the RX frame buffer so the next IDLE detection
 *         captures a fresh response.
 */
void wifi_uart_rx_restart(void)
{
    g_rx_frame.sta.len   = 0;
    g_rx_frame.sta.finsh = 0;
}

/**
 * @brief  Return the last complete RX frame.
 *
 *         A frame is "complete" when the UART IDLE flag fires,
 *         indicating a gap ≥ 1 character time since the last byte.
 *
 * @return Pointer to null-terminated frame, or NULL.
 */
uint8_t *wifi_uart_rx_get_frame(void)
{
    if (g_rx_frame.sta.finsh == 1)
    {
        g_rx_frame.buf[g_rx_frame.sta.len] = '\0';
        return g_rx_frame.buf;
    }
    return NULL;
}

/**
 * @brief  Return the length of the last complete RX frame.
 */
uint16_t wifi_uart_rx_get_frame_len(void)
{
    if (g_rx_frame.sta.finsh == 1)
    {
        return g_rx_frame.sta.len;
    }
    return 0;
}

/* ====================== UART IRQ Handler =================================== */

/**
 * @brief  USART2 Interrupt Service Routine.
 *
 *         Handles three events:
 *           1. ORE (overrun error)    — cleared gracefully.
 *           2. RXNE (byte received)   — pushed into RX buffer.
 *           3. IDLE (line idle)       — marks the frame complete.
 *
 *         The caller must place wifi_uart_irq_handler() inside the
 *         project's USART2_IRQHandler() in stm32h7xx_it.c.
 */
void wifi_uart_irq_handler(void)
{
    uint8_t ch;

    /* --- Overrun error: clear flag and drain --- */
    if (__HAL_UART_GET_FLAG(&g_wifi_uart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&g_wifi_uart);
        (void)g_wifi_uart.Instance->ISR;
        (void)g_wifi_uart.Instance->RDR;
    }

    /* --- Byte received --- */
    if (__HAL_UART_GET_FLAG(&g_wifi_uart, UART_FLAG_RXNE) != RESET)
    {
        /* Read RDR directly — non-blocking, ISR-safe */
        ch = (uint8_t)(g_wifi_uart.Instance->RDR & 0xFFU);

        if (g_rx_frame.sta.len < (WIFI_UART_RX_BUF_SIZE - 1))
        {
            /* Room for one more char + trailing '\0' */
            g_rx_frame.buf[g_rx_frame.sta.len] = ch;
            g_rx_frame.sta.len++;
        }
        else
        {
            /* Overflow: wrap to start — discard previous data */
            g_rx_frame.sta.len = 0;
            g_rx_frame.buf[g_rx_frame.sta.len] = ch;
            g_rx_frame.sta.len++;
        }
    }

    /* --- IDLE line: frame complete --- */
    if (__HAL_UART_GET_FLAG(&g_wifi_uart, UART_FLAG_IDLE) != RESET)
    {
        g_rx_frame.sta.finsh = 1;
        __HAL_UART_CLEAR_IDLEFLAG(&g_wifi_uart);
    }
}

/* ==================== Integration Notes ====================================
 *
 * This driver initializes USART2 GPIO and clocks *before* calling
 * HAL_UART_Init(), so it is self-contained and does NOT require
 * changes to the project's stm32h7xx_hal_msp.c.
 *
 * However, two additions to the project framework are still needed:
 *
 *   1. IRQ handler (in stm32h7xx_it.c):
 *
 *        void USART2_IRQHandler(void)
 *        {
 *            wifi_uart_irq_handler();
 *        }
 *
 *   2. If you prefer centralized MSP init, move the pin/clock setup
 *      from wifi_uart_clk_enable() / wifi_uart_gpio_init() into your
 *      existing HAL_UART_MspInit() in stm32h7xx_hal_msp.c, keyed on
 *      huart->Instance == USART2.  Then remove the explicit calls
 *      from wifi_uart_init().
 *
 * ========================================================================= */
