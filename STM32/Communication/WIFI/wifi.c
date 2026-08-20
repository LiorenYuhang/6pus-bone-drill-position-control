/**
 ******************************************************************************
 * @file    wifi.c
 * @brief   ATK-MW8266D WiFi module AT-command driver implementation.
 *
 *          All functions are blocking — they send an AT command, then
 *          poll the UART RX frame buffer for the expected response.
 *          The UART layer uses IDLE-line detection to mark frame
 *          boundaries, so a "frame" here is the complete reply to a
 *          single AT command.
 *
 *          Target: STM32H743VIT6, HAL library, USART2.
 ******************************************************************************
 */

#include "wifi.h"
#include "wifi_uart.h"
#include <stdio.h>
#include <string.h>

/* ======================= Local Helpers ===================================== */

/**
 * @brief  Pull RST low then high to reset the module via hardware.
 */
void wifi_hw_reset(void)
{
    HAL_GPIO_WritePin(WIFI_RST_PORT, WIFI_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(WIFI_RST_PORT, WIFI_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(500);
}

/* ===================== Core: send AT command =============================== */

/**
 * @brief  Send an AT command and (optionally) wait for a response.
 *
 *         Implementation notes:
 *          - The command is sent with trailing "\r\n".
 *          - If ack is NULL or timeout_ms is 0, the function returns
 *            immediately after sending (fire-and-forget).
 *          - Otherwise it polls every 1 ms until the expected ack
 *            substring appears in the received frame, or the timeout
 *            expires.
 *          - When a frame is received that does NOT contain the ack,
 *            the RX buffer is restarted and the wait continues — this
 *            correctly handles intermediate echo / status lines.
 */
uint8_t wifi_send_at_cmd(const char *cmd, const char *ack, uint32_t timeout_ms)
{
    uint8_t *rx = NULL;

    /* Restart RX before sending so we capture the response cleanly */
    wifi_uart_rx_restart();

    /* Send command */
    wifi_uart_printf("%s\r\n", cmd);

    /* Fire-and-forget mode */
    if ((ack == NULL) || (timeout_ms == 0))
    {
        return WIFI_EOK;
    }

    /* Poll for expected response */
    while (timeout_ms > 0)
    {
        rx = wifi_uart_rx_get_frame();
        if (rx != NULL)
        {
            if (strstr((const char *)rx, ack) != NULL)
            {
                return WIFI_EOK;
            }
            /* Frame received but not what we wanted; restart for next */
            wifi_uart_rx_restart();
        }

        HAL_Delay(1);
        timeout_ms--;
    }

    return WIFI_ETIMEOUT;
}

/* ======================== Init / Test ====================================== */

/**
 * @brief  Fully initialize the WiFi module.
 *
 *         Sequence:
 *           1. Hardware reset
 *           2. UART init at requested baud rate
 *           3. AT test — try up to 10 times
 */
uint8_t wifi_init(uint32_t baudrate)
{
    uint8_t i;

    wifi_hw_reset();
    wifi_uart_init(baudrate);

    /* AT test with retries — module may need a moment after boot */
    for (i = 0; i < 10; i++)
    {
        if (wifi_at_test() == WIFI_EOK)
        {
            return WIFI_EOK;
        }
    }

    return WIFI_ERROR;
}

/**
 * @brief  Basic AT communication check.
 *
 *         Sends "AT" and waits for "OK".  Retries up to 10 times
 *         internally; the calling wifi_init() may also retry at a
 *         higher level.
 */
uint8_t wifi_at_test(void)
{
    uint8_t ret;
    uint8_t i;

    for (i = 0; i < 10; i++)
    {
        ret = wifi_send_at_cmd("AT", "OK", 500);
        if (ret == WIFI_EOK)
        {
            return WIFI_EOK;
        }
    }

    return WIFI_ERROR;
}

/* ======================== Configuration ==================================== */

/**
 * @brief  Restore factory defaults.  Module will reboot afterward.
 */
uint8_t wifi_restore(void)
{
    uint8_t ret;

    ret = wifi_send_at_cmd("AT+RESTORE", "ready", 3000);
    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}

/**
 * @brief  Set WiFi mode (Station / AP / AP+Station).
 */
uint8_t wifi_set_mode(uint8_t mode)
{
    uint8_t ret;
    char cmd[16];

    if (mode < 1 || mode > 3)  { return WIFI_EINVAL; }

    snprintf(cmd, sizeof(cmd), "AT+CWMODE=%u", mode);
    ret = wifi_send_at_cmd(cmd, "OK", 500);

    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}

/**
 * @brief  Software reset via AT+RST.  Waits 1 s for reboot.
 */
uint8_t wifi_sw_reset(void)
{
    uint8_t ret;

    ret = wifi_send_at_cmd("AT+RST", "OK", 500);
    if (ret == WIFI_EOK)
    {
        HAL_Delay(1000);
        return WIFI_EOK;
    }

    return WIFI_ERROR;
}

/**
 * @brief  Enable or disable AT command echo.
 */
uint8_t wifi_echo_config(uint8_t cfg)
{
    uint8_t ret;

    switch (cfg)
    {
        case WIFI_ECHO_OFF:
            ret = wifi_send_at_cmd("ATE0", "OK", 500);
            break;
        case WIFI_ECHO_ON:
            ret = wifi_send_at_cmd("ATE1", "OK", 500);
            break;
        default:
            return WIFI_EINVAL;
    }

    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}

/* ======================== WiFi Network ===================================== */

/**
 * @brief  Join an access point.  Waits up to 10 s for DHCP.
 */
uint8_t wifi_join_ap(const char *ssid, const char *pwd)
{
    uint8_t ret;
    char cmd[128];

    /* AT+CWJAP="ssid","password" */
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, pwd);
    ret = wifi_send_at_cmd(cmd, "WIFI GOT IP", 10000);

    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}

/**
 * @brief  Query the assigned IP address via AT+CIFSR.
 *
 *         The response format is:
 *           +CIFSR:STAIP,"192.168.1.x"
 *           +CIFSR:STAMAC,"xx:xx:xx:xx:xx:xx"
 *
 *         We parse the first quoted string (the IP) and copy it into
 *         the caller-provided buffer (≥ 16 bytes).
 */
uint8_t wifi_get_ip(char *ip_buf)
{
    uint8_t ret;
    char *p_start;
    char *p_end;
    uint8_t *rx;

    ret = wifi_send_at_cmd("AT+CIFSR", "OK", 500);
    if (ret != WIFI_EOK)
    {
        return WIFI_ERROR;
    }

    rx = wifi_uart_rx_get_frame();
    if (rx == NULL)
    {
        return WIFI_ERROR;
    }

    /* Find the first quoted string: the STA IP address */
    p_start = strchr((const char *)rx, '\"');
    if (p_start == NULL) { return WIFI_ERROR; }

    p_end = strchr(p_start + 1, '\"');
    if (p_end == NULL) { return WIFI_ERROR; }

    *p_end = '\0';
    snprintf(ip_buf, 16, "%s", p_start + 1);

    return WIFI_EOK;
}

/* ======================== TCP Client ======================================= */

/**
 * @brief  Establish a TCP connection to a remote server.
 *
 *         AT+CIPSTART="TCP","<ip>",<port>
 *         Waits up to 5 s for "CONNECT".
 */
uint8_t wifi_connect_tcp_server(const char *server_ip, const char *server_port)
{
    uint8_t ret;
    char cmd[128];

    snprintf(cmd, sizeof(cmd),
             "AT+CIPSTART=\"TCP\",\"%s\",%s",
             server_ip, server_port);

    ret = wifi_send_at_cmd(cmd, "CONNECT", 5000);

    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}

/**
 * @brief  Close the active TCP/UDP/SSL connection.
 */
uint8_t wifi_close_connection(void)
{
    uint8_t ret;

    ret = wifi_send_at_cmd("AT+CIPCLOSE", "OK", 500);

    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}

/* ================== Transparent Mode ======================================= */

/**
 * @brief  Enter transparent transmission mode.
 *
 *         Sends:
 *           AT+CIPMODE=1   — set transparent mode
 *           AT+CIPSEND      — initiate send (expects ">")
 */
uint8_t wifi_enter_transparent_mode(void)
{
    uint8_t ret;

    ret  = wifi_send_at_cmd("AT+CIPMODE=1", "OK", 500);
    ret += wifi_send_at_cmd("AT+CIPSEND",   ">",  500);

    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}

/**
 * @brief  Exit transparent mode by sending the "+++" escape sequence.
 *
 *         IMPORTANT: According to the ESP8266 AT spec, there must be
 *         no data traffic for at least 1 s before and after "+++".
 *         The caller must enforce this; we only send the bytes.
 */
void wifi_exit_transparent_mode(void)
{
    /* "+++" without trailing CR/LF — this is the escape sequence */
    wifi_uart_printf("+++");
}

/* ======================= ATK Cloud ========================================= */

/**
 * @brief  Connect to the ATK Cloud (原子云) IoT platform.
 *
 *         AT+ATKCLDSTA="<device_id>","<password>"
 *         Waits up to 10 s for "CLOUD CONNECTED".
 */
uint8_t wifi_connect_atk_cloud(const char *device_id, const char *device_pwd)
{
    uint8_t ret;
    char cmd[128];

    snprintf(cmd, sizeof(cmd),
             "AT+ATKCLDSTA=\"%s\",\"%s\"",
             device_id, device_pwd);

    ret = wifi_send_at_cmd(cmd, "CLOUD CONNECTED", 10000);

    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}

/**
 * @brief  Disconnect from the ATK Cloud platform.
 */
uint8_t wifi_disconnect_atk_cloud(void)
{
    uint8_t ret;

    ret = wifi_send_at_cmd("AT+ATKCLDCLS", "CLOUD DISCONNECT", 500);

    return (ret == WIFI_EOK) ? WIFI_EOK : WIFI_ERROR;
}
