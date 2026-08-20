/**
 ******************************************************************************
 * @file    wifi.h
 * @brief   ATK-MW8266D WiFi module AT-command driver.
 *
 *          Provides a high-level API for controlling the ATK-MW8266D
 *          (ESP8266-based) module via AT commands over UART:
 *
 *            - Hardware / software reset
 *            - AT communication test
 *            - WiFi mode selection (Station / AP / AP+Station)
 *            - Join an access point
 *            - Query assigned IP address
 *            - TCP client connection
 *            - Transparent (passthrough) mode
 *            - ATK Cloud (原子云) connection
 *
 *          All functions are blocking and return a status code.
 *
 *          Dependencies:
 *            wifi_uart.h   — low-level UART TX/RX
 *            HAL_Delay()   — timing (via HAL_GetTick / SysTick)
 *            <string.h>
 *            <stdio.h>
 ******************************************************************************
 */

#ifndef __WIFI_H
#define __WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* =========================== Return Codes ================================== */

#define WIFI_EOK         0   /* success                            */
#define WIFI_ERROR       1   /* generic error                      */
#define WIFI_ETIMEOUT    2   /* timeout waiting for response       */
#define WIFI_EINVAL      3   /* invalid argument                   */
#define WIFI_ENOTINIT    4   /* module not initialized             */

/* =============================== Modes ===================================== */

#define WIFI_MODE_STATION       1   /* Station (client) mode        */
#define WIFI_MODE_AP            2   /* Soft-AP mode                 */
#define WIFI_MODE_AP_STATION    3   /* AP + Station combo           */

/* ========================== Echo Configuration ============================= */

#define WIFI_ECHO_OFF           0   /* ATE0 — disable echo          */
#define WIFI_ECHO_ON            1   /* ATE1 — enable echo           */

/* =========================== Public API ==================================== */

/**
 * @brief  Perform a full hardware reset by toggling the RST pin.
 *
 *         RST pulled LOW for 100 ms, then released for 500 ms to
 *         allow the module to boot and go through its startup
 *         sequence.
 */
void wifi_hw_reset(void);

/**
 * @brief  Send an AT command and wait for an expected response.
 *
 *         This is the core building block.  It transmits `cmd\r\n`,
 *         then polls the RX buffer for `ack` until `timeout_ms`
 *         elapses.
 *
 * @param  cmd        : AT command string (e.g. "AT").
 * @param  ack        : substring expected in the response (e.g. "OK").
 *                      Pass NULL to send-and-forget.
 * @param  timeout_ms : maximum wait time in milliseconds.
 *                      0 means send-only (no response checking).
 * @return WIFI_EOK on match, WIFI_ETIMEOUT otherwise.
 */
uint8_t wifi_send_at_cmd(const char *cmd, const char *ack, uint32_t timeout_ms);

/**
 * @brief  Initialize the WiFi module.
 *
 *         Sequence: hardware init → hard reset → UART init →
 *         AT test (retries up to 10 times).
 *
 * @param  baudrate : UART baud rate (115200 is factory default).
 * @return WIFI_EOK or WIFI_ERROR.
 */
uint8_t wifi_init(uint32_t baudrate);

/**
 * @brief  Restore the module to factory defaults (AT+RESTORE).
 * @return WIFI_EOK or WIFI_ERROR.
 */
uint8_t wifi_restore(void);

/**
 * @brief  Run a basic AT communication test (send "AT", expect "OK").
 *
 *         Retries up to 10 times.
 *
 * @return WIFI_EOK or WIFI_ERROR.
 */
uint8_t wifi_at_test(void);

/**
 * @brief  Set the WiFi operating mode.
 * @param  mode : WIFI_MODE_STATION, WIFI_MODE_AP, or WIFI_MODE_AP_STATION.
 * @return WIFI_EOK, WIFI_ERROR, or WIFI_EINVAL.
 */
uint8_t wifi_set_mode(uint8_t mode);

/**
 * @brief  Soft-reset the module (AT+RST) and wait for reboot.
 * @return WIFI_EOK or WIFI_ERROR.
 */
uint8_t wifi_sw_reset(void);

/**
 * @brief  Enable or disable AT command echo.
 * @param  cfg : WIFI_ECHO_OFF or WIFI_ECHO_ON.
 * @return WIFI_EOK, WIFI_ERROR, or WIFI_EINVAL.
 */
uint8_t wifi_echo_config(uint8_t cfg);

/**
 * @brief  Connect to an access point (WiFi network).
 *
 * @param  ssid : network name.
 * @param  pwd  : network password.
 * @return WIFI_EOK on "WIFI GOT IP", WIFI_ERROR otherwise.
 */
uint8_t wifi_join_ap(const char *ssid, const char *pwd);

/**
 * @brief  Retrieve the module's current IP address (AT+CIFSR).
 *
 * @param  ip_buf : output buffer, must be at least 16 bytes.
 * @return WIFI_EOK or WIFI_ERROR.
 */
uint8_t wifi_get_ip(char *ip_buf);

/**
 * @brief  Establish a TCP connection to a remote server.
 *
 * @param  server_ip   : IPv4 address string (e.g. "192.168.1.100").
 * @param  server_port : port string (e.g. "8080").
 * @return WIFI_EOK on "CONNECT", WIFI_ERROR otherwise.
 */
uint8_t wifi_connect_tcp_server(const char *server_ip, const char *server_port);

/**
 * @brief  Enter transparent (passthrough) transmission mode.
 *
 *         After this call, raw data sent via wifi_uart_printf()
 *         will be forwarded directly to the remote TCP peer.
 *
 * @return WIFI_EOK or WIFI_ERROR.
 */
uint8_t wifi_enter_transparent_mode(void);

/**
 * @brief  Exit transparent transmission mode.
 *
 *         Sends the "+++" escape sequence to return to AT command mode.
 */
void wifi_exit_transparent_mode(void);

/**
 * @brief  Close the current TCP/UDP/SSL connection (AT+CIPCLOSE).
 * @return WIFI_EOK or WIFI_ERROR.
 */
uint8_t wifi_close_connection(void);

/**
 * @brief  Connect to the ATK Cloud (原子云) IoT platform.
 *
 * @param  device_id  : device identifier registered on the cloud.
 * @param  device_pwd : device password.
 * @return WIFI_EOK on "CLOUD CONNECTED", WIFI_ERROR otherwise.
 */
uint8_t wifi_connect_atk_cloud(const char *device_id, const char *device_pwd);

/**
 * @brief  Disconnect from the ATK Cloud platform.
 * @return WIFI_EOK or WIFI_ERROR.
 */
uint8_t wifi_disconnect_atk_cloud(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_H */
