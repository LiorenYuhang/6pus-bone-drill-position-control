# ATK-MW8266D WiFi Module Driver for STM32H743VIT6

基于 HAL 库的 ATK-MW8266D (ESP8266) WiFi 模块驱动，使用 USART2 + AT 指令集。

## 硬件连接

| STM32H743VIT6     | ATK-MW8266D V1.4  |
|-------------------|--------------------|
| PD5  (USART2_TX)  | RXD (模块串口接收)  |
| PD6  (USART2_RX)  | TXD (模块串口发送)  |
| PD7  (GPIO输出)    | RST (模块复位,低有效) |
| 3.3V              | VCC                |
| GND               | GND                |

> **注意**: IO 电平已匹配 — 模块 VCC 接 3.3V 时 IO 为 3.3V，可直接连接 STM32H743。

## 文件结构

```
Communication/WIFI/
├── README.md        # 本文件
├── wifi_uart.h      # UART 接口层头文件 (引脚定义, API声明)
├── wifi_uart.c      # UART 接口层实现 (初始化, printf, 中断接收)
├── wifi.h           # AT 命令驱动头文件 (API声明, 返回码)
└── wifi.c           # AT 命令驱动实现 (初始化, WiFi连接, TCP透传等)
```

## 在项目中集成

### 1. 添加文件到工程

将 `wifi_uart.c` 和 `wifi.c` 添加到 Makefile/IDE 的源文件列表中。

### 2. 添加中断处理函数 (stm32h7xx_it.c)

```c
#include "wifi_uart.h"

void USART2_IRQHandler(void)
{
    wifi_uart_irq_handler();
}
```

### 3. 基本使用流程

```c
#include "wifi.h"
#include <stdio.h>

/* WiFi 网络配置 */
#define WIFI_SSID   "你的WiFi名"
#define WIFI_PWD    "你的WiFi密码"
#define TCP_SERVER_IP   "192.168.1.100"
#define TCP_SERVER_PORT "8080"

void wifi_demo(void)
{
    uint8_t ret;
    char ip[16];

    /* 1. 初始化模块 (硬件复位 + AT测试) */
    ret = wifi_init(115200);
    if (ret != WIFI_EOK)
    {
        printf("WiFi init failed!\r\n");
        return;
    }
    printf("WiFi module initialized.\r\n");

    /* 2. 连接 WiFi 热点 */
    ret  = wifi_restore();                      /* 恢复出厂设置 */
    ret += wifi_set_mode(WIFI_MODE_STATION);    /* Station 模式 */
    ret += wifi_sw_reset();                     /* 软复位 */
    ret += wifi_echo_config(WIFI_ECHO_OFF);     /* 关闭回显 */
    ret += wifi_join_ap(WIFI_SSID, WIFI_PWD);   /* 连接 WiFi */
    ret += wifi_get_ip(ip);                     /* 获取 IP */
    if (ret != WIFI_EOK)
    {
        printf("Failed to join AP!\r\n");
        return;
    }
    printf("Connected. IP: %s\r\n", ip);

    /* 3. 连接 TCP 服务器 */
    ret = wifi_connect_tcp_server(TCP_SERVER_IP, TCP_SERVER_PORT);
    if (ret != WIFI_EOK)
    {
        printf("TCP connect failed!\r\n");
        return;
    }
    printf("TCP connected.\r\n");

    /* 4. 进入透传模式 */
    ret = wifi_enter_transparent_mode();
    if (ret != WIFI_EOK)
    {
        printf("Enter transparent mode failed!\r\n");
        return;
    }

    /* 5. 现在可以直接发送数据到 TCP 服务器 */
    wifi_uart_printf("Hello from STM32H743!\r\n");

    /* 6. 接收来自 TCP 服务器的数据 */
    wifi_uart_rx_restart();
    while (1)
    {
        uint8_t *rx = wifi_uart_rx_get_frame();
        if (rx != NULL)
        {
            printf("Received: %s\r\n", rx);
            wifi_uart_rx_restart();    /* 重新开始捕获下一帧 */
        }
        HAL_Delay(10);
    }
}
```

## AT 命令 API 速查

| 函数                        | AT 指令            | 说明               |
|-----------------------------|--------------------|--------------------|
| `wifi_init(baudrate)`       | —                  | 硬件初始化 + AT 测试|
| `wifi_at_test()`            | AT                 | 通信测试 (重试10次) |
| `wifi_restore()`            | AT+RESTORE         | 恢复出厂设置        |
| `wifi_set_mode(mode)`       | AT+CWMODE          | 设置 WiFi 模式     |
| `wifi_sw_reset()`           | AT+RST             | 软复位模块          |
| `wifi_echo_config(cfg)`     | ATE0/ATE1          | 开关 AT 回显       |
| `wifi_join_ap(ssid, pwd)`   | AT+CWJAP           | 连接 WiFi 热点     |
| `wifi_get_ip(buf)`          | AT+CIFSR           | 获取 IP 地址       |
| `wifi_connect_tcp_server()` | AT+CIPSTART        | 连接 TCP 服务器    |
| `wifi_close_connection()`   | AT+CIPCLOSE        | 关闭 TCP 连接      |
| `wifi_enter_transparent_mode()` | AT+CIPMODE+AT+CIPSEND | 进入透传模式   |
| `wifi_exit_transparent_mode()`  | +++                | 退出透传模式        |
| `wifi_connect_atk_cloud()`  | AT+ATKCLDSTA       | 连接原子云          |
| `wifi_disconnect_atk_cloud()`| AT+ATKCLDCLS      | 断开原子云连接      |

## 透传模式注意事项

1. 退出透传时需先等待 ≥1 秒无数据收发，再发送 `+++`，再等 ≥1 秒。
2. 透传模式下每帧最大 2048 字节 (ESP8266 限制)。
3. 退出透传后模块回到 AT 命令模式。

## 修改引脚

编辑 `wifi_uart.h` 中的宏定义即可更换 UART 和 RST 引脚:

```c
#define WIFI_UART_TX_PORT   GPIOD   /* 改为你的 TX 端口 */
#define WIFI_UART_TX_PIN    GPIO_PIN_5
#define WIFI_UART_TX_AF     GPIO_AF7_USART2

#define WIFI_UART_RX_PORT   GPIOD
#define WIFI_UART_RX_PIN    GPIO_PIN_6
#define WIFI_UART_RX_AF     GPIO_AF7_USART2

#define WIFI_UART_INSTANCE  USART2
#define WIFI_UART_IRQn      USART2_IRQn
#define WIFI_UART_IRQHandler USART2_IRQHandler

#define WIFI_RST_PORT       GPIOD
#define WIFI_RST_PIN        GPIO_PIN_7
```

记得同步修改 `wifi_uart.c` 中的 `wifi_uart_clk_enable()` 函数（对应的 `__HAL_RCC_xxx_CLK_ENABLE()`）。

## 参考资料

- ATK-MW8266D 模块用户手册 V1.3 (正点原子)
- ATK-MW8266D 模块使用说明 V1.2 (正点原子)
- ESP8266 AT 指令集文档
- 正点原子论坛: http://openedv.com/thread-308397-1-1.html
