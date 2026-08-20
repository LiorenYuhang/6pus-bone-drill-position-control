/**
  ******************************************************************************
  * @file    main.c
  * @brief   IMU Application: reads WHEELTEC H30 IMU data via I2C and
  *          prints readings over USART1 (printf) for display.
  *
  *          Hardware connections:
  *            IMU SCL -> PB6 (I2C1_SCL)
  *            IMU SDA -> PB7 (I2C1_SDA)
  *            IMU INT -> PA12 (optional data-ready interrupt)
  *            IMU VCC -> 3.3V
  *            IMU GND -> GND
  *
  *          USART1 TX (PA9) -> PC serial terminal @ 115200 baud
  ******************************************************************************
  */

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "imu.h"
#include <stdio.h>
#include <string.h>

volatile uint8_t imu_data_ready = 0;

/* Private function prototypes */
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

/**
  * @brief  Main program entry point.
  */
int main(void)
{
    /* Enable CPU caches for performance */
    CPU_CACHE_Enable();

    /* HAL Library initialization */
    HAL_Init();

    /* Configure system clock to 400 MHz */
    SystemClock_Config();

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    /* Initialize IMU data structure */
    IMU_Init();

    printf("\r\n========================================\r\n");
    printf("  WHEELTEC H30 IMU Reader\r\n");
    printf("  MCU: STM32H743VIT6 @ 400 MHz\r\n");
    printf("  I2C addr: 0x%02X\r\n", IMU_I2C_ADDR);
    printf("========================================\r\n\r\n");

    /* Main loop: read and print IMU data every 100 ms (10 Hz) */
    uint32_t tick = 0;
    uint32_t error_count = 0;

    while (1)
    {
        tick = HAL_GetTick();

        if (IMU_ReadAll(&g_imu) == 0)
        {
            printf("=== IMU Data @ %lu ms ===\r\n", tick);

            /* Euler angles */
            printf("  Euler:  Pitch=%8.2f deg  Roll=%8.2f deg  Yaw=%8.2f deg\r\n",
                   g_imu.attitude.pitch, g_imu.attitude.roll, g_imu.attitude.yaw);

            /* Accelerometer */
            printf("  Accel:  X=%8.2f m/s2  Y=%8.2f m/s2  Z=%8.2f m/s2\r\n",
                   g_imu.accel.x, g_imu.accel.y, g_imu.accel.z);

            /* Gyroscope */
            printf("  Gyro:   X=%8.1f dps   Y=%8.1f dps   Z=%8.1f dps\r\n",
                   g_imu.gyro.x, g_imu.gyro.y, g_imu.gyro.z);

            /* Quaternion */
            printf("  Quat:   W=%7.4f  X=%7.4f  Y=%7.4f  Z=%7.4f\r\n",
                   g_imu.attitude.q0, g_imu.attitude.q1,
                   g_imu.attitude.q2, g_imu.attitude.q3);

            printf("\r\n");
        }
        else
        {
            error_count++;
            printf("[ERROR] I2C read failed! (errors: %lu)\r\n", error_count);
        }

        /* Delay ~100ms for 10Hz output rate */
        HAL_Delay(100);
    }
}

/**
  * @brief  System Clock Configuration: 400 MHz CPU, 200 MHz buses.
  *         HSE = 25 MHz, PLLM = 5, PLLN = 160, PLLP = 2
  */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* Voltage scaling for max frequency */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /* Enable HSE oscillator and configure PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 160;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure bus clocks: CPU 400MHz, AXI/AHB 200MHz, APB 100MHz */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 |
                                   RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable I/O compensation cell for high-speed operation */
    __HAL_RCC_CSI_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_EnableCompensationCell();
}

/**
  * @brief  Enable CPU L1 Caches (I-Cache + D-Cache).
  */
static void CPU_CACHE_Enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}

/**
  * @brief  Called on unrecoverable error.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: %s:%lu\r\n", (char *)file, line);
    while (1)
    {
    }
}
#endif
