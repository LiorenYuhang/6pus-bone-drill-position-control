/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   GPIO configuration.
  *          PA12 - IMU INT pin (external interrupt, falling edge)
  ******************************************************************************
  */

#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H */
