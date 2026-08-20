# IMU 惯导模块项目

基于 STM32H743VIT6 的 WHEELTEC H30 惯导模块（IMU）数据读取项目。

## 功能

通过 I2C 接口读取 H30 IMU 的欧拉角、加速度、角速度、四元数、磁力计数据，通过 USART1 输出。

## 开发环境

- **MCU**: STM32H743VIT6 (Cortex-M7, 400MHz, 2MB Flash, 128KB RAM)
- **编译器**: arm-none-eabi-gcc 13.2.1
- **HAL库**: STM32Cube_FW_H7_V1.13.0

## 快速开始

```bash
cd ~/bone_drill/STM32/sensor/IMU
make clean
make          # 编译
make flash    # 烧录（需 STM32CubeProgrammer 或 OpenOCD）
```

## 硬件引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| I2C1_SCL | PB6 | I2C总线时钟 |
| I2C1_SDA | PB7 | I2C总线数据 |
| USART1_TX | PA9 | 调试串口输出 |
| USART1_RX | PA10 | 调试串口输入 |
| IMU_INT | PA12 | IMU数据就绪中断（可选） |

## 目录结构

```text
sensor/
└── IMU/
    ├── README.md             # 本文件
    ├── TUTORIAL.md           # 详细教程
    ├── Makefile
    ├── STM32H743VITx_FLASH.ld
    ├── startup/
    │   └── startup_stm32h743xx.s
    └── Core/
        ├── Inc/              # 头文件
        └── Src/              # 源文件
```
