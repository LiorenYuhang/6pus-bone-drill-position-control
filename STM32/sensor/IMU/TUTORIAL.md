# STM32H743 IMU 项目教程

本教程从零开始讲解如何使用 STM32H743VIT6 读取 WHEELTEC H30 惯导模块（IMU）数据。

## 目录

1. [开发环境准备](#1-开发环境准备)
2. [项目结构解析](#2-项目结构解析)
3. [H30 IMU 模块介绍](#3-h30-imu-模块介绍)
4. [代码详解](#4-代码详解)
5. [编译与烧录](#5-编译与烧录)
6. [调试与验证](#6-调试与验证)
7. [扩展开发](#7-扩展开发)

---

## 1. 开发环境准备

### 1.1 必备工具

本项目的开发环境已在当前系统中配置完毕，以下是已安装的工具：

| 工具 | 用途 | 版本/路径 |
|------|------|----------|
| arm-none-eabi-gcc | ARM Cortex-M 交叉编译器 | 13.2.1 |
| make | 构建自动化 | 系统自带 |
| STM32CubeMX | STM32 外设配置工具 | `~/STM32CubeMX/` |
| STM32Cube_FW_H7 | H7 系列 HAL 固件库 | `~/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/` |
| STM32 VS Code Extension | IDE 集成 | 已安装 |

### 1.2 固件库说明

STM32Cube_FW_H7 固件包包含：

- **HAL 驱动**: `Drivers/STM32H7xx_HAL_Driver/` — 硬件抽象层
- **CMSIS**: `Drivers/CMSIS/` — ARM Cortex-M 软件接口标准
- **启动文件**: `Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/` — GCC 汇编启动代码
- **示例项目**: `Projects/` — 官方板卡示例

---

## 2. 项目结构解析

```text
IMU/
├── Makefile                    # GNU Make 构建脚本
├── STM32H743VITx_FLASH.ld     # 链接脚本（内存布局）
├── startup/
│   └── startup_stm32h743xx.s  # 启动文件（向量表、Reset_Handler）
└── Core/
    ├── Inc/                    # 头文件
    └── Src/                    # 源文件
```

### 2.1 启动流程

MCU 上电后的启动顺序：

```
Reset → startup_stm32h743xx.s:
  1. 设置栈指针 (SP = _estack)
  2. 调用 ExitRun0Mode() — 配置供电
  3. 调用 SystemInit() — 配置 FPU、Cache、电源
  4. 复制 .data 段（Flash → RAM）
  5. 清零 .bss 段
  6. 调用 __libc_init_array() — 运行静态构造
  7. 调用 main()
```

### 2.2 main() 执行流程

```
main():
  CPU_CACHE_Enable()          # 启用 I-Cache + D-Cache
  HAL_Init()                  # HAL 库初始化（SysTick 1ms）
  SystemClock_Config()        # 配置时钟 400MHz
  MX_GPIO_Init()              # GPIO 初始化（IMU INT 引脚）
  MX_I2C1_Init()              # I2C1 初始化（400kHz）
  MX_USART1_UART_Init()       # USART1 初始化（115200）
  IMU_Init()                  # IMU 数据结构清零
  while(1):
    IMU_ReadAll(&g_imu)       # 读取全部 IMU 数据
    printf(...)                # 串口输出
    HAL_Delay(100)             # 等待 100ms（10Hz）
```

### 2.3 链接脚本

链接脚本 `STM32H743VITx_FLASH.ld` 定义了 MCU 的内存布局：

| 内存区域 | 起始地址 | 大小 | 用途 |
|---------|---------|------|------|
| ITCMRAM | 0x00000000 | 64 KB | 紧耦合指令 RAM |
| RAM (DTCM) | 0x20000000 | 128 KB | 数据 RAM |
| ROM (Flash) | 0x08000000 | 2048 KB | 程序存储 |

---

## 3. H30 IMU 模块介绍

### 3.1 模块概述

WHEELTEC H30 是一款集成了 IMU（惯性测量单元）和 GNSS 的惯导模块。核心芯片为 YIS106。

### 3.2 通信接口

H30 支持两种通信方式：

| 接口 | 特点 | 优势 |
|------|------|------|
| **I2C**（本工程使用）| 地址 0x6A，400 kHz | 简单，直接从寄存器读取解析好的数据 |
| **UART** | 460800 baud，帧协议 | 可获取 GPS 位置、速度、UTC 时间等更多数据 |

### 3.3 I2C 数据寄存器

I2C 通信更简单——直接从寄存器读取 32 位有符号整数，乘以转换因子即可得到实际物理量。

| 地址 | 名称 | 数据 | 转换因子 |
|------|------|------|----------|
| 0x10 | 加速度 | Pitch/Roll/Yaw int32 × 3 | × 10⁻⁶ = m/s² |
| 0x20 | 角速度 | X/Y/Z int32 × 3 | × 10⁻⁶ = dps |
| 0x30 | 磁力计（归一化）| X/Y/Z int32 × 3 | × 10⁻⁶ |
| 0x40 | 欧拉角 | Pitch/Roll/Yaw int32 × 3 | × 10⁻⁶ = deg |
| 0x41 | 四元数 | W/X/Y/Z int32 × 4 | × 10⁻⁶ |

### 3.4 电气连接

```
H30 IMU (4-pin)          STM32H743VIT6
┌───────────┐           ┌──────────────┐
│ VCC (3.3V)├───────────┤ 3.3V         │
│ GND       ├───────────┤ GND          │
│ SCL       ├───────────┤ PB6 (I2C1)   │
│ SDA       ├───────────┤ PB7 (I2C1)   │
│ INT       ├───────────┤ PA12 (GPIO)  │
└───────────┘           └──────────────┘
```

---

## 4. 代码详解

### 4.1 IMU 驱动 (`imu.c` / `imu.h`)

核心数据结构：

```c
typedef struct {
    float x, y, z;           // 三轴数据
} imu_axis_data_t;

typedef struct {
    float pitch, roll, yaw;  // 欧拉角 (deg)
    float q0, q1, q2, q3;   // 四元数
} imu_attitude_t;

typedef struct {
    imu_axis_data_t accel;   // 加速度 (m/s²)
    imu_axis_data_t gyro;    // 角速度 (dps)
    imu_axis_data_t mag_norm;// 磁力计归一化值
    imu_axis_data_t mag_raw; // 磁力计原始值 (mGauss)
    imu_attitude_t attitude; // 姿态数据
} imu_data_t;
```

关键函数：

- `IMU_ReadEuler()` — 读取欧拉角
- `IMU_ReadAccel()` — 读取加速度
- `IMU_ReadGyro()` — 读取角速度
- `IMU_ReadQuaternion()` — 读取四元数
- `IMU_ReadAll()` — 一次性读取全部数据

内部实现以 `IMU_ReadEuler()` 为例：

```c
int IMU_ReadEuler(imu_data_t *data)
{
    uint8_t buf[12];

    // 1. 通过 I2C 从寄存器 0x40 读取 12 字节
    if (IMU_ReadReg(IMU_REG_EULER, buf, 12) != HAL_OK)
        return -1;

    // 2. 将 4 字节 little-endian 转换为 int32，再乘以因子得到 float
    data->attitude.pitch = buf_to_int32(buf)      * 1e-6f;
    data->attitude.roll  = buf_to_int32(buf + 4)  * 1e-6f;
    data->attitude.yaw   = buf_to_int32(buf + 8)  * 1e-6f;

    return 0;
}
```

### 4.2 I2C 配置 (`i2c.c`)

关键配置参数：

| 参数 | 值 | 说明 |
|------|------|------|
| 时钟频率 | 400 kHz | Fast Mode |
| 寻址模式 | 7-bit | 标准 I2C 地址 |
| Timing | 0x00B03FDB | 100MHz PCLK1 → 400kHz SCL |
| 引脚 | PB6 (SCL), PB7 (SDA) | AF4 (I2C1) |

### 4.3 USART 配置 (`usart.c`)

| 参数 | 值 | 说明 |
|------|------|------|
| 波特率 | 115200 | 标准调试波特率 |
| 数据位 | 8 | 8N1 格式 |
| 停止位 | 1 | |
| 校验 | 无 | |
| 引脚 | PA9 (TX), PA10 (RX) | AF7 (USART1) |

**printf 重定向**：通过实现 `_write()` 函数将 printf 输出重定向到 USART1。

```c
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
```

### 4.4 HAL MSP 配置 (`stm32h7xx_hal_msp.c`)

MSP (MCU Support Package) 函数由 HAL 库在初始化外设时自动调用，负责：

1. **使能外设时钟**（`__HAL_RCC_I2C1_CLK_ENABLE()`）
2. **配置 GPIO 引脚复用**（PB6/PB7 → AF4_I2C1, PA9/PA10 → AF7_USART1）
3. **配置外设时钟源**（I2C1 ← D2PCLK1, USART1 ← D2PCLK2）

### 4.5 中断处理 (`stm32h7xx_it.c`)

- **SysTick_Handler**：每 1ms 触发一次，调用 `HAL_IncTick()`
- **EXTI15_10_IRQHandler**：IMU 数据就绪中断（PA12 下降沿触发），设置 `imu_data_ready` 标志
- **I2C1_EV/ER_IRQHandler**：I2C 事件/错误中断，转发给 HAL 库处理

---

## 5. 编译与烧录

### 5.1 编译

```bash
cd ~/bone_drill/STM32/sensor/IMU
make clean
make
```

编译步骤：
1. 编译所有 Core/Src/*.c 源文件
2. 编译 HAL 驱动文件（stm32h7xx_hal_*.c）
3. 汇编启动文件（startup_stm32h743xx.s）
4. 链接生成 ELF → 生成 HEX 和 BIN

### 5.2 理解 Makefile

关键变量：

```makefile
# MCU 架构标志
MCFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard

# 预定义宏
DEFS = -DSTM32H743xx -DUSE_HAL_DRIVER

# 链接脚本
LDFLAGS = -T./STM32H743VITx_FLASH.ld
```

### 5.3 烧录

烧录前确保 ST-Link 或其他调试器已连接。

```bash
# 使用 STM32CubeProgrammer CLI
make flash

# 或使用 OpenOCD
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/IMU.bin 0x08000000 verify reset"
```

### 5.4 VS Code 集成

如果安装了 STM32 VS Code Extension，可以直接用 VS Code 打开 `sensor/IMU/` 目录进行开发，利用：
- 代码补全和跳转
- 寄存器查看（Debug 时）
- 图形化调试

---

## 6. 调试与验证

### 6.1 串口输出验证

连接 USB 转串口模块后，打开终端：

```bash
# 串口参数: 115200, 8N1
screen /dev/ttyUSB0 115200
```

预期输出：

```text
========================================
  WHEELTEC H30 IMU Reader
  MCU: STM32H743VIT6 @ 400 MHz
  I2C addr: 0x6A
========================================

=== IMU Data @ 1234 ms ===
  Euler:  Pitch=    0.00 deg  Roll=    0.00 deg  Yaw=    0.00 deg
  Accel:  X=    0.00 m/s2  Y=    0.00 m/s2  Z=    9.81 m/s2
  Gyro:   X=    0.0 dps   Y=    0.0 dps   Z=    0.0 dps
  Quat:   W= 1.0000  X= 0.0000  Y= 0.0000  Z= 0.0000
```

- 静止时 Z 轴加速度应接近 9.81 m/s²（重力加速度）
- 转动模块时欧拉角应相应变化
- 如果显示 `[ERROR] I2C read failed`，检查接线和供电

### 6.2 常见问题排查

| 现象 | 可能原因 | 解决方案 |
|------|---------|----------|
| 无串口输出 | USART 引脚错误 | 确认 PA9 连接到 USB-TTL 的 RX |
| 串口乱码 | 波特率不匹配 | 确认终端设为 115200, 8N1 |
| I2C 读取失败 | 接线错误 | 检查 SCL→PB6, SDA→PB7 |
| I2C 读取失败 | 供电不足 | H30 需 3.3V 稳定供电 |
| I2C 读取失败 | I2C 地址错误 | 确认 H30 测量地址为 0x6A |
| 读取数据全为 0 | IMU 未初始化完成 | 上电后等待 1-2 秒 |
| 程序卡死 | 未焊接 HSE 晶振 | 修改 SystemClock_Config 使用 HSI |

### 6.3 GDB 调试

如果安装了 `arm-none-eabi-gdb`，可以通过 OpenOCD 进行在线调试：

```bash
# 终端1：启动 OpenOCD
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg

# 终端2：启动 GDB
arm-none-eabi-gdb build/IMU.elf
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) continue
```

---

## 7. 扩展开发

### 7.1 添加 UART 通信模式

H30 也支持 UART 通信（460800 baud），可以获取 GPS 位置、速度、UTC 时间等更多数据。

实现步骤：
1. 在 `stm32h7xx_hal_msp.c` 中配置另一个 UART（如 UART4）
2. 移植 `analysis_data.c/h` 解析协议
3. 实现 UART 数据的帧同步和 CRC 校验

### 7.2 提高数据输出速率

当前为 10 Hz，可将 `HAL_Delay(100)` 改为更短的延迟，但注意：
- I2C 400kHz 下每帧通信约需 ~300μs
- 如果使用 INT 中断驱动（`imu_data_ready` 标志），可实现更高的实时性

### 7.3 添加 OLED 显示

参考官方 I2C 例程中的 OLED 显示代码，可以在本地添加 OLED 模块直接显示 IMU 数据。

### 7.4 数据记录到 SD 卡

利用 STM32H7 的 SDMMC 接口，将 IMU 数据实时写入 SD 卡，用于离线分析。

### 7.5 ROS2 集成

通过串口将 IMU 数据发送到 ROS2 节点（参见 inspire_motor_control 项目的 `ros2_jazzy/`）。

---

## 参考资料

- [WHEELTEC H30 产品资料目录](../../docs/WHEELTEC_H30惯导产品资料/)
  - 用户手册
  - YIS 通讯协议文档
  - I2C/串口例程
  - 上位机工具
- [STM32H743VI 数据手册](https://www.st.com/resource/en/datasheet/stm32h743vi.pdf)
- [STM32H7 参考手册](https://www.st.com/resource/en/reference_manual/rm0433-stm32h743i-eval.pdf)
- [ARM Cortex-M7 技术参考手册](https://developer.arm.com/documentation/ddi0489/)
