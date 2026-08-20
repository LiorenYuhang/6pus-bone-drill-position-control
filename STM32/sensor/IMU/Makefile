###############################################################################
# Makefile for STM32H743VIT6 IMU Project
# Reads WHEELTEC H30 IMU data via I2C, outputs via USART1
###############################################################################

# ----- Toolchain -----
PREFIX = arm-none-eabi-
CC      = $(PREFIX)gcc
CXX     = $(PREFIX)g++
AS      = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size
GDB     = $(PREFIX)gdb

# ----- Directories -----
PROJECT_DIR  = .
CORE_DIR     = $(PROJECT_DIR)/Core
FW_BASE      = $(HOME)/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0
HAL_DIR      = $(FW_BASE)/Drivers/STM32H7xx_HAL_Driver/Src
HAL_INC      = $(FW_BASE)/Drivers/STM32H7xx_HAL_Driver/Inc
CMSIS_DIR    = $(FW_BASE)/Drivers/CMSIS
DEVICE_DIR   = $(CMSIS_DIR)/Device/ST/STM32H7xx

# ----- Target -----
TARGET  = IMU
BUILD   = build

# ----- MCU Flags -----
MCU  = cortex-m7
FPU  = fpv5-sp-d16
FLOAT_ABI = hard
MCFLAGS = -mcpu=$(MCU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI)

# ----- Compiler Flags -----
DEFS  = -DSTM32H743xx -DUSE_HAL_DRIVER
INC   = -I$(CORE_DIR)/Inc
INC  += -I$(HAL_INC)
INC  += -I$(HAL_INC)/Legacy
INC  += -I$(CMSIS_DIR)/Include
INC  += -I$(DEVICE_DIR)/Include

CFLAGS  = $(MCFLAGS) $(DEFS) $(INC)
CFLAGS += -Wall -Wextra -Os -g3 -ffunction-sections -fdata-sections
CFLAGS += -fno-common -ffreestanding -fstack-usage
CFLAGS += -Wno-unused-parameter
CFLAGS += --specs=nano.specs --specs=nosys.specs

ASFLAGS = $(MCFLAGS) $(DEFS) $(INC) -g3

LDFLAGS  = $(MCFLAGS) -T$(PROJECT_DIR)/STM32H743VITx_FLASH.ld
LDFLAGS += -Wl,-Map=$(BUILD)/$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs --specs=nosys.specs
LDFLAGS += -Wl,--print-memory-usage

# ----- Source Files -----
# Application sources
C_SRCS  = $(wildcard $(CORE_DIR)/Src/*.c)

# HAL driver sources (only needed modules)
HAL_SRCS  = $(HAL_DIR)/stm32h7xx_hal.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_cortex.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_rcc.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_rcc_ex.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_gpio.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_i2c.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_i2c_ex.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_uart.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_dma.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_dma_ex.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_exti.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_pwr.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_pwr_ex.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_tim.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_tim_ex.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_flash.c
HAL_SRCS += $(HAL_DIR)/stm32h7xx_hal_flash_ex.c

# Startup (assembly)
ASM_SRCS = $(PROJECT_DIR)/startup/startup_stm32h743xx.s

# ----- Object Files -----
C_OBJS   = $(patsubst %.c, $(BUILD)/%.o, $(notdir $(C_SRCS)))
HAL_OBJS = $(patsubst %.c, $(BUILD)/%.o, $(notdir $(HAL_SRCS)))
ASM_OBJS = $(patsubst %.s, $(BUILD)/%.o, $(notdir $(ASM_SRCS)))

ALL_OBJS = $(C_OBJS) $(HAL_OBJS) $(ASM_OBJS)

# ----- VPATH for source lookup -----
VPATH  = $(CORE_DIR)/Src
VPATH += $(HAL_DIR)
VPATH += $(PROJECT_DIR)/startup

# ----- Build Rules -----
.PHONY: all clean flash debug

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin
	@echo ""
	@echo "========== Build Complete =========="
	$(SIZE) $(BUILD)/$(TARGET).elf

# Create build directory
$(BUILD):
	mkdir -p $(BUILD)

# Link
$(BUILD)/$(TARGET).elf: $(BUILD) $(ALL_OBJS)
	@echo "Linking..."
	$(CC) $(ALL_OBJS) $(LDFLAGS) -o $@

# Compile C sources
$(BUILD)/%.o: %.c
	@echo "CC $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup
$(BUILD)/%.o: %.s
	@echo "AS $<"
	$(AS) $(ASFLAGS) -c $< -o $@

# Generate hex
$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

# Generate bin
$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

# ----- Flash to MCU using STM32CubeProgrammer or OpenOCD -----
flash: $(BUILD)/$(TARGET).bin
	@echo "Flashing..."
	@if command -v STM32_Programmer_CLI >/dev/null 2>&1; then \
		STM32_Programmer_CLI -c port=SWD -w $(BUILD)/$(TARGET).bin 0x08000000 -v; \
	elif command -v openocd >/dev/null 2>&1; then \
		openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
			-c "program $(BUILD)/$(TARGET).bin 0x08000000 verify reset"; \
	else \
		echo "No flash tool found. Install STM32CubeProgrammer or OpenOCD."; \
	fi

# ----- Clean -----
clean:
	rm -rf $(BUILD)

# ----- Debug helper -----
info:
	@echo "C sources: $(notdir $(C_SRCS))"
	@echo "HAL sources: $(notdir $(HAL_SRCS))"
	@echo "ASM sources: $(notdir $(ASM_SRCS))"
