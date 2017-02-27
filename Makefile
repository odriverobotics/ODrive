######################################
# Makefile by CubeMX2Makefile.py
######################################

######################################
# target
######################################
TARGET = ODriveFirmware

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
# OPT = -O3 -ffast-math -flto
# OPT = -O3 -ffast-math
OPT = -O0 -ffast-math

#######################################
# pathes
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
C_SOURCES = \
  Middlewares/Third_Party/FreeRTOS/Source/queue.c \
  Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
  Middlewares/Third_Party/FreeRTOS/Source/list.c \
  Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
  Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
  Middlewares/Third_Party/FreeRTOS/Source/timers.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
  Src/stm32f4xx_it.c \
  Src/tim.c \
  Src/gpio.c \
  Src/main.c \
  Src/adc.c \
  Src/freertos.c \
  Src/spi.c \
  Src/stm32f4xx_hal_msp.c \
  Src/can.c \
  Drivers/DRV8301/drv8301.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
  Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c \
  MotorControl/utils.c \
  MotorControl/low_level.c  
ASM_SOURCES = \
  Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s

#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# macros for gcc
AS_DEFS =
C_DEFS = -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F405xx
# includes for gcc
AS_INCLUDES =
C_INCLUDES = -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
C_INCLUDES += -IMiddlewares/Third_Party/FreeRTOS/Source/include
C_INCLUDES += -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
C_INCLUDES += -IDrivers/DRV8301
C_INCLUDES += -IDrivers/STM32F4xx_HAL_Driver/Inc
C_INCLUDES += -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy
C_INCLUDES += -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
C_INCLUDES += -IDrivers/CMSIS/Include
C_INCLUDES += -IInc
C_INCLUDES += -IMotorControl
# compile gcc flags
ASFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
# Generate dependency information
CFLAGS += -std=c99 -MD -MP -MF .dep/$(@F).d

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F405RGTx_FLASH.ld
# libraries
LIBS = -lc -lm -lnosys -larm_cortexM4lf_math
LIBDIR = -LDrivers/CMSIS/Lib
LDFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nano.specs $(OPT) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@$(BIN) $< $@
	
$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

#######################################
# flashing / debug
#######################################

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c reset\ halt -c flash\ write_image\ erase\ $(BUILD_DIR)/$(TARGET).elf -c reset\ run -c exit

gdb: $(BUILD_DIR)/$(TARGET).elf
	arm-none-eabi-gdb $(BUILD_DIR)/$(TARGET).elf -x openocd.gdbinit

bmp: $(BUILD_DIR)/$(TARGET).elf
	arm-none-eabi-gdb --ex 'target extended-remote /dev/stlink' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' $(BUILD_DIR)/$(TARGET).elf

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

.PHONY: clean all flash gdb

# *** EOF ***
