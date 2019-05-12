
tup.include('build.lua')

-- Switch between board versions
boardversion = tup.getconfig("BOARD_VERSION")
if boardversion == "v3.1" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=1"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.2" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=2"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.3" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=3"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.4-24V" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=4"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.4-48V" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=4"
    FLAGS += "-DHW_VERSION_VOLTAGE=48"
elseif boardversion == "v3.5-24V" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.5-48V" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
    FLAGS += "-DHW_VERSION_VOLTAGE=48"
elseif boardversion == "v3.6-24V" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=6"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.6-56V" then
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=6"
    FLAGS += "-DHW_VERSION_VOLTAGE=56"
elseif boardversion == "" then
    error("board version not specified - take a look at tup.config.default")
else
    error("unknown board version "..boardversion)
end
buildsuffix = boardversion

-- USB I/O settings
if tup.getconfig("USB_PROTOCOL") == "native" or tup.getconfig("USB_PROTOCOL") == "" then
    FLAGS += "-DUSB_PROTOCOL_NATIVE"
elseif tup.getconfig("USB_PROTOCOL") == "native-stream" then
    FLAGS += "-DUSB_PROTOCOL_NATIVE_STREAM_BASED"
elseif tup.getconfig("USB_PROTOCOL") == "stdout" then
    FLAGS += "-DUSB_PROTOCOL_STDOUT"
elseif tup.getconfig("USB_PROTOCOL") == "none" then
    FLAGS += "-DUSB_PROTOCOL_NONE"
else
    error("unknown USB protocol")
end

-- UART I/O settings
if tup.getconfig("UART_PROTOCOL") == "native" then
    FLAGS += "-DUART_PROTOCOL_NATIVE"
elseif tup.getconfig("UART_PROTOCOL") == "ascii" or tup.getconfig("UART_PROTOCOL") == "" then
    FLAGS += "-DUART_PROTOCOL_ASCII"
elseif tup.getconfig("UART_PROTOCOL") == "stdout" then
    FLAGS += "-DUART_PROTOCOL_STDOUT"
elseif tup.getconfig("UART_PROTOCOL") == "none" then
    FLAGS += "-DUART_PROTOCOL_NONE"
else
    error("unknown UART protocol "..tup.getconfig("UART_PROTOCOL"))
end

-- GPIO settings
if tup.getconfig("STEP_DIR") == "y" then
    if tup.getconfig("UART_PROTOCOL") == "none" then
        FLAGS += "-DUSE_GPIO_MODE_STEP_DIR"
    else
        error("Step/dir mode conflicts with UART. Set CONFIG_UART_PROTOCOL to none.")
    end
end

-- Compiler settings
if tup.getconfig("STRICT") == "true" then
    FLAGS += '-Werror'
end


-- C-specific flags
FLAGS += '-D__weak="__attribute__((weak))"'
FLAGS += '-D__packed="__attribute__((__packed__))"'
FLAGS += '-DUSE_HAL_DRIVER'
FLAGS += '-DSTM32F405xx'

FLAGS += '-mthumb'
FLAGS += '-mcpu=cortex-m4'
FLAGS += '-mfpu=fpv4-sp-d16'
FLAGS += '-mfloat-abi=hard'
FLAGS += { '-Wall', '-Wdouble-promotion', '-Wfloat-conversion', '-fdata-sections', '-ffunction-sections'}

-- linker flags
LDFLAGS += '-TSTM32/STM32F405RGTx_FLASH.ld' -- todo: make the stm32_platform package export this
LDFLAGS += '-LThirdParty/CMSIS/Lib' -- todo: make the FreeRTOS/CMSIS package export this
LDFLAGS += '-lc -lm -lnosys -larm_cortexM4lf_math' -- libs
LDFLAGS += '-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-Wl,--undefined=uxTopUsedPriority'

-- debug build
if tup.getconfig("DEBUG") == "true" then
    FLAGS += '-g -gdwarf-2'
    OPT += '-Og'
else
    OPT += '-O2'
end

-- common flags for ASM, C and C++
OPT += '-ffast-math -fno-finite-math-only'
tup.append_table(FLAGS, OPT)
tup.append_table(LDFLAGS, OPT)

toolchain = GCCToolchain('arm-none-eabi-', 'build', FLAGS, LDFLAGS)


-- TODO: cleaner separation of the platform code and the rest
build{
    name='stm_platform',
    type='objects',
    toolchains={toolchain},
    packages={},
    sources={
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c',
        'ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_wwdg.c',
    },
    includes={
        'ThirdParty/STM32F4xx_HAL_Driver/Inc',
        'ThirdParty/STM32F4xx_HAL_Driver/Inc/Legacy',
        'ThirdParty/CMSIS/Include',
        'ThirdParty/CMSIS/Device/ST/STM32F4xx/Include',
        'STM32'
    }
}

build{
    name='FreeRTOS',
    type='objects',
    toolchains={toolchain},
    packages={},
    sources={
        'ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c',
        'ThirdParty/FreeRTOS/Source/croutine.c',
        'ThirdParty/FreeRTOS/Source/portable/MemMang/heap_4.c',
        'ThirdParty/FreeRTOS/Source/list.c',
        'ThirdParty/FreeRTOS/Source/queue.c',
        'ThirdParty/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c',
        'ThirdParty/FreeRTOS/Source/tasks.c',
        'ThirdParty/FreeRTOS/Source/timers.c',
        'ThirdParty/FreeRTOS/Source/event_groups.c'
    },
    includes={
        'ThirdParty/FreeRTOS/Source/include',
        'ThirdParty/FreeRTOS/Source/CMSIS_RTOS',
        'ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F',
        'ThirdParty/CMSIS/Include',
        'ThirdParty/CMSIS/Device/ST/STM32F4xx/Include',
        'STM32'
    }
}

tup.frule{
    command='python ../tools/odrive/version.py --output %o',
    outputs={'build/version.h'}
}

build{
    name='ODriveFirmware',
    toolchains={toolchain},
    --toolchains={LLVMToolchain('x86_64', {'-Ofast'}, {'-flto'})},
    packages={'stm_platform', 'FreeRTOS'},
    sources={
        -- STM32 HAL
        'STM32/stm32_adc.cpp',
        'STM32/stm32_can.cpp',
        'STM32/stm32_dma.cpp',
        'STM32/stm32_gpio.cpp',
        'STM32/stm32_i2c.cpp',
        'STM32/stm32_spi.cpp',
        'STM32/stm32_system.cpp',
        'STM32/stm32_tim.cpp',
        'STM32/stm32_usart.cpp',
        'STM32/stm32_usb.cpp',
        'STM32/startup_stm32f405xx.s',
        'ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c',
        'ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_core.c',
        'ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c',
        'STM32/stm32f4xx_hal_timebase_TIM.c',
        'STM32/usbd_conf.c',
        'STM32/usbd_desc.c',
        'STM32/system_stm32f4xx.c',

        'STM32/freertos.cpp',
        'FreeRTOS-openocd.c',

        'Drivers/DRV8301/drv8301.cpp',
        'Drivers/DRV8301/drv8301.c',

        'USB/usb.cpp',
        'USB/usb_cdc.cpp',
        'USB/winusb_compat.cpp',

        'MotorControl/main.cpp',
        'MotorControl/utils.c',
        'MotorControl/arm_sin_f32.c',
        'MotorControl/arm_cos_f32.c',
        'MotorControl/low_level.cpp',
        'MotorControl/nvm.c',
        'MotorControl/axis.cpp',
        'MotorControl/motor.cpp',
        'MotorControl/encoder.cpp',
        'MotorControl/controller.cpp',
        'MotorControl/sensorless_estimator.cpp',
        'MotorControl/trapTraj.cpp',
        'MotorControl/pwm_in.cpp',
        'communication/can_simple.cpp',
        'communication/communication.cpp',
        'communication/ascii_protocol.cpp',
        'communication/interface_uart.cpp',
        'communication/interface_usb.cpp',
--        'communication/interface_i2c.cpp',
        'fibre/cpp/protocol.cpp',
    },
    includes={
        'Drivers/DRV8301',
        'MotorControl',
        'USB',
        'STM32',
        'ThirdParty/STM32_USB_Device_Library/Core/Inc',
        'fibre/cpp/include',
        '.'
    }
}
