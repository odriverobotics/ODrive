
-- Utility functions -----------------------------------------------------------

function run_now(command)
    local handle
    handle = io.popen(command)
    local output = handle:read("*a")
    local rc = {handle:close()}
    return rc[1], output
end

-- If we simply invoke python or python3 on a pristine Windows 10, it will try
-- to open the Microsoft Store which will not work and hang tup instead. The
-- command "python --version" does not open the Microsoft Store.
-- On some systems this may return a python2 command if Python3 is not installed.
function find_python3()
    success, python_version = run_now("python --version 2>&1")
    if success and string.match(python_version, "Python 3") then return "python -B" end
    success, python_version = run_now("python3 --version 2>&1")
    if success and string.match(python_version, "Python 3") then return "python3 -B" end
    error("Python 3 not found.")
end

function add_pkg(pkg)
    if pkg.is_included == true then
        return
    end
    pkg.is_included = true
    for _, file in pairs(pkg.code_files or {}) do
        code_files += (pkg.root or '.')..'/'..file
    end
    for _, dir in pairs(pkg.include_dirs or {}) do
        CFLAGS += '-I'..(pkg.root or '.')..'/'..dir
    end
    tup.append_table(CFLAGS, pkg.cflags or {})
    tup.append_table(LDFLAGS, pkg.ldflags or {})
    for _, pkg in pairs(pkg.include or {}) do
        add_pkg(pkg)
    end
end

function compile(src_file, obj_file)
    compiler = (tup.ext(src_file) == 'c') and CC or CXX
    tup.frule{
        inputs={src_file},
        extra_inputs = {'autogen/interfaces.hpp', 'autogen/function_stubs.hpp', 'autogen/endpoints.hpp', 'autogen/type_info.hpp'},
        command='^o^ '..compiler..' -c %f '..tostring(CFLAGS)..' -o %o',
        outputs={obj_file}
    }
end

-- Packages --------------------------------------------------------------------

tup.include('fibre-cpp/package.lua')
fibre_pkg = get_fibre_package({
    enable_server=true,
    enable_client=false,
    allow_heap=false,
    max_log_verbosity=0,
    pkgconf=false,
})

odrive_firmware_pkg = {
    root = '.',
    include_dirs = {
        '.',
        'MotorControl',
        'fibre-cpp/include',
    },
    code_files = {
        'syscalls.c',
        'MotorControl/utils.cpp',
        'MotorControl/arm_sin_f32.c',
        'MotorControl/arm_cos_f32.c',
        'MotorControl/low_level.cpp',
        'MotorControl/axis.cpp',
        'MotorControl/motor.cpp',
        'MotorControl/thermistor.cpp',
        'MotorControl/encoder.cpp',
        'MotorControl/endstop.cpp',
        'MotorControl/acim_estimator.cpp',
        'MotorControl/mechanical_brake.cpp',
        'MotorControl/controller.cpp',
        'MotorControl/foc.cpp',
        'MotorControl/open_loop_controller.cpp',
        'MotorControl/oscilloscope.cpp',
        'MotorControl/sensorless_estimator.cpp',
        'MotorControl/trapTraj.cpp',
        'MotorControl/pwm_input.cpp',
        'MotorControl/main.cpp',
        'Drivers/STM32/stm32_system.cpp',
        'Drivers/STM32/stm32_gpio.cpp',
        'Drivers/STM32/stm32_nvm.c',
        'Drivers/STM32/stm32_spi_arbiter.cpp',
        'communication/can/can_simple.cpp',
        'communication/can/odrive_can.cpp',    
        'communication/communication.cpp',
        'communication/ascii_protocol.cpp',
        'communication/interface_uart.cpp',
        'communication/interface_usb.cpp',
        'communication/interface_i2c.cpp',
        'FreeRTOS-openocd.c',
        'autogen/version.c'
    }
}

stm32f4xx_hal_pkg = {
    root = 'ThirdParty/STM32F4xx_HAL_Driver',
    include_dirs = {
        'Inc',
    },
    code_files = {
        'Src/stm32f4xx_hal.c',
        'Src/stm32f4xx_hal_adc.c',
        'Src/stm32f4xx_hal_adc_ex.c',
        'Src/stm32f4xx_hal_can.c',
        'Src/stm32f4xx_hal_cortex.c',
        'Src/stm32f4xx_hal_dma.c',
        'Src/stm32f4xx_hal_dma_ex.c',
        'Src/stm32f4xx_hal_flash.c',
        'Src/stm32f4xx_hal_flash_ex.c',
        'Src/stm32f4xx_hal_flash_ramfunc.c',
        'Src/stm32f4xx_hal_gpio.c',
        'Src/stm32f4xx_hal_i2c.c',
        'Src/stm32f4xx_hal_i2c_ex.c',
        'Src/stm32f4xx_hal_pcd.c',
        'Src/stm32f4xx_hal_pcd_ex.c',
        'Src/stm32f4xx_hal_pwr.c',
        'Src/stm32f4xx_hal_pwr_ex.c',
        'Src/stm32f4xx_hal_rcc.c',
        'Src/stm32f4xx_hal_rcc_ex.c',
        'Src/stm32f4xx_hal_spi.c',
        'Src/stm32f4xx_hal_tim.c',
        'Src/stm32f4xx_hal_tim_ex.c',
        'Src/stm32f4xx_hal_uart.c',
        'Src/stm32f4xx_ll_usb.c',
    },
    cflags = {'-DARM_MATH_CM4', '-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-DFPU_FPV4'}
}

stm32f7xx_hal_pkg = {
    root = 'Private/ThirdParty/STM32F7xx_HAL_Driver',
    include_dirs = {
        'Inc',
    },
    code_files = {
        'Src/stm32f7xx_hal.c',
        'Src/stm32f7xx_hal_adc.c',
        'Src/stm32f7xx_hal_adc_ex.c',
        'Src/stm32f7xx_hal_can.c',
        'Src/stm32f7xx_hal_cortex.c',
        'Src/stm32f7xx_hal_dma.c',
        'Src/stm32f7xx_hal_dma_ex.c',
        'Src/stm32f7xx_hal_exti.c',
        'Src/stm32f7xx_hal_flash.c',
        'Src/stm32f7xx_hal_flash_ex.c',
        'Src/stm32f7xx_hal_gpio.c',
        'Src/stm32f7xx_hal_i2c.c',
        'Src/stm32f7xx_hal_i2c_ex.c',
        'Src/stm32f7xx_hal_i2s.c',
        'Src/stm32f7xx_hal_pcd.c',
        'Src/stm32f7xx_hal_pcd_ex.c',
        'Src/stm32f7xx_hal_pwr.c',
        'Src/stm32f7xx_hal_pwr_ex.c',
        'Src/stm32f7xx_hal_rcc.c',
        'Src/stm32f7xx_hal_rcc_ex.c',
        'Src/stm32f7xx_hal_spi.c',
        'Src/stm32f7xx_hal_spi_ex.c',
        'Src/stm32f7xx_hal_tim.c',
        'Src/stm32f7xx_hal_tim_ex.c',
        'Src/stm32f7xx_hal_uart.c',
        'Src/stm32f7xx_hal_uart_ex.c',
        'Src/stm32f7xx_ll_usb.c',
    },
    cflags = {'-DARM_MATH_CM7', '-mcpu=cortex-m7', '-mfpu=fpv5-sp-d16'}
}

freertos_pkg = {
    root = 'ThirdParty/FreeRTOS',
    include_dirs = {
        'Source/include',
        'Source/CMSIS_RTOS',
    },
    code_files = {
        'Source/croutine.c',
        'Source/event_groups.c',
        'Source/list.c',
        'Source/queue.c',
        'Source/stream_buffer.c',
        'Source/tasks.c',
        'Source/timers.c',
        'Source/CMSIS_RTOS/cmsis_os.c',
        'Source/portable/MemMang/heap_4.c',
    }
}

cmsis_pkg = {
    root = 'ThirdParty/CMSIS',
    include_dirs = {
        'Include',
        'Device/ST/STM32F7xx/Include',
        'Device/ST/STM32F4xx/Include'
    },
    ldflags = {'-LThirdParty/CMSIS/Lib/GCC'},
}

stm32_usb_device_library_pkg = {
    root = 'ThirdParty/STM32_USB_Device_Library',
    include_dirs = {
        'Core/Inc',
        'Class/CDC/Inc',
    },
    code_files = {
        'Core/Src/usbd_core.c',
        'Core/Src/usbd_ctlreq.c',
        'Core/Src/usbd_ioreq.c',
        'Class/CDC/Src/usbd_cdc.c',
    }
}

crypto_pkg = {
    root = 'Private',
    include_dirs = {
        'ThirdParty/sha-2',
        'ThirdParty/rsa_embedded',
    },
    code_files = {
        'ThirdParty/sha-2/sha-256.c',
        'ThirdParty/rsa_embedded/rsa.c',
    }
}

board_v3 = {
    root = 'Board/v3',
    root_interface = 'ODrive3',
    include = {stm32f4xx_hal_pkg},
    include_dirs = {
        'Inc',
        '../../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F',
    },
    code_files = {
        'startup_stm32f405xx.s',
        '../../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c',
        '../../Drivers/DRV8301/drv8301.cpp',
        'board.cpp',
        'Src/stm32f4xx_hal_timebase_TIM.c',
        'Src/tim.c',
        'Src/dma.c',
        'Src/freertos.c',
        'Src/main.c',
        'Src/usbd_conf.c',
        'Src/spi.c',
        'Src/usart.c',
        'Src/usbd_cdc_if.c',
        'Src/adc.c',
        'Src/stm32f4xx_hal_msp.c',
        'Src/usbd_desc.c',
        'Src/stm32f4xx_it.c',
        'Src/usb_device.c',
        'Src/can.c',
        'Src/system_stm32f4xx.c',
        'Src/gpio.c',
        'Src/i2c.c',
    },
    cflags = {'-DSTM32F405xx', '-DHW_VERSION_MAJOR=3'},
    ldflags = {
        '-TBoard/v3/STM32F405RGTx_FLASH.ld',
        '-larm_cortexM4lf_math',
    }
}

board_v4 = {
    root = 'Private/v4',
    root_interface = 'ODrive4',
    include = {stm32f7xx_hal_pkg, crypto_pkg},
    include_dirs = {
        '..',
        'Inc',
        '../../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1',
    },
    code_files = {
        'startup_stm32f722xx.s',
        '../../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c',
        '../Drivers/DRV8353/drv8353.cpp',
        '../Drivers/status_led.cpp',
        'board.cpp',
        'Src/main.c',
        'Src/gpio.c',
        'Src/adc.c',
        'Src/can.c',
        'Src/dma.c',
        'Src/freertos.c',
        'Src/spi.c',
        'Src/tim.c',
        'Src/stm32f7xx_it.c',
        'Src/stm32f7xx_hal_msp.c',
        'Src/stm32f7xx_hal_timebase_tim.c',
        'Src/system_stm32f7xx.c',
        'Src/i2s.c',
        'Src/usart.c',
        'Src/usb_device.c',
        'Src/usbd_conf.c',
        'Src/usbd_desc.c',
        'Src/usbd_cdc_if.c',
        'Src/i2c.c',
    },
    cflags = {'-DSTM32F722xx', '-DHW_VERSION_MAJOR=4'},
    ldflags = {
        '-TPrivate/v4/STM32F722RETx_FLASH.ld',
        '-larm_cortexM7lfsp_math',
    }
}

boards = {
    ["v3.1"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=1 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.2"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=2 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.3"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=3 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.4-24V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=4 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.4-48V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=4 -DHW_VERSION_VOLTAGE=48"}},
    ["v3.5-24V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=5 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.5-48V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=5 -DHW_VERSION_VOLTAGE=48"}},
    ["v3.6-24V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=6 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.6-56V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=6 -DHW_VERSION_VOLTAGE=56"}},
    ["v4.0-56V"] = {include={board_v4}, cflags={"-DHW_VERSION_MINOR=0 -DHW_VERSION_VOLTAGE=56"}},
    ["v4.1-58V"] = {include={board_v4}, cflags={"-DHW_VERSION_MINOR=1 -DHW_VERSION_VOLTAGE=58"}},
}


-- Toolchain setup -------------------------------------------------------------

CCPATH = tup.getconfig('ARM_COMPILER_PATH')
if CCPATH == "" then
    CCPATH=''
else
    CCPATH = CCPATH..'/'
end

CC=CCPATH..'arm-none-eabi-gcc -std=c99'
CXX=CCPATH..'arm-none-eabi-g++ -std=c++17 -Wno-register'
LINKER=CCPATH..'arm-none-eabi-g++'

-- C-specific flags
CFLAGS += '-D__weak="__attribute__((weak))"'
CFLAGS += '-D__packed="__attribute__((__packed__))"'
CFLAGS += '-DUSE_HAL_DRIVER'

CFLAGS += '-mthumb'
CFLAGS += '-mfloat-abi=hard'
CFLAGS += '-Wno-psabi' -- suppress unimportant note about ABI compatibility in GCC 10
CFLAGS += { '-Wall', '-Wdouble-promotion', '-Wfloat-conversion', '-fdata-sections', '-ffunction-sections'}
CFLAGS += '-g'
CFLAGS += '-DFIBRE_ENABLE_SERVER'
CFLAGS += '-Wno-nonnull'

-- linker flags
LDFLAGS += '-flto -lc -lm -lnosys' -- libs
-- LDFLAGS += '-mthumb -mfloat-abi=hard -specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-mthumb -mfloat-abi=hard -specs=nosys.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-Wl,--undefined=uxTopUsedPriority'


-- Handle Configuration Options ------------------------------------------------

-- Switch between board versions
boardversion = tup.getconfig("BOARD_VERSION")
if boardversion == "" then
    error("board version not specified - take a look at tup.config.default")
elseif boards[boardversion] == nil then
    error("unknown board version "..boardversion)
end
board = boards[boardversion]

-- --not 
-- TODO: remove this setting
if tup.getconfig("USB_PROTOCOL") ~= "native" and tup.getconfig("USB_PROTOCOL") ~= "" then
    error("CONFIG_USB_PROTOCOL is deprecated")
end

-- UART I/O settings
if tup.getconfig("UART_PROTOCOL") ~= "ascii" and tup.getconfig("UART_PROTOCOL") ~= "" then
    error("CONFIG_UART_PROTOCOL is deprecated")
end

-- Compiler settings
if tup.getconfig("STRICT") == "true" then
    CFLAGS += '-Werror'
end

if tup.getconfig("NO_DRM") == "true" then
    CFLAGS += '-DNO_DRM'
end

-- debug build
if tup.getconfig("DEBUG") == "true" then
    CFLAGS += '-gdwarf-2 -Og'
else
    CFLAGS += '-O2'
end

if tup.getconfig("USE_LTO") == "true" then
    CFLAGS += '-flto'
end


-- Generate Tup Rules ----------------------------------------------------------

python_command = find_python3()
print('Using python command "'..python_command..'"')

-- TODO: use CI to verify that on PRs the enums.py file is consistent with the YAML.
-- Note: we currently check this file into source control for two reasons:
--  - Don't require tup to run in order to use odrivetool from the repo
--  - On Windows, tup is unhappy with writing outside of the tup directory
--tup.frule{command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template enums_template.j2 --output ../tools/odrive/enums.py'}

-- tup.frule{
--     command=python_command..' ../tools/odrive/version.py --output %o',
--     outputs={'autogen/version.c'}
-- }

-- Autogen files from YAML interface definitions
root_interface = board.include[1].root_interface
tup.frule{inputs={'fibre-cpp/interfaces_template.j2', extra_inputs='odrive-interface.yaml'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/interfaces.hpp'}
tup.frule{inputs={'fibre-cpp/function_stubs_template.j2', extra_inputs='odrive-interface.yaml'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/function_stubs.hpp'}
tup.frule{inputs={'fibre-cpp/endpoints_template.j2', extra_inputs='odrive-interface.yaml'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --generate-endpoints '..root_interface..' --template %f --output %o', outputs='autogen/endpoints.hpp'}
tup.frule{inputs={'fibre-cpp/type_info_template.j2', extra_inputs='odrive-interface.yaml'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/type_info.hpp'}


add_pkg(board)
add_pkg(freertos_pkg)
add_pkg(cmsis_pkg)
add_pkg(stm32_usb_device_library_pkg)
add_pkg(fibre_pkg)
add_pkg(odrive_firmware_pkg)


for _, src_file in pairs(code_files) do
    obj_file = "build/obj/"..src_file:gsub("/","_"):gsub("%.","")..".o"
    object_files += obj_file
    compile(src_file, obj_file)
end

tup.frule{
    inputs=object_files,
    command='^o^ '..LINKER..' %f '..tostring(CFLAGS)..' '..tostring(LDFLAGS)..
            ' -Wl,-Map=%O.map -o %o',
    outputs={'build/ODriveFirmware.elf', extra_outputs={'build/ODriveFirmware.map'}}
}
-- display the size
tup.frule{inputs={'build/ODriveFirmware.elf'}, command=CCPATH..'arm-none-eabi-size %f'}
-- create *.hex and *.bin output formats
tup.frule{inputs={'build/ODriveFirmware.elf'}, command=CCPATH..'arm-none-eabi-objcopy -O ihex %f %o', outputs={'build/ODriveFirmware.hex'}}
tup.frule{inputs={'build/ODriveFirmware.elf'}, command=CCPATH..'arm-none-eabi-objcopy -O binary -S %f %o', outputs={'build/ODriveFirmware.bin'}}

if tup.getconfig('ENABLE_DISASM') == 'true' then
    tup.frule{inputs={'build/ODriveFirmware.elf'}, command=CCPATH..'arm-none-eabi-objdump %f -dSC > %o', outputs={'build/ODriveFirmware.asm'}}
end

if tup.getconfig('DOCTEST') == 'true' then
    TEST_INCLUDES = '-I. -I./MotorControl -I./fibre-cpp/include -I./Drivers/DRV8301 -I./doctest'
    tup.foreach_rule('Tests/*.cpp', 'g++ -O3 -std=c++17 '..TEST_INCLUDES..' -c %f -o %o', 'Tests/bin/%B.o')
    tup.frule{inputs='Tests/bin/*.o', command='g++ %f -o %o', outputs='Tests/test_runner.exe'}
    tup.frule{inputs='Tests/test_runner.exe', command='%f'}
end
