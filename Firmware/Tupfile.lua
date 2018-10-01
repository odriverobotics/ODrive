
tup.include('build.lua')

tup.include('HAL_Config.lua') -- Flags for each HAL & board used.


-- C-specific flags
FLAGS += '-D__weak="__attribute__((weak))"'
FLAGS += '-D__packed="__attribute__((__packed__))"'
FLAGS += '-DUSE_HAL_DRIVER'

FLAGS += '-mthumb'
FLAGS += '-mcpu=cortex-m4'
FLAGS += '-mfpu=fpv4-sp-d16'
FLAGS += '-mfloat-abi=hard'
FLAGS += { '-Wall', '-Wfloat-conversion', '-fdata-sections', '-ffunction-sections'}

-- debug build
FLAGS += '-g -gdwarf-2'


-- linker flags
LDFLAGS += '-L'..boarddir..'/Drivers/CMSIS/Lib' -- lib dir
LDFLAGS += '-lc -lm -lnosys -larm_cortexM4lf_math' -- libs
LDFLAGS += '-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-Wl,--undefined=uxTopUsedPriority'


-- common flags for ASM, C and C++
OPT += '-Og'
OPT += '-ffast-math -fno-finite-math-only'
tup.append_table(FLAGS, OPT)
tup.append_table(LDFLAGS, OPT)

toolchain = GCCToolchain('arm-none-eabi-', 'build', FLAGS, LDFLAGS)


-- Load list of source files Makefile that was autogenerated by CubeMX
vars = parse_makefile_vars(boarddir..'/Makefile')
all_stm_sources = (vars['C_SOURCES'] or '')..' '..(vars['CPP_SOURCES'] or '')..' '..(vars['ASM_SOURCES'] or '')
for src in string.gmatch(all_stm_sources, "%S+") do
stm_sources += boarddir..'/'..src
end
for src in string.gmatch(vars['C_INCLUDES'] or '', "%S+") do
stm_includes += boarddir..'/'..string.sub(src, 3, -1) -- remove "-I" from each include path
end

-- TODO: cleaner separation of the platform code and the rest
stm_includes += '.'
stm_includes += 'Drivers/DRV8301'
stm_sources += boarddir..'/Src/syscalls.c'
build{
name='stm_platform',
type='objects',
toolchains={toolchain},
packages={},
sources=stm_sources,
includes=stm_includes
}

tup.frule{
command='python ../tools/odrive/version.py --output %o',
outputs={'build/version.h'}
}

build{
name='ODriveFirmware',
toolchains={toolchain},
--toolchains={LLVMToolchain('x86_64', {'-Ofast'}, {'-flto'})},
packages={'stm_platform'},
sources={
'Drivers/DRV8301/drv8301.c',
'MotorControl/utils.c',
'MotorControl/low_level.cpp',
'MotorControl/nvm.c',
'MotorControl/axis.cpp',
'MotorControl/motor.cpp',
'MotorControl/encoder.cpp',
'MotorControl/controller.cpp',
'MotorControl/sensorless_estimator.cpp',
'MotorControl/main.cpp',
'communication/communication.cpp',
'communication/ascii_protocol.cpp',
'communication/interface_uart.cpp',
'communication/interface_usb.cpp',
'communication/interface_can.cpp',
'communication/interface_i2c.cpp',
'fibre/cpp/protocol.cpp',
'FreeRTOS-openocd.c'
},
includes={
'Drivers/DRV8301',
'MotorControl',
'fibre/cpp/include',
'.'
}
}
