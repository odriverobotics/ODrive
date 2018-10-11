MCU_Defined = false
FPU_Defined = false

function DefineBoardVersion(Major,Minor,Voltage)
    FLAGS += "-DHW_VERSION_MAJOR="..Major
    FLAGS += "-DHW_VERSION_MINOR="..Minor
    FLAGS += "-DHW_VERSION_VOLTAGE="..Voltage
end

function MCU_Set_STM32F405()
    MCU_Defined = true
    FLAGS += '-DUSE_HAL_DRIVER'
    FLAGS += '-DSTM32F405xx'
    FLAGS += '-mcpu=cortex-m4'
    FLAGS += '-mfpu=fpv4-sp-d16'
    LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'
    LDFLAGS += '-L'..boarddir..'/Drivers/CMSIS/Lib' -- lib dir
    LDFLAGS += '-mfpu=fpv4-sp-d16  -mcpu=cortex-m4 -lc -lm -lnosys -larm_cortexM4lf_math' -- libs
end

function Hardware_FPU_Enable()
    if not MCU_Defined then error("Define MCU First!") end
    if FPU_Defined then error("FPU State Already Set. Remove Duplicate.") end
    FPU_Defined = true
    FLAGS += '-mfloat-abi=hard'
    LDFLAGS += '-mfloat-abi=hard'
end

function Hardware_FPU_Disable()
    if not MCU_Defined then error("Define MCU First!") end
    if FPU_Defined then error("FPU State Already Set. Remove Duplicate.") end
    FPU_Defined = true
    FLAGS += '-mfloat-abi=soft'
    LDFLAGS += '-mfloat-abi=soft'
end