if boardversion == "v3.1" then
    boarddir = 'Board/v3' -- currently all platform code is in the same v3.3 directory
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=1"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.2" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=2"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.3" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=3"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.4-24V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=4"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.4-48V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=4"
    FLAGS += "-DHW_VERSION_VOLTAGE=48"
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.5-24V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.5-48V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
    FLAGS += "-DHW_VERSION_VOLTAGE=48"
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

end
