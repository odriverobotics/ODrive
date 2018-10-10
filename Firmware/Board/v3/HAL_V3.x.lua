if boardversion == "v3.1" then
    boarddir = 'Board/v3' -- currently all platform code is in the same v3.3 directory
    DefineBoardVersion(3,1,24)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.2" then
    boarddir = 'Board/v3'
    DefineBoardVersion(3,2,24)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.3" then
    boarddir = 'Board/v3'
    DefineBoardVersion(3,3,24)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.4-24V" then
    boarddir = 'Board/v3'
    DefineBoardVersion(3,4,24)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.4-48V" then
    boarddir = 'Board/v3'
    DefineBoardVersion(3,4,48)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.5-24V" then
    boarddir = 'Board/v3'
    DefineBoardVersion(3,5,24)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.5-48V" then
    boarddir = 'Board/v3'
    DefineBoardVersion(3,5,48)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.6-24V" then
    boarddir = 'Board/v3'
    DefineBoardVersion(3,6,24)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()

elseif boardversion == "v3.6-56V" then
    boarddir = 'Board/v3'
    DefineBoardVersion(3,6,56)
    MCU_Set_STM32F405()
    Hardware_FPU_Enable()
end
