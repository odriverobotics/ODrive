-- Switch between board versions using TUP.config even if using CLion build system
boardversion = tup.getconfig("BOARD_VERSION")
if boardversion == "v3.1" then
    boarddir = 'Board/v3' -- currently all platform code is in the same v3.3 directory
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=1"
    FLAGS += '-DSTM32F405xx'
    LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'

elseif boardversion == "v3.2" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=2"
    FLAGS += '-DSTM32F405xx'
    LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'

elseif boardversion == "v3.3" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=3"
    FLAGS += '-DSTM32F405xx'
    LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'
elseif boardversion == "v3.4-24V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=4"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
    FLAGS += '-DSTM32F405xx'
    LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'

elseif boardversion == "v3.4-48V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=4"
    FLAGS += "-DHW_VERSION_VOLTAGE=48"
    FLAGS += '-DSTM32F405xx'
    LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'

elseif boardversion == "v3.5-24V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
    FLAGS += '-DSTM32F405xx'
    LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'

elseif boardversion == "v3.5-48V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
    FLAGS += "-DHW_VERSION_VOLTAGE=48"
    FLAGS += '-DSTM32F405xx'
    LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'
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

