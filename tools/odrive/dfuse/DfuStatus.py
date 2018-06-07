class DfuStatus:
    OK                 = 0x00
    ERROR_TARGET       = 0x01
    ERROR_FILE         = 0x02
    ERROR_WRITE        = 0x03
    ERROR_ERASE        = 0x04
    ERROR_CHECK_ERASED = 0x05
    ERROR_PROG         = 0x06
    ERROR_VERIFY       = 0x07
    ERROR_ADDRESS      = 0x08
    ERROR_NOTDONE      = 0x09
    ERROR_FIRMWARE     = 0x0a
    ERROR_VENDOR       = 0x0b
    ERROR_USBR         = 0x0c
    ERROR_POR          = 0x0d
    ERROR_UNKNOWN      = 0x0e
    ERROR_STALLEDPKT   = 0x0f

