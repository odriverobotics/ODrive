class DfuState():
    APP_IDLE                = 0x00
    APP_DETACH              = 0x01
    DFU_IDLE                = 0x02
    DFU_DOWNLOAD_SYNC       = 0x03
    DFU_DOWNLOAD_BUSY       = 0x04
    DFU_DOWNLOAD_IDLE       = 0x05
    DFU_MANIFEST_SYNC       = 0x06
    DFU_MANIFEST            = 0x07
    DFU_MANIFEST_WAIT_RESET = 0x08
    DFU_UPLOAD_IDLE         = 0x09
    DFU_ERROR               = 0x0a

