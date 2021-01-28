
#include "interface_usb.h"
#include "ascii_protocol.hpp"

#include <MotorControl/utils.hpp>

#include <fibre/async_stream.hpp>
#include <fibre/../../legacy_protocol.hpp>
#include <usbd_cdc.h>
#include "usbd_core.h"
#include <cmsis_os.h>
#include <freertos_vars.h>

#include <odrive_main.h>

osThreadId usb_thread;
const uint32_t stack_size_usb_thread = 4096; // Bytes
USBStats_t usb_stats_;

#define USBD_VID    0x1209
#define USBD_PID    0x0D32
#define USBD_LANGID 1033

#define STRINGIFY(s) QUOTE(s)
#define QUOTE(s) #s

#define USB_TX_DATA_SIZE 64 // See remark in code

namespace fibre {

class Stm32UsbTxStream : public AsyncStreamSink {
public:
    Stm32UsbTxStream(uint8_t endpoint_num) : endpoint_num_(endpoint_num) {}

    void start_write(cbufptr_t buffer, TransferHandle* handle, Callback<void, WriteResult> completer) final;
    void cancel_write(TransferHandle transfer_handle) final;
    void did_finish();

    const uint8_t endpoint_num_;
    bool connected_ = false;
    Callback<void, WriteResult> completer_;
    const uint8_t* tx_end_ = nullptr;
};

class Stm32UsbRxStream : public AsyncStreamSource {
public:
    Stm32UsbRxStream(uint8_t endpoint_num) : endpoint_num_(endpoint_num) {}

    void start_read(bufptr_t buffer, TransferHandle* handle, Callback<void, ReadResult> completer) final;
    void cancel_read(TransferHandle transfer_handle) final;
    void did_finish();
    
    const uint8_t endpoint_num_;
    bool connected_ = false;
    Callback<void, ReadResult> completer_;
    uint8_t* rx_end_ = nullptr;
};

}

using namespace fibre;

void Stm32UsbTxStream::start_write(cbufptr_t buffer, TransferHandle* handle, Callback<void, WriteResult> completer) {
    if (handle) {
        *handle = reinterpret_cast<TransferHandle>(this);
    }

    if (!connected_) {
        completer.invoke({kStreamClosed, buffer.begin()});
        return;
    }

    // Note on MTU: on the physical layer, a full speed device can transmit up
    // to 64 bytes of payload per bulk package. However a single logical
    // transfer can consist of multiple 64 byte packets terminated by a 0 byte
    // packet. Currently we don't implement this segmentation. Therefore we
    // must ensure that all packets are < 64 bytes, otherwise the host will wait
    // for more.
    if (buffer.size() >= USB_TX_DATA_SIZE) {
        completer.invoke({kStreamError, buffer.begin()});
        return;
    }

    // Transmission already in progress?
    if (completer_ || tx_end_) {
        completer.invoke({kStreamError, buffer.begin()});
        return;
    }

    completer_ = completer;
    tx_end_ = buffer.end();

    USBD_StatusTypeDef status = USBD_LL_Transmit(&usb_dev_handle, endpoint_num_,
            const_cast<uint8_t*>(buffer.begin()), buffer.size());

    if (status != USBD_OK) {
        tx_end_ = nullptr;
        completer_.invoke_and_clear({kStreamError, buffer.begin()});
    }
}

void Stm32UsbTxStream::cancel_write(TransferHandle transfer_handle) {
    // not implemented
}

void Stm32UsbTxStream::did_finish() {
    const uint8_t* tx_end = tx_end_;
    tx_end_ = nullptr;
    completer_.invoke_and_clear({connected_ ? kStreamOk : kStreamClosed, tx_end});
}

void Stm32UsbRxStream::start_read(bufptr_t buffer, TransferHandle* handle, Callback<void, ReadResult> completer) {
    if (handle) {
        *handle = reinterpret_cast<TransferHandle>(this);
    }

    if (!connected_) {
        completer.invoke({kStreamClosed, buffer.begin()});
        return;
    }

    if (completer_ || rx_end_) {
        completer.invoke({kStreamError, buffer.begin()});
        return;
    }

    completer_ = completer;
    rx_end_ = buffer.begin(); // the pointer is updated at the end of the transfer

    if (USBD_CDC_ReceivePacket(&usb_dev_handle, buffer.begin(), buffer.size(), endpoint_num_) != USBD_OK) {
        rx_end_ = nullptr;
        completer_.invoke_and_clear({kStreamError, buffer.begin()});
        return;
    }
}

void Stm32UsbRxStream::cancel_read(TransferHandle transfer_handle) {
    // not implemented
}

void Stm32UsbRxStream::did_finish() {
    uint8_t* rx_end = rx_end_;
    rx_end_ = nullptr;
    completer_.invoke_and_clear({connected_ ? kStreamOk : kStreamClosed, rx_end});
}

Stm32UsbTxStream usb_cdc_tx_stream(CDC_IN_EP);
Stm32UsbTxStream usb_native_tx_stream(ODRIVE_IN_EP);
Stm32UsbRxStream usb_cdc_rx_stream(CDC_OUT_EP);
Stm32UsbRxStream usb_native_rx_stream(ODRIVE_OUT_EP);

LegacyProtocolStreamBased fibre_over_cdc(&usb_cdc_rx_stream, &usb_cdc_tx_stream);
LegacyProtocolPacketBased fibre_over_usb(&usb_native_rx_stream, &usb_native_tx_stream, USB_TX_DATA_SIZE - 1); // See note on MTU above

fibre::AsyncStreamSinkMultiplexer<2> usb_cdc_tx_multiplexer(usb_cdc_tx_stream);
fibre::BufferedStreamSink<64> usb_cdc_stdout_sink(usb_cdc_tx_multiplexer); // Used in communication.cpp
AsciiProtocol ascii_over_cdc(&usb_cdc_rx_stream, &usb_cdc_tx_multiplexer);

bool usb_cdc_stdout_pending = false;

static void usb_server_thread(void * ctx) {
    (void) ctx;
 
    for (;;) {
        osEvent event = osMessageGet(usb_event_queue, osWaitForever);

        if (event.status != osEventMessage) {
            continue;
        }

        usb_stats_.rx_cnt++;

        switch (event.value.v) {
            case 1: { // USB connected event
                usb_cdc_tx_stream.connected_ = true;
                usb_native_tx_stream.connected_ = true;
                usb_cdc_rx_stream.connected_ = true;
                usb_native_rx_stream.connected_ = true;

                fibre_over_usb.start({});

                if (odrv.config_.usb_cdc_protocol == ODrive::STREAM_PROTOCOL_TYPE_FIBRE) {
                    fibre_over_cdc.start({});
                } else if (odrv.config_.usb_cdc_protocol == ODrive::STREAM_PROTOCOL_TYPE_ASCII
                        || odrv.config_.usb_cdc_protocol == ODrive::STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT) {
                    ascii_over_cdc.start();
                }
            } break;

            case 2: { // USB disconnected event
                usb_cdc_tx_stream.connected_ = false;
                usb_native_tx_stream.connected_ = false;
                usb_cdc_rx_stream.connected_ = false;
                usb_native_rx_stream.connected_ = false;
                usb_cdc_tx_stream.did_finish();
                usb_native_tx_stream.did_finish();
                usb_cdc_rx_stream.did_finish();
                usb_native_rx_stream.did_finish();
            } break;

            case 3: { // TX on CDC interface done
                usb_cdc_tx_stream.did_finish();
            } break;

            case 4: { // TX on custom interface done
                usb_native_tx_stream.did_finish();
            } break;

            case 5: { // RX on CDC interface done
                usb_cdc_rx_stream.did_finish();
            } break;

            case 6: { // RX on custom interface done
                usb_native_rx_stream.did_finish();
            } break;

            case 7: { // stdout has data
                usb_cdc_stdout_pending = false;
                usb_cdc_stdout_sink.maybe_start_async_write();
            } break;
        }
    }
}

const char* get_string_descriptor(int str_id) {
    switch (str_id) {
        case USBD_IDX_MFC_STR: return "ODrive Robotics";
        case USBD_IDX_PRODUCT_STR: return STRINGIFY(ODrive HW_VERSION_MAJOR.HW_VERSION_MINOR);
        case USBD_IDX_SERIAL_STR: return serial_number_str;
        case USBD_IDX_CONFIG_STR: return "CDC Config"; // TODO: probably not what we want
        case USBD_IDX_INTERFACE_STR: return "CDC Interface"; // TODO: probably not what we want
        case USBD_IDX_ODRIVE_INTF_STR: return "ODrive Native Interface";

        // MS OS String descriptor to tell Windows that it may query for other
        // Windows-specific descriptors. This is needed for automatic WinUSB support.
        // Windows will only query for OS descriptors once.
        // Delete the information about already queried devices in registry by deleting:
        // HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\usbflags\VVVVPPPPRRRR
        case USBD_IDX_MICROSOFT_DESC_STR: return "MSFT100P";
        default: return nullptr;
    }
}

int get_string_descriptor(uint8_t str_id, uint8_t* buf, uint16_t* length) {
    if (str_id == USBD_IDX_LANGID_STR) {
        if (*length < 4) {
            return -1;
        }
        buf[0] = USB_LEN_LANGID_STR_DESC;
        buf[1] = USB_DESC_TYPE_STRING;
        buf[2] = LOBYTE(USBD_LANGID);
        buf[3] = HIBYTE(USBD_LANGID);
        *length = 4;
        return 0;
    }

    const char* ascii_str = get_string_descriptor(str_id);

    // Copy ASCII string into UTF-16 output buffer

    size_t n_chars = strlen(ascii_str);
    size_t desc_size = 2 + 2 * n_chars;

    if (*length < desc_size) {
        return -1;
    }

    *length = desc_size;
    buf[0] = desc_size;
    buf[1] = USB_DESC_TYPE_STRING;

    for (size_t i = 0; i < n_chars; ++i) {
      buf[2 + 2 * i] = ascii_str[i];
      buf[3 + 2 * i] = 0;
    }

    return 0;
}


__ALIGN_BEGIN static uint8_t usb_device_desc[USB_LEN_DEV_DESC] __ALIGN_END = {
    USB_LEN_DEV_DESC,           /*bLength */
    USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
#if (USBD_LPM_ENABLED == 1)
    0x01,                       /*bcdUSB */ /* changed to USB version 2.01
                                               in order to support LPM L1 suspend
                                               resume test of USBCV3.0*/
#else
    0x00,                       /*bcdUSB */
#endif /* (USBD_LPM_ENABLED == 1) */
    0x02,
    // Notify OS that this is a composite device
    0xEF,                       /*bDeviceClass*/
    0x02,                       /*bDeviceSubClass*/
    0x01,                       /*bDeviceProtocol*/
    USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
    LOBYTE(USBD_VID),           /*idVendor*/
    HIBYTE(USBD_VID),           /*idVendor*/
    LOBYTE(USBD_PID),        /*idProduct*/
    HIBYTE(USBD_PID),        /*idProduct*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x03,                       /* bNumInterfaces */
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

uint8_t* get_device_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    UNUSED(speed);
    *length = sizeof(usb_device_desc);
    return usb_device_desc;
}

#if ((USBD_LPM_ENABLED == 1) || (USBD_CLASS_BOS_ENABLED == 1))
#define USB_SIZ_BOS_DESC            0x0C

__ALIGN_BEGIN uint8_t USBD_HS_BOSDesc[USB_SIZ_BOS_DESC] __ALIGN_END = {
    0x5,
    USB_DESC_TYPE_BOS,
    LOBYTE(USB_SIZ_BOS_DESC),
    HIBYTE(USB_SIZ_BOS_DESC),
    0x1,  /* 1 device capability */
            /* device capability */
    0x7,
    USB_DEVICE_CAPABITY_TYPE,
    0x2,
    0x2,  /*LPM capability bit set */
    0x0,
    0x0,
    0x0
};

uint8_t * get_bos_descriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  UNUSED(speed);
  *length = sizeof(USBD_HS_BOSDesc);
  return (uint8_t*)USBD_HS_BOSDesc;
}
#endif /* (USBD_LPM_ENABLED == 1) */

USBD_DescriptorsTypeDef odrive_usb_descriptors = {
    get_device_descriptor,
    get_string_descriptor,
#if ((USBD_LPM_ENABLED == 1) || (USBD_CLASS_BOS_ENABLED == 1))
    get_bos_descriptor,
#endif /* (USBD_LPM_ENABLED == 1) */
};

/**
 * @brief Called from usb_cdc.c when a connection is established.
 */
static int8_t CDC_Init(void) {
    osMessagePut(usb_event_queue, 1, 0);
    return USBD_OK;
}

/**
 * @brief Called from usb_cdc.c when the connection breaks.
 */
static int8_t CDC_DeInit(void) {
    osMessagePut(usb_event_queue, 2, 0);
    return USBD_OK;
}

/**
 * @brief Called from usb_cdc.c when the host sends a CDC control request.
 */
static int8_t CDC_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length) {
    switch(cmd) {
        case CDC_SEND_ENCAPSULATED_COMMAND: break;
        case CDC_GET_ENCAPSULATED_RESPONSE: break;
        case CDC_SET_COMM_FEATURE: break;
        case CDC_GET_COMM_FEATURE: break;
        case CDC_CLEAR_COMM_FEATURE: break;

        /*******************************************************************************/
        /* Line Coding Structure                                                       */
        /*-----------------------------------------------------------------------------*/
        /* Offset | Field       | Size | Value  | Description                          */
        /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
        /* 4      | bCharFormat |   1  | Number | Stop bits                            */
        /*                                        0 - 1 Stop bit                       */
        /*                                        1 - 1.5 Stop bits                    */
        /*                                        2 - 2 Stop bits                      */
        /* 5      | bParityType |  1   | Number | Parity                               */
        /*                                        0 - None                             */
        /*                                        1 - Odd                              */
        /*                                        2 - Even                             */
        /*                                        3 - Mark                             */
        /*                                        4 - Space                            */
        /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
        /*******************************************************************************/
        case CDC_SET_LINE_CODING: break;
        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(115200);
            pbuf[1] = (uint8_t)(115200 >> 8);
            pbuf[2] = (uint8_t)(115200 >> 16);
            pbuf[3] = (uint8_t)(115200 >> 24);
            pbuf[4] = 0;  // stop bits (1)
            pbuf[5] = 0;  // parity (none)
            pbuf[6] = 8;  // number of bits (8)
        break;

        case CDC_SET_CONTROL_LINE_STATE: break;
        case CDC_SEND_BREAK: break;
        default: break;
    }

    return USBD_OK;
}

/**
  * @brief Called from usb_cdc.c when data arrives on USB.
  * 
  * The USB device driver will not accept more data until USBD_CDC_ReceivePacket
  * is called.
  */
static int8_t CDC_Receive(uint8_t* Buf, uint32_t *Len, uint8_t endpoint_pair) {
    if (endpoint_pair == CDC_OUT_EP && usb_cdc_rx_stream.rx_end_) {
        usb_cdc_rx_stream.rx_end_ += *Len;
        osMessagePut(usb_event_queue, 5, 0);
    } else if (endpoint_pair == ODRIVE_OUT_EP && usb_native_rx_stream.rx_end_) {
        usb_native_rx_stream.rx_end_ += *Len;
        osMessagePut(usb_event_queue, 6, 0);
    }
    return USBD_OK;
}

/**
 * @brief Unused
  */
static int8_t CDC_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum) {
    UNUSED(Buf);
    UNUSED(Len);
    UNUSED(epnum);
    return USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops = {
    CDC_Init,
    CDC_DeInit,
    CDC_Control,
    CDC_Receive,
    CDC_TransmitCplt
};


void start_usb_server() {
    // Init USB device driver
    if (USBD_Init(&usb_dev_handle, &odrive_usb_descriptors, DEVICE_HS) != USBD_OK) {
        for (;;); // TODO: handle
    }
    // TODO: we should register a composite class interface, not a CDC interface
    if (USBD_RegisterClass(&usb_dev_handle, &USBD_CDC) != USBD_OK) {
        for (;;); // TODO: handle
    }
    if (USBD_CDC_RegisterInterface(&usb_dev_handle, &USBD_Interface_fops) != USBD_OK) {
        for (;;); // TODO: handle
    }
    if (USBD_Start(&usb_dev_handle) != USBD_OK) {
        for (;;); // TODO: handle
    }

    HAL_PWREx_EnableUSBVoltageDetector();

    // Start USB communication thread
    osThreadDef(usb_server_thread_def, usb_server_thread, osPriorityNormal, 0, stack_size_usb_thread / sizeof(StackType_t));
    usb_thread = osThreadCreate(osThread(usb_server_thread_def), NULL);
}
