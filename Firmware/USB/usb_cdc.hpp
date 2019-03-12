#ifndef __USB_CDC_HPP
#define __USB_CDC_HPP

#include  "usbd_ioreq.h"

#include <stm32_usb.hpp>

//#define CDC_IN_EP                                   0x81  
//#define CDC_OUT_EP                                  0x01  
//#define CDC_CMD_EP                                  0x82  
//#define ODRIVE_IN_EP                                0x83  
//#define ODRIVE_OUT_EP                               0x03  

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_DATA_HS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */







/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define USB_RX_DATA_SIZE  64
#define USB_TX_DATA_SIZE  64


class USB_CDC_t : public USBClass_t {
public:
    USB_CDC_t(USBIntEndpoint_t* cmd_endpoint, USBTxEndpoint_t* tx_endpoint, USBRxEndpoint_t* rx_endpoint) :
        cmd_endpoint_(cmd_endpoint),
        tx_endpoint_(tx_endpoint),
        rx_endpoint_(rx_endpoint) {}

    uint8_t get_class() final { return 0x2; /* bFunctionClass: Communication Interface Class */ }
    uint8_t get_sub_class() final { return 0x2; /* bFunctionSubClass: Abstract Control Model */ }
    uint8_t get_protocol() final { return 0x1; /* bFunctionProtocol: Common AT commands */ }
    uint8_t get_interface_count() final { return 0x02; }
    uint8_t get_func_str_idx() final { return USBD_IDX_LANGID_STR /* why this ID? makes no sense */; }

    bool get_interface_descriptors(uint8_t* buf, size_t max_len, size_t* n_written, uint8_t interface_id) final;
    USB_t::STATUS handle_setup_request(USB_t::SetupRequest_t* req) final;

    USB_t::STATUS start_tx(uint8_t* buf, uint16_t len);
    USB_t::STATUS start_rx(uint8_t* buf, uint16_t len);

    USBIntEndpoint_t* cmd_endpoint_;
    USBTxEndpoint_t* tx_endpoint_;
    USBRxEndpoint_t* rx_endpoint_;
};

class USB_ODriveNativeClass_t : public USBClass_t {
public:
    USB_ODriveNativeClass_t(USBTxEndpoint_t* tx_endpoint, USBRxEndpoint_t* rx_endpoint) :
        tx_endpoint_(tx_endpoint),
        rx_endpoint_(rx_endpoint) {}

    uint8_t get_interface_count() final { return 0x01; }
    uint8_t get_func_str_idx() final { return USBD_IDX_ODRIVE_INTF_STR; }

    bool get_interface_descriptors(uint8_t* buf, size_t max_len, size_t* n_written, uint8_t interface_id) final;

    USB_t::STATUS handle_setup_request(USB_t::SetupRequest_t* req) { return USB_t::FAIL; /* no setup requests supported on this class */ };

    USBTxEndpoint_t* tx_endpoint_;
    USBRxEndpoint_t* rx_endpoint_;
};


#endif // __USB_CDC_HPP