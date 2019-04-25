
#include "usb_cdc.hpp"
#include <algorithm>

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23

USB_t::STATUS USB_CDC_t::handle_setup_request(USB_t::SetupRequest_t* req) {
    static const uint8_t line_coding[] = {
        (uint8_t)(115200),
        (uint8_t)(115200 >> 8),
        (uint8_t)(115200 >> 16),
        (uint8_t)(115200 >> 24),
        0,  // stop bits (1)
        0,  // parity (none)
        8   // number of bits (8)
    };

    if (!req) {
        return USB_t::FAIL;
    }

    switch (req->request) {
    case CDC_SEND_ENCAPSULATED_COMMAND: return USB_t::OK;
    case CDC_GET_ENCAPSULATED_RESPONSE: return USB_t::OK;
    case CDC_SET_COMM_FEATURE: return USB_t::OK;
    case CDC_GET_COMM_FEATURE: return USB_t::OK;
    case CDC_CLEAR_COMM_FEATURE: return USB_t::OK;

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
    case CDC_SET_LINE_CODING: return USB_t::OK;

    case CDC_GET_LINE_CODING:
        req->data = line_coding;
        req->length = std::min(req->length, sizeof(line_coding));
        return USB_t::OK;

    case CDC_SET_CONTROL_LINE_STATE: return USB_t::OK;

    case CDC_SEND_BREAK: return USB_t::OK;
    default: return USB_t::OK;
    }
}

USB_t::STATUS USB_CDC_t::start_tx(uint8_t* buf, uint16_t len) {
    if (tx_endpoint_) {
        return tx_endpoint_->start_tx(buf, len);
    } else {
        return USB_t::FAIL;
    }
}

USB_t::STATUS USB_CDC_t::start_rx(uint8_t* buf, uint16_t len) {
    if (rx_endpoint_) {
        return rx_endpoint_->start_rx(buf, len);
    } else {
        return USB_t::FAIL;
    }
}


bool USB_CDC_t::get_interface_descriptors(uint8_t* buf, size_t max_len, size_t* n_written, uint8_t interface_id) {
    static const uint8_t intf_desc_0[] = {
        /* Interface Descriptor */
        0x09,   /* bLength: Interface Descriptor size */
        USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
        /* Interface descriptor type */
        0x00,   /* bInterfaceNumber: Number of Interface (overridden at runtime) */
        0x00,   /* bAlternateSetting: Alternate setting */
        0x01,   /* bNumEndpoints: One endpoints used */
        0x02,   /* bInterfaceClass: Communication Interface Class */
        0x02,   /* bInterfaceSubClass: Abstract Control Model */
        0x01,   /* bInterfaceProtocol: Common AT commands */
        0x00,   /* iInterface: */

        /* Header Functional Descriptor */
        0x05,   /* bLength: Endpoint Descriptor size */
        0x24,   /* bDescriptorType: CS_INTERFACE */
        0x00,   /* bDescriptorSubtype: Header Func Desc */
        0x10,   /* bcdCDC: spec release number */
        0x01,
        
        /* Call Management Functional Descriptor */
        0x05,   /* bFunctionLength */
        0x24,   /* bDescriptorType: CS_INTERFACE */
        0x01,   /* bDescriptorSubtype: Call Management Func Desc */
        0x00,   /* bmCapabilities: D0+D1 */
        0x01,   /* bDataInterface: 1 */
        
        /* ACM Functional Descriptor */
        0x04,   /* bFunctionLength */
        0x24,   /* bDescriptorType: CS_INTERFACE */
        0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
        0x02,   /* bmCapabilities */
        
        /* Union Functional Descriptor */
        0x05,   /* bFunctionLength */
        0x24,   /* bDescriptorType: CS_INTERFACE */
        0x06,   /* bDescriptorSubtype: Union func desc */
        0x00,   /* bMasterInterface: Communication class interface */
        0x01   /* bSlaveInterface0: Data Class Interface */
    };

    static const uint8_t intf_desc_1[] = {
        /* Data class interface descriptor */
        0x09,   /* bLength: Endpoint Descriptor size */
        USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
        0x00,   /* bInterfaceNumber: Number of Interface (overridden at runtime) */
        0x00,   /* bAlternateSetting: Alternate setting */
        0x02,   /* bNumEndpoints: Two endpoints used */
        0x0A,   /* bInterfaceClass: CDC */
        0x00,   /* bInterfaceSubClass: */
        0x00,   /* bInterfaceProtocol: */
        0x00,   /* iInterface: */
    };

    size_t offset = 0;

    if (offset + sizeof(intf_desc_0) > max_len) {
        return false;
    }
    memcpy(buf + offset, intf_desc_0, sizeof(intf_desc_0));
    buf[2] = interface_id++; // override bInterfaceNumber
    offset += sizeof(intf_desc_0);

    if (offset > max_len) {
        return false;
    }
    if (!cmd_endpoint_->get_descriptor(buf + offset, max_len - offset, &offset)) {
        return false;
    }

    if (offset + sizeof(intf_desc_1) > max_len) {
        return false;
    }
    memcpy(buf + offset, intf_desc_1, sizeof(intf_desc_1));
    buf[2] = interface_id++; // override bInterfaceNumber
    offset += sizeof(intf_desc_1);

    if (offset > max_len) {
        return false;
    }
    if (!rx_endpoint_->get_descriptor(buf + offset, max_len - offset, &offset)) {
        return false;
    }
    if (offset > max_len) {
        return false;
    }
    if (!tx_endpoint_->get_descriptor(buf + offset, max_len - offset, &offset)) {
        return false;
    }

    if (n_written) {
        *n_written += offset;
    }

    return true;
}



bool USB_ODriveNativeClass_t::get_interface_descriptors(uint8_t* buf, size_t max_len, size_t* n_written, uint8_t interface_id) {
    static const uint8_t intf_desc_0[] = {
        /* Data class interface descriptor */
        0x09,   /* bLength: Endpoint Descriptor size */
        USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
        0x00,   /* bInterfaceNumber: Number of Interface (overridden at runtime) */
        0x00,   /* bAlternateSetting: Alternate setting */
        0x02,   /* bNumEndpoints: Two endpoints used */
        0x00,   /* bInterfaceClass: CDC */
        0x01,   /* bInterfaceSubClass: */
        0x00,   /* bInterfaceProtocol: */
        0x00,   /* iInterface: */
    };

    size_t offset = 0;

    if (offset + sizeof(intf_desc_0) > max_len) {
        return false;
    }
    memcpy(buf + offset, intf_desc_0, sizeof(intf_desc_0));
    buf[2] = interface_id++; // override bInterfaceNumber
    offset += sizeof(intf_desc_0);

    if (offset > max_len) {
        return false;
    }
    if (!rx_endpoint_->get_descriptor(buf + offset, max_len - offset, &offset)) {
        return false;
    }
    if (offset > max_len) {
        return false;
    }
    if (!tx_endpoint_->get_descriptor(buf + offset, max_len - offset, &offset)) {
        return false;
    }

    if (n_written) {
        *n_written += offset;
    }

    return true;
}
