#ifndef __USB_HPP
#define __USB_HPP

#include <subscriber.hpp>

#include <stdint.h>
#include <stdlib.h>

#include "usbd_def.h" // TODO: get rid of this. Only needed for align macros

#define USB_CONFIG_DESC_SIZ                         128 // expected size: 67 + 39 TODO: make automatic using templates

class USBClass_t;
class USBVendorRequestHandler_t;

class USB_t {
public:
    enum STATUS {
        OK,
        BUSY,
        FAIL
    };
    struct SetupRequest_t {
        uint8_t request_type;
        uint8_t request;
        uint16_t value;
        uint16_t index;
        const uint8_t* data;
        size_t length;
    };

    bool register_device_class(USBClass_t* device_class) {
        if (device_class_) {
            return false;
        }
        if (!device_class) {
            return false;
        }

        device_class_ = device_class;
        return true;
    }

    /**
     * @brief Shall be invoked by the USB driver for handling USB Setup Requests
     * that are not directly handled by the driver.
     * 
     * If the request had a Host => Device data phase, the data field is
     * non-NULL and the length indicates the length of the data. Otherwise
     * the data field is NULL.
     * 
     * If the request has a Device => Host data phase, the length field indicates
     * the requested data length. In this case the data field shall be set to 
     * the data to be sent and the length field shall be updated in case the
     * data is smaller than requested.
     */
    USB_t::STATUS handle_setup_request(SetupRequest_t* req);

    bool add_vendor_request_handler(USBVendorRequestHandler_t* handler) {
        if (n_vendor_request_handlers_ < sizeof(vendor_request_handlers_) / sizeof(vendor_request_handlers_[0])) {
            vendor_request_handlers_[n_vendor_request_handlers_++] = handler;
            return true;
        } else {
            return false;
        }
    }


    uint8_t* get_config_descriptor(uint16_t *length); // currently only one configuration is supported

protected:

    USBClass_t* device_class_ = nullptr;

private:
    /* USB Configuration Descriptor */
    __ALIGN_BEGIN uint8_t cfg_desc_[USB_CONFIG_DESC_SIZ] __ALIGN_END;

    USBVendorRequestHandler_t* vendor_request_handlers_[5];
    size_t n_vendor_request_handlers_ = 0;
};

class USBEndpoint_t {
public:
    USBEndpoint_t(uint8_t endpoint_type, uint8_t endpoint_id, uint16_t max_packet_size) :
        endpoint_type_(endpoint_type),
        endpoint_id_(endpoint_id),
        max_packet_size_(max_packet_size) {}

    bool get_descriptor(uint8_t* buf, size_t len, size_t* n_written) {
        const uint8_t descriptor[] = {
            0x07,   /* bLength: Endpoint Descriptor size */
            USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType: Endpoint */
            endpoint_id_,               /* bEndpointAddress */
            endpoint_type_,             /* bmAttributes: Bulk */
            LOBYTE(max_packet_size_),   /* wMaxPacketSize: */
            HIBYTE(max_packet_size_),
            0x00                        /* bInterval: ignore for Bulk transfer */
        };
        if (buf && len >= sizeof(descriptor)) {
            memcpy(buf, descriptor, sizeof(descriptor));
            if (n_written) {
                *n_written += sizeof(descriptor);
            }
            return true;
        } else {
            return false;
        }
    }

    uint8_t endpoint_type_;
    uint8_t endpoint_id_;
    uint16_t max_packet_size_;

    /** @brief Will be invoked after the low level resources of the endpoint are
     * initialized (usually every time the cable is plugged in) */
    Subscriber<> on_init_;

    /** @brief Will be invoked right before the low level resources of the
     * endpoint are deinitialized (usually every time the cable is pulled out) */
    Subscriber<> on_deinit_;
};

class USBTxEndpoint_t : public virtual USBEndpoint_t {
public:
    virtual USB_t::STATUS start_tx(const uint8_t* buffer, size_t length) = 0;
    Subscriber<> on_tx_complete_;
};

class USBRxEndpoint_t : public virtual USBEndpoint_t {
public:
    virtual USB_t::STATUS start_rx(uint8_t* buffer, size_t length) = 0;
    Subscriber<size_t> on_rx_complete_;
};

class USBIntEndpoint_t : public virtual USBEndpoint_t {
public:
    //USBIntEndpoint_t() {}
    virtual void function_that_prevents_stupid_compiler_error() = 0;
};

class USBClass_t {
public:
    //USBClass_t() {}
    //~USBClass_t() {}
    
    virtual uint8_t get_class() { return 0x0; /* custom */ }
    virtual uint8_t get_sub_class() { return 0x0; /* custom */ }
    virtual uint8_t get_protocol() { return 0x0; /* custom */ }
    virtual uint8_t get_interface_count() = 0;
    virtual uint8_t get_func_str_idx() = 0;

    virtual bool get_interface_descriptors(uint8_t* buf, size_t max_len, size_t* n_written, uint8_t interface_id) = 0;

    /**
     * @brief Invoked when a setup request with type "class" comes in.
     * @param req: See USB_t::handle_setup_request() for details.
     */
    virtual USB_t::STATUS handle_setup_request(USB_t::SetupRequest_t* req) = 0;
};

class USBCompositeDevice : public USBClass_t {
public:
    bool register_class(USBClass_t* device_class) {
        if (n_classes_ < sizeof(classes_) / sizeof(classes_[0])) {
            classes_[n_classes_++] = device_class;
            return true;
        } else {
            return false;
        }
    }

    uint8_t get_class() final { return 0xEF; }
    uint8_t get_sub_class() final { return 0x02; }
    uint8_t get_protocol() final { return 0x01; }
    uint8_t get_interface_count() final;
    uint8_t get_func_str_idx() final { return 0x00; /* not currently used for composite device */ }

    bool get_interface_descriptors(uint8_t* buf, size_t len, size_t* n_written, uint8_t interface_id) final;
    USB_t::STATUS handle_setup_request(USB_t::SetupRequest_t* req) final;

private:
    USBClass_t* classes_[5] = { nullptr }; // TODO: make size not arbitrary
    size_t n_classes_ = 0;
};

struct USBVendorRequestHandler_t {
    using handler_t = USB_t::STATUS (*)(USB_t::SetupRequest_t* req);

    uint16_t vendor_id_;
    handler_t handler_;
};

#endif // __USB_HPP