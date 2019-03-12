
#include "usb.hpp"
#include "usbd_def.h"
#include <algorithm>

USB_t::STATUS USB_t::handle_setup_request(SetupRequest_t* req) {
    static const uint8_t ifalt[] = { 0 };

    if (!req) {
        return USB_t::FAIL;
    }

    switch (req->request_type & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
        if (device_class_) {
            return device_class_->handle_setup_request(req);
        } else {
            return USB_t::FAIL;
        }

    case USB_REQ_TYPE_STANDARD:
        if (req->request == USB_REQ_GET_INTERFACE) {
            req->data = ifalt;
            req->length = std::min(req->length, sizeof(ifalt));
            return USB_t::OK;
        } else if (req->request == USB_REQ_GET_INTERFACE) {
            return USB_t::OK;
        } else {
            return USB_t::FAIL;
        }

    case USB_REQ_TYPE_VENDOR:
        for (size_t i = 0; i < n_vendor_request_handlers_; ++i) {
            if (vendor_request_handlers_[i] && vendor_request_handlers_[i]->vendor_id_ == req->request && vendor_request_handlers_[i]->handler_) {
                return vendor_request_handlers_[i]->handler_(req);
            }
        }
        return USB_t::FAIL;
    
    default: 
        return USB_t::FAIL;
    }
}

uint8_t* USB_t::get_config_descriptor(uint16_t *length) {
    static const uint8_t cgf_desc_header[] = {
        /* Configuration Descriptor */
        0x09,   /* bLength: Configuration Descriptor size */
        USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
        0x00,   /* wTotalLength:no of returned bytes (overridden at runtime) */
        0x00,
        0x00,   /* bNumInterfaces (overridden at runtime) */
        0x01,   /* bConfigurationValue: Configuration value */
        0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
        0xC0,   /* bmAttributes: self powered */
        0x32,   /* MaxPower 0 mA */
    };

    uint8_t* buf = cfg_desc_;
    size_t max_len = sizeof(cfg_desc_);
    size_t offset = 0;

    if (offset + sizeof(cgf_desc_header) > max_len) {
        return nullptr;
    }
    memcpy(buf + offset, cgf_desc_header, sizeof(cgf_desc_header));
    offset += sizeof(cgf_desc_header);

    if (!device_class_) {
        return nullptr;
    }
    device_class_->get_interface_descriptors(buf + offset, max_len - offset, &offset, 0);

    buf[2] = offset;
    buf[4] = device_class_->get_interface_count();

    if (length)
        *length = offset;
    return buf;
}


uint8_t USBCompositeDevice::get_interface_count() {
    uint8_t count = 0;
    for (size_t i = 0; i < n_classes_; ++i) {
        if (classes_[i]) {
            count += classes_[i]->get_interface_count();
        }
    }
    return count;
}

bool USBCompositeDevice::get_interface_descriptors(uint8_t* buf, size_t max_len, size_t* n_written, uint8_t interface_id) {
    size_t offset = 0;

    for (size_t i = 0; i < n_classes_; ++i) {
        if (!classes_[i]) {
            return false;
        }

        const uint8_t intf_asoc_desc[] = {
            /* Interface Association Descriptor */
            0x08,                               /* bLength: IAD size */
            0x0B,                               /* bDescriptorType: Interface Association Descriptor */
            interface_id,                       /* bFirstInterface */
            classes_[i]->get_interface_count(), /* bInterfaceCount */
            classes_[i]->get_class(),           /* bFunctionClass */
            classes_[i]->get_sub_class(),        /* bFunctionSubClass */
            classes_[i]->get_protocol(),        /* bFunctionProtocol */
            classes_[i]->get_func_str_idx(),    /* iFunction */
        };
        
        if (offset + sizeof(intf_asoc_desc) > max_len) {
            return false;
        }
        memcpy(buf + offset, intf_asoc_desc, sizeof(intf_asoc_desc));
        offset += sizeof(intf_asoc_desc);

        if (offset > max_len) {
            return false;
        }
        if (!classes_[i]->get_interface_descriptors(buf + offset, max_len - offset, &offset, interface_id)) {
            return false;
        }

        interface_id += classes_[i]->get_interface_count();
    }

    if (n_written) {
        *n_written += offset;
    }

    return true;
};

USB_t::STATUS USBCompositeDevice::handle_setup_request(USB_t::SetupRequest_t* req) {
    // TODO: not sure how to find out at which of the classes this request is directed. Assume class 0 for now.
    if (n_classes_ > 0 && classes_[0]) {
        return classes_[0]->handle_setup_request(req);
    } else {
        return USB_t::FAIL;
    }
}

