
#include "stm32_usb.hpp"

#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include <usb_cdc.hpp>
#include <algorithm>

extern PCD_HandleTypeDef hpcd_USB_OTG_FS; // todo: make this a member of the USB object

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t device_qualifier_descriptor_[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

static uint8_t* get_config_descriptor_static(uint16_t* length) {
    USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef *)hpcd_USB_OTG_FS.pData;
    if (pdev && pdev->pUserData) {
        return ((STM32_USB_t*)pdev->pUserData)->get_config_descriptor(length);
    } else {
        return nullptr;
    }
}

static void deferred_interrupt_thread(void* ctx) {
    STM32_USB_t* obj = (STM32_USB_t*)ctx;
    if (!obj)
        return;

    for (;;) {
        // Wait for signalling from USB interrupt (OTG_FS_IRQHandler)
        osStatus semaphore_status = osSemaphoreWait(obj->sem_irq, osWaitForever);
        if (semaphore_status == osOK) {
            // We have a new incoming USB transmission: handle it
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
            // Let the irq (OTG_FS_IRQHandler) fire again.
            HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        }
    }
}

typedef uint8_t* (*descriptor_func_t)(USBD_SpeedTypeDef speed, uint16_t *length);

template<size_t N, uint8_t (STM32_USB_t::*field)[N]>
descriptor_func_t make_descriptor_func() {
    return [](USBD_SpeedTypeDef speed, uint16_t *length) {
        if (length) {
            *length = N;
        }
        return usb.*field;
    };
}
template<const char* STM32_USB_t::*field>
descriptor_func_t make_descriptor_func() {
    return [](USBD_SpeedTypeDef speed, uint16_t *length) {
        USBD_GetString((uint8_t *)(usb.*field), usb.str_buffer, length);
        return usb.str_buffer;
    };
}

/**
* @brief  Called by USB-CDC driver to fetch non-standard string descriptors
*/
// TODO: this is a relic, make a setup function to register custom strings
uint8_t* USBD_UsrStrDescriptor(struct _USBD_HandleTypeDef *pdev, uint8_t index,  uint16_t *length) {
    if (USBD_IDX_MICROSOFT_DESC_STR == index) {
        return make_descriptor_func<sizeof(STM32_USB_t::microsoft_os_descriptor_), &STM32_USB_t::microsoft_os_descriptor_>()(USBD_SPEED_FULL, length);
    } else if (USBD_IDX_ODRIVE_INTF_STR == index) {
        return make_descriptor_func<&STM32_USB_t::native_interface_string_>()(USBD_SPEED_FULL, length);
    } else {
        if (length) {
           *length = 0;
        }
        return nullptr;
    }
    return NULL;
}

bool STM32_USB_t::init(
        uint16_t vid, uint16_t pid, uint16_t langid,
        const char* manufacturer_string, const char* product_string, const char* serial_string, const char* config_string, const char* interface_string, const char* native_interface_string,
        USBClass_t* device_class) {
    if (!USB_t::register_device_class(device_class)) {
        return false;
    }
    if (!device_class_)
        return false;

    // Init usb irq binary semaphore, and start with no tokens by removing the starting one.
    osSemaphoreDef(sem_irq);
    sem_irq = osSemaphoreCreate(osSemaphore(sem_irq), 1);
    osSemaphoreWait(sem_irq, 0);

    // Start USB interrupt handler thread
    osThreadDef(task_usb_pump, deferred_interrupt_thread, osPriorityAboveNormal, 0, 1024);
    irq_thread = osThreadCreate(osThread(task_usb_pump), this);

    uint8_t device_descriptor[] = {
        0x12,                       /*bLength */
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
        device_class_->get_class(),     /*bDeviceClass*/
        device_class_->get_sub_class(), /*bDeviceSubClass*/
        device_class_->get_protocol(),  /*bDeviceProtocol*/
        USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
        LOBYTE(vid),                /*idVendor*/
        HIBYTE(vid),                /*idVendor*/
        LOBYTE(pid),                /*idProduct*/
        HIBYTE(pid),                /*idProduct*/
        0x00,                       /*bcdDevice rel. 2.00*/
        0x03,                       /* bNumInterfaces */
        USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
        USBD_IDX_PRODUCT_STR,       /*Index of product string*/
        USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
        USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
    };
    memcpy(device_descriptor_, device_descriptor, sizeof(device_descriptor_));

    uint8_t langid_descriptor[] = {
        USB_LEN_LANGID_STR_DESC,
        USB_DESC_TYPE_STRING,
        LOBYTE(langid),
        HIBYTE(langid)
    };
    memcpy(langid_descriptor_, langid_descriptor, sizeof(langid_descriptor_));

    manufacturer_string_ = manufacturer_string;
    product_string_ = product_string;
    serial_string_ = serial_string;
    config_string_ = config_string;
    interface_string_ = interface_string;
    native_interface_string_ = native_interface_string;


    // Init Device Library, add supported class and start the library.
    hUsbDeviceFS.pUserData = this;
    hUsbDeviceFS.pClassData = device_class_;
    USBD_Init(&hUsbDeviceFS, &usb_desc, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, &usb_class);
    USBD_Start(&hUsbDeviceFS);

    return true;
}

USBD_StatusTypeDef STM32_USB_t::handle_init(uint8_t cfgidx) {
    for (size_t i = 0; i < n_endpoints_; ++i) {
        if (!(endpoints_[i] && endpoints_[i]->init())) {
            return USBD_FAIL;
        }
    }
    return USBD_OK;
}

USBD_StatusTypeDef STM32_USB_t::handle_deinit(uint8_t cfgidx) {
    bool result = true;
    for (size_t i = 0; i < n_endpoints_; ++i) {
        if (!(endpoints_[i] && endpoints_[i]->deinit())) {
            result = false;
        }
    }
    return result ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef STM32_USB_t::handle_setup_request(USBD_SetupReqTypedef *req) {
    USB_t::STATUS result;

    req_ = {
        .request_type = req->bmRequest,
        .request = req->bRequest,
        .value = req->wValue,
        .index = req->wIndex,
        .data = nullptr,
        .length = req->wLength
    };

    if (req->wLength) {
        // Data phase will follow
        if (req->bmRequest & 0x80) {
            // Data phase: Device => Host
            size_t requested_length = req->wLength;
            result = USB_t::handle_setup_request(&req_);
            if (!req_.data) {
                result = USB_t::FAIL;
            }
            if (result == USB_t::OK) {
                result = convert_status(USBD_CtlSendData(&hUsbDeviceFS, (uint8_t*)req_.data, std::min(req_.length, (size_t)requested_length)));
            }
        } else {
            // Data phase: Host => Device
            req_.length = std::min(req_.length, sizeof(ctl_req_data_));
            result = convert_status(USBD_CtlPrepareRx(&hUsbDeviceFS, ctl_req_data_, req_.length));
        }
    } else {
        // Device => Host or Host => Device
        result = USB_t::handle_setup_request(&req_);
    }

    if (result == USB_t::FAIL) {
        USBD_CtlError(&hUsbDeviceFS, req);
    }

    return convert_status(result);
}

USBD_StatusTypeDef STM32_USB_t::handle_ep0_rx_complete() {    
    USB_t::STATUS result;
    if (req_.request == 0xFF) {
        result = USB_t::FAIL;
    } else {
        result = USB_t::handle_setup_request(&req_);
        req_.request = 0xFF;
    }

    //if (result == USB_t::FAIL) {
    //    USBD_CtlError(&hUsbDeviceFS, req);
    //}
    
    return convert_status(result);
}

bool STM32_USB_t::handle_data_out_complete(uint8_t epnum) {
    for (size_t i = 0; i < n_rx_endpoints_; ++i) {
        if (rx_endpoints_[i] && rx_endpoints_[i]->endpoint_id_ == epnum) {
            return rx_endpoints_[i]->handle_rx_complete();
        }
    }
    return false; // endpoint not found
}

bool STM32_USB_t::handle_data_in_complete(uint8_t epnum) {
    // NOTE: We would logically expect xx_IN_EP here, but we actually get the xx_OUT_EP
    epnum |= 0x80;
    
    for (size_t i = 0; i < n_tx_endpoints_; ++i) {
        if (tx_endpoints_[i] && tx_endpoints_[i]->endpoint_id_ == epnum) {
            return tx_endpoints_[i]->handle_tx_complete();
        }
    }
    return false; // endpoint not found
}


/* callbacks for providing descriptors */
USBD_DescriptorsTypeDef STM32_USB_t::usb_desc = {
    make_descriptor_func<sizeof(STM32_USB_t::device_descriptor_), &STM32_USB_t::device_descriptor_>(),
    make_descriptor_func<sizeof(STM32_USB_t::langid_descriptor_), &STM32_USB_t::langid_descriptor_>(),
    make_descriptor_func<&STM32_USB_t::manufacturer_string_>(),
    make_descriptor_func<&STM32_USB_t::product_string_>(),
    make_descriptor_func<&STM32_USB_t::serial_string_>(),
    make_descriptor_func<&STM32_USB_t::config_string_>(),
    make_descriptor_func<&STM32_USB_t::interface_string_>(),
#if (USBD_LPM_ENABLED == 1)
    USBD_FS_USR_BOSDescriptor, // TODO
#endif /* (USBD_LPM_ENABLED == 1) */
};

/* callbacks for the USB driver */
USBD_ClassTypeDef STM32_USB_t::usb_class = {
    [](USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
        USBD_StatusTypeDef status = USBD_FAIL;
        if (pdev && pdev->pUserData) {
            status = ((STM32_USB_t*)pdev->pUserData)->handle_init(cfgidx);
        }
        return (uint8_t)status;
    },
    [](USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
        USBD_StatusTypeDef status = USBD_FAIL;
        if (pdev && pdev->pUserData) {
            status = ((STM32_USB_t*)pdev->pUserData)->handle_deinit(cfgidx);
        }
        return (uint8_t)status;
    },
    [](USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
        USBD_StatusTypeDef status = USBD_FAIL;
        if (pdev && pdev->pUserData) {
            status = ((STM32_USB_t*)pdev->pUserData)->handle_setup_request(req);
        }
        return (uint8_t)status;
    },
    nullptr, // EP0 TX Complete
    [](USBD_HandleTypeDef *pdev) { 
        USBD_StatusTypeDef status = USBD_FAIL;
        if (pdev && pdev->pUserData) {
            status = ((STM32_USB_t*)pdev->pUserData)->handle_ep0_rx_complete();
        }
        return (uint8_t)status;
    },
    [](USBD_HandleTypeDef *pdev, uint8_t epnum) {
        USBD_StatusTypeDef status = USBD_FAIL;
        if (pdev && pdev->pUserData) {
            status = ((STM32_USB_t*)pdev->pUserData)->handle_data_in_complete(epnum) ? USBD_OK : USBD_FAIL;
        }
        return (uint8_t)status;
    },
    [](USBD_HandleTypeDef *pdev, uint8_t epnum) {
        USBD_StatusTypeDef status = USBD_FAIL;
        if (pdev && pdev->pUserData) {
            status = ((STM32_USB_t*)pdev->pUserData)->handle_data_out_complete(epnum) ? USBD_OK : USBD_FAIL;
        }
        return (uint8_t)status;
    },
    nullptr, // SOF
    nullptr, // IsoINIncomplete
    nullptr, // IsoOUTIncomplete
    get_config_descriptor_static,
    get_config_descriptor_static,
    get_config_descriptor_static,
    [](uint16_t* length) {
        if (length) {
            *length = sizeof(device_qualifier_descriptor_);
        }
        return device_qualifier_descriptor_;
    },
    USBD_UsrStrDescriptor
};


/* Interrupt entrypoints -----------------------------------------------------*/

extern "C" {
void OTG_FS_IRQHandler(void);
}

/** @brief Entrypoint for the USB On The Go FS global interrupt. */
void OTG_FS_IRQHandler(void) {
    // Usually HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS) would be called here. But
    // instead we mask the interrupt and signal processing of interrupt by
    // deferred_interrupt_thread.
    // The thread will re-enable the interrupt when all pending irqs are clear.
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    osSemaphoreRelease(usb.sem_irq);
    //HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}


/* Peripheral definitions ----------------------------------------------------*/

STM32_USB_t usb;
