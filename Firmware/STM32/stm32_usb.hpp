#ifndef __STM32_USB_HPP
#define __STM32_USB_HPP

#include "usbd_def.h"
#include "usbd_core.h"
#include "cmsis_os.h"

#include <usb.hpp>

// TODO: these are CDC dependent, move them
#define CDC_DATA_HS_MAX_PACKET_SIZE 64  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE 64  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */ 
#define CTL_DATA_SIZE 64
#define USB_TX_DATA_SIZE 64


static inline USBD_StatusTypeDef convert_status(USB_t::STATUS status) {
    switch (status) {
        case USB_t::OK: return USBD_OK;
        case USB_t::BUSY: return USBD_BUSY;
        case USB_t::FAIL: return USBD_FAIL;
        default: return USBD_FAIL;
    }
}

static inline USB_t::STATUS convert_status(USBD_StatusTypeDef status) {
    switch (status) {
        case USBD_OK: return USB_t::OK;
        case USBD_BUSY: return USB_t::BUSY;
        case USBD_FAIL: return USB_t::FAIL;
        default: return USB_t::FAIL;
    }
}

class STM32_USBEndpoint_t : public virtual USBEndpoint_t {
public:
    STM32_USBEndpoint_t(USBD_HandleTypeDef *pdev) :
        USBEndpoint_t(0, 0, 0), // constructor ignored because of virtual inheritance
        pdev(pdev) {}

    bool init() {
        // Open endpoint
        if (USBD_LL_OpenEP(pdev, endpoint_id_, endpoint_type_, max_packet_size_) != USBD_OK) {
            return false;
        }
        on_init_.invoke();
        return true;
    }

    bool deinit() {
        on_deinit_.invoke();
        return USBD_LL_CloseEP(pdev, endpoint_id_) == USBD_OK;
    }

    USBD_HandleTypeDef *pdev;
};

class STM32_USBTxEndpoint_t : public STM32_USBEndpoint_t, public USBTxEndpoint_t {
public:
    STM32_USBTxEndpoint_t(USBD_HandleTypeDef *pdev, uint8_t endpoint_id) :
        USBEndpoint_t(USBD_EP_TYPE_BULK, endpoint_id, CDC_DATA_FS_IN_PACKET_SIZE), // TODO: distinguish HS and FS packet size
        STM32_USBEndpoint_t(pdev) {}

    bool init() {
        state_ = 0;
        return STM32_USBEndpoint_t::init();
    }

    USB_t::STATUS start_tx(const uint8_t* buffer, size_t length) final {
        // Check length
        if (length > USB_TX_DATA_SIZE) // TODO: this is artificial - probably can do more
            return USB_t::FAIL;

        if (state_ == 0) {
            /* Tx Transfer in progress */
            state_ = 1;
            
            /* Transmit next packet */
            return convert_status(USBD_LL_Transmit(pdev, endpoint_id_,
                    const_cast<uint8_t*>(buffer) /* casting this const away is safe because...
                    well... it's not actually. Stupid STM. */, length));
        } else {
            return USB_t::BUSY;
        }
    }

    bool handle_tx_complete() {
        state_ = 0;
        on_tx_complete_.invoke();
        return true;
    }

private:
    volatile uint8_t state_;
};

class STM32_USBRxEndpoint_t : public STM32_USBEndpoint_t, public USBRxEndpoint_t {
public:
    STM32_USBRxEndpoint_t(USBD_HandleTypeDef *pdev, uint8_t endpoint_id) :
        USBEndpoint_t(USBD_EP_TYPE_BULK, endpoint_id, CDC_DATA_FS_OUT_PACKET_SIZE),
        STM32_USBEndpoint_t(pdev) {}

    bool init() {
        state_ = 0;
        return STM32_USBEndpoint_t::init();
    }

    USB_t::STATUS start_rx(uint8_t* buffer, size_t length) final {
        return convert_status(USBD_LL_PrepareReceive(pdev, endpoint_id_, buffer, length));
    }

    bool handle_rx_complete() {
        /* Get the received data length */
        size_t length = USBD_LL_GetRxDataSize(pdev, endpoint_id_);
        on_rx_complete_.invoke(length);
        return true;
    }

private:
    volatile uint8_t state_;
};

class STM32_USBIntEndpoint_t : public STM32_USBEndpoint_t, public USBIntEndpoint_t {
public:
    STM32_USBIntEndpoint_t(USBD_HandleTypeDef *pdev, uint8_t endpoint_id) :
        USBEndpoint_t(USBD_EP_TYPE_INTR, endpoint_id, CDC_CMD_PACKET_SIZE),
        STM32_USBEndpoint_t(pdev) {}

    bool init() {
        return STM32_USBEndpoint_t::init();
    }

    void function_that_prevents_stupid_compiler_error() final {}
};

class STM32_USB_t : public USB_t {
public:
    USBD_HandleTypeDef hUsbDeviceFS;

    SetupRequest_t req_ = { .request = 0xff };
    __ALIGN_BEGIN uint8_t ctl_req_data_[CTL_DATA_SIZE] __ALIGN_END;
    
    __ALIGN_BEGIN uint8_t str_buffer[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

    // USB standard device descriptor (assigned in init())
    __ALIGN_BEGIN uint8_t device_descriptor_[USB_LEN_DEV_DESC] __ALIGN_END;

    // USB lang indentifier descriptor (assigned in init())
    __ALIGN_BEGIN uint8_t langid_descriptor_[USB_LEN_LANGID_STR_DESC] __ALIGN_END;

    // MS OS String descriptor to tell Windows that it may query for other descriptors
    // It's a standard string descriptor.
    // Windows will only query for OS descriptors once!
    // Delete the information about already queried devices in registry by deleting:
    // HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\usbflags\VVVVPPPPRRRR
    __ALIGN_BEGIN uint8_t microsoft_os_descriptor_[18] __ALIGN_END = {
        0x12,           //  bLength           1 0x12  Length of the descriptor
        0x03,           //  bDescriptorType   1 0x03  Descriptor type
                        //  qwSignature      14 ‘MSFT100’ Signature field
        0x4D, 0x00,     //  'M'
        0x53, 0x00,     //  'S'
        0x46, 0x00,     //  'F'
        0x54, 0x00,     //  'T'
        0x31, 0x00,     //  '1'
        0x30, 0x00,     //  '0'
        0x30, 0x00,     //  '0'
        MS_VendorCode,  //  bMS_VendorCode    1 Vendor-specific Vendor code
        0x00            //  bPad              1 0x00  Pad field
    };


    const char* manufacturer_string_ = nullptr;
    const char* product_string_ = nullptr;
    const char* serial_string_ = nullptr;
    const char* config_string_ = nullptr;
    const char* interface_string_ = nullptr;
    const char* native_interface_string_ = nullptr;

    osSemaphoreId sem_irq;
    osThreadId irq_thread;

    STM32_USB_t() {}

    bool init(
        uint16_t vid, uint16_t pid, uint16_t langid,
        const char* manufacturer_string, const char* product_string, const char* serial_string, const char* config_string, const char* interface_string, const char* native_interface_string,
        USBClass_t* device_class, osPriority task_priority);
    
    bool enable_interrupts(uint8_t priority);
    bool start();
    
    bool add_int_endpoint(STM32_USBIntEndpoint_t* ep) {
        if (n_int_endpoints_ < sizeof(int_endpoints_) / sizeof(int_endpoints_[0])) {
            int_endpoints_[n_int_endpoints_++] = ep;
            endpoints_[n_endpoints_++] = ep;
            return true;
        } else {
            return false;
        }
    }

    bool add_tx_endpoint(STM32_USBTxEndpoint_t* ep) {
        if (n_tx_endpoints_ < sizeof(tx_endpoints_) / sizeof(tx_endpoints_[0])) {
            tx_endpoints_[n_tx_endpoints_++] = ep;
            endpoints_[n_endpoints_++] = ep;
            return true;
        } else {
            return false;
        }
    }

    bool add_rx_endpoint(STM32_USBRxEndpoint_t* ep) {
        if (n_rx_endpoints_ < sizeof(rx_endpoints_) / sizeof(rx_endpoints_[0])) {
            rx_endpoints_[n_rx_endpoints_++] = ep;
            endpoints_[n_endpoints_++] = ep;
            return true;
        } else {
            return false;
        }
    }

private:
    USBD_StatusTypeDef handle_init(uint8_t cfgidx);
    USBD_StatusTypeDef handle_deinit(uint8_t cfgidx);

    /** @brief Low level handler of the USB setup request */
    USBD_StatusTypeDef handle_setup_request(USBD_SetupReqTypedef *req);

    /** @brief Higher level the USB setup request.
     * See USBClass_t::control_request for a detailed description */
    USB_t::STATUS handle_setup_request(uint8_t request_type, uint8_t request, const uint8_t* rx_data, const uint8_t** tx_data, size_t* length);

    USBD_StatusTypeDef handle_ep0_rx_complete();

    /** @brief Handles completion of an RX operation */
    bool handle_data_out_complete(uint8_t epnum);

    /** @brief Handles completion of a TX operation */
    bool handle_data_in_complete(uint8_t epnum);

    static USBD_DescriptorsTypeDef usb_desc;
    static USBD_ClassTypeDef usb_class;

    STM32_USBIntEndpoint_t* int_endpoints_[5];
    STM32_USBTxEndpoint_t* tx_endpoints_[5];
    STM32_USBRxEndpoint_t* rx_endpoints_[5];
    STM32_USBEndpoint_t* endpoints_[15]; // TODO: simplify
    size_t n_int_endpoints_ = 0;
    size_t n_tx_endpoints_ = 0;
    size_t n_rx_endpoints_ = 0;
    size_t n_endpoints_ = 0;
};



extern STM32_USB_t usb;

#endif // __STM32_USB_HPP