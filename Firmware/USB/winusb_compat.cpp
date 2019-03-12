
#include "winusb_compat.hpp"
#include <algorithm>


// TODO: make runtime configurable
#define NUM_INTERFACES 1

#if NUM_INTERFACES == 2
#define USB_WINUSBCOMM_COMPAT_ID_OS_DESC_SIZ       (16 + 24 + 24)
#else
#define USB_WINUSBCOMM_COMPAT_ID_OS_DESC_SIZ       (16 + 24)
#endif

// This descriptor associates the WinUSB driver with the device
__ALIGN_BEGIN static const uint8_t ms_extended_compat_id_os_desc[USB_WINUSBCOMM_COMPAT_ID_OS_DESC_SIZ] __ALIGN_END = {
                                                    //    +-- Offset in descriptor
                                                    //    |             +-- Size
                                                    //    v             v
  USB_WINUSBCOMM_COMPAT_ID_OS_DESC_SIZ, 0, 0, 0,    //    0 dwLength    4 DWORD The length, in bytes, of the complete extended compat ID descriptor
  0x00, 0x01,                                       //    4 bcdVersion  2 BCD The descriptor’s version number, in binary coded decimal (BCD) format
  0x04, 0x00,                                       //    6 wIndex      2 WORD  An index that identifies the particular OS feature descriptor
  NUM_INTERFACES,                                   //    8 bCount      1 BYTE  The number of custom property sections
  0, 0, 0, 0, 0, 0, 0,                              //    9 RESERVED    7 BYTEs Reserved
                                                    //    =====================
                                                    //                 16

                                                    //   +-- Offset from function section start
                                                    //   |                        +-- Size
                                                    //   v                        v
  2,                                                //   0  bFirstInterfaceNumber 1 BYTE  The interface or function number
  0,                                                //   1  RESERVED              1 BYTE  Reserved
  0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00,   //   2  compatibleID          8 BYTEs The function’s compatible ID      ("WINUSB")
  0, 0, 0, 0, 0, 0, 0, 0,                           //  10  subCompatibleID       8 BYTEs The function’s subcompatible ID
  0, 0, 0, 0, 0, 0,                                 //  18  RESERVED              6 BYTEs Reserved
                                                    //  =================================
                                                    //                           24
#if NUM_INTERFACES == 2
                                                    //   +-- Offset from function section start
                                                    //   |                        +-- Size
                                                    //   v                        v
  2,                                                //   0  bFirstInterfaceNumber 1 BYTE  The interface or function number
  0,                                                //   1  RESERVED              1 BYTE  Reserved
  0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00,   //   2  compatibleID          8 BYTEs The function’s compatible ID      ("WINUSB")
  0, 0, 0, 0, 0, 0, 0, 0,                           //  10  subCompatibleID       8 BYTEs The function’s subcompatible ID
  0, 0, 0, 0, 0, 0,                                 //  18  RESERVED              6 BYTEs Reserved
                                                    //  =================================
                                                    //                           24
#endif
};


// Properties are added to:
// HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\VID_xxxx&PID_xxxx\sssssssss\Device Parameters
// Use USBDeview or similar to uninstall
__ALIGN_BEGIN static const uint8_t ms_extended_properties_os_desc[0xB6] __ALIGN_END =
{
  0xB6, 0x00, 0x00, 0x00,   // 0 dwLength   4 DWORD The length, in bytes, of the complete extended properties descriptor
  0x00, 0x01,               // 4 bcdVersion 2 BCD   The descriptor’s version number, in binary coded decimal (BCD) format
  0x05, 0x00,               // 6 wIndex     2 WORD  The index for extended properties OS descriptors
  0x02, 0x00,               // 8 wCount     2 WORD  The number of custom property sections that follow the header section
                            // ====================
                            //             10
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  0x84, 0x00, 0x00, 0x00,   //  0       dwSize                  4 DWORD             The size of this custom properties section
  0x01, 0x00, 0x00, 0x00,   //  4       dwPropertyDataType      4 DWORD             Property data format
  0x28, 0x00,               //  8       wPropertyNameLength     2 DWORD             Property name length
                            // ========================================
                            //                                 10
                            // 10       bPropertyName         PNL WCHAR[]           The property name
  'D',0, 'e',0, 'v',0, 'i',0, 'c',0, 'e',0, 'I',0, 'n',0,
  't',0, 'e',0, 'r',0, 'f',0, 'a',0, 'c',0, 'e',0, 'G',0,
  'U',0, 'I',0, 'D',0, 0,0,
                            // ========================================
                            //                                 40 (0x28)

  0x4E, 0x00, 0x00, 0x00,   // 10 + PNL dwPropertyDataLength    4 DWORD             Length of the buffer holding the property data
                            // ========================================
                            //                                  4
    // 14 + PNL bPropertyData         PDL Format-dependent  Property data
  '{',0, 'E',0, 'A',0, '0',0, 'B',0, 'D',0, '5',0, 'C',0,
  '3',0, '-',0, '5',0, '0',0, 'F',0, '3',0, '-',0, '4',0,
  '8',0, '8',0, '8',0, '-',0, '8',0, '4',0, 'B',0, '4',0,
  '-',0, '7',0, '4',0, 'E',0, '5',0, '0',0, 'E',0, '1',0,
  '6',0, '4',0, '9',0, 'D',0, 'B',0, '}',0,  0 ,0,
                            // ========================================
                            //                                 78 (0x4E)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  0x3E, 0x00, 0x00, 0x00,   //  0 dwSize 0x00000030 (62 bytes)
  0x01, 0x00, 0x00, 0x00,   //  4 dwPropertyDataType 0x00000001 (Unicode string)
  0x0C, 0x00,               //  8 wPropertyNameLength 0x000C (12 bytes)
                            // ========================================
                            //                                  10
  'L',0, 'a',0, 'b',0, 'e',0, 'l',0, 0,0,
                            // 10 bPropertyName “Label”
                            // ========================================
                            //                                  12
  0x24, 0x00, 0x00, 0x00,   // 22 dwPropertyDataLength 0x00000016 (36 bytes)
                            // ========================================
                            //                                  4
  'O',0, 'D',0, 'r',0, 'i',0, 'v',0, 'e',0, 0,0
                            // 26 bPropertyData “ODrive”
                            // ========================================
                            //                                  14

};

static USB_t::STATUS get_ms_extended_compat_id_os_descriptor(USB_t::SetupRequest_t* req) {
    if (!req) {
        return USB_t::FAIL;
    }

    if (req->index == 0x04) {
        req->data = ms_extended_compat_id_os_desc;
        req->length = std::min(req->length, sizeof(ms_extended_compat_id_os_desc));
        return USB_t::OK;
    } else {
        return USB_t::FAIL;
    }
}
static USB_t::STATUS get_ms_extended_properties_os_descriptor(USB_t::SetupRequest_t* req) {
    if (!req) {
        return USB_t::FAIL;
    }

    if (req->index == 0x05) {
        uint8_t interface_id = (uint8_t)req->value;
        switch (interface_id) {
        case 0:
#if NUM_INTERFACES == 2
        case 1:
#endif
            req->data = ms_extended_properties_os_desc;
            req->length = std::min(req->length, sizeof(ms_extended_properties_os_desc));
            return USB_t::OK;
        default:
            return USB_t::FAIL;
        }
    } else {
        return USB_t::FAIL;
    }
}

static USB_t::STATUS handle_setup_request(USB_t::SetupRequest_t* req) {
    if (!req) {
        return USB_t::FAIL;
    }

    switch (req->request_type & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_DEVICE:
        return get_ms_extended_compat_id_os_descriptor(req);
    case USB_REQ_RECIPIENT_INTERFACE:
        return get_ms_extended_properties_os_descriptor(req);
    case USB_REQ_RECIPIENT_ENDPOINT:
        // fall through
    default:
        break;
    }
    return USB_t::FAIL;
}

USBVendorRequestHandler_t ms_request_handler{
    MS_VendorCode,
    handle_setup_request
};
