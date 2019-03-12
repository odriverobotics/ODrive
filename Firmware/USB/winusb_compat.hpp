/**
 * @brief Zero-config support for WinUSB
 * 
 * This section tells Windows that it should automatically load the WinUSB driver
 * for the device (more specifically, interface 2 because it's a composite device).
 * This allows for driverless communication with the device.
 */

#ifndef __WINUSB_COMPAT_HPP
#define __WINUSB_COMPAT_HPP

#include "usb.hpp"

extern USBVendorRequestHandler_t ms_request_handler;

#endif // __WINUSB_COMPAT_HPP