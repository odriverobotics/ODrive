# USB

This page documents the low level USB configuration. If you're looking for a higher level protocol documentation see [Native Protocol](native-protocol) and [ASCII Protocol](ascii-protocol).

This page assumes that you are familiar with the general USB architecture, in particular with terms like "configuration", "interface" and "endpoint".

On USB the ODrive provides a single configuration which is a composite device consisting of a CDC device (virtual COM port) and a vendor specific device.

<details><summary markdown="span">What is a composite device?</summary><div markdown="block">
A composite device is a device where interfaces are grouped by interface association descriptors. For such devices, the host OS loads an intermediate driver, so that each of the interface groups can be treated like a separate device and have its own host-side driver attached.
</div></details>

The following interface groups are present:

 * Interface Association: Communication Device Class (CDC)
    * Interface 0:
        * Endpoint `0x82`: CDC commands
    * Interface 1:
        * Endpoint `0x01`: CDC data OUT
        * Endpoint `0x81`: CDC data IN
 * Interface Association: Vendor Specific Device Class
    * Interface 2:
        * Endpoint `0x03`: data OUT
        * Endpoint `0x83`: data IN

The CDC interface (endpoint pair `0x01, 0x81`) runs the [ASCII Protocol](ascii-protocol) by default (see `odrv0.config.enable_ascii_protocol_on_usb`). The vendor specific interface (endpoint pair `0x03, 0x83`) runs the [Native Protocol](native-protocol) (the packet based variant).

The two interfaces can not (yet) be used simultaneously.
