/*
* ODrive I2C communication library
* This file implements I2C communication with the ODrive.
*
*   - Implement the C function I2C_transaction to provide low level I2C access.
*   - Use read_property<PropertyId>() to read properties from the ODrive.
*   - Use write_property<PropertyId>() to modify properties on the ODrive.
*   - Use trigger<PropertyId>() to trigger a function (such as reboot or save_configuration)
*   - Use endpoint_type_t<PropertyId> to retrieve the underlying type
*     of a given property.
*   - Refer to PropertyId for a list of available properties.
*
* To regenerate the interface definitions, flash an ODrive with
* the new firmware, connect it to your PC via USB and then run
*   ../tools/odrivetool generate-code --output [path to odrive_endpoints.h]
* This step can be done with any ODrive, it doesn't have to be the
* one that you'll be controlling over I2C.
*/


#include <limits.h>
#include <stdint.h>

#include "odrive_endpoints.h"

#ifdef __AVR__
// AVR-GCC doesn't ship with the STL, so we use our own little excerpt
#include "type_traits.h"
#else
#include <type_traits>
#endif


extern "C" {

/* @brief Send and receive data to/from an I2C slave
*
* This function carries out the following sequence:
* 1. generate a START condition
* 2. if the tx_buffer is not null:
*    a. send 7-bit slave address (with the LSB 0)
*    b. send all bytes in the tx_buffer
* 3. if both tx_buffer and rx_buffer are not null, generate a REPEATED START condition
* 4. if the rx_buffer is not null:
*    a. send 7-bit slave address (with the LSB 1)
*    b. read rx_length bytes into rx_buffer
* 5. send STOP condition
*
* @param slave_addr: 7-bit slave address (the MSB is ignored)
* @return true if all data was transmitted and received as requested by the caller, false otherwise
*/
bool I2C_transaction(uint8_t slave_addr, const uint8_t * tx_buffer, size_t tx_length, uint8_t * rx_buffer, size_t rx_length);

}


namespace odrive {
    static constexpr const uint8_t i2c_addr = (0xD << 3); // write: 1101xxx0, read: 1101xxx1

    template<typename T>
    using bit_width = std::integral_constant<unsigned int, CHAR_BIT * sizeof(T)>;

    template<typename T>
    using byte_width = std::integral_constant<unsigned int, (bit_width<T>::value + 7) / 8>;


    template<unsigned int IBitSize>
    struct unsigned_int_of_size;

    template<> struct unsigned_int_of_size<32> { typedef uint32_t type; };


    template<typename T>
    typename std::enable_if<std::is_integral<T>::value, T>::type
    read_le(const uint8_t buffer[byte_width<T>::value]) {
        T value = 0;
        for (size_t i = 0; i < byte_width<T>::value; ++i)
            value |= (static_cast<T>(buffer[i]) << (i << 3));
        return value;
    }

    template<typename T>
    typename std::enable_if<std::is_floating_point<T>::value, T>::type
    read_le(const uint8_t buffer[]) {
        using T_Int = typename unsigned_int_of_size<bit_width<T>::value>::type;
        T_Int value = read_le<T_Int>(buffer);
        return *reinterpret_cast<T*>(&value);
    }

    template<typename T>
    typename std::enable_if<std::is_integral<T>::value, void>::type
    write_le(uint8_t buffer[byte_width<T>::value], T value) {
        for (size_t i = 0; i < byte_width<T>::value; ++i)
            buffer[i] = (value >> (i << 3)) & 0xff;
    }

    template<typename T>
    typename std::enable_if<std::is_floating_point<T>::value, T>::type
    write_le(uint8_t buffer[byte_width<T>::value], T value) {
        using T_Int = typename unsigned_int_of_size<bit_width<T>::value>::type;
        write_le<T_Int>(buffer, *reinterpret_cast<T_Int*>(&value));
    }

    /* @brief Read from an endpoint on the ODrive.
    * To read from an axis specific endpoint use read_axis_property() instead.
    *
    * Usage example:
    *   float val;
    *   success = odrive::read_property<odrive::VBUS_VOLTAGE>(0, &val);
    *
    * @param num Selects the ODrive. For instance the value 4 selects
    * the ODrive that has [A2, A1, A0] connected to [VCC, GND, GND].
    * @return true if the I2C transaction succeeded, false otherwise
    */
    template<int IPropertyId>
    bool read_property(uint8_t num, endpoint_type_t<IPropertyId>* value, uint16_t address = IPropertyId) {
        uint8_t i2c_tx_buffer[4];
        write_le<uint16_t>(i2c_tx_buffer, address);
        write_le<uint16_t>(i2c_tx_buffer + sizeof(i2c_tx_buffer) - 2, json_crc);
        uint8_t i2c_rx_buffer[byte_width<endpoint_type_t<IPropertyId>>::value];
        if (!I2C_transaction(i2c_addr + num,
            i2c_tx_buffer, sizeof(i2c_tx_buffer),
            i2c_rx_buffer, sizeof(i2c_rx_buffer)))
            return false;
        if (value)
            *value = read_le<endpoint_type_t<IPropertyId>>(i2c_rx_buffer);
        return true;
    }

    /* @brief Write to an endpoint on the ODrive.
    * To write to an axis specific endpoint use write_axis_property() instead.
    *
    * Usage example:
    *   success = odrive::write_property<odrive::TEST_PROPERTY>(0, 42);
    *
    * @param num Selects the ODrive. For instance the value 4 selects
    * the ODrive that has [A2, A1, A0] connected to [VCC, GND, GND].
    * @return true if the I2C transaction succeeded, false otherwise
    */
    template<int IPropertyId>
    bool write_property(uint8_t num, endpoint_type_t<IPropertyId> value, uint16_t address = IPropertyId) {
        uint8_t i2c_tx_buffer[4 + byte_width<endpoint_type_t<IPropertyId>>::value];
        write_le<uint16_t>(i2c_tx_buffer, address);
        write_le<endpoint_type_t<IPropertyId>>(i2c_tx_buffer + 2, value);
        write_le<uint16_t>(i2c_tx_buffer + sizeof(i2c_tx_buffer) - 2, json_crc);
        return I2C_transaction(i2c_addr + num, i2c_tx_buffer, sizeof(i2c_tx_buffer), nullptr, 0);
    }

    /* @brief Trigger an parameter-less function on the ODrive
    *
    * Usage example:
    *   success = odrive::trigger<odrive::SAVE_CONFIGURATION>(0);
    *
    * @param num Selects the ODrive. For instance the value 4 selects
    * the ODrive that has [A2, A1, A0] connected to [VCC, GND, GND].
    * @return true if the I2C transaction succeeded, false otherwise
    */
    template<int IPropertyId,
             typename = typename std::enable_if<std::is_void<endpoint_type_t<IPropertyId>>::value>::type>
    bool trigger(uint8_t num, uint16_t address = IPropertyId) {
        uint8_t i2c_tx_buffer[4];
        write_le<uint16_t>(i2c_tx_buffer, address);
        write_le<uint16_t>(i2c_tx_buffer + sizeof(i2c_tx_buffer) - 2, json_crc);
        return I2C_transaction(i2c_addr + num, i2c_tx_buffer, sizeof(i2c_tx_buffer), nullptr, 0);
    }

    template<int IPropertyId>
    bool read_axis_property(uint8_t num, uint8_t axis, endpoint_type_t<IPropertyId>* value) {
        return read_property<IPropertyId>(num, value, IPropertyId + axis * per_axis_offset);
    }

    template<int IPropertyId>
    bool write_axis_property(uint8_t num, uint8_t axis, endpoint_type_t<IPropertyId> value) {
        return write_property<IPropertyId>(num, value, IPropertyId + axis * per_axis_offset);
    }


    /* @brief Checks if the axis is in the requested state and the error register is clear */
    bool check_axis_state(uint8_t num, uint8_t axis, uint8_t state) {
        endpoint_type_t<odrive::AXIS__CURRENT_STATE> observed_state = 0;
        endpoint_type_t<odrive::AXIS__ERROR> observed_error = 0;
        if (!read_axis_property<odrive::AXIS__CURRENT_STATE>(num, axis, &observed_state))
            return false;
        if (!read_axis_property<odrive::AXIS__ERROR>(num, axis, &observed_error))
            return false;
        return (observed_error == 0) && (observed_state == state);
    }

    /* @brief Clears any error state of the specified axis */
    bool clear_errors(uint8_t num, uint8_t axis) {
        if (!write_axis_property<odrive::AXIS__ERROR>(num, axis, 0))
            return false;
        if (!write_axis_property<odrive::AXIS__MOTOR__ERROR>(num, axis, 0))
            return false;
        if (!write_axis_property<odrive::AXIS__ENCODER__ERROR>(num, axis, 0))
            return false;
        return true;
    }
}
