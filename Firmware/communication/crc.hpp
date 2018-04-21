#ifndef __CRC_HPP
#define __CRC_HPP

#include <stdint.h>
#include <limits.h>

// Default CRC-8 Polynomial: x^8 + x^5 + x^4 + x^2 + x + 1
// Can protect a 4 byte payload against toggling of up to 5 bits
//  source: https://users.ece.cmu.edu/~koopman/crc/index.html
constexpr uint8_t CRC8_DEFAULT = 0x37;

// Default CRC-16 Polynomial: 0x9eb2 x^16 + x^13 + x^12 + x^11 + x^10 + x^8 + x^6 + x^5 + x^2 + 1
// Can protect a 135 byte payload against toggling of up to 5 bits
//  source: https://users.ece.cmu.edu/~koopman/crc/index.html
// Also known as CRC-16-DNP
constexpr uint16_t CRC16_DEFAULT = 0x3d65;

// Calculates an arbitrary CRC for one byte.
// Adapted from https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
template<typename T, unsigned POLYNOMIAL>
static T calc_crc(T remainder, uint8_t value) {
    constexpr T BIT_WIDTH = (CHAR_BIT * sizeof(T));
    constexpr T TOPBIT = ((T)1 << (BIT_WIDTH - 1));
    
    // Bring the next byte into the remainder.
    remainder ^= (value << (BIT_WIDTH - 8));

    // Perform modulo-2 division, a bit at a time.
    for (uint8_t bit = 8; bit; --bit) {
        if (remainder & TOPBIT) {
            remainder = (remainder << 1) ^ POLYNOMIAL;
        } else {
            remainder = (remainder << 1);
        }
    }

    return remainder;
}

template<typename T, unsigned POLYNOMIAL>
static T calc_crc(T remainder, const uint8_t* buffer, size_t length) {
    while (length--)
        remainder = calc_crc<T, POLYNOMIAL>(remainder, *(buffer++));
    return remainder;
}

template<unsigned POLYNOMIAL = CRC8_DEFAULT>
static uint8_t calc_crc8(uint8_t remainder, uint8_t value) {
    return calc_crc<uint8_t, POLYNOMIAL>(remainder, value);
}

template<unsigned POLYNOMIAL = CRC16_DEFAULT>
static uint16_t calc_crc16(uint16_t remainder, uint8_t value) {
    return calc_crc<uint16_t, POLYNOMIAL>(remainder, value);
}

template<unsigned POLYNOMIAL = CRC8_DEFAULT>
static uint8_t calc_crc8(uint8_t remainder, const uint8_t* buffer, size_t length) {
    return calc_crc<uint8_t, POLYNOMIAL>(remainder, buffer, length);
}

template<unsigned POLYNOMIAL = CRC16_DEFAULT>
static uint16_t calc_crc16(uint16_t remainder, const uint8_t* buffer, size_t length) {
    return calc_crc<uint16_t, POLYNOMIAL>(remainder, buffer, length);
}

#endif /* __CRC_HPP */
