#ifndef __FIBRE_SIMPLE_SERDES
#define __FIBRE_SIMPLE_SERDES

#include "cpp_utils.hpp"
#include "limits.h"
#include <optional> // TODO: make C++11 backport of this
#include <cstring>
#include <stdint.h>

template<typename T, bool BigEndian, typename = void>
struct SimpleSerializer;
template<typename T>
using LittleEndianSerializer = SimpleSerializer<T, false>;
template<typename T>
using BigEndianSerializer = SimpleSerializer<T, true>;


/* @brief Serializer/deserializer for arbitrary integral number types */
// TODO: allow reading an arbitrary number of bits
template<typename T, bool BigEndian>
struct SimpleSerializer<T, BigEndian, typename std::enable_if_t<std::is_integral<T>::value>> {
    static constexpr size_t BIT_WIDTH = std::numeric_limits<T>::digits;
    static constexpr size_t BYTE_WIDTH = (BIT_WIDTH + 7) / 8;

    template<typename TIterator>
    static std::optional<T> read(TIterator* begin, TIterator end = nullptr) {
        T result = 0;
        if (BigEndian) {
            for (size_t i = BYTE_WIDTH; i > 0; (i++, (*begin)++)) {
                if (end && !(*begin < end))
                    return std::nullopt;
                uint8_t byte = **begin;
                result |= static_cast<T>(byte) << ((i - 1) << 3);
            }
        } else {
            for (size_t i = 0; i < BYTE_WIDTH; (i++, (*begin)++)) {
                if (end && !(*begin < end))
                    return std::nullopt;
                uint8_t byte = **begin;
                result |= static_cast<T>(byte) << (i << 3);
            }
        }
        return result;
    }

    template<typename TIterator>
    static bool write(T value, TIterator* begin, TIterator end = nullptr) {
        if (BigEndian) {
            for (size_t i = BYTE_WIDTH; i > 0; (i--, (*begin)++)) {
                if (end && !(*begin < end))
                    return false;
                uint8_t byte = static_cast<uint8_t>((value >> ((i - 1) << 3)) & 0xff);
                **begin = byte;
            }
        } else {
            for (size_t i = 0; i < BYTE_WIDTH; (i++, (*begin)++)) {
                if (end && !(*begin < end))
                    return false;
                uint8_t byte = static_cast<uint8_t>((value >> (i << 3)) & 0xff);
                **begin = byte;
            }
        }
        return true;
    }
};

template<typename T>
inline std::optional<T> read_le(fibre::cbufptr_t* buffer) {
    static_assert(is_complete<LittleEndianSerializer<T>>(), "no LittleEndianSerializer is defined for type T");
    return LittleEndianSerializer<T>::read(&buffer->begin(), buffer->end());
}

template<typename T>
inline bool write_le(T value, fibre::bufptr_t* buffer) {
    static_assert(is_complete<LittleEndianSerializer<T>>(), "no LittleEndianSerializer is defined for type T");
    return LittleEndianSerializer<T>::write(value, &buffer->begin(), buffer->end());
}

template<typename T, typename = typename std::enable_if_t<!std::is_const<T>::value>>
inline size_t write_le(T value, uint8_t* buffer){
    //TODO: add static_assert that this is still a little endian machine
    std::memcpy(&buffer[0], &value, sizeof(value));
    return sizeof(value);
}

template<typename T>
typename std::enable_if_t<std::is_const<T>::value, size_t>
write_le(T value, uint8_t* buffer) {
    return write_le<std::remove_const_t<T>>(value, buffer);
}

template<>
inline size_t write_le<float>(float value, uint8_t* buffer) {
    static_assert(CHAR_BIT * sizeof(float) == 32, "32 bit floating point expected");
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 floating point expected");
    uint32_t value_as_uint32;
    std::memcpy(&value_as_uint32, &value, sizeof(uint32_t));
    return write_le<uint32_t>(value_as_uint32, buffer);
}

template<typename T>
inline size_t read_le(T* value, const uint8_t* buffer){
    // TODO: add static_assert that this is still a little endian machine
    std::memcpy(value, buffer, sizeof(*value));
    return sizeof(*value);
}

template<>
inline size_t read_le<float>(float* value, const uint8_t* buffer) {
    static_assert(CHAR_BIT * sizeof(float) == 32, "32 bit floating point expected");
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 floating point expected");
    return read_le(reinterpret_cast<uint32_t*>(value), buffer);
}

// @brief Reads a value of type T from the buffer.
// @param buffer    Pointer to the buffer to be read. The pointer is updated by the number of bytes that were read.
// @param length    The number of available bytes in buffer. This value is updated to subtract the bytes that were read.
template<typename T>
static inline T read_le(const uint8_t** buffer, size_t* length) {
    T result;
    size_t cnt = read_le(&result, *buffer);
    *buffer += cnt;
    *length -= cnt;
    return result;
}

#endif