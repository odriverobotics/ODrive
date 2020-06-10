#ifndef __FIBRE_SIMPLE_SERDES
#define __FIBRE_SIMPLE_SERDES

//#include "stream.hpp"


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


#endif