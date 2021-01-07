#ifndef __FIBRE_PRINT_UTILS_HPP
#define __FIBRE_PRINT_UTILS_HPP

#include <string>
#include <ostream>
#include <fibre/bufptr.hpp>

namespace fibre {

template<typename T>
constexpr size_t hex_digits() {
    return (std::numeric_limits<T>::digits + 3) / 4;
}

/* @brief Converts a hexadecimal digit to a uint8_t.
* @param output If not null, the digit's value is stored in this output
* Returns true if the char is a valid hex digit, false otherwise
*/
static inline bool hex_digit_to_byte(char ch, uint8_t* output) {
    uint8_t nil_output = 0;
    if (!output)
        output = &nil_output;
    if (ch >= '0' && ch <= '9')
        return (*output) = ch - '0', true;
    if (ch >= 'a' && ch <= 'f')
        return (*output) = ch - 'a' + 10, true;
    if (ch >= 'A' && ch <= 'F')
        return (*output) = ch - 'A' + 10, true;
    return false;
}

/* @brief Converts a hex string to an integer
* @param output If not null, the result is stored in this output
* Returns true if the string represents a valid hex value, false otherwise.
*/
template<typename TInt>
bool hex_string_to_int(const char * str, size_t length, TInt* output) {
    constexpr size_t N_DIGITS = hex_digits<TInt>();
    TInt result = 0;
    if (length > N_DIGITS)
        length = N_DIGITS;
    for (size_t i = 0; i < length && str[i]; i++) {
        uint8_t digit = 0;
        if (!hex_digit_to_byte(str[i], &digit))
            return false;
        result <<= 4;
        result += digit;
    }
    if (output)
        *output = result;
    return true;
}

template<typename TInt>
bool hex_string_to_int(const char * str, TInt* output) {
    return hex_string_to_int<TInt>(str, hex_digits<TInt>(), output);
}

template<typename TInt, size_t ICount>
bool hex_string_to_int_arr(const char * str, size_t length, TInt (&output)[ICount]) {
    for (size_t i = 0; i < ICount; i++) {
        if (!hex_string_to_int<TInt>(&str[i * hex_digits<TInt>()], &output[i]))
            return false;
    }
    return true;
}

template<typename TInt, size_t ICount>
bool hex_string_to_int_arr(const char * str, TInt (&output)[ICount]) {
    return hex_string_to_int_arr(str, hex_digits<TInt>() * ICount, output);
}

// TODO: move to print_utils.hpp
template<typename T>
class HexPrinter {
public:
    HexPrinter(T val, bool prefix) : val_(val) /*, prefix_(prefix)*/ {
        const char digits[] = "0123456789abcdef";
        size_t prefix_length = prefix ? 2 : 0;
        if (prefix) {
            str[0] = '0';
            str[1] = 'x';
        }
        str[prefix_length + hex_digits<T>()] = '\0';
        
        for (size_t i = 0; i < hex_digits<T>(); ++i) {
            str[prefix_length + hex_digits<T>() - i - 1] = digits[val & 0xf];
            val >>= 4;
        }
    }
    std::string to_string() const { return str; }
    void to_string(char* buf) const {
        for (size_t i = 0; (i < sizeof(str)) && str[i]; ++i)
            buf[i] = str[i];
    }

    T val_;
    //bool prefix_;
    char str[hex_digits<T>() + 3]; // 3 additional characters 0x and \0
};

template<typename T>
std::ostream& operator<<(std::ostream& stream, const HexPrinter<T>& printer) {
    // TODO: specialize for char
    return stream << printer.to_string();
}

template<typename T>
HexPrinter<T> as_hex(T val, bool prefix = true) { return HexPrinter<T>(val, prefix); }

template<typename T>
class HexArrayPrinter {
public:
    HexArrayPrinter(T* ptr, size_t length) : ptr_(ptr), length_(length) {}
    T* ptr_;
    size_t length_;
};

template<typename TStream, typename T>
TStream& operator<<(TStream& stream, const HexArrayPrinter<T>& printer) {
    for (size_t pos = 0; pos < printer.length_; ++pos) {
        stream << " " << as_hex(printer.ptr_[pos]);
        if (((pos + 1) % 16) == 0)
            stream << "\n";
    }
    return stream;
}

template<typename T, size_t ILength>
HexArrayPrinter<T> as_hex(T (&val)[ILength]) { return HexArrayPrinter<T>(val, ILength); }

template<typename T>
HexArrayPrinter<T> as_hex(generic_bufptr_t<T> buffer) { return HexArrayPrinter<T>(buffer.begin(), buffer.size()); }

}

#endif // __FIBRE_PRINT_UTILS_HPP