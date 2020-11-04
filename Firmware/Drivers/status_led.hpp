#ifndef __STATUS_LED_HPP
#define __STATUS_LED_HPP

#include <stdint.h>
#include <Drivers/ws2812.hpp>

struct rgb_t {
    rgb_t() {
        val = 0;
    }
    rgb_t(uint8_t r, uint8_t g, uint8_t b) {
        val = (r << 16) | (g << 8) | (b << 0);
    }
    rgb_t(uint32_t val) : val(val) {}

    uint8_t get_r() { return (val >> 16) & 0xff; }
    uint8_t get_g() { return (val >> 8) & 0xff; }
    uint8_t get_b() { return (val >> 0) & 0xff; }

    template<uint32_t max_val>
    static rgb_t mix(rgb_t color0, rgb_t color1, uint32_t ratio) {
        uint32_t ratio1 = (ratio >= max_val) ? (max_val - 1) : ratio;
        uint32_t ratio0 = max_val - ratio1;
        return rgb_t{
            (uint8_t)(((uint32_t)color0.get_r() * ratio0 + (uint32_t)color1.get_r() * ratio1) / max_val),
            (uint8_t)(((uint32_t)color0.get_g() * ratio0 + (uint32_t)color1.get_g() * ratio1) / max_val),
            (uint8_t)(((uint32_t)color0.get_b() * ratio0 + (uint32_t)color1.get_b() * ratio1) / max_val),
        };
    }

    uint32_t val;
};

struct Ws2812EncoderTraits {
    static constexpr uint32_t kBaudrate = 3310345ULL;
    static constexpr uint16_t kSymbolHigh = 0b1110; // 906ns on, 302ns off
    static constexpr uint16_t kSymbolLow = 0b1000; // 302ns on, 906ns off
    static constexpr size_t kNumBitsPerSymbol = 4;
    static constexpr size_t kBitsPerLed = 24;
    static constexpr size_t kNumLeds = 1;
    using TColor = rgb_t;
    using TEncoded = uint16_t;
    static uint32_t get_bits(TColor color) { return color.val; }
};

using I2sWs2812Encoder = Ws2812Encoder<Ws2812EncoderTraits>;
constexpr size_t kI2sBufLen = ((I2sWs2812Encoder::get_total_encoded_words(1) + 1) >> 1) << 1;

class I2sRgbLed {
public:
    void init();
    void set_color(rgb_t color);
private:
    uint16_t i2s_buf_[kI2sBufLen];
};

#endif // __STATUS_LED_HPP