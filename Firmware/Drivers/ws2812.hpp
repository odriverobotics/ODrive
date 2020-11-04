#ifndef __WS2812_HPP
#define __WS2812_HPP

#include <stdlib.h>
#include <limits.h>

/**
 * @tparam TTraits::TColor: The type representing a single LED's color
 * @tparam TTraits::TEncoded: The data type of the encoded bitstream.
 *         Typically uint8_t, but can also have a different word size.
 * @tparam TTraits::convert: A function that converts an instance of TTraits::TColor
 *        into the bit representation that should be sent out.
 *        kBitsPerLed bits are sent out.
 *        If the returned type has a larger size, it should be left-padded (MSBs
 *        ignored)
 *        The MSB (after padding) is sent out first (after padding).
 */
template<typename TTraits>
struct Ws2812Encoder {
    using TEncoded = typename TTraits::TEncoded;
    using TColor = typename TTraits::TColor;

    static constexpr size_t kResetTimeUs = 55; // officially 50us, but that doesn't always work
    static constexpr size_t kResetBits = (kResetTimeUs * TTraits::kBaudrate) / 1000000ULL;
    static constexpr size_t kEncodedWordSize = CHAR_BIT * sizeof(TEncoded);

    static constexpr size_t get_total_encoded_bits(size_t num_leds) {
        return num_leds * TTraits::kBitsPerLed * TTraits::kNumBitsPerSymbol + kResetBits;
    }
    static constexpr size_t get_total_encoded_words(size_t num_leds) {
        return (get_total_encoded_bits(num_leds) + kEncodedWordSize - 1) / kEncodedWordSize;
    }
    
    template<bool WrapAround>
    static void encode(TColor* colors, size_t num_colors, size_t encoded_offset, TEncoded* encoded_buffer, size_t encoded_buffer_length);
};



/**
 * @brief Encodes an array of colors into a bitstream that can be sent over a
 * real-time bit generator like I2S or SPI in order to control a WS2812-type LED chain.
 * 
 * The bits must be sent out MSB-first to generate the correct wave form.
 * 
 * @tparam WrapAround: if true, the encoder wraps around to the first LED when
 *                     the end of the stream is reached useful for continuous data streams.
 *                     If false, the encoded buffer is padded with zeros.
 * @param encoded_offset: The position in the encoded stream, indicated in number of encoded words.
 * @param encoded_buffer: Buffer where the encoded bit stream will be written.
 */
template<typename TTraits>
template<bool WrapAround>
void Ws2812Encoder<TTraits>::encode(TColor* colors, size_t num_colors, size_t encoded_offset, TEncoded* encoded_buffer, size_t encoded_buffer_length) {
    size_t total_encoded_bits = get_total_encoded_bits(num_colors);

    for (size_t i2s_word_id = 0; i2s_word_id < encoded_buffer_length; ++i2s_word_id) {
        uint16_t i2s_word = 0;

        for (size_t i2s_bit_id = 0; i2s_bit_id < kEncodedWordSize; ++i2s_bit_id) {
            size_t bitpos = (i2s_word_id + encoded_offset) * kEncodedWordSize + i2s_bit_id;

            if (WrapAround) {
                bitpos = bitpos % total_encoded_bits;
            }

            size_t symbol_bit_id = TTraits::kNumBitsPerSymbol - (bitpos % TTraits::kNumBitsPerSymbol) - 1;
            size_t led_bit_id = TTraits::kBitsPerLed - ((bitpos / TTraits::kNumBitsPerSymbol) % TTraits::kBitsPerLed) - 1;
            size_t led_id = (bitpos / TTraits::kNumBitsPerSymbol) / TTraits::kBitsPerLed;

            if (led_id < num_colors) {
                auto color = TTraits::get_bits(colors[led_id]);
                uint16_t symbol = ((color >> led_bit_id) & 1) ? TTraits::kSymbolHigh : TTraits::kSymbolLow;

                if ((symbol >> symbol_bit_id) & 1) {
                    i2s_word |= (1 << (kEncodedWordSize - i2s_bit_id - 1));
                }
            }
        }

        encoded_buffer[i2s_word_id] = i2s_word;
    }
}


#endif // __WS2812_HPP