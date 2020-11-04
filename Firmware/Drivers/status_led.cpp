
#include "status_led.hpp"
#include <i2s.h>

void I2sRgbLed::init() {
    uint16_t init_buf[1] = {0};
    HAL_I2S_Transmit_DMA(&hi2s1, init_buf, 1);
    while (hi2s1.State != HAL_I2S_STATE_READY);
}

void I2sRgbLed::set_color(rgb_t color) {
    rgb_t stripe[1] = {color};
    I2sWs2812Encoder::encode<false>(stripe, 1, 0, i2s_buf_, kI2sBufLen);
    HAL_I2S_Transmit_DMA(&hi2s1, i2s_buf_, kI2sBufLen);
}
