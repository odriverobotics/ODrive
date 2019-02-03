#ifndef _AMT203_HPP
#define _AMT203_HPP

#include <spi.h>
#include "absolute_encoder.hpp"

typedef enum {
    AMT203_NOP_CMD = 0x00,
    AMT203_NOP_A5 = 0xA5,
    AMT203_READ_POS_CMD = 0x10,
    AMT203_SET_ZERO_POINT_CMD = 0x70,
    AMT203_SET_ZERO_POINT_SUCCESS = 0x80,
} AMT203_Message_t;


typedef struct {
    uint16_t chipSelectPin = 5;
    bool use_absolute = true;
} amt203ConfigData_t;

class AMT203 : public AbsoluteEncoder {
   public:
    AMT203(SPI_HandleTypeDef* spiHandle, GPIO_TypeDef* chipSelectHandle, uint16_t chipSelectPin);
    bool init();
    uint32_t readPosition();
    bool setZeroPoint();

   private:
    SPI_HandleTypeDef* spiHandle;
    GPIO_TypeDef* chipSelectHandle;
    uint16_t chipSelectPin;
    uint16_t position;
    uint8_t sendCommand(AMT203_Message_t command);
    uint8_t sendNop();

};

#endif