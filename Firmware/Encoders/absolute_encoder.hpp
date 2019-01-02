#ifndef __ABS_ENCODER_HPP
#define __ABS_ENCODER_HPP

#include <stdint.h>

typedef enum {
    ABSOLUTE_ENCODER_AMT203
} absEncoderType_t;

class AbsoluteEncoder {
    public:
        virtual uint32_t readPosition() = 0;
        virtual bool init() = 0;
};

#endif