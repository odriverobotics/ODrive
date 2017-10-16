
// TODO: resolve assert
#define assert(expr)

#ifdef __cplusplus

#include <functional>
#include <limits>
#include "crc.hpp"

typedef enum {
    GPIO_MODE_UART,
    GPIO_MODE_STEP_DIR,
} GpioMode_t;

extern "C" {
#endif

void init_communication(void);
void communication_task(void const * argument);
void USB_receive_packet(const uint8_t *buffer, size_t length);

#ifdef __cplusplus
}
#endif
