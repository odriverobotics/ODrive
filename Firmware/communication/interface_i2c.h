#ifndef __INTERFACE_I2C_HPP
#define __INTERFACE_I2C_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct I2CStats_t {
    uint8_t addr;
    uint32_t addr_match_cnt;
    uint32_t rx_cnt;
    uint32_t error_cnt;
};

extern I2CStats_t i2c_stats_;

void start_i2c_server(void);

#ifdef __cplusplus
}
#endif

#endif // __INTERFACE_I2C_HPP
