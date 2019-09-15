/*
* Convenience functions to load and store multiple objects from and to NVM.
* 
* The NVM stores consecutive one-to-one copies of arbitrary objects.
* The types of these objects are passed as template arguments to Config<Ts...>.
*/

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <stm32f405xx.h>

#include "nvm.h"
#include <fibre/crc.hpp>


/* Private defines -----------------------------------------------------------*/
#define CONFIG_CRC16_INIT 0xabcd
#define CONFIG_CRC16_POLYNOMIAL 0x3d65

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

// IMPORTANT: if you change, reorder or otherwise modify any of the fields in
// the config structs, make sure to increment this number:
static constexpr uint16_t config_version = 0x0001;

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/


// @brief Manages configuration load and store operations from and to NVM
//
// The NVM stores consecutive one-to-one copies of arbitrary objects.
// The types of these objects are passed as template arguments to Config<Ts...>.
//
// Config<Ts...> has two template specializations to implement template recursion:
// - Config<T, Ts...> handles loading/storing of the first object (type T) and leaves
//   the rest of the objects to an "inner" class Config<Ts...>.
// - Config<> represents the leaf of the recursion.
template<typename ... Ts>
struct Config;

template<>
struct Config<> {
    static size_t get_size() {
        return 0;
    }
    static int load_config(size_t offset, uint16_t* crc16) {
        return 0;
    }
    static int store_config(size_t offset, uint16_t* crc16) {
        return 0;
    }
};

template<typename T, typename ... Ts>
struct Config<T, Ts...> {
    static size_t get_size() {
        return sizeof(T) + Config<Ts...>::get_size();
    }

    // @brief Loads one or more consecutive objects from the NVM.
    // During loading this function also calculates the CRC over the loaded data.
    // @param offset: 0 means that the function should start reading at the beginning
    // of the last comitted NVM block
    // @param crc16: the result of the CRC calculation is written to this address
    // @param val0, vals: the values to be loaded
    static int load_config(size_t offset, uint16_t* crc16, T* val0, Ts* ... vals) {
        size_t size = sizeof(T);
        // save current CRC (in case val0 and crc16 point to the same address)
        size_t previous_crc16 = *crc16;
        if (NVM_read(offset, (uint8_t *)val0, size))
            return -1;
        *crc16 = calc_crc16<CONFIG_CRC16_POLYNOMIAL>(previous_crc16, (uint8_t *)val0, size);
        if (Config<Ts...>::load_config(offset + size, crc16, vals...))
            return -1;
        return 0;
    }

    // @brief Stores one or more consecutive objects to the NVM.
    // During storing this function also calculates the CRC over the stored data.
    // @param offset: 0 means that the function should start writing at the beginning
    // of the currently active NVM write block
    // @param crc16: the result of the CRC calculation is written to this address
    // @param val0, vals: the values to be stored
    static int store_config(size_t offset, uint16_t* crc16, const T* val0, const Ts* ... vals) {
        size_t size = sizeof(T);
        if (NVM_write(offset, (uint8_t *)val0, size))
            return -1;
        // update CRC _after_ writing (in case val0 and crc16 point to the same address)
        if (crc16)
            *crc16 = calc_crc16<CONFIG_CRC16_POLYNOMIAL>(*crc16, (uint8_t *)val0, size);
        if (Config<Ts...>::store_config(offset + size, crc16, vals...))
            return -1;
        return 0;
    }

    // @brief Loads one or more consecutive objects from the NVM. The loaded data
    // is validated using a CRC value that is stored at the beginning of the data.
    static int safe_load_config(T* val0, Ts* ... vals) {
        //printf("have %d bytes\r\n", NVM_get_max_read_length()); osDelay(5);
        if (Config<T, Ts..., uint16_t>::get_size() > NVM_get_max_read_length())
            return -1;
        uint16_t crc16 = CONFIG_CRC16_INIT ^ config_version;
        if (Config<T, Ts..., uint16_t>::load_config(0, &crc16, val0, vals..., &crc16))
            return -1;
        if (crc16)
            return -1;
        return 0;
    }

    // @brief Stores one or more consecutive objects to the NVM. In addition to the
    // provided objects, a CRC of the data is stored.
    //
    // The CRC includes a version number and thus adds some protection against
    // changes of the config structs during firmware update. Note that if the total
    // config data length changes, the CRC validation will fail even if the developer
    // forgets to update the config version number.
    static int safe_store_config(const T* val0, const Ts* ... vals) {
        size_t size = Config<T, Ts...>::get_size() + 2;
        //printf("config is %d bytes\r\n", size); osDelay(5);
        if (size > NVM_get_max_write_length())
            return -1;
        if (NVM_start_write(size))
            return -1;
        uint16_t crc16 = CONFIG_CRC16_INIT ^ config_version;
        if (Config<T, Ts...>::store_config(0, &crc16, val0, vals...))
            return -1;
        if (Config<uint8_t, uint8_t>::store_config(size - 2, nullptr, (uint8_t *)&crc16 + 1, (uint8_t *)&crc16))
            return -1;
        if (NVM_commit())
            return -1;
        return 0;
    }
};
