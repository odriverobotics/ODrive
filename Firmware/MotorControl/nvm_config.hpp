/*
* Convenience functions to load and store multiple objects from and to NVM.
* 
* The NVM stores consecutive one-to-one copies of arbitrary objects.
* The types of these objects are passed as template arguments to Config<Ts...>.
*/

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>

#include <Drivers/STM32/stm32_nvm.h>
#include <fibre/../../crc.hpp>


/* Private defines -----------------------------------------------------------*/
#define CONFIG_CRC16_INIT 0xabcd
#define CONFIG_CRC16_POLYNOMIAL 0x3d65

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

// IMPORTANT: if you change, reorder or otherwise modify any of the fields in
// the config structs without changing its total length, make sure to increment this number:
static constexpr uint16_t config_version = 0x0001;

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

/**
 * @brief Manages configuration load and store operations from and to NVM
 * 
 * Usage:
 *  1. start_load()
 *  2. read() (as often needed)
 *  3. finish_load() (to see if all reads were successful and the CRC in the end is valid)
 * 
 *  1. prepare_store()
 *  2. write() (as often as needed)
 *  3. start_store()
 *  4. write() (same sequence as before)
 *  5. finish_store()
 * 
 * The two store passes are required in order to measure the size on the first
 * pass. If the size increases between the first and second pass, finish_store()
 * will return an error.
 */
class ConfigManager {
public:
    /**
     * @brief Starts a load operation. This can be called at any time, even half
     * way through a previous load operation.
     */
    bool start_load() {
        if (NVM_init() != 0) {
            return (load_state = kLoadStateFailed), false;
        }
        load_offset = 0;
        load_crc16 = CONFIG_CRC16_INIT ^ config_version;
        load_state = kLoadStateInProgress;
        return true;
    }

    /**
     * @brief Loads the next chunk from NVM.
     * Note that this may return true even if invalid data was read. The user
     * will know the final verdict by the return value of finish_load().
     */
    template<typename T>
    bool read(T* val) {
        if (load_state != 1) {
            return (load_state = kLoadStateFailed), false;
        }
        size_t size = sizeof(T);
        if (NVM_read(load_offset, (uint8_t *)val, size) != 0)
            return (load_state = kLoadStateFailed), false;
        load_crc16 = calc_crc16<CONFIG_CRC16_POLYNOMIAL>(load_crc16, (uint8_t *)val, size);
        load_offset += size;
        return true;
    }

    /**
     * @brief Checks the final state of the load operation.
     * If this function returns false, it is possible that previous read()
     * operations actually returned garbage.
     */
    bool finish_load(size_t* occupied_size) {
        if (occupied_size) {
            *occupied_size = load_offset + 2;
        }

        uint16_t crc16_calculated = load_crc16;
        uint16_t crc16_loaded;
        if (!read(&crc16_loaded)) {
            return (load_state = kLoadStateFailed), false;
        }
        bool result = (load_state == 1) && (crc16_loaded == crc16_calculated);
        load_state = kLoadStateIdle;
        return result;
    }

    /**
     * @brief Starts preparation of a new store operation.
     */
    bool prepare_store() {
        if (store_state != kStoreStateIdle) {
            // it might be possible to restart the store process from other states but let's be safe
            return (store_state = kStoreStateFailed), false;
        }
        store_offset = 0;
        store_crc16 = CONFIG_CRC16_INIT ^ config_version;
        store_state = kStoreStatePreparing;
        return true;
    }

    template<typename T>
    bool write(T* val) {
        if (store_state == kStoreStateInProgress) {
            if (NVM_write(store_offset, (uint8_t*)val, sizeof(T)) != 0) {
                return (store_state = kStoreStateFailed), false;
            }
        } else if (store_state != kStoreStatePreparing) {
            return (store_state = kStoreStateFailed), false;
        }
        store_crc16 = calc_crc16<CONFIG_CRC16_POLYNOMIAL>(store_crc16, (uint8_t *)val, sizeof(T));
        store_offset += sizeof(T);
        return true;
    }

    /**
     * @brief Finishes the prepare pass and starts the actual store pass.
     */
    bool start_store(size_t* occupied_size) {
        if (occupied_size) {
            *occupied_size = store_offset + 2;
        }

        if (store_state != kStoreStatePreparing) {
            return (store_state = kStoreStateFailed), false;
        }
        store_offset += 2; // account for CRC16
        if (store_offset > NVM_get_max_write_length()) {
            return (store_state = kStoreStateFailed), false;
        }
        if (NVM_start_write(store_offset) != 0) {
            return (store_state = kStoreStateFailed), false;
        }
        store_offset = 0;
        store_crc16 = CONFIG_CRC16_INIT ^ config_version;
        store_state = kStoreStateInProgress;
        return true;
    }

    /**
     * @brief Commits the store operation.
     * If this function succeeds, the new configuration was successfully saved.
     * If this function fails, the old configuration was not touched.
     */
    bool finish_store() {
        uint16_t crc16 = store_crc16;
        if (!write(&crc16)) {
            return (store_state = kStoreStateFailed), false;
        }
        if (NVM_commit() != 0) {
            return (store_state = kStoreStateFailed), false;
        }
        store_state = kStoreStateIdle;
        return true;
    }

    enum {
        kLoadStateIdle = 0,
        kLoadStateInProgress = 1,
        kLoadStateFailed = 2
    } load_state = kLoadStateIdle;
    size_t load_offset;
    size_t load_crc16;

    enum {
        kStoreStateIdle = 0,
        kStoreStatePreparing = 1,
        kStoreStateInProgress = 2,
        kStoreStateFailed = 3
    } store_state = kStoreStateIdle;
    size_t store_offset;
    size_t store_crc16;
};
