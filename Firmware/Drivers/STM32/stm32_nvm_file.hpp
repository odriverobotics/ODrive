#ifndef __STM32_NVM_FILE_HPP
#define __STM32_NVM_FILE_HPP

#include <interfaces/file.hpp>
#include "stm32_system.h"

class Stm32NvmFile : public File {
public:
#if defined(STM32F4) || defined(STM32F7)
    constexpr static const size_t kFlashGranularity = 4;
#elif defined(STM32H7)
    constexpr static const size_t kFlashGranularity = 4 * FLASH_NB_32BITWORD_IN_FLASHWORD;
#endif

    Stm32NvmFile(uint32_t flash_bank, const uint32_t* sector_ids,
            uint32_t n_sectors, uint32_t nvm_start, uint32_t nvm_length)
        : flash_bank_(flash_bank),
          sector_ids_(sector_ids), n_sectors_(n_sectors),
          nvm_start_((const uint8_t*)nvm_start),
          nvm_length_(nvm_length) {}

    bool init();

private:
    bool is_valid() final;
    bool open_write() final;
    bool write(const uint8_t* buffer, size_t length) final;
    bool open_read(size_t* length) final;
    bool read(uint8_t* buffer, size_t length) final;
    bool close() final;
    bool erase() final;

    enum {
        kStateUninitialized,
        kStateErased,
        kStateWriting,
        kStateReading,
        kStateValid
    } state_;

    const uint32_t flash_bank_;
    const uint32_t* const sector_ids_;
    const uint32_t n_sectors_;
    const uint8_t* const nvm_start_;
    const uint32_t nvm_length_;

    uint32_t file_length_ = 0;
    uint32_t offset_ = 0; // read or write offset in bytes
    uint16_t crc16_; // used during writing

    uint8_t write_buf_[kFlashGranularity]; // used to handle unaligned sequential writes
};

#endif //__STM32_NVM_FILE_HPP
