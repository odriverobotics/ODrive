#ifndef __STM32_NVM_FILE_HPP
#define __STM32_NVM_FILE_HPP

#include <interfaces/file.hpp>

class Stm32NvmFile : public File {
public:
    constexpr static const size_t kHeaderSizeB = 8;

    Stm32NvmFile(const uint32_t* sector_ids, uint32_t n_sectors,
            uint32_t nvm_start, uint32_t nvm_length)
        : sector_ids_(sector_ids), n_sectors_(n_sectors),
          nvm_start_((const uint32_t*)nvm_start),
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

    const uint32_t* const sector_ids_;
    const uint32_t n_sectors_;
    const uint32_t* const nvm_start_;
    const uint32_t nvm_length_;

    uint32_t file_length_ = 0;
    uint32_t offset_ = 0; // read or write offset in bytes
    uint16_t crc16_; // used during writing
    uint32_t write_carry_ = 0xffffffff;
};

#endif //__STM32_NVM_FILE_HPP
