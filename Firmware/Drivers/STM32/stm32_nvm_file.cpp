
#include "stm32_nvm_file.hpp"
#include "stm32_system.h"
#include <fibre/../../crc.hpp>
#include <string.h>
#include <algorithm>

#define NVM_CRC16_INIT 0xabcd
#define NVM_CRC16_POLYNOMIAL 0x3d65

//! This value is to be incremented when the NVM format changes between firmware
//! releases in a way that the new firmware would misinterpret NVM data stored
//! by the old firmware. This version only pretains to this low level NVM driver
//! and should not take into account the NVM file format defined by the
//! application. The application must provide its own appropriate mechanism to
//! allow for safe firmware upgrades.
#define NVM_FORMAT_VERSION 0x01

struct __attribute__((packed)) Header {
    uint8_t nvm_format_version; // see `NVM_FORMAT_VERSION`. A value of 0xff indicates that the NVM is erased.
    uint8_t reserved; // must be written as 0xff and ignored on reading
    uint16_t crc16; // checksum of the payload starting after the header
    uint32_t length; // length of the payload
};

constexpr static const size_t kHeaderSizeB = std::max(sizeof(Header), Stm32NvmFile::kFlashGranularity);


static const uint32_t FLASH_ERR_FLAGS =
#if defined(FLASH_FLAG_EOP)
        FLASH_FLAG_EOP |
#endif
#if defined(FLASH_FLAG_OPERR)
        FLASH_FLAG_OPERR |
#endif
#if defined(FLASH_FLAG_WRPERR)
        FLASH_FLAG_WRPERR |
#endif
#if defined(FLASH_FLAG_PGAERR)
        FLASH_FLAG_PGAERR |
#endif
#if defined(FLASH_FLAG_PGSERR)
        FLASH_FLAG_PGSERR |
#endif
#if defined(FLASH_FLAG_PGPERR)
        FLASH_FLAG_PGPERR |
#endif
        0;

static void HAL_FLASH_ClearErrors() {
    __HAL_FLASH_CLEAR_FLAG(FLASH_ERR_FLAGS);
}

bool Stm32NvmFile::init() {
    // If you get a hard fault on reading the header it could be because the
    // NVM write operation did something bad (like writing twice to the same
    // block). In that case, you can erase the NVM to rectify the issue.
    //erase();

    Header header = *(Header*)nvm_start_;
    bool is_erased = header.nvm_format_version == 0xff;

    bool is_valid = !is_erased && (header.nvm_format_version == NVM_FORMAT_VERSION)
                && (header.nvm_format_version == NVM_FORMAT_VERSION)
                && (header.length <= nvm_length_ - kHeaderSizeB)
                && (calc_crc16<NVM_CRC16_POLYNOMIAL>(NVM_CRC16_INIT,
                    ((uint8_t *)nvm_start_) + kHeaderSizeB, header.length) == header.crc16);
                
    if (is_valid) {
        file_length_ = header.length;
        state_ = kStateValid;
        return true;

    } else {
        // The flash area claims to be erased but let's be save and verify this claim.
        for (size_t i = 0; i < nvm_length_; i += 4) {
            if (nvm_start_[i >> 2] != 0xffffffff) {
                // The flash was in an inconsistent state. Erase now.
                return erase();
            }
        }

        // The flash area is fully erased
        state_ = kStateErased;
        file_length_ = 0;
        return true;
    }
}

bool Stm32NvmFile::is_valid() {
    return (state_ == kStateValid) || (state_ == kStateReading);
}

bool Stm32NvmFile::open_write() {
    if (state_ != kStateErased) {
        return false;
    }

    HAL_FLASH_ClearErrors();

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    state_ = kStateWriting;
    offset_ = 0;
    crc16_ = NVM_CRC16_INIT;
    return true;
}

#ifdef FLASH_TYPEPROGRAM_FLASHWORD
// H7
HAL_StatusTypeDef FLASH_write_block(uint8_t* flash_addr, uint32_t* data_addr) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)flash_addr, (uint32_t)data_addr);
}
#else
// F4, F7
HAL_StatusTypeDef FLASH_write_block(uint8_t* flash_addr, uint32_t* data_addr) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)flash_addr, *data_addr);
}
#endif


bool Stm32NvmFile::write(const uint8_t* buffer, size_t length) {
    if (state_ != kStateWriting) {
        return false;
    }

    if (length > nvm_length_ - offset_ - kHeaderSizeB) {
        return false;
    }

    crc16_ = calc_crc16<NVM_CRC16_POLYNOMIAL>(crc16_, buffer, length);

    // Handle unaligned start
    if (offset_ % kFlashGranularity) {
        size_t n_copy = std::min(length, (size_t)(kFlashGranularity - (offset_ % kFlashGranularity)));
        memcpy(write_buf_ + (offset_ % kFlashGranularity), buffer, n_copy);

        offset_ += n_copy;
        buffer += n_copy;
        length -= n_copy;

        if (offset_ % kFlashGranularity) {
            return true; // Block not yet full
        }

        if (FLASH_write_block((uint8_t*)nvm_start_ + kHeaderSizeB + (offset_ / kFlashGranularity - 1) * kFlashGranularity, (uint32_t*)write_buf_) != HAL_OK) {
            return false;
        }
    }

    // Write blocks of the size of kFlashGranularity
    while (length >= kFlashGranularity) {
        if (FLASH_write_block((uint8_t*)nvm_start_ + kHeaderSizeB + offset_, (uint32_t*)buffer) != HAL_OK) {
            return false;
        }
        offset_ += kFlashGranularity;
        buffer += kFlashGranularity;
        length -=kFlashGranularity;
    }

    // Handle unaligned end
    // On the H7 family we're not allowed to write to the same block multiple
    // times because it would trip the ECC. Therefore we have to coalesce
    // unaligned write operations into one.
    if (length) {
        memset(write_buf_, 0xff, sizeof(write_buf_));
        memcpy((uint8_t*)write_buf_, buffer, length);
        offset_ += length;
        // write_buf_ will be flushed on the next write() or close() call
    }
    
    return true;
}

bool Stm32NvmFile::open_read(size_t* length) {
    if (state_ != kStateValid) {
        return false;
    }

    if (length) {
        *length = file_length_;
    }

    state_ = kStateReading;
    offset_ = 0;
    return true;
}

bool Stm32NvmFile::read(uint8_t* buffer, size_t length) {
    if (state_ != kStateReading) {
        return false;
    }

    if (length > file_length_ - offset_) {
        return false;
    }

    memcpy(buffer, (const uint8_t*)nvm_start_ + kHeaderSizeB + offset_, length);
    offset_ += length;

    return true;
}

bool Stm32NvmFile::close() {
    if (state_ == kStateReading) {
        offset_ = 0;
        state_ = kStateValid;
        return true;

    } else if (state_ == kStateWriting) {
        // Flush unaligned write end if necessary
        if (offset_ % kFlashGranularity) {
            if (FLASH_write_block((uint8_t*)nvm_start_ + kHeaderSizeB + (offset_ / kFlashGranularity) * kFlashGranularity, (uint32_t*)write_buf_) != HAL_OK) {
                return false;
            }
        }

        // Write header to mark the NVM valid
        union PaddedHeader {
            Header header;
            uint8_t raw[kFlashGranularity];
        } padded_header;

        memset(&padded_header, 0xff, sizeof(padded_header));

        padded_header.header = {
            .nvm_format_version = NVM_FORMAT_VERSION,
            .reserved = 0xff,
            .crc16 = crc16_,
            .length = offset_
        };

        for (size_t i = 0; i < kHeaderSizeB; i += kFlashGranularity) {
            if (FLASH_write_block((uint8_t*)nvm_start_ + i, (uint32_t*)(padded_header.raw + i)) != HAL_OK) {
                return false;
            }
        }

        if (HAL_FLASH_Lock() != HAL_OK) {
            return false;
        }

        file_length_ = offset_;
        offset_ = 0;
        crc16_ = 0;
        state_ = kStateValid;
        return true;

    } else {
        return false;
    }
}

bool Stm32NvmFile::erase() {
    if ((state_ != kStateValid) && (state_ != kStateUninitialized)) {
        return false;
    }

    HAL_FLASH_ClearErrors();

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    for (size_t i = 0; i < n_sectors_; ++i) {
        FLASH_EraseInitTypeDef erase_struct = {
            .TypeErase = FLASH_TYPEERASE_SECTORS,
#if defined(FLASH_BANK_1)
            .Banks = flash_bank_,
#endif
            .Sector = sector_ids_[i],
            .NbSectors = 1,
#if defined(FLASH_VOLTAGE_RANGE_3)
            .VoltageRange = FLASH_VOLTAGE_RANGE_3
#endif
        };

        uint32_t sector_error = 0;
        if (HAL_FLASHEx_Erase(&erase_struct, &sector_error) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    if (HAL_FLASH_Lock() != HAL_OK) {
        return false;
    }

    state_ = kStateErased;
    file_length_ = 0;
    return true;
}
