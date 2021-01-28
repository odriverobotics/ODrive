
#include "stm32_nvm_file.hpp"
#include "stm32_system.h"
#include <fibre/../../crc.hpp>
#include <string.h>
#include <algorithm>

#define NVM_CRC16_INIT 0xabcd
#define NVM_CRC16_POLYNOMIAL 0x3d65
#define NVM_FORMAT_VERSION 0x01

bool Stm32NvmFile::init() {
    bool is_erased = nvm_start_[0] & 0x80000000;
    uint16_t crc16 = nvm_start_[0] & 0xffff;
    uint8_t nvm_format_version = (nvm_start_[0] >> 24) & 0x7f;
    uint32_t length = nvm_start_[1];

    bool is_valid = !is_erased && (nvm_format_version == NVM_FORMAT_VERSION)
                && (nvm_format_version == NVM_FORMAT_VERSION)
                && (length <= nvm_length_ - kHeaderSizeB)
                && (calc_crc16<NVM_CRC16_POLYNOMIAL>(NVM_CRC16_INIT,
                    ((uint8_t *)nvm_start_) + kHeaderSizeB, length) == crc16);
                
    if (is_valid) {
        file_length_ = length;
        state_ = kStateValid;
        return true;

    } else {
        // The flash area to be erased but let's be save and verify this claim.
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
HAL_StatusTypeDef FLASH_write_word(uint8_t* flash_addr, uint32_t* data_addr) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)flash_addr, (uint32_t)data_addr);
}
#else
// F4, F7
HAL_StatusTypeDef FLASH_write_word(uint8_t* flash_addr, uint32_t* data_addr) {
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
    if (offset_ & 0x3) {
        size_t n_copy = std::min(length, (size_t)(4U - (offset_ & 3U)));
        memcpy((uint8_t*)write_carry_ + (offset_ & 0x3), buffer, n_copy);

        offset_ += n_copy;
        buffer += n_copy;
        length -= n_copy;

        if ((offset_ & 0x3) != 0) {
            return true; // Didn't fill carry word yet
        } else {
            if (FLASH_write_word((uint8_t*)nvm_start_ + kHeaderSizeB + offset_ - 4, &write_carry_) != HAL_OK) {
                return false;
            }
            write_carry_ = 0xffffffff;
        }
    }

    // Write 32-bit values
    for (; length >= 4; buffer += 4, offset_ += 4, length -=4) {
        if (FLASH_write_word((uint8_t*)nvm_start_ + kHeaderSizeB + offset_, (uint32_t*)buffer) != HAL_OK) {
            return false;
        }
    }

    // Handle unaligned end
    if (length) {
        memcpy((uint8_t*)write_carry_, buffer, length);
        offset_ += length;
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

        // If there was an unaligned end, write it out before closing
        if (offset_ & 3) {
            if (FLASH_write_word((uint8_t*)nvm_start_ + kHeaderSizeB + (offset_ & ~0x3), &write_carry_) != HAL_OK) {
                return false;
            }
            write_carry_ = 0;
        }

        uint32_t header0 = (NVM_FORMAT_VERSION << 24) | crc16_;
        uint32_t header1 = offset_;

        if (FLASH_write_word((uint8_t*)&nvm_start_[0], &header0) != HAL_OK) {
            return false;
        }
        if (FLASH_write_word((uint8_t*)&nvm_start_[1], &header1) != HAL_OK) {
            return false;
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

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    for (size_t i = 0; i < n_sectors_; ++i) {
        FLASH_EraseInitTypeDef erase_struct = {
            .TypeErase = FLASH_TYPEERASE_SECTORS,
#if defined(FLASH_OPTCR_nDBANK)
            .Banks = 0, // only used for mass erase
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
