/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c
//! \brief  Contains the various functions related to the DRV8301 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

// **************************************************************************
// the includes

#include "drv8301.hpp"
#include "utils.hpp"

#include "cmsis_os.h"
#include <math.h>
#include <array>
#include <algorithm>

const SPI_InitTypeDef Drv8301::spi_config_ = {
    .Mode = SPI_MODE_MASTER,
    .Direction = SPI_DIRECTION_2LINES,
    .DataSize = SPI_DATASIZE_16BIT,
    .CLKPolarity = SPI_POLARITY_LOW,
    .CLKPhase = SPI_PHASE_2EDGE,
    .NSS = SPI_NSS_SOFT,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
    .FirstBit = SPI_FIRSTBIT_MSB,
    .TIMode = SPI_TIMODE_DISABLE,
    .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .CRCPolynomial = 10,
};

bool Drv8301::init() {
    enable_gpio_.write(true);
    
    // Wait for driver to come online
    osDelay(10);

    // Make sure the Fault bit is not set during startup
    uint16_t reg;
    while (!read_spi(RegName_Status_1, &reg) || (reg & DRV8301_STATUS1_FAULT_BITS))
        ; // TODO: don't spin

    // Wait for the DRV8301 registers to update
    osDelay(1);

    return true;
}

Drv8301::FaultType_e Drv8301::get_error() {
    uint16_t readWord;
    FaultType_e faultType = FaultType_NoFault;

    // read the data
    if (!read_spi(RegName_Status_1, &readWord)) {
        return (FaultType_e)0xffff;
    }

    if (readWord & DRV8301_STATUS1_FAULT_BITS) {
        faultType = (FaultType_e)(readWord & DRV8301_FAULT_TYPE_MASK);

        if (faultType == FaultType_NoFault) {
            // read the data
            if (!read_spi(RegName_Status_2, &readWord)) {
                return (FaultType_e)0xffff;
            }

            if (readWord & DRV8301_STATUS2_GVDD_OV_BITS) {
                faultType = FaultType_GVDD_OV;
            }
        }
    }

    return faultType;
}

bool Drv8301::set_gain(float requested_gain, float* actual_gain) {
    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    // Snap down to have equal or larger range as requested or largest possible range otherwise

    // Decoding array for snapping gain
    std::array<std::pair<float, ShuntAmpGain_e>, 4> gain_choices = { 
        std::make_pair(10.0f, ShuntAmpGain_10VpV),
        std::make_pair(20.0f, ShuntAmpGain_20VpV),
        std::make_pair(40.0f, ShuntAmpGain_40VpV),
        std::make_pair(80.0f, ShuntAmpGain_80VpV)
    };

    // We use lower_bound in reverse because it snaps up by default, we want to snap down.
    auto gain_snap_down = std::lower_bound(gain_choices.crbegin(), gain_choices.crend(), requested_gain, 
    [](std::pair<float, ShuntAmpGain_e> pair, float val){
        return (bool)(pair.first > val);
    });

    // If we snap to outside the array, clip to smallest val
    if (gain_snap_down == gain_choices.crend())
        --gain_snap_down;
    
    Registers_t regs;
    if (!read_regs(&regs)) {
        return false;
    }

    regs.Ctrl_Reg_1.OC_MODE = OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    regs.Ctrl_Reg_1.OC_ADJ_SET = VdsLevel_0p730_V;
    regs.Ctrl_Reg_2.GAIN = gain_snap_down->second;

    if (!write_regs(&regs)) {
        return false;
    }

    if (actual_gain) {
        *actual_gain = gain_snap_down->first;
    }

    return true;
}

bool Drv8301::check_fault() {
    if (nfault_gpio_) {
        return nfault_gpio_.read();
    } else {
        return true;
    }
}

bool Drv8301::read_spi(const RegName_e regName, uint16_t* data) {
    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Read, regName, 0);
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000)) {
        return false;
    }

    // Datasheet says you don't have to pulse the nCS between transfers, (16
    // clocks should commit the transfer) but for some reason you actually need
    // to pulse it.
    delay_us(1);

    tx_buf_ = 0;
    rx_buf_ = 0xbeef;
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 1, 1000)) {
        return false;
    }

    delay_us(1);

    if (rx_buf_ == 0xbeef) {
        return false;
    }

    if (data) {
        *data = rx_buf_ & DRV8301_DATA_MASK;
    }

    return true;
}

bool Drv8301::write_spi(const RegName_e regName, const uint16_t data) {
    // Do blocking write
    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Write, regName, data);
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000)) {
        return false;
    }
    delay_us(1);

    return true;
}

bool Drv8301::write_regs(Registers_t *regs) {
    uint16_t ctrl1 = regs->Ctrl_Reg_1.DRV8301_CURRENT |
                     regs->Ctrl_Reg_1.DRV8301_RESET |
                     regs->Ctrl_Reg_1.PWM_MODE |
                     regs->Ctrl_Reg_1.OC_MODE |
                     regs->Ctrl_Reg_1.OC_ADJ_SET;

    uint16_t ctrl2 = regs->Ctrl_Reg_2.OCTW_SET |
                     regs->Ctrl_Reg_2.GAIN |
                     regs->Ctrl_Reg_2.DC_CAL_CH1p2 |
                     regs->Ctrl_Reg_2.OC_TOFF;

    return write_spi(RegName_Control_1, ctrl1)
        && write_spi(RegName_Control_2, ctrl2);
}

bool Drv8301::read_regs(Registers_t *regs) {
    bool success = true;
    uint16_t drvDataNew;

    // Update Status Register 1
    if (read_spi(RegName_Status_1, &drvDataNew)) {
        regs->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
        regs->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
        regs->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
        regs->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
        regs->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
        regs->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
        regs->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
        regs->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
        regs->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
        regs->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
        regs->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);
        regs->Stat_Reg_1_Value = drvDataNew;
    } else {
        success = false;
    }

    // Update Status Register 2
    if (read_spi(RegName_Status_2, &drvDataNew)) {
        regs->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
        regs->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);
        regs->Stat_Reg_2_Value = drvDataNew;
    } else {
        success = false;
    }

    // Update Control Register 1
    if (read_spi(RegName_Control_1, &drvDataNew)) {
        regs->Ctrl_Reg_1.DRV8301_CURRENT = (PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
        regs->Ctrl_Reg_1.DRV8301_RESET = (Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
        regs->Ctrl_Reg_1.PWM_MODE = (PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
        regs->Ctrl_Reg_1.OC_MODE = (OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
        regs->Ctrl_Reg_1.OC_ADJ_SET = (VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);
        regs->Ctrl_Reg_1_Value = drvDataNew;
    } else {
        success = false;
    }

    // Update Control Register 2
    if (read_spi(RegName_Control_2, &drvDataNew)) {
        regs->Ctrl_Reg_2.OCTW_SET = (OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
        regs->Ctrl_Reg_2.GAIN = (ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
        regs->Ctrl_Reg_2.DC_CAL_CH1p2 = (DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
        regs->Ctrl_Reg_2.OC_TOFF = (OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);
        regs->Ctrl_Reg_2_Value = drvDataNew;
    } else {
        success = false;
    }

    return success;
}
