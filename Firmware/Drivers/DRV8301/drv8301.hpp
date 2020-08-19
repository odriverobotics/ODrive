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
#ifndef _DRV8301_HPP_
#define _DRV8301_HPP_

//! \file   drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8301 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "stdbool.h"
#include "stdint.h"

// drivers

#include "stm32f4xx_hal.h"

#include <Drivers/gate_driver.hpp>
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include <Drivers/STM32/stm32_gpio.hpp>


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8301_ADDR_MASK               (0x7800)


//! \brief Defines the data mask
//!
#define DRV8301_DATA_MASK               (0x07FF)


//! \brief Defines the R/W mask
//!
#define DRV8301_RW_MASK                 (0x8000)


//! \brief Defines the R/W mask
//!
#define DRV8301_FAULT_TYPE_MASK         (0x07FF)


//! \brief Defines the location of the FETLC_OC (FET Low side, Phase C Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETLC_OC_BITS   (1 << 0)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase C Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETHC_OC_BITS   (1 << 1)

//! \brief Defines the location of the FETLC_OC (FET Low side, Phase B Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETLB_OC_BITS   (1 << 2)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase B Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETHB_OC_BITS   (1 << 3)

//! \brief Defines the location of the FETLC_OC (FET Low side, Phase A Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETLA_OC_BITS   (1 << 4)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase A Over Current) bits in the Status 1 register
//!
#define DRV8301_STATUS1_FETHA_OC_BITS   (1 << 5)

//! \brief Defines the location of the OTW (Over Temperature Warning) bits in the Status 1 register
//!
#define DRV8301_STATUS1_OTW_BITS        (1 << 6)

//! \brief Defines the location of the OTSD (Over Temperature Shut Down) bits in the Status 1 register
//!
#define DRV8301_STATUS1_OTSD_BITS       (1 << 7)

//! \brief Defines the location of the PVDD_UV (Power supply Vdd, Under Voltage) bits in the Status 1 register
//!
#define DRV8301_STATUS1_PVDD_UV_BITS    (1 << 8)

//! \brief Defines the location of the GVDD_UV (DRV8301 Vdd, Under Voltage) bits in the Status 1 register
//!
#define DRV8301_STATUS1_GVDD_UV_BITS    (1 << 9)

//! \brief Defines the location of the FAULT bits in the Status 1 register
//!
#define DRV8301_STATUS1_FAULT_BITS      (1 << 10)


//! \brief Defines the location of the Device ID bits in the Status 2 register
//!
#define DRV8301_STATUS2_ID_BITS        (15 << 0)

//! \brief Defines the location of the GVDD_OV (DRV8301 Vdd, Over Voltage) bits in the Status 2 register
//!
#define DRV8301_STATUS2_GVDD_OV_BITS    (1 << 7)


//! \brief Defines the location of the GATE_CURRENT bits in the Control 1 register
//!
#define DRV8301_CTRL1_GATE_CURRENT_BITS  (3 << 0)

//! \brief Defines the location of the GATE_RESET bits in the Control 1 register
//!
#define DRV8301_CTRL1_GATE_RESET_BITS    (1 << 2)

//! \brief Defines the location of the PWM_MODE bits in the Control 1 register
//!
#define DRV8301_CTRL1_PWM_MODE_BITS      (1 << 3)

//! \brief Defines the location of the OC_MODE bits in the Control 1 register
//!
#define DRV8301_CTRL1_OC_MODE_BITS       (3 << 4)

//! \brief Defines the location of the OC_ADJ bits in the Control 1 register
//!
#define DRV8301_CTRL1_OC_ADJ_SET_BITS   (31 << 6)


//! \brief Defines the location of the OCTW_SET bits in the Control 2 register
//!
#define DRV8301_CTRL2_OCTW_SET_BITS      (3 << 0)

//! \brief Defines the location of the GAIN bits in the Control 2 register
//!
#define DRV8301_CTRL2_GAIN_BITS          (3 << 2)

//! \brief Defines the location of the DC_CAL_1 bits in the Control 2 register
//!
#define DRV8301_CTRL2_DC_CAL_1_BITS      (1 << 4)

//! \brief Defines the location of the DC_CAL_2 bits in the Control 2 register
//!
#define DRV8301_CTRL2_DC_CAL_2_BITS      (1 << 5)

//! \brief Defines the location of the OC_TOFF bits in the Control 2 register
//!
#define DRV8301_CTRL2_OC_TOFF_BITS       (1 << 6)


#ifdef __cplusplus
}
#endif // extern "C"


class Drv8301 : public GateDriverBase, public OpAmpBase {
public:
    typedef enum {
        FaultType_NoFault  = (0 << 0),  //!< No fault
        FaultType_FETLC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
        FaultType_FETHC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
        FaultType_FETLB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
        FaultType_FETHB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
        FaultType_FETLA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
        FaultType_FETHA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
        FaultType_OTW      = (1 << 6),  //!< Over Temperature Warning fault
        FaultType_OTSD     = (1 << 7),  //!< Over Temperature Shut Down fault
        FaultType_PVDD_UV  = (1 << 8),  //!< Power supply Vdd Under Voltage fault
        FaultType_GVDD_UV  = (1 << 9),  //!< DRV8301 Vdd Under Voltage fault
        FaultType_GVDD_OV  = (1 << 10)  //!< DRV8301 Vdd Over Voltage fault
    } FaultType_e;

    Drv8301(Stm32SpiArbiter* spi_arbiter, Stm32Gpio ncs_gpio,
            Stm32Gpio enable_gpio, Stm32Gpio nfault_gpio)
        : spi_arbiter_(spi_arbiter), ncs_gpio_(ncs_gpio),
          enable_gpio_(enable_gpio), nfault_gpio_(nfault_gpio) {}
    
    /**
     * @brief Initializes the gate driver to a hardcoded default configuration.
     * Returns true on success or false otherwise (e.g. if the gate driver is
     * not connected).
     */
    bool init();
    
    bool set_gain(float requested_gain, float* actual_gain) final;
    bool check_fault() final;
    bool set_enabled(bool enabled) final { return true; }
    FaultType_e get_error();

    float get_midpoint() final {
        return 0.5f; // [V]
    }

    float get_max_output_swing() final {
        return 1.35f / 1.65f; // +-1.35V, normalized from a scale of +-1.65V to +-0.5
    }

private:
    enum CtrlMode_e {
        DRV8301_CtrlMode_Read = 1 << 15,   //!< Read Mode
        DRV8301_CtrlMode_Write = 0 << 15   //!< Write Mode
    };

    enum RegName_e {
        RegName_Status_1  = 0 << 11,   //!< Status Register 1
        RegName_Status_2  = 1 << 11,   //!< Status Register 2
        RegName_Control_1 = 2 << 11,  //!< Control Register 1
        RegName_Control_2 = 3 << 11   //!< Control Register 2
    };

    //! \brief Enumeration for the DC calibration modes
    enum DcCalMode_e {
        DcCalMode_Ch1_Load   = (0 << 4),   //!< Shunt amplifier 1 connected to load via input pins
        DcCalMode_Ch1_NoLoad = (1 << 4),   //!< Shunt amplifier 1 disconnected from load and input pins are shorted
        DcCalMode_Ch2_Load   = (0 << 5),   //!< Shunt amplifier 2 connected to load via input pins
        DcCalMode_Ch2_NoLoad = (1 << 5)    //!< Shunt amplifier 2 disconnected from load and input pins are shorted
    };

    //! \brief Enumeration for the Over Current modes
    enum OcMode_e {
        OcMode_CurrentLimit  = 0 << 4,   //!< current limit when OC detected
        OcMode_LatchShutDown = 1 << 4,   //!< latch shut down when OC detected
        OcMode_ReportOnly    = 2 << 4,   //!< report only when OC detected
        OcMode_Disabled      = 3 << 4    //!< OC protection disabled
    };

    //! \brief Enumeration for the Over Current Off Time modes
    enum OcOffTimeMode_e {
        OcOffTimeMode_Normal  = 0 << 6,   //!< normal CBC operation
        OcOffTimeMode_Ctrl    = 1 << 6    //!< off time control during OC
    };

    //! \brief Enumeration for the Over Current, Temperature Warning modes
    enum OcTwMode_e {
        OcTwMode_Both    = 0 << 0,   //!< report both OT and OC at /OCTW pin
        OcTwMode_OT_Only = 1 << 0,   //!< report only OT at /OCTW pin
        OcTwMode_OC_Only = 2 << 0    //!< report only OC at /OCTW pin
    };

    //! \brief Enumeration for the drv8301 peak current levels
    enum PeakCurrent_e {
        PeakCurrent_1p70_A  = 0 << 0,   //!< drv8301 driver peak current 1.70A
        PeakCurrent_0p70_A  = 1 << 0,   //!< drv8301 driver peak current 0.70A
        PeakCurrent_0p25_A  = 2 << 0    //!< drv8301 driver peak current 0.25A
    };

    //! \brief Enumeration for the PWM modes
    enum PwmMode_e {
        PwmMode_Six_Inputs   = 0 << 3,   //!< six independent inputs
        PwmMode_Three_Inputs = 1 << 3    //!< three independent nputs
    };

    //! \brief Enumeration for the shunt amplifier gains
    enum Reset_e {
        Reset_Normal = 0 << 2,   //!< normal
        Reset_All = 1 << 2       //!< reset all
    };

    //! \brief Enumeration for the shunt amplifier gains
    enum ShuntAmpGain_e {
        ShuntAmpGain_10VpV = 0 << 2,   //!< 10 V per V
        ShuntAmpGain_20VpV = 1 << 2,   //!< 20 V per V
        ShuntAmpGain_40VpV = 2 << 2,   //!< 40 V per V
        ShuntAmpGain_80VpV = 3 << 2    //!< 80 V per V
    };

    //! \brief Enumeration for the shunt amplifier number
    enum ShuntAmpNumber_e {
        ShuntAmpNumber_1 = 1,      //!< Shunt amplifier number 1
        ShuntAmpNumber_2 = 2       //!< Shunt amplifier number 2
    };

    //! \brief Enumeration for the Vds level for th over current adjustment
    enum VdsLevel_e {
        VdsLevel_0p060_V =  0 << 6,      //!< Vds = 0.060 V
        VdsLevel_0p068_V =  1 << 6,      //!< Vds = 0.068 V
        VdsLevel_0p076_V =  2 << 6,      //!< Vds = 0.076 V
        VdsLevel_0p086_V =  3 << 6,      //!< Vds = 0.086 V
        VdsLevel_0p097_V =  4 << 6,      //!< Vds = 0.097 V
        VdsLevel_0p109_V =  5 << 6,      //!< Vds = 0.109 V
        VdsLevel_0p123_V =  6 << 6,      //!< Vds = 0.123 V
        VdsLevel_0p138_V =  7 << 6,      //!< Vds = 0.138 V
        VdsLevel_0p155_V =  8 << 6,      //!< Vds = 0.155 V
        VdsLevel_0p175_V =  9 << 6,      //!< Vds = 0.175 V
        VdsLevel_0p197_V = 10 << 6,      //!< Vds = 0.197 V
        VdsLevel_0p222_V = 11 << 6,      //!< Vds = 0.222 V
        VdsLevel_0p250_V = 12 << 6,      //!< Vds = 0.250 V
        VdsLevel_0p282_V = 13 << 6,      //!< Vds = 0.282 V
        VdsLevel_0p317_V = 14 << 6,      //!< Vds = 0.317 V
        VdsLevel_0p358_V = 15 << 6,      //!< Vds = 0.358 V
        VdsLevel_0p403_V = 16 << 6,      //!< Vds = 0.403 V
        VdsLevel_0p454_V = 17 << 6,      //!< Vds = 0.454 V
        VdsLevel_0p511_V = 18 << 6,      //!< Vds = 0.511 V
        VdsLevel_0p576_V = 19 << 6,      //!< Vds = 0.576 V
        VdsLevel_0p648_V = 20 << 6,      //!< Vds = 0.648 V
        VdsLevel_0p730_V = 21 << 6,      //!< Vds = 0.730 V
        VdsLevel_0p822_V = 22 << 6,      //!< Vds = 0.822 V
        VdsLevel_0p926_V = 23 << 6,      //!< Vds = 0.926 V
        VdsLevel_1p043_V = 24 << 6,      //!< Vds = 1.403 V
        VdsLevel_1p175_V = 25 << 6,      //!< Vds = 1.175 V
        VdsLevel_1p324_V = 26 << 6,      //!< Vds = 1.324 V
        VdsLevel_1p491_V = 27 << 6,      //!< Vds = 1.491 V
        VdsLevel_1p679_V = 28 << 6,      //!< Vds = 1.679 V
        VdsLevel_1p892_V = 29 << 6,      //!< Vds = 1.892 V
        VdsLevel_2p131_V = 30 << 6,      //!< Vds = 2.131 V
        VdsLevel_2p400_V = 31 << 6       //!< Vds = 2.400 V
    };

    struct Registers_t {
        struct {
            bool FAULT;
            bool GVDD_UV;
            bool PVDD_UV;
            bool OTSD;
            bool OTW;
            bool FETHA_OC;
            bool FETLA_OC;
            bool FETHB_OC;
            bool FETLB_OC;
            bool FETHC_OC;
            bool FETLC_OC;
        } Stat_Reg_1;

        struct {
            bool GVDD_OV;
            uint16_t DeviceID;
        } Stat_Reg_2;

        struct {
            PeakCurrent_e DRV8301_CURRENT;
            Reset_e DRV8301_RESET;
            PwmMode_e PWM_MODE;
            OcMode_e OC_MODE;
            VdsLevel_e OC_ADJ_SET;
        } Ctrl_Reg_1;

        struct {
            OcTwMode_e OCTW_SET;
            ShuntAmpGain_e GAIN;
            DcCalMode_e DC_CAL_CH1p2;
            OcOffTimeMode_e OC_TOFF;
        } Ctrl_Reg_2;

        uint16_t Stat_Reg_1_Value;
        uint16_t Stat_Reg_2_Value;
        uint16_t Ctrl_Reg_1_Value;
        uint16_t Ctrl_Reg_2_Value;
    };

    //! \brief     Builds the control word
    //! \param[in] ctrlMode  The control mode
    //! \param[in] regName   The register name
    //! \param[in] data      The data
    //! \return    The control word
    static inline uint16_t build_ctrl_word(const CtrlMode_e ctrlMode,
                                                    const RegName_e regName,
                                                    const uint16_t data) {
        return ctrlMode | regName | (data & DRV8301_DATA_MASK);
    } // end of DRV8301_buildCtrlWord() function

    //! \brief     Reads data from the DRV8301 register
    //! \param[in] regName  The register name
    //! \return    The data value
    bool read_spi(const RegName_e regName, uint16_t* data);

    //! \brief     Writes data to the DRV8301 register
    //! \param[in] regName  The register name
    //! \param[in] data     The data value
    bool write_spi(const RegName_e regName, const uint16_t data);

    //! \brief     Interface to all 8301 SPI variables
    //!
    //! \details   Call this function periodically to be able to read the DRV8301 Status1, Status2,
    //!            Control1, and Control2 registers and write the Control1 and Control2 registers.
    //!            This function updates the members of the structure Registers_t.
    //!            <b>How to use in Setup</b>
    //!            <b>Code</b>
    //!            Add the structure declaration Registers_t to your code
    //!            Make sure the SPI and 8301 EN_Gate GPIO are setup for the 8301 by using HAL_init and HAL_setParams
    //!            During code setup, call HAL_enableDrv and HAL_setupDrvSpi
    //!            In background loop, call DRV8301_writeData and DRV8301_readData
    //!            <b>How to use in Runtime</b>
    //!            <b>Watch window</b>
    //!            Add the structure, declared by Registers_t above, to the watch window
    //!            <b>Runtime</b>
    //!            Pull down the menus from the Registers_t strcuture to the desired setting
    //!            Set SndCmd to send the settings to the DRV8301
    //!            If a read of the DRV8301 registers is required, se RcvCmd
    //!
    //! \param[in] regs  The (Registers_t) structure that contains all DRV8301 Status/Control register options
    bool write_regs(Registers_t *regs);

    //! \param[in] regs  The (Registers_t) structure that contains all DRV8301 Status/Control register options
    bool read_regs(Registers_t *regs);

    Stm32SpiArbiter* spi_arbiter_;
    Stm32Gpio ncs_gpio_;
    Stm32Gpio enable_gpio_;
    Stm32Gpio nfault_gpio_;

    // We don't put these buffers on the stack because we place the stack in
    // a RAM section which cannot be used by DMA.
    uint16_t tx_buf_;
    uint16_t rx_buf_;

    static const SPI_InitTypeDef spi_config_;
};


#endif // _DRV8301_HPP_
