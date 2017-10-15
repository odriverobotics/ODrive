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
#ifndef _DRV8301_H_
#define _DRV8301_H_

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

// Port
typedef SPI_HandleTypeDef* SPI_Handle;
typedef GPIO_TypeDef* GPIO_Handle;
typedef uint16_t GPIO_Number_e;


//!
//! \defgroup DRV8301

//!
//! \ingroup DRV8301
//@{


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


// **************************************************************************
// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum 
{
  DRV8301_CtrlMode_Read = 1 << 15,   //!< Read Mode
  DRV8301_CtrlMode_Write = 0 << 15   //!< Write Mode
} DRV8301_CtrlMode_e;


//! \brief Enumeration for the DC calibration modes
//!
typedef enum 
{
  DRV8301_DcCalMode_Ch1_Load   = (0 << 4),   //!< Shunt amplifier 1 connected to load via input pins
  DRV8301_DcCalMode_Ch1_NoLoad = (1 << 4),   //!< Shunt amplifier 1 disconnected from load and input pins are shorted
  DRV8301_DcCalMode_Ch2_Load   = (0 << 5),   //!< Shunt amplifier 2 connected to load via input pins
  DRV8301_DcCalMode_Ch2_NoLoad = (1 << 5)    //!< Shunt amplifier 2 disconnected from load and input pins are shorted
} DRV8301_DcCalMode_e;


//! \brief Enumeration for the fault types
//!
typedef enum 
{
  DRV8301_FaultType_NoFault  = (0 << 0),  //!< No fault
  DRV8301_FaultType_FETLC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
  DRV8301_FaultType_FETHC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
  DRV8301_FaultType_FETLB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
  DRV8301_FaultType_FETHB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
  DRV8301_FaultType_FETLA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
  DRV8301_FaultType_FETHA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
  DRV8301_FaultType_OTW      = (1 << 6),  //!< Over Temperature Warning fault
  DRV8301_FaultType_OTSD     = (1 << 7),  //!< Over Temperature Shut Down fault
  DRV8301_FaultType_PVDD_UV  = (1 << 8),  //!< Power supply Vdd Under Voltage fault
  DRV8301_FaultType_GVDD_UV  = (1 << 9),  //!< DRV8301 Vdd Under Voltage fault
  DRV8301_FaultType_GVDD_OV  = (1 << 10)  //!< DRV8301 Vdd Over Voltage fault
} DRV8301_FaultType_e;


//! \brief Enumeration for the Over Current modes
//!
typedef enum 
{
  DRV8301_OcMode_CurrentLimit  = 0 << 4,   //!< current limit when OC detected
  DRV8301_OcMode_LatchShutDown = 1 << 4,   //!< latch shut down when OC detected
  DRV8301_OcMode_ReportOnly    = 2 << 4,   //!< report only when OC detected
  DRV8301_OcMode_Disabled      = 3 << 4    //!< OC protection disabled
} DRV8301_OcMode_e;


//! \brief Enumeration for the Over Current Off Time modes
//!
typedef enum 
{
  DRV8301_OcOffTimeMode_Normal  = 0 << 6,   //!< normal CBC operation
  DRV8301_OcOffTimeMode_Ctrl    = 1 << 6    //!< off time control during OC
} DRV8301_OcOffTimeMode_e;


//! \brief Enumeration for the Over Current, Temperature Warning modes
//!
typedef enum 
{
  DRV8301_OcTwMode_Both    = 0 << 0,   //!< report both OT and OC at /OCTW pin
  DRV8301_OcTwMode_OT_Only = 1 << 0,   //!< report only OT at /OCTW pin
  DRV8301_OcTwMode_OC_Only = 2 << 0    //!< report only OC at /OCTW pin
} DRV8301_OcTwMode_e;


//! \brief Enumeration for the drv8301 peak current levels
//!
typedef enum 
{
  DRV8301_PeakCurrent_1p70_A  = 0 << 0,   //!< drv8301 driver peak current 1.70A
  DRV8301_PeakCurrent_0p70_A  = 1 << 0,   //!< drv8301 driver peak current 0.70A
  DRV8301_PeakCurrent_0p25_A  = 2 << 0    //!< drv8301 driver peak current 0.25A
} DRV8301_PeakCurrent_e;


//! \brief Enumeration for the PWM modes
//!
typedef enum 
{
  DRV8301_PwmMode_Six_Inputs   = 0 << 3,   //!< six independent inputs
  DRV8301_PwmMode_Three_Inputs = 1 << 3    //!< three independent nputs
} DRV8301_PwmMode_e;


//! \brief Enumeration for the register names
//!
typedef enum 
{
  DRV8301_RegName_Status_1  = 0 << 11,   //!< Status Register 1
  DRV8301_RegName_Status_2  = 1 << 11,   //!< Status Register 2
  DRV8301_RegName_Control_1 = 2 << 11,  //!< Control Register 1
  DRV8301_RegName_Control_2 = 3 << 11   //!< Control Register 2
} DRV8301_RegName_e;



//! \brief Enumeration for the shunt amplifier gains
//!
typedef enum 
{
  DRV8301_Reset_Normal = 0 << 2,   //!< normal
  DRV8301_Reset_All = 1 << 2       //!< reset all
} DRV8301_Reset_e;


//! \brief Enumeration for the shunt amplifier gains
//!
typedef enum 
{
  DRV8301_ShuntAmpGain_10VpV = 0 << 2,   //!< 10 V per V
  DRV8301_ShuntAmpGain_20VpV = 1 << 2,   //!< 20 V per V
  DRV8301_ShuntAmpGain_40VpV = 2 << 2,   //!< 40 V per V
  DRV8301_ShuntAmpGain_80VpV = 3 << 2    //!< 80 V per V
} DRV8301_ShuntAmpGain_e;


//! \brief Enumeration for the shunt amplifier number
//!
typedef enum 
{
  DRV8301_ShuntAmpNumber_1 = 1,      //!< Shunt amplifier number 1
  DRV8301_ShuntAmpNumber_2 = 2       //!< Shunt amplifier number 2
} DRV8301_ShuntAmpNumber_e;


//! \brief Enumeration for the Vds level for th over current adjustment
//!
typedef enum 
{
  DRV8301_VdsLevel_0p060_V =  0 << 6,      //!< Vds = 0.060 V
  DRV8301_VdsLevel_0p068_V =  1 << 6,      //!< Vds = 0.068 V
  DRV8301_VdsLevel_0p076_V =  2 << 6,      //!< Vds = 0.076 V
  DRV8301_VdsLevel_0p086_V =  3 << 6,      //!< Vds = 0.086 V
  DRV8301_VdsLevel_0p097_V =  4 << 6,      //!< Vds = 0.097 V
  DRV8301_VdsLevel_0p109_V =  5 << 6,      //!< Vds = 0.109 V
  DRV8301_VdsLevel_0p123_V =  6 << 6,      //!< Vds = 0.123 V
  DRV8301_VdsLevel_0p138_V =  7 << 6,      //!< Vds = 0.138 V
  DRV8301_VdsLevel_0p155_V =  8 << 6,      //!< Vds = 0.155 V
  DRV8301_VdsLevel_0p175_V =  9 << 6,      //!< Vds = 0.175 V
  DRV8301_VdsLevel_0p197_V = 10 << 6,      //!< Vds = 0.197 V
  DRV8301_VdsLevel_0p222_V = 11 << 6,      //!< Vds = 0.222 V
  DRV8301_VdsLevel_0p250_V = 12 << 6,      //!< Vds = 0.250 V
  DRV8301_VdsLevel_0p282_V = 13 << 6,      //!< Vds = 0.282 V
  DRV8301_VdsLevel_0p317_V = 14 << 6,      //!< Vds = 0.317 V
  DRV8301_VdsLevel_0p358_V = 15 << 6,      //!< Vds = 0.358 V
  DRV8301_VdsLevel_0p403_V = 16 << 6,      //!< Vds = 0.403 V
  DRV8301_VdsLevel_0p454_V = 17 << 6,      //!< Vds = 0.454 V
  DRV8301_VdsLevel_0p511_V = 18 << 6,      //!< Vds = 0.511 V
  DRV8301_VdsLevel_0p576_V = 19 << 6,      //!< Vds = 0.576 V
  DRV8301_VdsLevel_0p648_V = 20 << 6,      //!< Vds = 0.648 V
  DRV8301_VdsLevel_0p730_V = 21 << 6,      //!< Vds = 0.730 V
  DRV8301_VdsLevel_0p822_V = 22 << 6,      //!< Vds = 0.822 V
  DRV8301_VdsLevel_0p926_V = 23 << 6,      //!< Vds = 0.926 V
  DRV8301_VdsLevel_1p043_V = 24 << 6,      //!< Vds = 1.403 V
  DRV8301_VdsLevel_1p175_V = 25 << 6,      //!< Vds = 1.175 V
  DRV8301_VdsLevel_1p324_V = 26 << 6,      //!< Vds = 1.324 V
  DRV8301_VdsLevel_1p491_V = 27 << 6,      //!< Vds = 1.491 V
  DRV8301_VdsLevel_1p679_V = 28 << 6,      //!< Vds = 1.679 V
  DRV8301_VdsLevel_1p892_V = 29 << 6,      //!< Vds = 1.892 V
  DRV8301_VdsLevel_2p131_V = 30 << 6,      //!< Vds = 2.131 V
  DRV8301_VdsLevel_2p400_V = 31 << 6       //!< Vds = 2.400 V
} DRV8301_VdsLevel_e;


typedef enum
{
  DRV8301_GETID=0
} Drv8301SpiOutputDataSelect_e;


typedef struct _DRV_SPI_8301_Stat1_t_
{
  bool                  FAULT;
  bool                  GVDD_UV;
  bool                  PVDD_UV;
  bool                  OTSD;
  bool                  OTW;
  bool                  FETHA_OC;
  bool                  FETLA_OC;
  bool                  FETHB_OC;
  bool                  FETLB_OC;
  bool                  FETHC_OC;
  bool                  FETLC_OC;
}DRV_SPI_8301_Stat1_t_;


typedef struct _DRV_SPI_8301_Stat2_t_
{
  bool                  GVDD_OV;
  uint16_t              DeviceID;
}DRV_SPI_8301_Stat2_t_;


typedef struct _DRV_SPI_8301_CTRL1_t_
{
  DRV8301_PeakCurrent_e    DRV8301_CURRENT;
  DRV8301_Reset_e          DRV8301_RESET;
  DRV8301_PwmMode_e        PWM_MODE;
  DRV8301_OcMode_e         OC_MODE;
  DRV8301_VdsLevel_e       OC_ADJ_SET;
}DRV_SPI_8301_CTRL1_t_;


typedef struct _DRV_SPI_8301_CTRL2_t_
{
  DRV8301_OcTwMode_e       OCTW_SET;
  DRV8301_ShuntAmpGain_e   GAIN;
  DRV8301_DcCalMode_e      DC_CAL_CH1p2;
  DRV8301_OcOffTimeMode_e  OC_TOFF;
}DRV_SPI_8301_CTRL2_t_;


typedef struct _DRV_SPI_8301_Vars_t_
{
  DRV_SPI_8301_Stat1_t_     Stat_Reg_1;
  DRV_SPI_8301_Stat2_t_     Stat_Reg_2;
  DRV_SPI_8301_CTRL1_t_     Ctrl_Reg_1;
  DRV_SPI_8301_CTRL2_t_     Ctrl_Reg_2;
  bool                  SndCmd;
  bool                  RcvCmd;
}DRV_SPI_8301_Vars_t;


//! \brief Defines the DRV8301 object
//!
typedef struct _DRV8301_Obj_
{
  SPI_Handle       spiHandle;                  //!< the handle for the serial peripheral interface
  GPIO_Handle      EngpioHandle;               //!< the gpio handle that is connected to the drv8301 enable pin
  GPIO_Number_e    EngpioNumber;               //!< the gpio number that is connected to the drv8301 enable pin
  GPIO_Handle      nCSgpioHandle;              //!< the gpio handle that is connected to the drv8301 nCS pin
  GPIO_Number_e    nCSgpioNumber;               //!< the gpio number that is connected to the drv8301 nCS pin
  bool             RxTimeOut;                  //!< the timeout flag for the RX fifo
  bool             enableTimeOut;              //!< the timeout flag for drv8301 enable
} DRV8301_Obj;


//! \brief Defines the DRV8301 handle
//!
typedef struct _DRV8301_Obj_ *DRV8301_Handle;


//! \brief Defines the DRV8301 Word type
//!
typedef  uint16_t    DRV8301_Word_t;


// **************************************************************************
// the globals



// **************************************************************************
// the function prototypes


//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8301_Word_t DRV8301_buildCtrlWord(const DRV8301_CtrlMode_e ctrlMode,
                                                   const DRV8301_RegName_e regName,
                                                   const uint16_t data)
{
  DRV8301_Word_t ctrlWord = ctrlMode | regName | (data & DRV8301_DATA_MASK);

  return(ctrlWord);
} // end of DRV8301_buildCtrlWord() function


//! \brief     Gets the DC calibration mode
//! \param[in] handle     The DRV8301 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \return    The DC calibration mode
extern DRV8301_DcCalMode_e DRV8301_getDcCalMode(DRV8301_Handle handle,
                                                const DRV8301_ShuntAmpNumber_e ampNumber);

//! \brief     Enables the DRV8301
//! \param[in] handle     The DRV8301 handle
extern void DRV8301_enable(DRV8301_Handle handle);


//! \brief     Gets the fault type
//! \param[in] handle     The DRV8301 handle
//! \return    The fault type
extern DRV8301_FaultType_e DRV8301_getFaultType(DRV8301_Handle handle);


//! \brief     Gets the device ID
//! \param[in] handle     The DRV8301 handle
//! \return    The device ID
extern uint16_t DRV8301_getId(DRV8301_Handle handle);


//! \brief     Gets the over current level
//! \param[in] handle     The DRV8301 handle
//! \return    The over current level, V
extern DRV8301_VdsLevel_e DRV8301_getOcLevel(DRV8301_Handle handle);


//! \brief     Gets the over current mode
//! \param[in] handle     The DRV8301 handle
//! \return    The over current mode
extern DRV8301_OcMode_e DRV8301_getOcMode(DRV8301_Handle handle);


//! \brief     Gets the over current off time mode
//! \param[in] handle     The DRV8301 handle
//! \return    The over current off time mode
extern DRV8301_OcOffTimeMode_e DRV8301_getOcOffTimeMode(DRV8301_Handle handle);


//! \brief     Gets the over current, temperature warning mode
//! \param[in] handle     The DRV8301 handle
//! \return    The over current, temperature warning mode
extern DRV8301_OcTwMode_e DRV8301_getOcTwMode(DRV8301_Handle handle);


//! \brief     Gets the peak current value
//! \param[in] handle     The DRV8301 handle
//! \return    The peak current value
extern DRV8301_PeakCurrent_e DRV8301_getPeakCurrent(DRV8301_Handle handle);


//! \brief     Gets the PWM mode
//! \param[in] handle     The DRV8301 handle
//! \return    The PWM mode
extern DRV8301_PwmMode_e DRV8301_getPwmMode(DRV8301_Handle handle);


//! \brief     Gets the shunt amplifier gain value
//! \param[in] handle     The DRV8301 handle
//! \return    The shunt amplifier gain value
extern DRV8301_ShuntAmpGain_e DRV8301_getShuntAmpGain(DRV8301_Handle handle);


//! \brief     Gets the status register 1 value
//! \param[in] handle     The DRV8301 handle
//! \return    The status register1 value
extern uint16_t DRV8301_getStatusRegister1(DRV8301_Handle handle);


//! \brief     Gets the status register 2 value
//! \param[in] handle     The DRV8301 handle
//! \return    The status register2 value
extern uint16_t DRV8301_getStatusRegister2(DRV8301_Handle handle);


//! \brief     Initializes the DRV8301 object
//! \param[in] pMemory   A pointer to the memory for the DRV8301 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8301 object, bytes
//! \return    The DRV8301 object handle
extern DRV8301_Handle DRV8301_init(void *pMemory,const size_t numBytes);


//! \brief     Determines if DRV8301 fault has occurred
//! \param[in] handle     The DRV8301 handle
//! \return    A boolean value denoting if a fault has occurred (true) or not (false)
extern bool DRV8301_isFault(DRV8301_Handle handle);


//! \brief     Determines if DRV8301 is in reset
//! \param[in] handle     The DRV8301 handle
//! \return    A boolean value denoting if the DRV8301 is in reset (true) or not (false)
extern bool DRV8301_isReset(DRV8301_Handle handle);


//! \brief     Reads data from the DRV8301 register
//! \param[in] handle   The DRV8301 handle
//! \param[in] regName  The register name
//! \return    The data value
extern uint16_t DRV8301_readSpi(DRV8301_Handle handle,const DRV8301_RegName_e regName);


//! \brief     Resets the DRV8301
//! \param[in] handle   The DRV8301 handle
extern void DRV8301_reset(DRV8301_Handle handle);


//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8301 handle
static inline void DRV8301_resetEnableTimeout(DRV8301_Handle handle)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  obj->enableTimeOut = false;

  return;
}


//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8301 handle
static inline void DRV8301_resetRxTimeout(DRV8301_Handle handle)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  obj->RxTimeOut = false;

  return;
}


//! \brief     Sets the DC calibration mode
//! \param[in] handle     The DRV8301 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \param[in] mode       The DC calibration mode
extern void DRV8301_setDcCalMode(DRV8301_Handle handle,
                                 const DRV8301_ShuntAmpNumber_e ampNumber,
                                 const DRV8301_DcCalMode_e mode);


//! \brief     Sets the GPIO handle in the DRV8301
//! \param[in] handle     The DRV8301 handle
//! \param[in] gpioHandle  The GPIO handle to use
void DRV8301_setGpioHandle(DRV8301_Handle handle,GPIO_Handle gpioHandle);


//! \brief     Sets the GPIO number in the DRV8301
//! \param[in] handle     The DRV8301 handle
//! \param[in] gpioHandle  The GPIO number to use
void DRV8301_setGpioNumber(DRV8301_Handle handle,GPIO_Number_e gpioNumber);


//! \brief     Sets the over current level in terms of Vds
//! \param[in] handle    The DRV8301 handle
//! \param[in] VdsLevel  The over current level, V
extern void DRV8301_setOcLevel(DRV8301_Handle handle,const DRV8301_VdsLevel_e VdsLevel);


//! \brief     Sets the over current mode
//! \param[in] handle  The DRV8301 handle
//! \param[in] mode    The over current mode
extern void DRV8301_setOcMode(DRV8301_Handle handle,const DRV8301_OcMode_e mode);


//! \brief     Sets the over current off time mode
//! \param[in] handle   The DRV8301 handle
//! \param[in] mode     The over current off time mode
extern void DRV8301_setOcOffTimeMode(DRV8301_Handle handle,const DRV8301_OcOffTimeMode_e mode);


//! \brief     Sets the over current, temperature warning mode
//! \param[in] handle  The DRV8301 handle
//! \param[in] mode    The over current, temperature warning mode
extern void DRV8301_setOcTwMode(DRV8301_Handle handle,const DRV8301_OcTwMode_e mode);


//! \brief     Sets the peak current value
//! \param[in] handle       The DRV8301 handle
//! \param[in] peakCurrent  The peak current value
extern void DRV8301_setPeakCurrent(DRV8301_Handle handle,const DRV8301_PeakCurrent_e peakCurrent);


//! \brief     Sets the PWM mode
//! \param[in] handle  The DRV8301 handle
//! \param[in] mode    The PWM mode
extern void DRV8301_setPwmMode(DRV8301_Handle handle,const DRV8301_PwmMode_e mode);


//! \brief     Sets the shunt amplifier gain value
//! \param[in] handle  The DRV8301 handle
//! \param[in] gain    The shunt amplifier gain value
extern void DRV8301_setShuntAmpGain(DRV8301_Handle handle,const DRV8301_ShuntAmpGain_e gain);


//! \brief     Sets the SPI handle in the DRV8301
//! \param[in] handle     The DRV8301 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8301_setSpiHandle(DRV8301_Handle handle,SPI_Handle spiHandle);


//! \brief     Writes data to the DRV8301 register
//! \param[in] handle   The DRV8301 handle
//! \param[in] regName  The register name
//! \param[in] data     The data value
extern void DRV8301_writeSpi(DRV8301_Handle handle,const DRV8301_RegName_e regName,const uint16_t data);


//! \brief     Interface to all 8301 SPI variables
//!
//! \details   Call this function periodically to be able to read the DRV8301 Status1, Status2,
//!            Control1, and Control2 registers and write the Control1 and Control2 registers.
//!            This function updates the members of the structure DRV_SPI_8301_Vars_t.
//!            <b>How to use in Setup</b>
//!            <b>Code</b>
//!            Add the structure declaration DRV_SPI_8301_Vars_t to your code
//!            Make sure the SPI and 8301 EN_Gate GPIO are setup for the 8301 by using HAL_init and HAL_setParams
//!            During code setup, call HAL_enableDrv and HAL_setupDrvSpi
//!            In background loop, call DRV8301_writeData and DRV8301_readData
//!            <b>How to use in Runtime</b>
//!            <b>Watch window</b>
//!            Add the structure, declared by DRV_SPI_8301_Vars_t above, to the watch window
//!            <b>Runtime</b>
//!            Pull down the menus from the DRV_SPI_8301_Vars_t strcuture to the desired setting
//!            Set SndCmd to send the settings to the DRV8301
//!            If a read of the DRV8301 registers is required, se RcvCmd
//!
//! \param[in] handle  The DRV8301 handle
//! \param[in] Spi_8301_Vars  The (DRV_SPI_8301_Vars_t) structure that contains all DRV8301 Status/Control register options
extern void DRV8301_writeData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);


//! \param[in] handle  The DRV8301 handle
//! \param[in] Spi_8301_Vars  The (DRV_SPI_8301_Vars_t) structure that contains all DRV8301 Status/Control register options
extern void DRV8301_readData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);


//! \brief     Initialize the interface to all 8301 SPI variables
//! \param[in] handle  The DRV8301 handle
extern void DRV8301_setupSpi(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _DRV8301_H_ definition





