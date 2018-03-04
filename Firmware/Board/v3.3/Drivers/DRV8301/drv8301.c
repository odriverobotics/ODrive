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

#include "assert.h"
#include <math.h>
#include "cmsis_os.h"

// drivers
#include "drv8301.h"

#include "utils.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

void DRV8301_enable(DRV8301_Handle handle)
{

  //Enable driver
  HAL_GPIO_WritePin(handle->EngpioHandle, handle->EngpioNumber, GPIO_PIN_SET);

  //Wait for driver to come online
  osDelay(10);

  // Make sure the Fault bit is not set during startup
  while((DRV8301_readSpi(handle,DRV8301_RegName_Status_1) & DRV8301_STATUS1_FAULT_BITS) != 0);

  // Wait for the DRV8301 registers to update
  osDelay(1);

  return;
}

DRV8301_DcCalMode_e DRV8301_getDcCalMode(DRV8301_Handle handle,const DRV8301_ShuntAmpNumber_e ampNumber)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

  // clear the bits
  if(ampNumber == DRV8301_ShuntAmpNumber_1)
    {
      data &= (~DRV8301_CTRL2_DC_CAL_1_BITS);

    }
  else if(ampNumber == DRV8301_ShuntAmpNumber_2)
    {
      data &= (~DRV8301_CTRL2_DC_CAL_2_BITS);
    }

  return((DRV8301_DcCalMode_e)data);
} // end of DRV8301_getDcCalMode() function


DRV8301_FaultType_e DRV8301_getFaultType(DRV8301_Handle handle)
{
  DRV8301_Word_t      readWord;
  DRV8301_FaultType_e faultType = DRV8301_FaultType_NoFault;


  // read the data
  readWord = DRV8301_readSpi(handle,DRV8301_RegName_Status_1);

  if(readWord & DRV8301_STATUS1_FAULT_BITS)
    {
      faultType = (DRV8301_FaultType_e)(readWord & DRV8301_FAULT_TYPE_MASK);

      if(faultType == DRV8301_FaultType_NoFault)
        {
          // read the data
          readWord = DRV8301_readSpi(handle,DRV8301_RegName_Status_2);

          if(readWord & DRV8301_STATUS2_GVDD_OV_BITS)
            {
              faultType = DRV8301_FaultType_GVDD_OV;
            }
        }
    }

  return(faultType);
} // end of DRV8301_getFaultType() function


uint16_t DRV8301_getId(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Status_2);

  // mask bits
  data &= DRV8301_STATUS2_ID_BITS;

  return(data);
} // end of DRV8301_getId() function


DRV8301_VdsLevel_e DRV8301_getOcLevel(DRV8301_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

  return((DRV8301_VdsLevel_e)data);
} // end of DRV8301_getOcLevel() function


DRV8301_OcMode_e DRV8301_getOcMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_MODE_BITS);

  return((DRV8301_OcMode_e)data);
} // end of DRV8301_getOcMode() function


DRV8301_OcOffTimeMode_e DRV8301_getOcOffTimeMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

  return((DRV8301_OcOffTimeMode_e)data);
} // end of DRV8301_getOcOffTimeMode() function


DRV8301_OcTwMode_e DRV8301_getOcTwMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

  return((DRV8301_OcTwMode_e)data);
} // end of DRV8301_getOcTwMode() function


DRV8301_PeakCurrent_e DRV8301_getPeakCurrent(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

  return((DRV8301_PeakCurrent_e)data);
} // end of DRV8301_getPeakCurrent() function


DRV8301_PwmMode_e DRV8301_getPwmMode(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_PWM_MODE_BITS);

  return((DRV8301_PwmMode_e)data);
} // end of DRV8301_getPwmMode() function


DRV8301_ShuntAmpGain_e DRV8301_getShuntAmpGain(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_GAIN_BITS);

  return((DRV8301_ShuntAmpGain_e)data);
} // end of DRV8301_getShuntAmpGain() function


DRV8301_Handle DRV8301_init(void *pMemory,const size_t numBytes)
{
  DRV8301_Handle handle;


  if(numBytes < sizeof(DRV8301_Obj))
    return((DRV8301_Handle)NULL);


  // assign the handle
  handle = (DRV8301_Handle)pMemory;

  DRV8301_resetRxTimeout(handle);
  DRV8301_resetEnableTimeout(handle);


  return(handle);
} // end of DRV8301_init() function


void DRV8301_setEnGpioHandle(DRV8301_Handle handle,GPIO_Handle gpioHandle)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  // initialize the gpio interface object
  obj->EngpioHandle = gpioHandle;

  return;
} // end of DRV8301_setGpioHandle() function


void DRV8301_setEnGpioNumber(DRV8301_Handle handle,GPIO_Number_e gpioNumber)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  // initialize the gpio interface object
  obj->EngpioNumber = gpioNumber;

  return;
} // end of DRV8301_setGpioNumber() function


void DRV8301_setnCSGpioHandle(DRV8301_Handle handle,GPIO_Handle gpioHandle)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  // initialize the gpio interface object
  obj->nCSgpioHandle = gpioHandle;

  return;
} // end of DRV8301_setGpioHandle() function


void DRV8301_setnCSGpioNumber(DRV8301_Handle handle,GPIO_Number_e gpioNumber)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  // initialize the gpio interface object
  obj->nCSgpioNumber = gpioNumber;

  return;
} // end of DRV8301_setGpioNumber() function


void DRV8301_setSpiHandle(DRV8301_Handle handle,SPI_Handle spiHandle)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  // initialize the serial peripheral interface object
  obj->spiHandle = spiHandle;

  return;
} // end of DRV8301_setSpiHandle() function


bool DRV8301_isFault(DRV8301_Handle handle)
{
  DRV8301_Word_t readWord;
  bool status=false;


  // read the data
  readWord = DRV8301_readSpi(handle,DRV8301_RegName_Status_1);

  if(readWord & DRV8301_STATUS1_FAULT_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8301_isFault() function


bool DRV8301_isReset(DRV8301_Handle handle)
{
  DRV8301_Word_t readWord;
  bool status=false;


  // read the data
  readWord = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  if(readWord & DRV8301_CTRL1_GATE_RESET_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8301_isReset() function


uint16_t DRV8301_readSpi(DRV8301_Handle handle, const DRV8301_RegName_e regName)
{

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  // Do blocking read
  uint16_t zerobuff = 0;
  uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Read, regName, 0);
  uint16_t recbuff = 0xbeef;
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);

  // Datasheet says you don't have to pulse the nCS between transfers, (16 clocks should commit the transfer)
  // but for some reason you actually need to pulse it.
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  HAL_SPI_TransmitReceive(handle->spiHandle, (uint8_t*)(&zerobuff), (uint8_t*)(&recbuff), 1, 1000);
  delay_us(1);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);

  assert(recbuff != 0xbeef);

  return(recbuff & DRV8301_DATA_MASK);
}  // end of DRV8301_readSpi() function


void DRV8301_reset(DRV8301_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // set the bits
  data |= DRV8301_CTRL1_GATE_RESET_BITS;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

  return;
}  // end of DRV8301_reset() function

  
void DRV8301_setDcCalMode(DRV8301_Handle handle,const DRV8301_ShuntAmpNumber_e ampNumber,const DRV8301_DcCalMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

  // clear the bits
  if(ampNumber == DRV8301_ShuntAmpNumber_1)
    {
      data &= (~DRV8301_CTRL2_DC_CAL_1_BITS);

    }
  else if(ampNumber == DRV8301_ShuntAmpNumber_2)
    {
      data &= (~DRV8301_CTRL2_DC_CAL_2_BITS);
    }

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_2,data);

  return;
} // end of DRV8301_setDcCalMode() function


void DRV8301_setOcLevel(DRV8301_Handle handle,const DRV8301_VdsLevel_e VdsLevel)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

  // set the bits
  data |= VdsLevel;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

  return;
} // end of DRV8301_setOcLevel() function


void DRV8301_setOcMode(DRV8301_Handle handle,const DRV8301_OcMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_OC_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

  return;
} // end of DRV8301_setOcMode() function


void DRV8301_setOcOffTimeMode(DRV8301_Handle handle,const DRV8301_OcOffTimeMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_2,data);

  return;
} // end of DRV8301_setOcOffTimeMode() function


void DRV8301_setOcTwMode(DRV8301_Handle handle,const DRV8301_OcTwMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_2,data);

  return;
} // end of DRV8301_setOcTwMode() function


void DRV8301_setPeakCurrent(DRV8301_Handle handle,const DRV8301_PeakCurrent_e peakCurrent)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

  // set the bits
  data |= peakCurrent;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

  return;
} // end of DRV8301_setPeakCurrent() function


void DRV8301_setPwmMode(DRV8301_Handle handle,const DRV8301_PwmMode_e mode)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

  // clear the bits
  data &= (~DRV8301_CTRL1_PWM_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

  return;
} // end of DRV8301_setPwmMode() function


void DRV8301_setShuntAmpGain(DRV8301_Handle handle,const DRV8301_ShuntAmpGain_e gain)
{
  uint16_t data;


  // read data
  data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

  // clear the bits
  data &= (~DRV8301_CTRL2_GAIN_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8301_writeSpi(handle,DRV8301_RegName_Control_2,data);

  return;
} // end of DRV8301_setShuntAmpGain() function


void DRV8301_writeSpi(DRV8301_Handle handle, const DRV8301_RegName_e regName,const uint16_t data)
{
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  // Do blocking write
  uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, regName, data);
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);
  delay_us(1);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);

  return;
}  // end of DRV8301_writeSpi() function


void DRV8301_writeData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;


  if(Spi_8301_Vars->SndCmd)
  {
    // Update Control Register 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT |  \
                 Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET   |  \
                 Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE     |  \
                 Spi_8301_Vars->Ctrl_Reg_1.OC_MODE      |  \
                 Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET;
    DRV8301_writeSpi(handle,drvRegName,drvDataNew);

    // Update Control Register 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET      |  \
                 Spi_8301_Vars->Ctrl_Reg_2.GAIN          |  \
                 Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2  |  \
                 Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF;
    DRV8301_writeSpi(handle,drvRegName,drvDataNew);

    Spi_8301_Vars->SndCmd = false;
  }

  return;
}  // end of DRV8301_writeData() function


void DRV8301_readData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;


  if(Spi_8301_Vars->RcvCmd)
  {
    // Update Status Register 1
    drvRegName = DRV8301_RegName_Status_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
    Spi_8301_Vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
    Spi_8301_Vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
    Spi_8301_Vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
    Spi_8301_Vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1_Value = drvDataNew;

    // Update Status Register 2
    drvRegName = DRV8301_RegName_Status_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
    Spi_8301_Vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);
    Spi_8301_Vars->Stat_Reg_2_Value = drvDataNew;

    // Update Control Register 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);
    Spi_8301_Vars->Ctrl_Reg_1_Value = drvDataNew;

    // Update Control Register 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
    Spi_8301_Vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
    Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
    Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);
    Spi_8301_Vars->Ctrl_Reg_2_Value = drvDataNew;
    Spi_8301_Vars->RcvCmd = false;
  }

  return;
}  // end of DRV8301_readData() function


void DRV8301_setupSpi(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;

//  Why impose hardcoded values?
//  Defaults should be device defaults or application level specified.
//  Setting other hardcoded here is just confusing!

#if 0
  // Update Control Register 1
  drvRegName = DRV8301_RegName_Control_1;
  drvDataNew = (DRV8301_PeakCurrent_0p25_A   | \
                DRV8301_Reset_Normal         | \
                DRV8301_PwmMode_Six_Inputs   | \
                DRV8301_OcMode_CurrentLimit  | \
                DRV8301_VdsLevel_0p730_V);
  DRV8301_writeSpi(handle,drvRegName,drvDataNew);

  // Update Control Register 2
  drvRegName = DRV8301_RegName_Control_2;
  drvDataNew = (DRV8301_OcTwMode_Both        | \
                DRV8301_ShuntAmpGain_10VpV   | \
                DRV8301_DcCalMode_Ch1_Load   | \
                DRV8301_DcCalMode_Ch2_Load   | \
                DRV8301_OcOffTimeMode_Normal);
  DRV8301_writeSpi(handle,drvRegName,drvDataNew);
#endif


  Spi_8301_Vars->SndCmd = false;
  Spi_8301_Vars->RcvCmd = false;


  // Wait for the DRV8301 registers to update
  osDelay(1);


  // Update Status Register 1
  drvRegName = DRV8301_RegName_Status_1;
  drvDataNew = DRV8301_readSpi(handle,drvRegName);
  Spi_8301_Vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
  Spi_8301_Vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
  Spi_8301_Vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
  Spi_8301_Vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
  Spi_8301_Vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
  Spi_8301_Vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

  // Update Status Register 2
  drvRegName = DRV8301_RegName_Status_2;
  drvDataNew = DRV8301_readSpi(handle,drvRegName);
  Spi_8301_Vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
  Spi_8301_Vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

  // Update Control Register 1
  drvRegName = DRV8301_RegName_Control_1;
  drvDataNew = DRV8301_readSpi(handle,drvRegName);
  Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
  Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
  Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
  Spi_8301_Vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
  Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

  // Update Control Register 2
  drvRegName = DRV8301_RegName_Control_2;
  drvDataNew = DRV8301_readSpi(handle,drvRegName);
  Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
  Spi_8301_Vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
  Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
  Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);

  return;
}


// end of file
