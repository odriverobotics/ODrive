/**
  ******************************************************************************
  * @file    stm32f410tx.h
  * @author  MCD Application Team
  * @version V2.4.3
  * @date    22-January-2016
  * @brief   CMSIS STM32F410Tx Device Peripheral Access Layer Header File. 
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - peripherals registers declarations and bits definition
  *           - Macros to access peripheral’s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f410tx
  * @{
  */
    
#ifndef __STM32F410Tx_H
#define __STM32F410Tx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
  

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals 
  */
#define __CM4_REV                 0x0001U  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1U       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1U       /*!< FPU present                                   */

/**
  * @}
  */
   
/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device 
 *        in @ref Library_configuration_section 
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1 global Interrupts                                            */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                             */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global Interrupt and DAC Global Interrupt                    */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  RNG_IRQn                    = 80,     /*!< RNG global Interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96,     /*!< FMPI2C1 Error Interrupt                                           */
  LPTIM1_IRQn                 = 97      /*!< LPTIM1 interrupt                                                  */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
#include "system_stm32f4xx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */   

/** 
  * @brief Analog to Digital Converter  
  */

typedef struct
{
  __IO uint32_t SR;     /*!< ADC status register,                         Address offset: 0x00 */
  __IO uint32_t CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */
  __IO uint32_t CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
  __IO uint32_t SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
  __IO uint32_t SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
  __IO uint32_t JOFR1;  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
  __IO uint32_t JOFR2;  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
  __IO uint32_t JOFR3;  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
  __IO uint32_t JOFR4;  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
  __IO uint32_t HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
  __IO uint32_t LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
  __IO uint32_t SQR1;   /*!< ADC regular sequence register 1,             Address offset: 0x2C */
  __IO uint32_t SQR2;   /*!< ADC regular sequence register 2,             Address offset: 0x30 */
  __IO uint32_t SQR3;   /*!< ADC regular sequence register 3,             Address offset: 0x34 */
  __IO uint32_t JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
  __IO uint32_t JDR1;   /*!< ADC injected data register 1,                Address offset: 0x3C */
  __IO uint32_t JDR2;   /*!< ADC injected data register 2,                Address offset: 0x40 */
  __IO uint32_t JDR3;   /*!< ADC injected data register 3,                Address offset: 0x44 */
  __IO uint32_t JDR4;   /*!< ADC injected data register 4,                Address offset: 0x48 */
  __IO uint32_t DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
  __IO uint32_t CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
  __IO uint32_t CDR;    /*!< ADC common regular data register for dual
                             AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

/** 
  * @brief CRC calculation unit 
  */

typedef struct
{
  __IO uint32_t DR;         /*!< CRC Data register,             Address offset: 0x00 */
  __IO uint8_t  IDR;        /*!< CRC Independent data register, Address offset: 0x04 */
  uint8_t       RESERVED0;  /*!< Reserved, 0x05                                      */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                      */
  __IO uint32_t CR;         /*!< CRC Control register,          Address offset: 0x08 */
} CRC_TypeDef;

/** 
  * @brief Digital to Analog Converter
  */

typedef struct
{
  __IO uint32_t CR;       /*!< DAC control register,                                    Address offset: 0x00 */
  __IO uint32_t SWTRIGR;  /*!< DAC software trigger register,                           Address offset: 0x04 */
  __IO uint32_t DHR12R1;  /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
  __IO uint32_t DHR12L1;  /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
  __IO uint32_t DHR8R1;   /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
  __IO uint32_t DHR12R2;  /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
  __IO uint32_t DHR12L2;  /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
  __IO uint32_t DHR8R2;   /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
  __IO uint32_t DHR12RD;  /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
  __IO uint32_t DHR12LD;  /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
  __IO uint32_t DHR8RD;   /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
  __IO uint32_t DOR1;     /*!< DAC channel1 data output register,                       Address offset: 0x2C */
  __IO uint32_t DOR2;     /*!< DAC channel2 data output register,                       Address offset: 0x30 */
  __IO uint32_t SR;       /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;

/** 
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;  /*!< MCU device ID code,               Address offset: 0x00 */
  __IO uint32_t CR;      /*!< Debug MCU configuration register, Address offset: 0x04 */
  __IO uint32_t APB1FZ;  /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
  __IO uint32_t APB2FZ;  /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}DBGMCU_TypeDef;


/** 
  * @brief DMA Controller
  */

typedef struct
{
  __IO uint32_t CR;     /*!< DMA stream x configuration register      */
  __IO uint32_t NDTR;   /*!< DMA stream x number of data register     */
  __IO uint32_t PAR;    /*!< DMA stream x peripheral address register */
  __IO uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  __IO uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  __IO uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  __IO uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  __IO uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  __IO uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  __IO uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;


/** 
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __IO uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  __IO uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __IO uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __IO uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __IO uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

/** 
  * @brief FLASH Registers
  */

typedef struct
{
  __IO uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  __IO uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  __IO uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  __IO uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
  __IO uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
  __IO uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  __IO uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;

/** 
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/** 
  * @brief System configuration controller
  */
  
typedef struct
{
  __IO uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __IO uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  __IO uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED;     /*!< Reserved, 0x18                                                               */
  uint32_t      CFGR2;        /*!< Reserved, 0x1C                                                               */
  __IO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
  uint32_t      RESERVED1[2]; /*!< Reserved, 0x24-0x28                                                          */
  __IO uint32_t CFGR;         /*!< SYSCFG Configuration register,                     Address offset: 0x2C      */
} SYSCFG_TypeDef;

/** 
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  __IO uint32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  __IO uint32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  __IO uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  __IO uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  __IO uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  __IO uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  __IO uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
  __IO uint32_t FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< FMPI2C Control register 1,            Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< FMPI2C Control register 2,            Address offset: 0x04 */
  __IO uint32_t OAR1;        /*!< FMPI2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;        /*!< FMPI2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;     /*!< FMPI2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR;    /*!< FMPI2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;         /*!< FMPI2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;         /*!< FMPI2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;        /*!< FMPI2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;        /*!< FMPI2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;        /*!< FMPI2C Transmit data register,        Address offset: 0x28 */
} FMPI2C_TypeDef;

/** 
  * @brief Independent WATCHDOG
  */

typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;

/** 
  * @brief Power Control
  */

typedef struct
{
  __IO uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  __IO uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

/** 
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  uint32_t      RESERVED0[3];  /*!< Reserved, 0x14-0x1C                                                               */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  uint32_t      RESERVED2[3];  /*!< Reserved, 0x34-0x3C                                                               */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  uint32_t      RESERVED4[3];  /*!< Reserved, 0x54-0x5C                                                               */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  uint32_t      RESERVED7[2];  /*!< Reserved, 0x84-0x88                                                               */
  __IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
  __IO uint32_t CKGATENR;       /*!< RCC Clocks Gated ENable Register,                           Address offset: 0x90 */
  __IO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */

} RCC_TypeDef;

/** 
  * @brief Real-Time Clock
  */

typedef struct
{
  __IO uint32_t TR;      /*!< RTC time register,                                        Address offset: 0x00 */
  __IO uint32_t DR;      /*!< RTC date register,                                        Address offset: 0x04 */
  __IO uint32_t CR;      /*!< RTC control register,                                     Address offset: 0x08 */
  __IO uint32_t ISR;     /*!< RTC initialization and status register,                   Address offset: 0x0C */
  __IO uint32_t PRER;    /*!< RTC prescaler register,                                   Address offset: 0x10 */
  __IO uint32_t WUTR;    /*!< RTC wakeup timer register,                                Address offset: 0x14 */
  __IO uint32_t CALIBR;  /*!< RTC calibration register,                                 Address offset: 0x18 */
  __IO uint32_t ALRMAR;  /*!< RTC alarm A register,                                     Address offset: 0x1C */
  __IO uint32_t ALRMBR;  /*!< RTC alarm B register,                                     Address offset: 0x20 */
  __IO uint32_t WPR;     /*!< RTC write protection register,                            Address offset: 0x24 */
  __IO uint32_t SSR;     /*!< RTC sub second register,                                  Address offset: 0x28 */
  __IO uint32_t SHIFTR;  /*!< RTC shift control register,                               Address offset: 0x2C */
  __IO uint32_t TSTR;    /*!< RTC time stamp time register,                             Address offset: 0x30 */
  __IO uint32_t TSDR;    /*!< RTC time stamp date register,                             Address offset: 0x34 */
  __IO uint32_t TSSSR;   /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
  __IO uint32_t CALR;    /*!< RTC calibration register,                                 Address offset: 0x3C */
  __IO uint32_t TAFCR;   /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
  __IO uint32_t ALRMASSR;/*!< RTC alarm A sub second register,                          Address offset: 0x44 */
  __IO uint32_t ALRMBSSR;/*!< RTC alarm B sub second register,                          Address offset: 0x48 */
  uint32_t RESERVED7;    /*!< Reserved, 0x4C                                                                 */
  __IO uint32_t BKP0R;   /*!< RTC backup register 1,                                    Address offset: 0x50 */
  __IO uint32_t BKP1R;   /*!< RTC backup register 1,                                    Address offset: 0x54 */
  __IO uint32_t BKP2R;   /*!< RTC backup register 2,                                    Address offset: 0x58 */
  __IO uint32_t BKP3R;   /*!< RTC backup register 3,                                    Address offset: 0x5C */
  __IO uint32_t BKP4R;   /*!< RTC backup register 4,                                    Address offset: 0x60 */
  __IO uint32_t BKP5R;   /*!< RTC backup register 5,                                    Address offset: 0x64 */
  __IO uint32_t BKP6R;   /*!< RTC backup register 6,                                    Address offset: 0x68 */
  __IO uint32_t BKP7R;   /*!< RTC backup register 7,                                    Address offset: 0x6C */
  __IO uint32_t BKP8R;   /*!< RTC backup register 8,                                    Address offset: 0x70 */
  __IO uint32_t BKP9R;   /*!< RTC backup register 9,                                    Address offset: 0x74 */
  __IO uint32_t BKP10R;  /*!< RTC backup register 10,                                   Address offset: 0x78 */
  __IO uint32_t BKP11R;  /*!< RTC backup register 11,                                   Address offset: 0x7C */
  __IO uint32_t BKP12R;  /*!< RTC backup register 12,                                   Address offset: 0x80 */
  __IO uint32_t BKP13R;  /*!< RTC backup register 13,                                   Address offset: 0x84 */
  __IO uint32_t BKP14R;  /*!< RTC backup register 14,                                   Address offset: 0x88 */
  __IO uint32_t BKP15R;  /*!< RTC backup register 15,                                   Address offset: 0x8C */
  __IO uint32_t BKP16R;  /*!< RTC backup register 16,                                   Address offset: 0x90 */
  __IO uint32_t BKP17R;  /*!< RTC backup register 17,                                   Address offset: 0x94 */
  __IO uint32_t BKP18R;  /*!< RTC backup register 18,                                   Address offset: 0x98 */
  __IO uint32_t BKP19R;  /*!< RTC backup register 19,                                   Address offset: 0x9C */
} RTC_TypeDef;

/** 
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;

/** 
  * @brief TIM
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __IO uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

/** 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
 
typedef struct
{
  __IO uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  __IO uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  __IO uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  __IO uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  __IO uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  __IO uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  __IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

/** 
  * @brief Window WATCHDOG
  */

typedef struct
{
  __IO uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;


/** 
  * @brief RNG
  */
  
typedef struct 
{
  __IO uint32_t CR;  /*!< RNG control register, Address offset: 0x00 */
  __IO uint32_t SR;  /*!< RNG status register,  Address offset: 0x04 */
  __IO uint32_t DR;  /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;


/**
  * @brief LPTIMER
  */
typedef struct
{
  __IO uint32_t ISR;         /*!< LPTIM Interrupt and Status register,                Address offset: 0x00 */
  __IO uint32_t ICR;         /*!< LPTIM Interrupt Clear register,                     Address offset: 0x04 */
  __IO uint32_t IER;         /*!< LPTIM Interrupt Enable register,                    Address offset: 0x08 */
  __IO uint32_t CFGR;        /*!< LPTIM Configuration register,                       Address offset: 0x0C */
  __IO uint32_t CR;          /*!< LPTIM Control register,                             Address offset: 0x10 */
  __IO uint32_t CMP;         /*!< LPTIM Compare register,                             Address offset: 0x14 */
  __IO uint32_t ARR;         /*!< LPTIM Autoreload register,                          Address offset: 0x18 */
  __IO uint32_t CNT;         /*!< LPTIM Counter register,                             Address offset: 0x1C */
  __IO uint32_t OR;          /*!< LPTIM Option register,                              Address offset: 0x20 */
} LPTIM_TypeDef;

/** 
  * @brief Peripheral_memory_map
  */
#define FLASH_BASE            ((uint32_t)0x08000000U) /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define SRAM1_BASE            ((uint32_t)0x20000000U) /*!< SRAM1(32 KB) base address in the alias region                              */
#define PERIPH_BASE           ((uint32_t)0x40000000U) /*!< Peripheral base address in the alias region                                */
#define SRAM1_BB_BASE         ((uint32_t)0x22000000U) /*!< SRAM1(32 KB) base address in the bit-band region                           */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000U) /*!< Peripheral base address in the bit-band region                             */
#define FLASH_END             ((uint32_t)0x0801FFFFU) /*!< FLASH end address */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)

/*!< APB1 peripherals */
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00U)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000U)
#define LPTIM1_BASE           (APB1PERIPH_BASE + 0x2400U)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800U)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00U)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000U)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400U)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800U)
#define FMPI2C1_BASE          (APB1PERIPH_BASE + 0x6000U)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000U)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400U)
/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000U)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400U)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000U)
#define ADC_BASE              (APB2PERIPH_BASE + 0x2300U)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000U)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800U)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00U)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000U)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800U)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800U)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00U)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000U)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800U)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00U)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000U)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010U)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028U)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040U)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058U)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070U)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088U)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0U)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8U)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400U)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010U)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028U)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040U)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058U)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070U)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088U)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0U)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8U)
#define RNG_BASE              (PERIPH_BASE + 0x80000U)

/* Debug MCU registers base address */
#define DBGMCU_BASE           ((uint32_t )0xE0042000U)

/**
  * @}
  */
  
/** @addtogroup Peripheral_declaration
  * @{
  */  
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define FMPI2C1             ((FMPI2C_TypeDef *) FMPI2C1_BASE)
#define LPTIM1              ((LPTIM_TypeDef *) LPTIM1_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define ADC                 ((ADC_Common_TypeDef *) ADC_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)
#define RNG                 ((RNG_TypeDef *) RNG_BASE)

#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */
  
  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */
    
/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for ADC_SR register  ********************/
#define  ADC_SR_AWD                          ((uint32_t)0x00000001U)       /*!<Analog watchdog flag */
#define  ADC_SR_EOC                          ((uint32_t)0x00000002U)       /*!<End of conversion */
#define  ADC_SR_JEOC                         ((uint32_t)0x00000004U)       /*!<Injected channel end of conversion */
#define  ADC_SR_JSTRT                        ((uint32_t)0x00000008U)       /*!<Injected channel Start flag */
#define  ADC_SR_STRT                         ((uint32_t)0x00000010U)       /*!<Regular channel Start flag */
#define  ADC_SR_OVR                          ((uint32_t)0x00000020U)       /*!<Overrun flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       ((uint32_t)0x0000001FU)        /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  ADC_CR1_AWDCH_1                     ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  ADC_CR1_AWDCH_2                     ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  ADC_CR1_AWDCH_3                     ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  ADC_CR1_AWDCH_4                     ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  ADC_CR1_EOCIE                       ((uint32_t)0x00000020U)        /*!<Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       ((uint32_t)0x00000040U)        /*!<AAnalog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      ((uint32_t)0x00000080U)        /*!<Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        ((uint32_t)0x00000100U)        /*!<Scan mode */
#define  ADC_CR1_AWDSGL                      ((uint32_t)0x00000200U)        /*!<Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       ((uint32_t)0x00000400U)        /*!<Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      ((uint32_t)0x00000800U)        /*!<Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     ((uint32_t)0x00001000U)        /*!<Discontinuous mode on injected channels */
#define  ADC_CR1_DISCNUM                     ((uint32_t)0x0000E000U)        /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   ((uint32_t)0x00002000U)        /*!<Bit 0 */
#define  ADC_CR1_DISCNUM_1                   ((uint32_t)0x00004000U)        /*!<Bit 1 */
#define  ADC_CR1_DISCNUM_2                   ((uint32_t)0x00008000U)        /*!<Bit 2 */
#define  ADC_CR1_JAWDEN                      ((uint32_t)0x00400000U)        /*!<Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       ((uint32_t)0x00800000U)        /*!<Analog watchdog enable on regular channels */
#define  ADC_CR1_RES                         ((uint32_t)0x03000000U)        /*!<RES[2:0] bits (Resolution) */
#define  ADC_CR1_RES_0                       ((uint32_t)0x01000000U)        /*!<Bit 0 */
#define  ADC_CR1_RES_1                       ((uint32_t)0x02000000U)        /*!<Bit 1 */
#define  ADC_CR1_OVRIE                       ((uint32_t)0x04000000U)         /*!<overrun interrupt enable */
  
/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        ((uint32_t)0x00000001U)        /*!<A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        ((uint32_t)0x00000002U)        /*!<Continuous Conversion */
#define  ADC_CR2_DMA                         ((uint32_t)0x00000100U)        /*!<Direct Memory access mode */
#define  ADC_CR2_DDS                         ((uint32_t)0x00000200U)        /*!<DMA disable selection (Single ADC) */
#define  ADC_CR2_EOCS                        ((uint32_t)0x00000400U)        /*!<End of conversion selection */
#define  ADC_CR2_ALIGN                       ((uint32_t)0x00000800U)        /*!<Data Alignment */
#define  ADC_CR2_JEXTSEL                     ((uint32_t)0x000F0000U)        /*!<JEXTSEL[3:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   ((uint32_t)0x00010000U)        /*!<Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   ((uint32_t)0x00020000U)        /*!<Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   ((uint32_t)0x00040000U)        /*!<Bit 2 */
#define  ADC_CR2_JEXTSEL_3                   ((uint32_t)0x00080000U)        /*!<Bit 3 */
#define  ADC_CR2_JEXTEN                      ((uint32_t)0x00300000U)        /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
#define  ADC_CR2_JEXTEN_0                    ((uint32_t)0x00100000U)        /*!<Bit 0 */
#define  ADC_CR2_JEXTEN_1                    ((uint32_t)0x00200000U)        /*!<Bit 1 */
#define  ADC_CR2_JSWSTART                    ((uint32_t)0x00400000U)        /*!<Start Conversion of injected channels */
#define  ADC_CR2_EXTSEL                      ((uint32_t)0x0F000000U)        /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    ((uint32_t)0x01000000U)        /*!<Bit 0 */
#define  ADC_CR2_EXTSEL_1                    ((uint32_t)0x02000000U)        /*!<Bit 1 */
#define  ADC_CR2_EXTSEL_2                    ((uint32_t)0x04000000U)        /*!<Bit 2 */
#define  ADC_CR2_EXTSEL_3                    ((uint32_t)0x08000000U)        /*!<Bit 3 */
#define  ADC_CR2_EXTEN                       ((uint32_t)0x30000000U)        /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
#define  ADC_CR2_EXTEN_0                     ((uint32_t)0x10000000U)        /*!<Bit 0 */
#define  ADC_CR2_EXTEN_1                     ((uint32_t)0x20000000U)        /*!<Bit 1 */
#define  ADC_CR2_SWSTART                     ((uint32_t)0x40000000U)        /*!<Start Conversion of regular channels */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     ((uint32_t)0x00000007U)        /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP10_1                   ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP10_2                   ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP11                     ((uint32_t)0x00000038U)        /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   ((uint32_t)0x00000008U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP11_1                   ((uint32_t)0x00000010U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP11_2                   ((uint32_t)0x00000020U)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP12                     ((uint32_t)0x000001C0U)        /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   ((uint32_t)0x00000040U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP12_1                   ((uint32_t)0x00000080U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP12_2                   ((uint32_t)0x00000100U)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP13                     ((uint32_t)0x00000E00U)        /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   ((uint32_t)0x00000200U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP13_1                   ((uint32_t)0x00000400U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP13_2                   ((uint32_t)0x00000800U)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP14                     ((uint32_t)0x00007000U)        /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   ((uint32_t)0x00001000U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP14_1                   ((uint32_t)0x00002000U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP14_2                   ((uint32_t)0x00004000U)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP15                     ((uint32_t)0x00038000U)        /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   ((uint32_t)0x00008000U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP15_1                   ((uint32_t)0x00010000U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP15_2                   ((uint32_t)0x00020000U)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP16                     ((uint32_t)0x001C0000U)        /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   ((uint32_t)0x00040000U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP16_1                   ((uint32_t)0x00080000U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP16_2                   ((uint32_t)0x00100000U)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP17                     ((uint32_t)0x00E00000U)        /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   ((uint32_t)0x00200000U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP17_1                   ((uint32_t)0x00400000U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP17_2                   ((uint32_t)0x00800000U)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP18                     ((uint32_t)0x07000000U)        /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
#define  ADC_SMPR1_SMP18_0                   ((uint32_t)0x01000000U)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP18_1                   ((uint32_t)0x02000000U)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP18_2                   ((uint32_t)0x04000000U)        /*!<Bit 2 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      ((uint32_t)0x00000007U)        /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP0_1                    ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP0_2                    ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP1                      ((uint32_t)0x00000038U)        /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    ((uint32_t)0x00000008U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP1_1                    ((uint32_t)0x00000010U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP1_2                    ((uint32_t)0x00000020U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP2                      ((uint32_t)0x000001C0U)        /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPR2_SMP2_0                    ((uint32_t)0x00000040U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP2_1                    ((uint32_t)0x00000080U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP2_2                    ((uint32_t)0x00000100U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP3                      ((uint32_t)0x00000E00U)        /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPR2_SMP3_0                    ((uint32_t)0x00000200U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP3_1                    ((uint32_t)0x00000400U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP3_2                    ((uint32_t)0x00000800U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP4                      ((uint32_t)0x00007000U)        /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPR2_SMP4_0                    ((uint32_t)0x00001000U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP4_1                    ((uint32_t)0x00002000U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP4_2                    ((uint32_t)0x00004000U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP5                      ((uint32_t)0x00038000U)        /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPR2_SMP5_0                    ((uint32_t)0x00008000U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP5_1                    ((uint32_t)0x00010000U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP5_2                    ((uint32_t)0x00020000U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP6                      ((uint32_t)0x001C0000U)        /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPR2_SMP6_0                    ((uint32_t)0x00040000U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP6_1                    ((uint32_t)0x00080000U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP6_2                    ((uint32_t)0x00100000U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP7                      ((uint32_t)0x00E00000U)        /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPR2_SMP7_0                    ((uint32_t)0x00200000U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP7_1                    ((uint32_t)0x00400000U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP7_2                    ((uint32_t)0x00800000U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP8                      ((uint32_t)0x07000000U)        /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPR2_SMP8_0                    ((uint32_t)0x01000000U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP8_1                    ((uint32_t)0x02000000U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP8_2                    ((uint32_t)0x04000000U)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP9                      ((uint32_t)0x38000000U)        /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPR2_SMP9_0                    ((uint32_t)0x08000000U)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP9_1                    ((uint32_t)0x10000000U)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP9_2                    ((uint32_t)0x20000000U)        /*!<Bit 2 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  ((uint32_t)0x0FFFU)            /*!<Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  ((uint32_t)0x0FFFU)            /*!<Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  ((uint32_t)0x0FFFU)            /*!<Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  ((uint32_t)0x0FFFU)            /*!<Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          ((uint32_t)0x0FFFU)            /*!<Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          ((uint32_t)0x0FFFU)            /*!<Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  ADC_SQR1_SQ13                       ((uint32_t)0x0000001FU)        /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQR1_SQ13_0                     ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  ADC_SQR1_SQ13_1                     ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  ADC_SQR1_SQ13_2                     ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  ADC_SQR1_SQ13_3                     ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  ADC_SQR1_SQ13_4                     ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  ADC_SQR1_SQ14                       ((uint32_t)0x000003E0U)        /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQR1_SQ14_0                     ((uint32_t)0x00000020U)        /*!<Bit 0 */
#define  ADC_SQR1_SQ14_1                     ((uint32_t)0x00000040U)        /*!<Bit 1 */
#define  ADC_SQR1_SQ14_2                     ((uint32_t)0x00000080U)        /*!<Bit 2 */
#define  ADC_SQR1_SQ14_3                     ((uint32_t)0x00000100U)        /*!<Bit 3 */
#define  ADC_SQR1_SQ14_4                     ((uint32_t)0x00000200U)        /*!<Bit 4 */
#define  ADC_SQR1_SQ15                       ((uint32_t)0x00007C00U)        /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQR1_SQ15_0                     ((uint32_t)0x00000400U)        /*!<Bit 0 */
#define  ADC_SQR1_SQ15_1                     ((uint32_t)0x00000800U)        /*!<Bit 1 */
#define  ADC_SQR1_SQ15_2                     ((uint32_t)0x00001000U)        /*!<Bit 2 */
#define  ADC_SQR1_SQ15_3                     ((uint32_t)0x00002000U)        /*!<Bit 3 */
#define  ADC_SQR1_SQ15_4                     ((uint32_t)0x00004000U)        /*!<Bit 4 */
#define  ADC_SQR1_SQ16                       ((uint32_t)0x000F8000U)        /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQR1_SQ16_0                     ((uint32_t)0x00008000U)        /*!<Bit 0 */
#define  ADC_SQR1_SQ16_1                     ((uint32_t)0x00010000U)        /*!<Bit 1 */
#define  ADC_SQR1_SQ16_2                     ((uint32_t)0x00020000U)        /*!<Bit 2 */
#define  ADC_SQR1_SQ16_3                     ((uint32_t)0x00040000U)        /*!<Bit 3 */
#define  ADC_SQR1_SQ16_4                     ((uint32_t)0x00080000U)        /*!<Bit 4 */
#define  ADC_SQR1_L                          ((uint32_t)0x00F00000U)        /*!<L[3:0] bits (Regular channel sequence length) */
#define  ADC_SQR1_L_0                        ((uint32_t)0x00100000U)        /*!<Bit 0 */
#define  ADC_SQR1_L_1                        ((uint32_t)0x00200000U)        /*!<Bit 1 */
#define  ADC_SQR1_L_2                        ((uint32_t)0x00400000U)        /*!<Bit 2 */
#define  ADC_SQR1_L_3                        ((uint32_t)0x00800000U)        /*!<Bit 3 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  ADC_SQR2_SQ7                        ((uint32_t)0x0000001FU)        /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQR2_SQ7_0                      ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  ADC_SQR2_SQ7_1                      ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  ADC_SQR2_SQ7_2                      ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  ADC_SQR2_SQ7_3                      ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  ADC_SQR2_SQ7_4                      ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  ADC_SQR2_SQ8                        ((uint32_t)0x000003E0U)        /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQR2_SQ8_0                      ((uint32_t)0x00000020U)        /*!<Bit 0 */
#define  ADC_SQR2_SQ8_1                      ((uint32_t)0x00000040U)        /*!<Bit 1 */
#define  ADC_SQR2_SQ8_2                      ((uint32_t)0x00000080U)        /*!<Bit 2 */
#define  ADC_SQR2_SQ8_3                      ((uint32_t)0x00000100U)        /*!<Bit 3 */
#define  ADC_SQR2_SQ8_4                      ((uint32_t)0x00000200U)        /*!<Bit 4 */
#define  ADC_SQR2_SQ9                        ((uint32_t)0x00007C00U)        /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQR2_SQ9_0                      ((uint32_t)0x00000400U)        /*!<Bit 0 */
#define  ADC_SQR2_SQ9_1                      ((uint32_t)0x00000800U)        /*!<Bit 1 */
#define  ADC_SQR2_SQ9_2                      ((uint32_t)0x00001000U)        /*!<Bit 2 */
#define  ADC_SQR2_SQ9_3                      ((uint32_t)0x00002000U)        /*!<Bit 3 */
#define  ADC_SQR2_SQ9_4                      ((uint32_t)0x00004000U)        /*!<Bit 4 */
#define  ADC_SQR2_SQ10                       ((uint32_t)0x000F8000U)        /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQR2_SQ10_0                     ((uint32_t)0x00008000U)        /*!<Bit 0 */
#define  ADC_SQR2_SQ10_1                     ((uint32_t)0x00010000U)        /*!<Bit 1 */
#define  ADC_SQR2_SQ10_2                     ((uint32_t)0x00020000U)        /*!<Bit 2 */
#define  ADC_SQR2_SQ10_3                     ((uint32_t)0x00040000U)        /*!<Bit 3 */
#define  ADC_SQR2_SQ10_4                     ((uint32_t)0x00080000U)        /*!<Bit 4 */
#define  ADC_SQR2_SQ11                       ((uint32_t)0x01F00000U)        /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQR2_SQ11_0                     ((uint32_t)0x00100000U)        /*!<Bit 0 */
#define  ADC_SQR2_SQ11_1                     ((uint32_t)0x00200000U)        /*!<Bit 1 */
#define  ADC_SQR2_SQ11_2                     ((uint32_t)0x00400000U)        /*!<Bit 2 */
#define  ADC_SQR2_SQ11_3                     ((uint32_t)0x00800000U)        /*!<Bit 3 */
#define  ADC_SQR2_SQ11_4                     ((uint32_t)0x01000000U)        /*!<Bit 4 */
#define  ADC_SQR2_SQ12                       ((uint32_t)0x3E000000U)        /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQR2_SQ12_0                     ((uint32_t)0x02000000U)        /*!<Bit 0 */
#define  ADC_SQR2_SQ12_1                     ((uint32_t)0x04000000U)        /*!<Bit 1 */
#define  ADC_SQR2_SQ12_2                     ((uint32_t)0x08000000U)        /*!<Bit 2 */
#define  ADC_SQR2_SQ12_3                     ((uint32_t)0x10000000U)        /*!<Bit 3 */
#define  ADC_SQR2_SQ12_4                     ((uint32_t)0x20000000U)        /*!<Bit 4 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  ADC_SQR3_SQ1                        ((uint32_t)0x0000001FU)        /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQR3_SQ1_0                      ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  ADC_SQR3_SQ1_1                      ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  ADC_SQR3_SQ1_2                      ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  ADC_SQR3_SQ1_3                      ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  ADC_SQR3_SQ1_4                      ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  ADC_SQR3_SQ2                        ((uint32_t)0x000003E0U)        /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQR3_SQ2_0                      ((uint32_t)0x00000020U)        /*!<Bit 0 */
#define  ADC_SQR3_SQ2_1                      ((uint32_t)0x00000040U)        /*!<Bit 1 */
#define  ADC_SQR3_SQ2_2                      ((uint32_t)0x00000080U)        /*!<Bit 2 */
#define  ADC_SQR3_SQ2_3                      ((uint32_t)0x00000100U)        /*!<Bit 3 */
#define  ADC_SQR3_SQ2_4                      ((uint32_t)0x00000200U)        /*!<Bit 4 */
#define  ADC_SQR3_SQ3                        ((uint32_t)0x00007C00U)        /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQR3_SQ3_0                      ((uint32_t)0x00000400U)        /*!<Bit 0 */
#define  ADC_SQR3_SQ3_1                      ((uint32_t)0x00000800U)        /*!<Bit 1 */
#define  ADC_SQR3_SQ3_2                      ((uint32_t)0x00001000U)        /*!<Bit 2 */
#define  ADC_SQR3_SQ3_3                      ((uint32_t)0x00002000U)        /*!<Bit 3 */
#define  ADC_SQR3_SQ3_4                      ((uint32_t)0x00004000U)        /*!<Bit 4 */
#define  ADC_SQR3_SQ4                        ((uint32_t)0x000F8000U)        /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQR3_SQ4_0                      ((uint32_t)0x00008000U)        /*!<Bit 0 */
#define  ADC_SQR3_SQ4_1                      ((uint32_t)0x00010000U)        /*!<Bit 1 */
#define  ADC_SQR3_SQ4_2                      ((uint32_t)0x00020000U)        /*!<Bit 2 */
#define  ADC_SQR3_SQ4_3                      ((uint32_t)0x00040000U)        /*!<Bit 3 */
#define  ADC_SQR3_SQ4_4                      ((uint32_t)0x00080000U)        /*!<Bit 4 */
#define  ADC_SQR3_SQ5                        ((uint32_t)0x01F00000U)        /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQR3_SQ5_0                      ((uint32_t)0x00100000U)        /*!<Bit 0 */
#define  ADC_SQR3_SQ5_1                      ((uint32_t)0x00200000U)        /*!<Bit 1 */
#define  ADC_SQR3_SQ5_2                      ((uint32_t)0x00400000U)        /*!<Bit 2 */
#define  ADC_SQR3_SQ5_3                      ((uint32_t)0x00800000U)        /*!<Bit 3 */
#define  ADC_SQR3_SQ5_4                      ((uint32_t)0x01000000U)        /*!<Bit 4 */
#define  ADC_SQR3_SQ6                        ((uint32_t)0x3E000000U)        /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQR3_SQ6_0                      ((uint32_t)0x02000000U)        /*!<Bit 0 */
#define  ADC_SQR3_SQ6_1                      ((uint32_t)0x04000000U)        /*!<Bit 1 */
#define  ADC_SQR3_SQ6_2                      ((uint32_t)0x08000000U)        /*!<Bit 2 */
#define  ADC_SQR3_SQ6_3                      ((uint32_t)0x10000000U)        /*!<Bit 3 */
#define  ADC_SQR3_SQ6_4                      ((uint32_t)0x20000000U)        /*!<Bit 4 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define  ADC_JSQR_JSQ1                       ((uint32_t)0x0000001FU)        /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */  
#define  ADC_JSQR_JSQ1_0                     ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ1_1                     ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ1_2                     ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ1_3                     ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ1_4                     ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ2                       ((uint32_t)0x000003E0U)        /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQR_JSQ2_0                     ((uint32_t)0x00000020U)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ2_1                     ((uint32_t)0x00000040U)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ2_2                     ((uint32_t)0x00000080U)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ2_3                     ((uint32_t)0x00000100U)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ2_4                     ((uint32_t)0x00000200U)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ3                       ((uint32_t)0x00007C00U)        /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQR_JSQ3_0                     ((uint32_t)0x00000400U)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ3_1                     ((uint32_t)0x00000800U)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ3_2                     ((uint32_t)0x00001000U)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ3_3                     ((uint32_t)0x00002000U)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ3_4                     ((uint32_t)0x00004000U)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ4                       ((uint32_t)0x000F8000U)        /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQR_JSQ4_0                     ((uint32_t)0x00008000U)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ4_1                     ((uint32_t)0x00010000U)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ4_2                     ((uint32_t)0x00020000U)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ4_3                     ((uint32_t)0x00040000U)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ4_4                     ((uint32_t)0x00080000U)        /*!<Bit 4 */
#define  ADC_JSQR_JL                         ((uint32_t)0x00300000U)        /*!<JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQR_JL_0                       ((uint32_t)0x00100000U)        /*!<Bit 0 */
#define  ADC_JSQR_JL_1                       ((uint32_t)0x00200000U)        /*!<Bit 1 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      ((uint32_t)0xFFFFU)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      ((uint32_t)0xFFFFU)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      ((uint32_t)0xFFFFU)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      ((uint32_t)0xFFFFU)            /*!<Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         ((uint32_t)0x0000FFFFU)        /*!<Regular data */
#define  ADC_DR_ADC2DATA                     ((uint32_t)0xFFFF0000U)        /*!<ADC2 data */

/*******************  Bit definition for ADC_CSR register  ********************/
#define  ADC_CSR_AWD1                        ((uint32_t)0x00000001U)        /*!<ADC1 Analog watchdog flag */
#define  ADC_CSR_EOC1                        ((uint32_t)0x00000002U)        /*!<ADC1 End of conversion */
#define  ADC_CSR_JEOC1                       ((uint32_t)0x00000004U)        /*!<ADC1 Injected channel end of conversion */
#define  ADC_CSR_JSTRT1                      ((uint32_t)0x00000008U)        /*!<ADC1 Injected channel Start flag */
#define  ADC_CSR_STRT1                       ((uint32_t)0x00000010U)        /*!<ADC1 Regular channel Start flag */
#define  ADC_CSR_OVR1                        ((uint32_t)0x00000020U)        /*!<ADC1 DMA overrun  flag */
#define  ADC_CSR_AWD2                        ((uint32_t)0x00000100U)        /*!<ADC2 Analog watchdog flag */
#define  ADC_CSR_EOC2                        ((uint32_t)0x00000200U)        /*!<ADC2 End of conversion */
#define  ADC_CSR_JEOC2                       ((uint32_t)0x00000400U)        /*!<ADC2 Injected channel end of conversion */
#define  ADC_CSR_JSTRT2                      ((uint32_t)0x00000800U)        /*!<ADC2 Injected channel Start flag */
#define  ADC_CSR_STRT2                       ((uint32_t)0x00001000U)        /*!<ADC2 Regular channel Start flag */
#define  ADC_CSR_OVR2                        ((uint32_t)0x00002000U)        /*!<ADC2 DMA overrun  flag */
#define  ADC_CSR_AWD3                        ((uint32_t)0x00010000U)        /*!<ADC3 Analog watchdog flag */
#define  ADC_CSR_EOC3                        ((uint32_t)0x00020000U)        /*!<ADC3 End of conversion */
#define  ADC_CSR_JEOC3                       ((uint32_t)0x00040000U)        /*!<ADC3 Injected channel end of conversion */
#define  ADC_CSR_JSTRT3                      ((uint32_t)0x00080000U)        /*!<ADC3 Injected channel Start flag */
#define  ADC_CSR_STRT3                       ((uint32_t)0x00100000U)        /*!<ADC3 Regular channel Start flag */
#define  ADC_CSR_OVR3                        ((uint32_t)0x00200000U)        /*!<ADC3 DMA overrun  flag */

/* Legacy defines */
#define  ADC_CSR_DOVR1                        ADC_CSR_OVR1
#define  ADC_CSR_DOVR2                        ADC_CSR_OVR2
#define  ADC_CSR_DOVR3                        ADC_CSR_OVR3

/*******************  Bit definition for ADC_CCR register  ********************/
#define  ADC_CCR_MULTI                       ((uint32_t)0x0000001FU)        /*!<MULTI[4:0] bits (Multi-ADC mode selection) */  
#define  ADC_CCR_MULTI_0                     ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  ADC_CCR_MULTI_1                     ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  ADC_CCR_MULTI_2                     ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  ADC_CCR_MULTI_3                     ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  ADC_CCR_MULTI_4                     ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  ADC_CCR_DELAY                       ((uint32_t)0x00000F00U)        /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */  
#define  ADC_CCR_DELAY_0                     ((uint32_t)0x00000100U)        /*!<Bit 0 */
#define  ADC_CCR_DELAY_1                     ((uint32_t)0x00000200U)        /*!<Bit 1 */
#define  ADC_CCR_DELAY_2                     ((uint32_t)0x00000400U)        /*!<Bit 2 */
#define  ADC_CCR_DELAY_3                     ((uint32_t)0x00000800U)        /*!<Bit 3 */
#define  ADC_CCR_DDS                         ((uint32_t)0x00002000U)        /*!<DMA disable selection (Multi-ADC mode) */
#define  ADC_CCR_DMA                         ((uint32_t)0x0000C000U)        /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */  
#define  ADC_CCR_DMA_0                       ((uint32_t)0x00004000U)        /*!<Bit 0 */
#define  ADC_CCR_DMA_1                       ((uint32_t)0x00008000U)        /*!<Bit 1 */
#define  ADC_CCR_ADCPRE                      ((uint32_t)0x00030000U)        /*!<ADCPRE[1:0] bits (ADC prescaler) */  
#define  ADC_CCR_ADCPRE_0                    ((uint32_t)0x00010000U)        /*!<Bit 0 */
#define  ADC_CCR_ADCPRE_1                    ((uint32_t)0x00020000U)        /*!<Bit 1 */
#define  ADC_CCR_VBATE                       ((uint32_t)0x00400000U)        /*!<VBAT Enable */
#define  ADC_CCR_TSVREFE                     ((uint32_t)0x00800000U)        /*!<Temperature Sensor and VREFINT Enable */

/*******************  Bit definition for ADC_CDR register  ********************/
#define  ADC_CDR_DATA1                      ((uint32_t)0x0000FFFFU)         /*!<1st data of a pair of regular conversions */
#define  ADC_CDR_DATA2                      ((uint32_t)0xFFFF0000U)         /*!<2nd data of a pair of regular conversions */

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((uint32_t)0xFFFFFFFFU) /*!< Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((uint32_t)0xFFU)        /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((uint32_t)0x01U)        /*!< RESET bit */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMA_SxCR register  *****************/
#define DMA_SxCR_CHSEL                       ((uint32_t)0x0E000000U)
#define DMA_SxCR_CHSEL_0                     ((uint32_t)0x02000000U)
#define DMA_SxCR_CHSEL_1                     ((uint32_t)0x04000000U)
#define DMA_SxCR_CHSEL_2                     ((uint32_t)0x08000000U)
#define DMA_SxCR_MBURST                      ((uint32_t)0x01800000U)
#define DMA_SxCR_MBURST_0                    ((uint32_t)0x00800000U)
#define DMA_SxCR_MBURST_1                    ((uint32_t)0x01000000U)
#define DMA_SxCR_PBURST                      ((uint32_t)0x00600000U)
#define DMA_SxCR_PBURST_0                    ((uint32_t)0x00200000U)
#define DMA_SxCR_PBURST_1                    ((uint32_t)0x00400000U)
#define DMA_SxCR_CT                          ((uint32_t)0x00080000U)
#define DMA_SxCR_DBM                         ((uint32_t)0x00040000U)
#define DMA_SxCR_PL                          ((uint32_t)0x00030000U)
#define DMA_SxCR_PL_0                        ((uint32_t)0x00010000U)
#define DMA_SxCR_PL_1                        ((uint32_t)0x00020000U)
#define DMA_SxCR_PINCOS                      ((uint32_t)0x00008000U)
#define DMA_SxCR_MSIZE                       ((uint32_t)0x00006000U)
#define DMA_SxCR_MSIZE_0                     ((uint32_t)0x00002000U)
#define DMA_SxCR_MSIZE_1                     ((uint32_t)0x00004000U)
#define DMA_SxCR_PSIZE                       ((uint32_t)0x00001800U)
#define DMA_SxCR_PSIZE_0                     ((uint32_t)0x00000800U)
#define DMA_SxCR_PSIZE_1                     ((uint32_t)0x00001000U)
#define DMA_SxCR_MINC                        ((uint32_t)0x00000400U)
#define DMA_SxCR_PINC                        ((uint32_t)0x00000200U)
#define DMA_SxCR_CIRC                        ((uint32_t)0x00000100U)
#define DMA_SxCR_DIR                         ((uint32_t)0x000000C0U)
#define DMA_SxCR_DIR_0                       ((uint32_t)0x00000040U)
#define DMA_SxCR_DIR_1                       ((uint32_t)0x00000080U)
#define DMA_SxCR_PFCTRL                      ((uint32_t)0x00000020U)
#define DMA_SxCR_TCIE                        ((uint32_t)0x00000010U)
#define DMA_SxCR_HTIE                        ((uint32_t)0x00000008U)
#define DMA_SxCR_TEIE                        ((uint32_t)0x00000004U)
#define DMA_SxCR_DMEIE                       ((uint32_t)0x00000002U)
#define DMA_SxCR_EN                          ((uint32_t)0x00000001U)

/* Legacy defines */
#define DMA_SxCR_ACK                         ((uint32_t)0x00100000U)

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT                            ((uint32_t)0x0000FFFFU)
#define DMA_SxNDT_0                          ((uint32_t)0x00000001U)
#define DMA_SxNDT_1                          ((uint32_t)0x00000002U)
#define DMA_SxNDT_2                          ((uint32_t)0x00000004U)
#define DMA_SxNDT_3                          ((uint32_t)0x00000008U)
#define DMA_SxNDT_4                          ((uint32_t)0x00000010U)
#define DMA_SxNDT_5                          ((uint32_t)0x00000020U)
#define DMA_SxNDT_6                          ((uint32_t)0x00000040U)
#define DMA_SxNDT_7                          ((uint32_t)0x00000080U)
#define DMA_SxNDT_8                          ((uint32_t)0x00000100U)
#define DMA_SxNDT_9                          ((uint32_t)0x00000200U)
#define DMA_SxNDT_10                         ((uint32_t)0x00000400U)
#define DMA_SxNDT_11                         ((uint32_t)0x00000800U)
#define DMA_SxNDT_12                         ((uint32_t)0x00001000U)
#define DMA_SxNDT_13                         ((uint32_t)0x00002000U)
#define DMA_SxNDT_14                         ((uint32_t)0x00004000U)
#define DMA_SxNDT_15                         ((uint32_t)0x00008000U)

/********************  Bits definition for DMA_SxFCR register  ****************/ 
#define DMA_SxFCR_FEIE                       ((uint32_t)0x00000080U)
#define DMA_SxFCR_FS                         ((uint32_t)0x00000038U)
#define DMA_SxFCR_FS_0                       ((uint32_t)0x00000008U)
#define DMA_SxFCR_FS_1                       ((uint32_t)0x00000010U)
#define DMA_SxFCR_FS_2                       ((uint32_t)0x00000020U)
#define DMA_SxFCR_DMDIS                      ((uint32_t)0x00000004U)
#define DMA_SxFCR_FTH                        ((uint32_t)0x00000003U)
#define DMA_SxFCR_FTH_0                      ((uint32_t)0x00000001U)
#define DMA_SxFCR_FTH_1                      ((uint32_t)0x00000002U)

/********************  Bits definition for DMA_LISR register  *****************/ 
#define DMA_LISR_TCIF3                       ((uint32_t)0x08000000U)
#define DMA_LISR_HTIF3                       ((uint32_t)0x04000000U)
#define DMA_LISR_TEIF3                       ((uint32_t)0x02000000U)
#define DMA_LISR_DMEIF3                      ((uint32_t)0x01000000U)
#define DMA_LISR_FEIF3                       ((uint32_t)0x00400000U)
#define DMA_LISR_TCIF2                       ((uint32_t)0x00200000U)
#define DMA_LISR_HTIF2                       ((uint32_t)0x00100000U)
#define DMA_LISR_TEIF2                       ((uint32_t)0x00080000U)
#define DMA_LISR_DMEIF2                      ((uint32_t)0x00040000U)
#define DMA_LISR_FEIF2                       ((uint32_t)0x00010000U)
#define DMA_LISR_TCIF1                       ((uint32_t)0x00000800U)
#define DMA_LISR_HTIF1                       ((uint32_t)0x00000400U)
#define DMA_LISR_TEIF1                       ((uint32_t)0x00000200U)
#define DMA_LISR_DMEIF1                      ((uint32_t)0x00000100U)
#define DMA_LISR_FEIF1                       ((uint32_t)0x00000040U)
#define DMA_LISR_TCIF0                       ((uint32_t)0x00000020U)
#define DMA_LISR_HTIF0                       ((uint32_t)0x00000010U)
#define DMA_LISR_TEIF0                       ((uint32_t)0x00000008U)
#define DMA_LISR_DMEIF0                      ((uint32_t)0x00000004U)
#define DMA_LISR_FEIF0                       ((uint32_t)0x00000001U)

/********************  Bits definition for DMA_HISR register  *****************/ 
#define DMA_HISR_TCIF7                       ((uint32_t)0x08000000U)
#define DMA_HISR_HTIF7                       ((uint32_t)0x04000000U)
#define DMA_HISR_TEIF7                       ((uint32_t)0x02000000U)
#define DMA_HISR_DMEIF7                      ((uint32_t)0x01000000U)
#define DMA_HISR_FEIF7                       ((uint32_t)0x00400000U)
#define DMA_HISR_TCIF6                       ((uint32_t)0x00200000U)
#define DMA_HISR_HTIF6                       ((uint32_t)0x00100000U)
#define DMA_HISR_TEIF6                       ((uint32_t)0x00080000U)
#define DMA_HISR_DMEIF6                      ((uint32_t)0x00040000U)
#define DMA_HISR_FEIF6                       ((uint32_t)0x00010000U)
#define DMA_HISR_TCIF5                       ((uint32_t)0x00000800U)
#define DMA_HISR_HTIF5                       ((uint32_t)0x00000400U)
#define DMA_HISR_TEIF5                       ((uint32_t)0x00000200U)
#define DMA_HISR_DMEIF5                      ((uint32_t)0x00000100U)
#define DMA_HISR_FEIF5                       ((uint32_t)0x00000040U)
#define DMA_HISR_TCIF4                       ((uint32_t)0x00000020U)
#define DMA_HISR_HTIF4                       ((uint32_t)0x00000010U)
#define DMA_HISR_TEIF4                       ((uint32_t)0x00000008U)
#define DMA_HISR_DMEIF4                      ((uint32_t)0x00000004U)
#define DMA_HISR_FEIF4                       ((uint32_t)0x00000001U)

/********************  Bits definition for DMA_LIFCR register  ****************/ 
#define DMA_LIFCR_CTCIF3                     ((uint32_t)0x08000000U)
#define DMA_LIFCR_CHTIF3                     ((uint32_t)0x04000000U)
#define DMA_LIFCR_CTEIF3                     ((uint32_t)0x02000000U)
#define DMA_LIFCR_CDMEIF3                    ((uint32_t)0x01000000U)
#define DMA_LIFCR_CFEIF3                     ((uint32_t)0x00400000U)
#define DMA_LIFCR_CTCIF2                     ((uint32_t)0x00200000U)
#define DMA_LIFCR_CHTIF2                     ((uint32_t)0x00100000U)
#define DMA_LIFCR_CTEIF2                     ((uint32_t)0x00080000U)
#define DMA_LIFCR_CDMEIF2                    ((uint32_t)0x00040000U)
#define DMA_LIFCR_CFEIF2                     ((uint32_t)0x00010000U)
#define DMA_LIFCR_CTCIF1                     ((uint32_t)0x00000800U)
#define DMA_LIFCR_CHTIF1                     ((uint32_t)0x00000400U)
#define DMA_LIFCR_CTEIF1                     ((uint32_t)0x00000200U)
#define DMA_LIFCR_CDMEIF1                    ((uint32_t)0x00000100U)
#define DMA_LIFCR_CFEIF1                     ((uint32_t)0x00000040U)
#define DMA_LIFCR_CTCIF0                     ((uint32_t)0x00000020U)
#define DMA_LIFCR_CHTIF0                     ((uint32_t)0x00000010U)
#define DMA_LIFCR_CTEIF0                     ((uint32_t)0x00000008U)
#define DMA_LIFCR_CDMEIF0                    ((uint32_t)0x00000004U)
#define DMA_LIFCR_CFEIF0                     ((uint32_t)0x00000001U)

/********************  Bits definition for DMA_HIFCR  register  ****************/ 
#define DMA_HIFCR_CTCIF7                     ((uint32_t)0x08000000U)
#define DMA_HIFCR_CHTIF7                     ((uint32_t)0x04000000U)
#define DMA_HIFCR_CTEIF7                     ((uint32_t)0x02000000U)
#define DMA_HIFCR_CDMEIF7                    ((uint32_t)0x01000000U)
#define DMA_HIFCR_CFEIF7                     ((uint32_t)0x00400000U)
#define DMA_HIFCR_CTCIF6                     ((uint32_t)0x00200000U)
#define DMA_HIFCR_CHTIF6                     ((uint32_t)0x00100000U)
#define DMA_HIFCR_CTEIF6                     ((uint32_t)0x00080000U)
#define DMA_HIFCR_CDMEIF6                    ((uint32_t)0x00040000U)
#define DMA_HIFCR_CFEIF6                     ((uint32_t)0x00010000U)
#define DMA_HIFCR_CTCIF5                     ((uint32_t)0x00000800U)
#define DMA_HIFCR_CHTIF5                     ((uint32_t)0x00000400U)
#define DMA_HIFCR_CTEIF5                     ((uint32_t)0x00000200U)
#define DMA_HIFCR_CDMEIF5                    ((uint32_t)0x00000100U)
#define DMA_HIFCR_CFEIF5                     ((uint32_t)0x00000040U)
#define DMA_HIFCR_CTCIF4                     ((uint32_t)0x00000020U)
#define DMA_HIFCR_CHTIF4                     ((uint32_t)0x00000010U)
#define DMA_HIFCR_CTEIF4                     ((uint32_t)0x00000008U)
#define DMA_HIFCR_CDMEIF4                    ((uint32_t)0x00000004U)
#define DMA_HIFCR_CFEIF4                     ((uint32_t)0x00000001U)


/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define  EXTI_IMR_MR0                        ((uint32_t)0x00000001U)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1                        ((uint32_t)0x00000002U)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2                        ((uint32_t)0x00000004U)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3                        ((uint32_t)0x00000008U)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4                        ((uint32_t)0x00000010U)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5                        ((uint32_t)0x00000020U)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6                        ((uint32_t)0x00000040U)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7                        ((uint32_t)0x00000080U)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8                        ((uint32_t)0x00000100U)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9                        ((uint32_t)0x00000200U)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10                       ((uint32_t)0x00000400U)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11                       ((uint32_t)0x00000800U)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12                       ((uint32_t)0x00001000U)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13                       ((uint32_t)0x00002000U)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14                       ((uint32_t)0x00004000U)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15                       ((uint32_t)0x00008000U)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16                       ((uint32_t)0x00010000U)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17                       ((uint32_t)0x00020000U)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18                       ((uint32_t)0x00040000U)        /*!< Interrupt Mask on line 18 */
#define  EXTI_IMR_MR19                       ((uint32_t)0x00080000U)        /*!< Interrupt Mask on line 19 */
#define  EXTI_IMR_MR20                       ((uint32_t)0x00100000U)        /*!< Interrupt Mask on line 20 */
#define  EXTI_IMR_MR21                       ((uint32_t)0x00200000U)        /*!< Interrupt Mask on line 21 */
#define  EXTI_IMR_MR22                       ((uint32_t)0x00400000U)        /*!< Interrupt Mask on line 22 */
#define  EXTI_IMR_MR23                       ((uint32_t)0x00800000U)        /*!< Interrupt Mask on line 23 */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define  EXTI_EMR_MR0                        ((uint32_t)0x00000001U)        /*!< Event Mask on line 0 */
#define  EXTI_EMR_MR1                        ((uint32_t)0x00000002U)        /*!< Event Mask on line 1 */
#define  EXTI_EMR_MR2                        ((uint32_t)0x00000004U)        /*!< Event Mask on line 2 */
#define  EXTI_EMR_MR3                        ((uint32_t)0x00000008U)        /*!< Event Mask on line 3 */
#define  EXTI_EMR_MR4                        ((uint32_t)0x00000010U)        /*!< Event Mask on line 4 */
#define  EXTI_EMR_MR5                        ((uint32_t)0x00000020U)        /*!< Event Mask on line 5 */
#define  EXTI_EMR_MR6                        ((uint32_t)0x00000040U)        /*!< Event Mask on line 6 */
#define  EXTI_EMR_MR7                        ((uint32_t)0x00000080U)        /*!< Event Mask on line 7 */
#define  EXTI_EMR_MR8                        ((uint32_t)0x00000100U)        /*!< Event Mask on line 8 */
#define  EXTI_EMR_MR9                        ((uint32_t)0x00000200U)        /*!< Event Mask on line 9 */
#define  EXTI_EMR_MR10                       ((uint32_t)0x00000400U)        /*!< Event Mask on line 10 */
#define  EXTI_EMR_MR11                       ((uint32_t)0x00000800U)        /*!< Event Mask on line 11 */
#define  EXTI_EMR_MR12                       ((uint32_t)0x00001000U)        /*!< Event Mask on line 12 */
#define  EXTI_EMR_MR13                       ((uint32_t)0x00002000U)        /*!< Event Mask on line 13 */
#define  EXTI_EMR_MR14                       ((uint32_t)0x00004000U)        /*!< Event Mask on line 14 */
#define  EXTI_EMR_MR15                       ((uint32_t)0x00008000U)        /*!< Event Mask on line 15 */
#define  EXTI_EMR_MR16                       ((uint32_t)0x00010000U)        /*!< Event Mask on line 16 */
#define  EXTI_EMR_MR17                       ((uint32_t)0x00020000U)        /*!< Event Mask on line 17 */
#define  EXTI_EMR_MR18                       ((uint32_t)0x00040000U)        /*!< Event Mask on line 18 */
#define  EXTI_EMR_MR19                       ((uint32_t)0x00080000U)        /*!< Event Mask on line 19 */
#define  EXTI_EMR_MR20                       ((uint32_t)0x00100000U)        /*!< Event Mask on line 20 */
#define  EXTI_EMR_MR21                       ((uint32_t)0x00200000U)        /*!< Event Mask on line 21 */
#define  EXTI_EMR_MR22                       ((uint32_t)0x00400000U)        /*!< Event Mask on line 22 */
#define  EXTI_EMR_MR23                       ((uint32_t)0x00800000U)        /*!< Event Mask on line 23 */

/******************  Bit definition for EXTI_RTSR register  *******************/
#define  EXTI_RTSR_TR0                       ((uint32_t)0x00000001U)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1                       ((uint32_t)0x00000002U)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2                       ((uint32_t)0x00000004U)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3                       ((uint32_t)0x00000008U)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4                       ((uint32_t)0x00000010U)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5                       ((uint32_t)0x00000020U)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6                       ((uint32_t)0x00000040U)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7                       ((uint32_t)0x00000080U)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8                       ((uint32_t)0x00000100U)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9                       ((uint32_t)0x00000200U)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10                      ((uint32_t)0x00000400U)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11                      ((uint32_t)0x00000800U)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12                      ((uint32_t)0x00001000U)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13                      ((uint32_t)0x00002000U)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14                      ((uint32_t)0x00004000U)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15                      ((uint32_t)0x00008000U)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16                      ((uint32_t)0x00010000U)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17                      ((uint32_t)0x00020000U)        /*!< Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18                      ((uint32_t)0x00040000U)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR_TR19                      ((uint32_t)0x00080000U)        /*!< Rising trigger event configuration bit of line 19 */
#define  EXTI_RTSR_TR20                      ((uint32_t)0x00100000U)        /*!< Rising trigger event configuration bit of line 20 */
#define  EXTI_RTSR_TR21                      ((uint32_t)0x00200000U)        /*!< Rising trigger event configuration bit of line 21 */
#define  EXTI_RTSR_TR22                      ((uint32_t)0x00400000U)        /*!< Rising trigger event configuration bit of line 22 */
#define  EXTI_RTSR_TR23                      ((uint32_t)0x00800000U)        /*!< Rising trigger event configuration bit of line 23 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define  EXTI_FTSR_TR0                       ((uint32_t)0x00000001U)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1                       ((uint32_t)0x00000002U)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2                       ((uint32_t)0x00000004U)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3                       ((uint32_t)0x00000008U)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4                       ((uint32_t)0x00000010U)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5                       ((uint32_t)0x00000020U)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6                       ((uint32_t)0x00000040U)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7                       ((uint32_t)0x00000080U)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8                       ((uint32_t)0x00000100U)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9                       ((uint32_t)0x00000200U)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10                      ((uint32_t)0x00000400U)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11                      ((uint32_t)0x00000800U)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12                      ((uint32_t)0x00001000U)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13                      ((uint32_t)0x00002000U)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14                      ((uint32_t)0x00004000U)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15                      ((uint32_t)0x00008000U)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16                      ((uint32_t)0x00010000U)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17                      ((uint32_t)0x00020000U)        /*!< Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18                      ((uint32_t)0x00040000U)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR_TR19                      ((uint32_t)0x00080000U)        /*!< Falling trigger event configuration bit of line 19 */
#define  EXTI_FTSR_TR20                      ((uint32_t)0x00100000U)        /*!< Falling trigger event configuration bit of line 20 */
#define  EXTI_FTSR_TR21                      ((uint32_t)0x00200000U)        /*!< Falling trigger event configuration bit of line 21 */
#define  EXTI_FTSR_TR22                      ((uint32_t)0x00400000U)        /*!< Falling trigger event configuration bit of line 22 */
#define  EXTI_FTSR_TR23                      ((uint32_t)0x00800000U)        /*!< Falling trigger event configuration bit of line 23 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define  EXTI_SWIER_SWIER0                   ((uint32_t)0x00000001U)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1                   ((uint32_t)0x00000002U)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2                   ((uint32_t)0x00000004U)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3                   ((uint32_t)0x00000008U)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4                   ((uint32_t)0x00000010U)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5                   ((uint32_t)0x00000020U)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6                   ((uint32_t)0x00000040U)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7                   ((uint32_t)0x00000080U)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8                   ((uint32_t)0x00000100U)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9                   ((uint32_t)0x00000200U)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10                  ((uint32_t)0x00000400U)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11                  ((uint32_t)0x00000800U)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12                  ((uint32_t)0x00001000U)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13                  ((uint32_t)0x00002000U)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14                  ((uint32_t)0x00004000U)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15                  ((uint32_t)0x00008000U)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16                  ((uint32_t)0x00010000U)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17                  ((uint32_t)0x00020000U)        /*!< Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18                  ((uint32_t)0x00040000U)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIER_SWIER19                  ((uint32_t)0x00080000U)        /*!< Software Interrupt on line 19 */
#define  EXTI_SWIER_SWIER20                  ((uint32_t)0x00100000U)        /*!< Software Interrupt on line 20 */
#define  EXTI_SWIER_SWIER21                  ((uint32_t)0x00200000U)        /*!< Software Interrupt on line 21 */
#define  EXTI_SWIER_SWIER22                  ((uint32_t)0x00400000U)        /*!< Software Interrupt on line 22 */
#define  EXTI_SWIER_SWIER23                  ((uint32_t)0x00800000U)        /*!< Software Interrupt on line 23 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define  EXTI_PR_PR0                         ((uint32_t)0x00000001U)        /*!< Pending bit for line 0 */
#define  EXTI_PR_PR1                         ((uint32_t)0x00000002U)        /*!< Pending bit for line 1 */
#define  EXTI_PR_PR2                         ((uint32_t)0x00000004U)        /*!< Pending bit for line 2 */
#define  EXTI_PR_PR3                         ((uint32_t)0x00000008U)        /*!< Pending bit for line 3 */
#define  EXTI_PR_PR4                         ((uint32_t)0x00000010U)        /*!< Pending bit for line 4 */
#define  EXTI_PR_PR5                         ((uint32_t)0x00000020U)        /*!< Pending bit for line 5 */
#define  EXTI_PR_PR6                         ((uint32_t)0x00000040U)        /*!< Pending bit for line 6 */
#define  EXTI_PR_PR7                         ((uint32_t)0x00000080U)        /*!< Pending bit for line 7 */
#define  EXTI_PR_PR8                         ((uint32_t)0x00000100U)        /*!< Pending bit for line 8 */
#define  EXTI_PR_PR9                         ((uint32_t)0x00000200U)        /*!< Pending bit for line 9 */
#define  EXTI_PR_PR10                        ((uint32_t)0x00000400U)        /*!< Pending bit for line 10 */
#define  EXTI_PR_PR11                        ((uint32_t)0x00000800U)        /*!< Pending bit for line 11 */
#define  EXTI_PR_PR12                        ((uint32_t)0x00001000U)        /*!< Pending bit for line 12 */
#define  EXTI_PR_PR13                        ((uint32_t)0x00002000U)        /*!< Pending bit for line 13 */
#define  EXTI_PR_PR14                        ((uint32_t)0x00004000U)        /*!< Pending bit for line 14 */
#define  EXTI_PR_PR15                        ((uint32_t)0x00008000U)        /*!< Pending bit for line 15 */
#define  EXTI_PR_PR16                        ((uint32_t)0x00010000U)        /*!< Pending bit for line 16 */
#define  EXTI_PR_PR17                        ((uint32_t)0x00020000U)        /*!< Pending bit for line 17 */
#define  EXTI_PR_PR18                        ((uint32_t)0x00040000U)        /*!< Pending bit for line 18 */
#define  EXTI_PR_PR19                        ((uint32_t)0x00080000U)        /*!< Pending bit for line 19 */
#define  EXTI_PR_PR20                        ((uint32_t)0x00100000U)        /*!< Pending bit for line 20 */
#define  EXTI_PR_PR21                        ((uint32_t)0x00200000U)        /*!< Pending bit for line 21 */
#define  EXTI_PR_PR22                        ((uint32_t)0x00400000U)        /*!< Pending bit for line 22 */
#define  EXTI_PR_PR23                        ((uint32_t)0x00800000U)        /*!< Pending bit for line 23 */

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY                    ((uint32_t)0x0000000FU)
#define FLASH_ACR_LATENCY_0WS                ((uint32_t)0x00000000U)
#define FLASH_ACR_LATENCY_1WS                ((uint32_t)0x00000001U)
#define FLASH_ACR_LATENCY_2WS                ((uint32_t)0x00000002U)
#define FLASH_ACR_LATENCY_3WS                ((uint32_t)0x00000003U)
#define FLASH_ACR_LATENCY_4WS                ((uint32_t)0x00000004U)
#define FLASH_ACR_LATENCY_5WS                ((uint32_t)0x00000005U)
#define FLASH_ACR_LATENCY_6WS                ((uint32_t)0x00000006U)
#define FLASH_ACR_LATENCY_7WS                ((uint32_t)0x00000007U)

#define FLASH_ACR_PRFTEN                     ((uint32_t)0x00000100U)
#define FLASH_ACR_ICEN                       ((uint32_t)0x00000200U)
#define FLASH_ACR_DCEN                       ((uint32_t)0x00000400U)
#define FLASH_ACR_ICRST                      ((uint32_t)0x00000800U)
#define FLASH_ACR_DCRST                      ((uint32_t)0x00001000U)
#define FLASH_ACR_BYTE0_ADDRESS              ((uint32_t)0x40023C00U)
#define FLASH_ACR_BYTE2_ADDRESS              ((uint32_t)0x40023C03U)

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP                         ((uint32_t)0x00000001U)
#define FLASH_SR_SOP                         ((uint32_t)0x00000002U)
#define FLASH_SR_WRPERR                      ((uint32_t)0x00000010U)
#define FLASH_SR_PGAERR                      ((uint32_t)0x00000020U)
#define FLASH_SR_PGPERR                      ((uint32_t)0x00000040U)
#define FLASH_SR_PGSERR                      ((uint32_t)0x00000080U)
#define FLASH_SR_BSY                         ((uint32_t)0x00010000U)

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG                          ((uint32_t)0x00000001U)
#define FLASH_CR_SER                         ((uint32_t)0x00000002U)
#define FLASH_CR_MER                         ((uint32_t)0x00000004U)
#define FLASH_CR_SNB                         ((uint32_t)0x000000F8U)
#define FLASH_CR_SNB_0                       ((uint32_t)0x00000008U)
#define FLASH_CR_SNB_1                       ((uint32_t)0x00000010U)
#define FLASH_CR_SNB_2                       ((uint32_t)0x00000020U)
#define FLASH_CR_SNB_3                       ((uint32_t)0x00000040U)
#define FLASH_CR_SNB_4                       ((uint32_t)0x00000080U)
#define FLASH_CR_PSIZE                       ((uint32_t)0x00000300U)
#define FLASH_CR_PSIZE_0                     ((uint32_t)0x00000100U)
#define FLASH_CR_PSIZE_1                     ((uint32_t)0x00000200U)
#define FLASH_CR_STRT                        ((uint32_t)0x00010000U)
#define FLASH_CR_EOPIE                       ((uint32_t)0x01000000U)
#define FLASH_CR_LOCK                        ((uint32_t)0x80000000U)

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK                 ((uint32_t)0x00000001U)
#define FLASH_OPTCR_OPTSTRT                 ((uint32_t)0x00000002U)
#define FLASH_OPTCR_BOR_LEV_0               ((uint32_t)0x00000004U)
#define FLASH_OPTCR_BOR_LEV_1               ((uint32_t)0x00000008U)
#define FLASH_OPTCR_BOR_LEV                 ((uint32_t)0x0000000CU)

#define FLASH_OPTCR_WDG_SW                  ((uint32_t)0x00000020U)
#define FLASH_OPTCR_nRST_STOP               ((uint32_t)0x00000040U)
#define FLASH_OPTCR_nRST_STDBY              ((uint32_t)0x00000080U)
#define FLASH_OPTCR_RDP                     ((uint32_t)0x0000FF00U)
#define FLASH_OPTCR_RDP_0                   ((uint32_t)0x00000100U)
#define FLASH_OPTCR_RDP_1                   ((uint32_t)0x00000200U)
#define FLASH_OPTCR_RDP_2                   ((uint32_t)0x00000400U)
#define FLASH_OPTCR_RDP_3                   ((uint32_t)0x00000800U)
#define FLASH_OPTCR_RDP_4                   ((uint32_t)0x00001000U)
#define FLASH_OPTCR_RDP_5                   ((uint32_t)0x00002000U)
#define FLASH_OPTCR_RDP_6                   ((uint32_t)0x00004000U)
#define FLASH_OPTCR_RDP_7                   ((uint32_t)0x00008000U)
#define FLASH_OPTCR_nWRP                    ((uint32_t)0x0FFF0000U)
#define FLASH_OPTCR_nWRP_0                  ((uint32_t)0x00010000U)
#define FLASH_OPTCR_nWRP_1                  ((uint32_t)0x00020000U)
#define FLASH_OPTCR_nWRP_2                  ((uint32_t)0x00040000U)
#define FLASH_OPTCR_nWRP_3                  ((uint32_t)0x00080000U)
#define FLASH_OPTCR_nWRP_4                  ((uint32_t)0x00100000U)
#define FLASH_OPTCR_nWRP_5                  ((uint32_t)0x00200000U)
#define FLASH_OPTCR_nWRP_6                  ((uint32_t)0x00400000U)
#define FLASH_OPTCR_nWRP_7                  ((uint32_t)0x00800000U)
#define FLASH_OPTCR_nWRP_8                  ((uint32_t)0x01000000U)
#define FLASH_OPTCR_nWRP_9                  ((uint32_t)0x02000000U)
#define FLASH_OPTCR_nWRP_10                 ((uint32_t)0x04000000U)
#define FLASH_OPTCR_nWRP_11                 ((uint32_t)0x08000000U)
                                             
/******************  Bits definition for FLASH_OPTCR1 register  ***************/
#define FLASH_OPTCR1_nWRP                    ((uint32_t)0x0FFF0000U)
#define FLASH_OPTCR1_nWRP_0                  ((uint32_t)0x00010000U)
#define FLASH_OPTCR1_nWRP_1                  ((uint32_t)0x00020000U)
#define FLASH_OPTCR1_nWRP_2                  ((uint32_t)0x00040000U)
#define FLASH_OPTCR1_nWRP_3                  ((uint32_t)0x00080000U)
#define FLASH_OPTCR1_nWRP_4                  ((uint32_t)0x00100000U)
#define FLASH_OPTCR1_nWRP_5                  ((uint32_t)0x00200000U)
#define FLASH_OPTCR1_nWRP_6                  ((uint32_t)0x00400000U)
#define FLASH_OPTCR1_nWRP_7                  ((uint32_t)0x00800000U)
#define FLASH_OPTCR1_nWRP_8                  ((uint32_t)0x01000000U)
#define FLASH_OPTCR1_nWRP_9                  ((uint32_t)0x02000000U)
#define FLASH_OPTCR1_nWRP_10                 ((uint32_t)0x04000000U)
#define FLASH_OPTCR1_nWRP_11                 ((uint32_t)0x08000000U)

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0                    ((uint32_t)0x00000003U)
#define GPIO_MODER_MODER0_0                  ((uint32_t)0x00000001U)
#define GPIO_MODER_MODER0_1                  ((uint32_t)0x00000002U)

#define GPIO_MODER_MODER1                    ((uint32_t)0x0000000CU)
#define GPIO_MODER_MODER1_0                  ((uint32_t)0x00000004U)
#define GPIO_MODER_MODER1_1                  ((uint32_t)0x00000008U)

#define GPIO_MODER_MODER2                    ((uint32_t)0x00000030U)
#define GPIO_MODER_MODER2_0                  ((uint32_t)0x00000010U)
#define GPIO_MODER_MODER2_1                  ((uint32_t)0x00000020U)

#define GPIO_MODER_MODER3                    ((uint32_t)0x000000C0U)
#define GPIO_MODER_MODER3_0                  ((uint32_t)0x00000040U)
#define GPIO_MODER_MODER3_1                  ((uint32_t)0x00000080U)

#define GPIO_MODER_MODER4                    ((uint32_t)0x00000300U)
#define GPIO_MODER_MODER4_0                  ((uint32_t)0x00000100U)
#define GPIO_MODER_MODER4_1                  ((uint32_t)0x00000200U)

#define GPIO_MODER_MODER5                    ((uint32_t)0x00000C00U)
#define GPIO_MODER_MODER5_0                  ((uint32_t)0x00000400U)
#define GPIO_MODER_MODER5_1                  ((uint32_t)0x00000800U)

#define GPIO_MODER_MODER6                    ((uint32_t)0x00003000U)
#define GPIO_MODER_MODER6_0                  ((uint32_t)0x00001000U)
#define GPIO_MODER_MODER6_1                  ((uint32_t)0x00002000U)

#define GPIO_MODER_MODER7                    ((uint32_t)0x0000C000U)
#define GPIO_MODER_MODER7_0                  ((uint32_t)0x00004000U)
#define GPIO_MODER_MODER7_1                  ((uint32_t)0x00008000U)

#define GPIO_MODER_MODER8                    ((uint32_t)0x00030000U)
#define GPIO_MODER_MODER8_0                  ((uint32_t)0x00010000U)
#define GPIO_MODER_MODER8_1                  ((uint32_t)0x00020000U)

#define GPIO_MODER_MODER9                    ((uint32_t)0x000C0000U)
#define GPIO_MODER_MODER9_0                  ((uint32_t)0x00040000U)
#define GPIO_MODER_MODER9_1                  ((uint32_t)0x00080000U)

#define GPIO_MODER_MODER10                   ((uint32_t)0x00300000U)
#define GPIO_MODER_MODER10_0                 ((uint32_t)0x00100000U)
#define GPIO_MODER_MODER10_1                 ((uint32_t)0x00200000U)

#define GPIO_MODER_MODER11                   ((uint32_t)0x00C00000U)
#define GPIO_MODER_MODER11_0                 ((uint32_t)0x00400000U)
#define GPIO_MODER_MODER11_1                 ((uint32_t)0x00800000U)

#define GPIO_MODER_MODER12                   ((uint32_t)0x03000000U)
#define GPIO_MODER_MODER12_0                 ((uint32_t)0x01000000U)
#define GPIO_MODER_MODER12_1                 ((uint32_t)0x02000000U)

#define GPIO_MODER_MODER13                   ((uint32_t)0x0C000000U)
#define GPIO_MODER_MODER13_0                 ((uint32_t)0x04000000U)
#define GPIO_MODER_MODER13_1                 ((uint32_t)0x08000000U)

#define GPIO_MODER_MODER14                   ((uint32_t)0x30000000U)
#define GPIO_MODER_MODER14_0                 ((uint32_t)0x10000000U)
#define GPIO_MODER_MODER14_1                 ((uint32_t)0x20000000U)

#define GPIO_MODER_MODER15                   ((uint32_t)0xC0000000U)
#define GPIO_MODER_MODER15_0                 ((uint32_t)0x40000000U)
#define GPIO_MODER_MODER15_1                 ((uint32_t)0x80000000U)

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT_0                     ((uint32_t)0x00000001U)
#define GPIO_OTYPER_OT_1                     ((uint32_t)0x00000002U)
#define GPIO_OTYPER_OT_2                     ((uint32_t)0x00000004U)
#define GPIO_OTYPER_OT_3                     ((uint32_t)0x00000008U)
#define GPIO_OTYPER_OT_4                     ((uint32_t)0x00000010U)
#define GPIO_OTYPER_OT_5                     ((uint32_t)0x00000020U)
#define GPIO_OTYPER_OT_6                     ((uint32_t)0x00000040U)
#define GPIO_OTYPER_OT_7                     ((uint32_t)0x00000080U)
#define GPIO_OTYPER_OT_8                     ((uint32_t)0x00000100U)
#define GPIO_OTYPER_OT_9                     ((uint32_t)0x00000200U)
#define GPIO_OTYPER_OT_10                    ((uint32_t)0x00000400U)
#define GPIO_OTYPER_OT_11                    ((uint32_t)0x00000800U)
#define GPIO_OTYPER_OT_12                    ((uint32_t)0x00001000U)
#define GPIO_OTYPER_OT_13                    ((uint32_t)0x00002000U)
#define GPIO_OTYPER_OT_14                    ((uint32_t)0x00004000U)
#define GPIO_OTYPER_OT_15                    ((uint32_t)0x00008000U)

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDER_OSPEEDR0               ((uint32_t)0x00000003U)
#define GPIO_OSPEEDER_OSPEEDR0_0             ((uint32_t)0x00000001U)
#define GPIO_OSPEEDER_OSPEEDR0_1             ((uint32_t)0x00000002U)

#define GPIO_OSPEEDER_OSPEEDR1               ((uint32_t)0x0000000CU)
#define GPIO_OSPEEDER_OSPEEDR1_0             ((uint32_t)0x00000004U)
#define GPIO_OSPEEDER_OSPEEDR1_1             ((uint32_t)0x00000008U)

#define GPIO_OSPEEDER_OSPEEDR2               ((uint32_t)0x00000030U)
#define GPIO_OSPEEDER_OSPEEDR2_0             ((uint32_t)0x00000010U)
#define GPIO_OSPEEDER_OSPEEDR2_1             ((uint32_t)0x00000020U)

#define GPIO_OSPEEDER_OSPEEDR3               ((uint32_t)0x000000C0U)
#define GPIO_OSPEEDER_OSPEEDR3_0             ((uint32_t)0x00000040U)
#define GPIO_OSPEEDER_OSPEEDR3_1             ((uint32_t)0x00000080U)

#define GPIO_OSPEEDER_OSPEEDR4               ((uint32_t)0x00000300U)
#define GPIO_OSPEEDER_OSPEEDR4_0             ((uint32_t)0x00000100U)
#define GPIO_OSPEEDER_OSPEEDR4_1             ((uint32_t)0x00000200U)

#define GPIO_OSPEEDER_OSPEEDR5               ((uint32_t)0x00000C00U)
#define GPIO_OSPEEDER_OSPEEDR5_0             ((uint32_t)0x00000400U)
#define GPIO_OSPEEDER_OSPEEDR5_1             ((uint32_t)0x00000800U)

#define GPIO_OSPEEDER_OSPEEDR6               ((uint32_t)0x00003000U)
#define GPIO_OSPEEDER_OSPEEDR6_0             ((uint32_t)0x00001000U)
#define GPIO_OSPEEDER_OSPEEDR6_1             ((uint32_t)0x00002000U)

#define GPIO_OSPEEDER_OSPEEDR7               ((uint32_t)0x0000C000U)
#define GPIO_OSPEEDER_OSPEEDR7_0             ((uint32_t)0x00004000U)
#define GPIO_OSPEEDER_OSPEEDR7_1             ((uint32_t)0x00008000U)

#define GPIO_OSPEEDER_OSPEEDR8               ((uint32_t)0x00030000U)
#define GPIO_OSPEEDER_OSPEEDR8_0             ((uint32_t)0x00010000U)
#define GPIO_OSPEEDER_OSPEEDR8_1             ((uint32_t)0x00020000U)

#define GPIO_OSPEEDER_OSPEEDR9               ((uint32_t)0x000C0000U)
#define GPIO_OSPEEDER_OSPEEDR9_0             ((uint32_t)0x00040000U)
#define GPIO_OSPEEDER_OSPEEDR9_1             ((uint32_t)0x00080000U)

#define GPIO_OSPEEDER_OSPEEDR10              ((uint32_t)0x00300000U)
#define GPIO_OSPEEDER_OSPEEDR10_0            ((uint32_t)0x00100000U)
#define GPIO_OSPEEDER_OSPEEDR10_1            ((uint32_t)0x00200000U)

#define GPIO_OSPEEDER_OSPEEDR11              ((uint32_t)0x00C00000U)
#define GPIO_OSPEEDER_OSPEEDR11_0            ((uint32_t)0x00400000U)
#define GPIO_OSPEEDER_OSPEEDR11_1            ((uint32_t)0x00800000U)

#define GPIO_OSPEEDER_OSPEEDR12              ((uint32_t)0x03000000U)
#define GPIO_OSPEEDER_OSPEEDR12_0            ((uint32_t)0x01000000U)
#define GPIO_OSPEEDER_OSPEEDR12_1            ((uint32_t)0x02000000U)

#define GPIO_OSPEEDER_OSPEEDR13              ((uint32_t)0x0C000000U)
#define GPIO_OSPEEDER_OSPEEDR13_0            ((uint32_t)0x04000000U)
#define GPIO_OSPEEDER_OSPEEDR13_1            ((uint32_t)0x08000000U)

#define GPIO_OSPEEDER_OSPEEDR14              ((uint32_t)0x30000000U)
#define GPIO_OSPEEDER_OSPEEDR14_0            ((uint32_t)0x10000000U)
#define GPIO_OSPEEDER_OSPEEDR14_1            ((uint32_t)0x20000000U)

#define GPIO_OSPEEDER_OSPEEDR15              ((uint32_t)0xC0000000U)
#define GPIO_OSPEEDER_OSPEEDR15_0            ((uint32_t)0x40000000U)
#define GPIO_OSPEEDER_OSPEEDR15_1            ((uint32_t)0x80000000U)

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPDR0                    ((uint32_t)0x00000003U)
#define GPIO_PUPDR_PUPDR0_0                  ((uint32_t)0x00000001U)
#define GPIO_PUPDR_PUPDR0_1                  ((uint32_t)0x00000002U)

#define GPIO_PUPDR_PUPDR1                    ((uint32_t)0x0000000CU)
#define GPIO_PUPDR_PUPDR1_0                  ((uint32_t)0x00000004U)
#define GPIO_PUPDR_PUPDR1_1                  ((uint32_t)0x00000008U)

#define GPIO_PUPDR_PUPDR2                    ((uint32_t)0x00000030U)
#define GPIO_PUPDR_PUPDR2_0                  ((uint32_t)0x00000010U)
#define GPIO_PUPDR_PUPDR2_1                  ((uint32_t)0x00000020U)

#define GPIO_PUPDR_PUPDR3                    ((uint32_t)0x000000C0U)
#define GPIO_PUPDR_PUPDR3_0                  ((uint32_t)0x00000040U)
#define GPIO_PUPDR_PUPDR3_1                  ((uint32_t)0x00000080U)

#define GPIO_PUPDR_PUPDR4                    ((uint32_t)0x00000300U)
#define GPIO_PUPDR_PUPDR4_0                  ((uint32_t)0x00000100U)
#define GPIO_PUPDR_PUPDR4_1                  ((uint32_t)0x00000200U)

#define GPIO_PUPDR_PUPDR5                    ((uint32_t)0x00000C00U)
#define GPIO_PUPDR_PUPDR5_0                  ((uint32_t)0x00000400U)
#define GPIO_PUPDR_PUPDR5_1                  ((uint32_t)0x00000800U)

#define GPIO_PUPDR_PUPDR6                    ((uint32_t)0x00003000U)
#define GPIO_PUPDR_PUPDR6_0                  ((uint32_t)0x00001000U)
#define GPIO_PUPDR_PUPDR6_1                  ((uint32_t)0x00002000U)

#define GPIO_PUPDR_PUPDR7                    ((uint32_t)0x0000C000U)
#define GPIO_PUPDR_PUPDR7_0                  ((uint32_t)0x00004000U)
#define GPIO_PUPDR_PUPDR7_1                  ((uint32_t)0x00008000U)

#define GPIO_PUPDR_PUPDR8                    ((uint32_t)0x00030000U)
#define GPIO_PUPDR_PUPDR8_0                  ((uint32_t)0x00010000U)
#define GPIO_PUPDR_PUPDR8_1                  ((uint32_t)0x00020000U)

#define GPIO_PUPDR_PUPDR9                    ((uint32_t)0x000C0000U)
#define GPIO_PUPDR_PUPDR9_0                  ((uint32_t)0x00040000U)
#define GPIO_PUPDR_PUPDR9_1                  ((uint32_t)0x00080000U)

#define GPIO_PUPDR_PUPDR10                   ((uint32_t)0x00300000U)
#define GPIO_PUPDR_PUPDR10_0                 ((uint32_t)0x00100000U)
#define GPIO_PUPDR_PUPDR10_1                 ((uint32_t)0x00200000U)

#define GPIO_PUPDR_PUPDR11                   ((uint32_t)0x00C00000U)
#define GPIO_PUPDR_PUPDR11_0                 ((uint32_t)0x00400000U)
#define GPIO_PUPDR_PUPDR11_1                 ((uint32_t)0x00800000U)

#define GPIO_PUPDR_PUPDR12                   ((uint32_t)0x03000000U)
#define GPIO_PUPDR_PUPDR12_0                 ((uint32_t)0x01000000U)
#define GPIO_PUPDR_PUPDR12_1                 ((uint32_t)0x02000000U)

#define GPIO_PUPDR_PUPDR13                   ((uint32_t)0x0C000000U)
#define GPIO_PUPDR_PUPDR13_0                 ((uint32_t)0x04000000U)
#define GPIO_PUPDR_PUPDR13_1                 ((uint32_t)0x08000000U)

#define GPIO_PUPDR_PUPDR14                   ((uint32_t)0x30000000U)
#define GPIO_PUPDR_PUPDR14_0                 ((uint32_t)0x10000000U)
#define GPIO_PUPDR_PUPDR14_1                 ((uint32_t)0x20000000U)

#define GPIO_PUPDR_PUPDR15                   ((uint32_t)0xC0000000U)
#define GPIO_PUPDR_PUPDR15_0                 ((uint32_t)0x40000000U)
#define GPIO_PUPDR_PUPDR15_1                 ((uint32_t)0x80000000U)

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR_0                       ((uint32_t)0x00000001U)
#define GPIO_IDR_IDR_1                       ((uint32_t)0x00000002U)
#define GPIO_IDR_IDR_2                       ((uint32_t)0x00000004U)
#define GPIO_IDR_IDR_3                       ((uint32_t)0x00000008U)
#define GPIO_IDR_IDR_4                       ((uint32_t)0x00000010U)
#define GPIO_IDR_IDR_5                       ((uint32_t)0x00000020U)
#define GPIO_IDR_IDR_6                       ((uint32_t)0x00000040U)
#define GPIO_IDR_IDR_7                       ((uint32_t)0x00000080U)
#define GPIO_IDR_IDR_8                       ((uint32_t)0x00000100U)
#define GPIO_IDR_IDR_9                       ((uint32_t)0x00000200U)
#define GPIO_IDR_IDR_10                      ((uint32_t)0x00000400U)
#define GPIO_IDR_IDR_11                      ((uint32_t)0x00000800U)
#define GPIO_IDR_IDR_12                      ((uint32_t)0x00001000U)
#define GPIO_IDR_IDR_13                      ((uint32_t)0x00002000U)
#define GPIO_IDR_IDR_14                      ((uint32_t)0x00004000U)
#define GPIO_IDR_IDR_15                      ((uint32_t)0x00008000U)

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR_0                       ((uint32_t)0x00000001U)
#define GPIO_ODR_ODR_1                       ((uint32_t)0x00000002U)
#define GPIO_ODR_ODR_2                       ((uint32_t)0x00000004U)
#define GPIO_ODR_ODR_3                       ((uint32_t)0x00000008U)
#define GPIO_ODR_ODR_4                       ((uint32_t)0x00000010U)
#define GPIO_ODR_ODR_5                       ((uint32_t)0x00000020U)
#define GPIO_ODR_ODR_6                       ((uint32_t)0x00000040U)
#define GPIO_ODR_ODR_7                       ((uint32_t)0x00000080U)
#define GPIO_ODR_ODR_8                       ((uint32_t)0x00000100U)
#define GPIO_ODR_ODR_9                       ((uint32_t)0x00000200U)
#define GPIO_ODR_ODR_10                      ((uint32_t)0x00000400U)
#define GPIO_ODR_ODR_11                      ((uint32_t)0x00000800U)
#define GPIO_ODR_ODR_12                      ((uint32_t)0x00001000U)
#define GPIO_ODR_ODR_13                      ((uint32_t)0x00002000U)
#define GPIO_ODR_ODR_14                      ((uint32_t)0x00004000U)
#define GPIO_ODR_ODR_15                      ((uint32_t)0x00008000U)

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS_0                       ((uint32_t)0x00000001U)
#define GPIO_BSRR_BS_1                       ((uint32_t)0x00000002U)
#define GPIO_BSRR_BS_2                       ((uint32_t)0x00000004U)
#define GPIO_BSRR_BS_3                       ((uint32_t)0x00000008U)
#define GPIO_BSRR_BS_4                       ((uint32_t)0x00000010U)
#define GPIO_BSRR_BS_5                       ((uint32_t)0x00000020U)
#define GPIO_BSRR_BS_6                       ((uint32_t)0x00000040U)
#define GPIO_BSRR_BS_7                       ((uint32_t)0x00000080U)
#define GPIO_BSRR_BS_8                       ((uint32_t)0x00000100U)
#define GPIO_BSRR_BS_9                       ((uint32_t)0x00000200U)
#define GPIO_BSRR_BS_10                      ((uint32_t)0x00000400U)
#define GPIO_BSRR_BS_11                      ((uint32_t)0x00000800U)
#define GPIO_BSRR_BS_12                      ((uint32_t)0x00001000U)
#define GPIO_BSRR_BS_13                      ((uint32_t)0x00002000U)
#define GPIO_BSRR_BS_14                      ((uint32_t)0x00004000U)
#define GPIO_BSRR_BS_15                      ((uint32_t)0x00008000U)
#define GPIO_BSRR_BR_0                       ((uint32_t)0x00010000U)
#define GPIO_BSRR_BR_1                       ((uint32_t)0x00020000U)
#define GPIO_BSRR_BR_2                       ((uint32_t)0x00040000U)
#define GPIO_BSRR_BR_3                       ((uint32_t)0x00080000U)
#define GPIO_BSRR_BR_4                       ((uint32_t)0x00100000U)
#define GPIO_BSRR_BR_5                       ((uint32_t)0x00200000U)
#define GPIO_BSRR_BR_6                       ((uint32_t)0x00400000U)
#define GPIO_BSRR_BR_7                       ((uint32_t)0x00800000U)
#define GPIO_BSRR_BR_8                       ((uint32_t)0x01000000U)
#define GPIO_BSRR_BR_9                       ((uint32_t)0x02000000U)
#define GPIO_BSRR_BR_10                      ((uint32_t)0x04000000U)
#define GPIO_BSRR_BR_11                      ((uint32_t)0x08000000U)
#define GPIO_BSRR_BR_12                      ((uint32_t)0x10000000U)
#define GPIO_BSRR_BR_13                      ((uint32_t)0x20000000U)
#define GPIO_BSRR_BR_14                      ((uint32_t)0x40000000U)
#define GPIO_BSRR_BR_15                      ((uint32_t)0x80000000U)

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0                       ((uint32_t)0x00000001U)
#define GPIO_LCKR_LCK1                       ((uint32_t)0x00000002U)
#define GPIO_LCKR_LCK2                       ((uint32_t)0x00000004U)
#define GPIO_LCKR_LCK3                       ((uint32_t)0x00000008U)
#define GPIO_LCKR_LCK4                       ((uint32_t)0x00000010U)
#define GPIO_LCKR_LCK5                       ((uint32_t)0x00000020U)
#define GPIO_LCKR_LCK6                       ((uint32_t)0x00000040U)
#define GPIO_LCKR_LCK7                       ((uint32_t)0x00000080U)
#define GPIO_LCKR_LCK8                       ((uint32_t)0x00000100U)
#define GPIO_LCKR_LCK9                       ((uint32_t)0x00000200U)
#define GPIO_LCKR_LCK10                      ((uint32_t)0x00000400U)
#define GPIO_LCKR_LCK11                      ((uint32_t)0x00000800U)
#define GPIO_LCKR_LCK12                      ((uint32_t)0x00001000U)
#define GPIO_LCKR_LCK13                      ((uint32_t)0x00002000U)
#define GPIO_LCKR_LCK14                      ((uint32_t)0x00004000U)
#define GPIO_LCKR_LCK15                      ((uint32_t)0x00008000U)
#define GPIO_LCKR_LCKK                       ((uint32_t)0x00010000U)

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define  I2C_CR1_PE                          ((uint32_t)0x00000001U)     /*!<Peripheral Enable                             */
#define  I2C_CR1_SMBUS                       ((uint32_t)0x00000002U)     /*!<SMBus Mode                                    */
#define  I2C_CR1_SMBTYPE                     ((uint32_t)0x00000008U)     /*!<SMBus Type                                    */
#define  I2C_CR1_ENARP                       ((uint32_t)0x00000010U)     /*!<ARP Enable                                    */
#define  I2C_CR1_ENPEC                       ((uint32_t)0x00000020U)     /*!<PEC Enable                                    */
#define  I2C_CR1_ENGC                        ((uint32_t)0x00000040U)     /*!<General Call Enable                           */
#define  I2C_CR1_NOSTRETCH                   ((uint32_t)0x00000080U)     /*!<Clock Stretching Disable (Slave mode)  */
#define  I2C_CR1_START                       ((uint32_t)0x00000100U)     /*!<Start Generation                              */
#define  I2C_CR1_STOP                        ((uint32_t)0x00000200U)     /*!<Stop Generation                               */
#define  I2C_CR1_ACK                         ((uint32_t)0x00000400U)     /*!<Acknowledge Enable                            */
#define  I2C_CR1_POS                         ((uint32_t)0x00000800U)     /*!<Acknowledge/PEC Position (for data reception) */
#define  I2C_CR1_PEC                         ((uint32_t)0x00001000U)     /*!<Packet Error Checking                         */
#define  I2C_CR1_ALERT                       ((uint32_t)0x00002000U)     /*!<SMBus Alert                                   */
#define  I2C_CR1_SWRST                       ((uint32_t)0x00008000U)     /*!<Software Reset                                */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_FREQ                        ((uint32_t)0x0000003FU)     /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
#define  I2C_CR2_FREQ_0                      ((uint32_t)0x00000001U)     /*!<Bit 0 */
#define  I2C_CR2_FREQ_1                      ((uint32_t)0x00000002U)     /*!<Bit 1 */
#define  I2C_CR2_FREQ_2                      ((uint32_t)0x00000004U)     /*!<Bit 2 */
#define  I2C_CR2_FREQ_3                      ((uint32_t)0x00000008U)     /*!<Bit 3 */
#define  I2C_CR2_FREQ_4                      ((uint32_t)0x00000010U)     /*!<Bit 4 */
#define  I2C_CR2_FREQ_5                      ((uint32_t)0x00000020U)     /*!<Bit 5 */

#define  I2C_CR2_ITERREN                     ((uint32_t)0x00000100U)     /*!<Error Interrupt Enable  */
#define  I2C_CR2_ITEVTEN                     ((uint32_t)0x00000200U)     /*!<Event Interrupt Enable  */
#define  I2C_CR2_ITBUFEN                     ((uint32_t)0x00000400U)     /*!<Buffer Interrupt Enable */
#define  I2C_CR2_DMAEN                       ((uint32_t)0x00000800U)     /*!<DMA Requests Enable     */
#define  I2C_CR2_LAST                        ((uint32_t)0x00001000U)     /*!<DMA Last Transfer       */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define  I2C_OAR1_ADD1_7                     ((uint32_t)0x000000FEU)     /*!<Interface Address */
#define  I2C_OAR1_ADD8_9                     ((uint32_t)0x00000300U)     /*!<Interface Address */

#define  I2C_OAR1_ADD0                       ((uint32_t)0x00000001U)     /*!<Bit 0 */
#define  I2C_OAR1_ADD1                       ((uint32_t)0x00000002U)     /*!<Bit 1 */
#define  I2C_OAR1_ADD2                       ((uint32_t)0x00000004U)     /*!<Bit 2 */
#define  I2C_OAR1_ADD3                       ((uint32_t)0x00000008U)     /*!<Bit 3 */
#define  I2C_OAR1_ADD4                       ((uint32_t)0x00000010U)     /*!<Bit 4 */
#define  I2C_OAR1_ADD5                       ((uint32_t)0x00000020U)     /*!<Bit 5 */
#define  I2C_OAR1_ADD6                       ((uint32_t)0x00000040U)     /*!<Bit 6 */
#define  I2C_OAR1_ADD7                       ((uint32_t)0x00000080U)     /*!<Bit 7 */
#define  I2C_OAR1_ADD8                       ((uint32_t)0x00000100U)     /*!<Bit 8 */
#define  I2C_OAR1_ADD9                       ((uint32_t)0x00000200U)     /*!<Bit 9 */

#define  I2C_OAR1_ADDMODE                    ((uint32_t)0x00008000U)     /*!<Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_ENDUAL                     ((uint32_t)0x00000001U)        /*!<Dual addressing mode enable */
#define  I2C_OAR2_ADD2                       ((uint32_t)0x000000FEU)        /*!<Interface address           */

/********************  Bit definition for I2C_DR register  ********************/
#define  I2C_DR_DR                           ((uint32_t)0x000000FFU)        /*!<8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define  I2C_SR1_SB                          ((uint32_t)0x00000001U)     /*!<Start Bit (Master mode)                  */
#define  I2C_SR1_ADDR                        ((uint32_t)0x00000002U)     /*!<Address sent (master mode)/matched (slave mode) */
#define  I2C_SR1_BTF                         ((uint32_t)0x00000004U)     /*!<Byte Transfer Finished                          */
#define  I2C_SR1_ADD10                       ((uint32_t)0x00000008U)     /*!<10-bit header sent (Master mode)         */
#define  I2C_SR1_STOPF                       ((uint32_t)0x00000010U)     /*!<Stop detection (Slave mode)              */
#define  I2C_SR1_RXNE                        ((uint32_t)0x00000040U)     /*!<Data Register not Empty (receivers)      */
#define  I2C_SR1_TXE                         ((uint32_t)0x00000080U)     /*!<Data Register Empty (transmitters)       */
#define  I2C_SR1_BERR                        ((uint32_t)0x00000100U)     /*!<Bus Error                                       */
#define  I2C_SR1_ARLO                        ((uint32_t)0x00000200U)     /*!<Arbitration Lost (master mode)           */
#define  I2C_SR1_AF                          ((uint32_t)0x00000400U)     /*!<Acknowledge Failure                             */
#define  I2C_SR1_OVR                         ((uint32_t)0x00000800U)     /*!<Overrun/Underrun                                */
#define  I2C_SR1_PECERR                      ((uint32_t)0x00001000U)     /*!<PEC Error in reception                          */
#define  I2C_SR1_TIMEOUT                     ((uint32_t)0x00004000U)     /*!<Timeout or Tlow Error                           */
#define  I2C_SR1_SMBALERT                    ((uint32_t)0x00008000U)     /*!<SMBus Alert                                     */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define  I2C_SR2_MSL                         ((uint32_t)0x00000001U)     /*!<Master/Slave                              */
#define  I2C_SR2_BUSY                        ((uint32_t)0x00000002U)     /*!<Bus Busy                                  */
#define  I2C_SR2_TRA                         ((uint32_t)0x00000004U)     /*!<Transmitter/Receiver                      */
#define  I2C_SR2_GENCALL                     ((uint32_t)0x00000010U)     /*!<General Call Address (Slave mode)  */
#define  I2C_SR2_SMBDEFAULT                  ((uint32_t)0x00000020U)     /*!<SMBus Device Default Address (Slave mode) */
#define  I2C_SR2_SMBHOST                     ((uint32_t)0x00000040U)     /*!<SMBus Host Header (Slave mode)     */
#define  I2C_SR2_DUALF                       ((uint32_t)0x00000080U)     /*!<Dual Flag (Slave mode)             */
#define  I2C_SR2_PEC                         ((uint32_t)0x0000FF00U)     /*!<Packet Error Checking Register            */

/*******************  Bit definition for I2C_CCR register  ********************/
#define  I2C_CCR_CCR                         ((uint32_t)0x00000FFFU)     /*!<Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CCR_DUTY                        ((uint32_t)0x00004000U)     /*!<Fast Mode Duty Cycle                                       */
#define  I2C_CCR_FS                          ((uint32_t)0x00008000U)     /*!<I2C Master Mode Selection                                  */

/******************  Bit definition for I2C_TRISE register  *******************/
#define  I2C_TRISE_TRISE                     ((uint32_t)0x0000003FU)     /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************  Bit definition for I2C_FLTR register  *******************/
#define  I2C_FLTR_DNF                        ((uint32_t)0x0000000FU)     /*!<Digital Noise Filter */
#define  I2C_FLTR_ANOFF                      ((uint32_t)0x00000010U)     /*!<Analog Noise Filter OFF */

/******************************************************************************/
/*                                                                            */
/*        Fast Mode Plus Inter-integrated Circuit Interface (I2C)             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  *******************/
#define  FMPI2C_CR1_PE                        ((uint32_t)0x00000001U)        /*!< Peripheral enable                   */
#define  FMPI2C_CR1_TXIE                      ((uint32_t)0x00000002U)        /*!< TX interrupt enable                 */
#define  FMPI2C_CR1_RXIE                      ((uint32_t)0x00000004U)        /*!< RX interrupt enable                 */
#define  FMPI2C_CR1_ADDRIE                    ((uint32_t)0x00000008U)        /*!< Address match interrupt enable      */
#define  FMPI2C_CR1_NACKIE                    ((uint32_t)0x00000010U)        /*!< NACK received interrupt enable      */
#define  FMPI2C_CR1_STOPIE                    ((uint32_t)0x00000020U)        /*!< STOP detection interrupt enable     */
#define  FMPI2C_CR1_TCIE                      ((uint32_t)0x00000040U)        /*!< Transfer complete interrupt enable  */
#define  FMPI2C_CR1_ERRIE                     ((uint32_t)0x00000080U)        /*!< Errors interrupt enable             */
#define  FMPI2C_CR1_DFN                       ((uint32_t)0x00000F00U)        /*!< Digital noise filter                */
#define  FMPI2C_CR1_ANFOFF                    ((uint32_t)0x00001000U)        /*!< Analog noise filter OFF             */
#define  FMPI2C_CR1_SWRST                     ((uint32_t)0x00002000U)        /*!< Software reset                      */
#define  FMPI2C_CR1_TXDMAEN                   ((uint32_t)0x00004000U)        /*!< DMA transmission requests enable    */
#define  FMPI2C_CR1_RXDMAEN                   ((uint32_t)0x00008000U)        /*!< DMA reception requests enable       */
#define  FMPI2C_CR1_SBC                       ((uint32_t)0x00010000U)        /*!< Slave byte control                  */
#define  FMPI2C_CR1_NOSTRETCH                 ((uint32_t)0x00020000U)        /*!< Clock stretching disable            */
#define  FMPI2C_CR1_WUPEN                     ((uint32_t)0x00040000U)        /*!< Wakeup from STOP enable             */
#define  FMPI2C_CR1_GCEN                      ((uint32_t)0x00080000U)        /*!< General call enable                 */
#define  FMPI2C_CR1_ALERTEN                   ((uint32_t)0x00400000U)        /*!< SMBus alert enable                  */
#define  FMPI2C_CR1_PECEN                     ((uint32_t)0x00800000U)        /*!< PEC enable                          */

/******************  Bit definition for I2C_CR2 register  ********************/
#define  FMPI2C_CR2_SADD                      ((uint32_t)0x000003FFU)        /*!< Slave address (master mode)                             */
#define  FMPI2C_CR2_RD_WRN                    ((uint32_t)0x00000400U)        /*!< Transfer direction (master mode)                        */
#define  FMPI2C_CR2_ADD10                     ((uint32_t)0x00000800U)        /*!< 10-bit addressing mode (master mode)                    */
#define  FMPI2C_CR2_HEAD10R                   ((uint32_t)0x00001000U)        /*!< 10-bit address header only read direction (master mode) */
#define  FMPI2C_CR2_START                     ((uint32_t)0x00002000U)        /*!< START generation                                        */
#define  FMPI2C_CR2_STOP                      ((uint32_t)0x00004000U)        /*!< STOP generation (master mode)                           */
#define  FMPI2C_CR2_NACK                      ((uint32_t)0x00008000U)        /*!< NACK generation (slave mode)                            */
#define  FMPI2C_CR2_NBYTES                    ((uint32_t)0x00FF0000U)        /*!< Number of bytes                                         */
#define  FMPI2C_CR2_RELOAD                    ((uint32_t)0x01000000U)        /*!< NBYTES reload mode                                      */
#define  FMPI2C_CR2_AUTOEND                   ((uint32_t)0x02000000U)        /*!< Automatic end mode (master mode)                        */
#define  FMPI2C_CR2_PECBYTE                   ((uint32_t)0x04000000U)        /*!< Packet error checking byte                              */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define  FMPI2C_OAR1_OA1                      ((uint32_t)0x000003FFU)        /*!< Interface own address 1   */
#define  FMPI2C_OAR1_OA1MODE                  ((uint32_t)0x00000400U)        /*!< Own address 1 10-bit mode */
#define  FMPI2C_OAR1_OA1EN                    ((uint32_t)0x00008000U)        /*!< Own address 1 enable      */

/*******************  Bit definition for I2C_OAR2 register  ******************/
#define  FMPI2C_OAR2_OA2                      ((uint32_t)0x000000FEU)        /*!< Interface own address 2 */
#define  FMPI2C_OAR2_OA2MSK                   ((uint32_t)0x00000700U)        /*!< Own address 2 masks     */
#define  FMPI2C_OAR2_OA2EN                    ((uint32_t)0x00008000U)        /*!< Own address 2 enable    */

/*******************  Bit definition for I2C_TIMINGR register *******************/
#define  FMPI2C_TIMINGR_SCLL                  ((uint32_t)0x000000FFU)        /*!< SCL low period (master mode)  */
#define  FMPI2C_TIMINGR_SCLH                  ((uint32_t)0x0000FF00U)        /*!< SCL high period (master mode) */
#define  FMPI2C_TIMINGR_SDADEL                ((uint32_t)0x000F0000U)        /*!< Data hold time                */
#define  FMPI2C_TIMINGR_SCLDEL                ((uint32_t)0x00F00000U)        /*!< Data setup time               */
#define  FMPI2C_TIMINGR_PRESC                 ((uint32_t)0xF0000000U)        /*!< Timings prescaler             */

/******************* Bit definition for I2C_TIMEOUTR register *******************/
#define  FMPI2C_TIMEOUTR_TIMEOUTA             ((uint32_t)0x00000FFFU)        /*!< Bus timeout A                 */
#define  FMPI2C_TIMEOUTR_TIDLE                ((uint32_t)0x00001000U)        /*!< Idle clock timeout detection  */
#define  FMPI2C_TIMEOUTR_TIMOUTEN             ((uint32_t)0x00008000U)        /*!< Clock timeout enable          */
#define  FMPI2C_TIMEOUTR_TIMEOUTB             ((uint32_t)0x0FFF0000U)        /*!< Bus timeout B                 */
#define  FMPI2C_TIMEOUTR_TEXTEN               ((uint32_t)0x80000000U)        /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  *********************/
#define  FMPI2C_ISR_TXE                       ((uint32_t)0x00000001U)        /*!< Transmit data register empty     */
#define  FMPI2C_ISR_TXIS                      ((uint32_t)0x00000002U)        /*!< Transmit interrupt status        */
#define  FMPI2C_ISR_RXNE                      ((uint32_t)0x00000004U)        /*!< Receive data register not empty  */
#define  FMPI2C_ISR_ADDR                      ((uint32_t)0x00000008U)        /*!< Address matched (slave mode)     */
#define  FMPI2C_ISR_NACKF                     ((uint32_t)0x00000010U)        /*!< NACK received flag               */
#define  FMPI2C_ISR_STOPF                     ((uint32_t)0x00000020U)        /*!< STOP detection flag              */
#define  FMPI2C_ISR_TC                        ((uint32_t)0x00000040U)        /*!< Transfer complete (master mode)  */
#define  FMPI2C_ISR_TCR                       ((uint32_t)0x00000080U)        /*!< Transfer complete reload         */
#define  FMPI2C_ISR_BERR                      ((uint32_t)0x00000100U)        /*!< Bus error                        */
#define  FMPI2C_ISR_ARLO                      ((uint32_t)0x00000200U)        /*!< Arbitration lost                 */
#define  FMPI2C_ISR_OVR                       ((uint32_t)0x00000400U)        /*!< Overrun/Underrun                 */
#define  FMPI2C_ISR_PECERR                    ((uint32_t)0x00000800U)        /*!< PEC error in reception           */
#define  FMPI2C_ISR_TIMEOUT                   ((uint32_t)0x00001000U)        /*!< Timeout or Tlow detection flag   */
#define  FMPI2C_ISR_ALERT                     ((uint32_t)0x00002000U)        /*!< SMBus alert                      */
#define  FMPI2C_ISR_BUSY                      ((uint32_t)0x00008000U)        /*!< Bus busy                         */
#define  FMPI2C_ISR_DIR                       ((uint32_t)0x00010000U)        /*!< Transfer direction (slave mode)  */
#define  FMPI2C_ISR_ADDCODE                   ((uint32_t)0x00FE0000U)        /*!< Address match code (slave mode)  */

/******************  Bit definition for I2C_ICR register  *********************/
#define  FMPI2C_ICR_ADDRCF                    ((uint32_t)0x00000008U)        /*!< Address matched clear flag  */
#define  FMPI2C_ICR_NACKCF                    ((uint32_t)0x00000010U)        /*!< NACK clear flag             */
#define  FMPI2C_ICR_STOPCF                    ((uint32_t)0x00000020U)        /*!< STOP detection clear flag   */
#define  FMPI2C_ICR_BERRCF                    ((uint32_t)0x00000100U)        /*!< Bus error clear flag        */
#define  FMPI2C_ICR_ARLOCF                    ((uint32_t)0x00000200U)        /*!< Arbitration lost clear flag */
#define  FMPI2C_ICR_OVRCF                     ((uint32_t)0x00000400U)        /*!< Overrun/Underrun clear flag */
#define  FMPI2C_ICR_PECCF                     ((uint32_t)0x00000800U)        /*!< PAC error clear flag        */
#define  FMPI2C_ICR_TIMOUTCF                  ((uint32_t)0x00001000U)        /*!< Timeout clear flag          */
#define  FMPI2C_ICR_ALERTCF                   ((uint32_t)0x00002000U)        /*!< Alert clear flag            */

/******************  Bit definition for I2C_PECR register  *********************/
#define  FMPI2C_PECR_PEC                      ((uint32_t)0x000000FFU)        /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define  FMPI2C_RXDR_RXDATA                   ((uint32_t)0x000000FFU)        /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *********************/
#define  FMPI2C_TXDR_TXDATA                   ((uint32_t)0x000000FFU)        /*!< 8-bit transmit data */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         ((uint32_t)0xFFFFU)            /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          ((uint32_t)0x07U)               /*!<PR[2:0] (Prescaler divider)         */
#define  IWDG_PR_PR_0                        ((uint32_t)0x01U)               /*!<Bit 0 */
#define  IWDG_PR_PR_1                        ((uint32_t)0x02U)               /*!<Bit 1 */
#define  IWDG_PR_PR_2                        ((uint32_t)0x04U)               /*!<Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         ((uint32_t)0x0FFFU)            /*!<Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         ((uint32_t)0x01U)               /*!<Watchdog prescaler value update      */
#define  IWDG_SR_RVU                         ((uint32_t)0x02U)               /*!<Watchdog counter reload value update */


/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         ((uint32_t)0x00000001U)     /*!< Low-Power Deepsleep                 */
#define  PWR_CR_PDDS                         ((uint32_t)0x00000002U)     /*!< Power Down Deepsleep                */
#define  PWR_CR_CWUF                         ((uint32_t)0x00000004U)     /*!< Clear Wakeup Flag                   */
#define  PWR_CR_CSBF                         ((uint32_t)0x00000008U)     /*!< Clear Standby Flag                  */
#define  PWR_CR_PVDE                         ((uint32_t)0x00000010U)     /*!< Power Voltage Detector Enable       */

#define  PWR_CR_PLS                          ((uint32_t)0x000000E0U)     /*!< PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CR_PLS_0                        ((uint32_t)0x00000020U)     /*!< Bit 0 */
#define  PWR_CR_PLS_1                        ((uint32_t)0x00000040U)     /*!< Bit 1 */
#define  PWR_CR_PLS_2                        ((uint32_t)0x00000080U)     /*!< Bit 2 */

/*!< PVD level configuration */
#define  PWR_CR_PLS_LEV0                     ((uint32_t)0x00000000U)     /*!< PVD level 0 */
#define  PWR_CR_PLS_LEV1                     ((uint32_t)0x00000020U)     /*!< PVD level 1 */
#define  PWR_CR_PLS_LEV2                     ((uint32_t)0x00000040U)     /*!< PVD level 2 */
#define  PWR_CR_PLS_LEV3                     ((uint32_t)0x00000060U)     /*!< PVD level 3 */
#define  PWR_CR_PLS_LEV4                     ((uint32_t)0x00000080U)     /*!< PVD level 4 */
#define  PWR_CR_PLS_LEV5                     ((uint32_t)0x000000A0U)     /*!< PVD level 5 */
#define  PWR_CR_PLS_LEV6                     ((uint32_t)0x000000C0U)     /*!< PVD level 6 */
#define  PWR_CR_PLS_LEV7                     ((uint32_t)0x000000E0U)     /*!< PVD level 7 */

#define  PWR_CR_DBP                          ((uint32_t)0x00000100U)     /*!< Disable Backup Domain write protection                     */
#define  PWR_CR_FPDS                         ((uint32_t)0x00000200U)     /*!< Flash power down in Stop mode                              */
#define  PWR_CR_LPLVDS                       ((uint32_t)0x00000400U)     /*!< Low Power Regulator Low Voltage in Deep Sleep mode         */
#define  PWR_CR_MRLVDS                       ((uint32_t)0x00000800U)     /*!< Main Regulator Low Voltage in Deep Sleep mode              */
#define  PWR_CR_ADCDC1                       ((uint32_t)0x00002000U)     /*!< Refer to AN4073 on how to use this bit                     */

#define  PWR_CR_VOS                          ((uint32_t)0x0000C000U)     /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
#define  PWR_CR_VOS_0                        ((uint32_t)0x00004000U)     /*!< Bit 0 */
#define  PWR_CR_VOS_1                        ((uint32_t)0x00008000U)     /*!< Bit 1 */

#define  PWR_CR_FMSSR                        ((uint32_t)0x00100000U)     /*!< Flash Memory Sleep System Run        */
#define  PWR_CR_FISSR                        ((uint32_t)0x00200000U)     /*!< Flash Interface Stop while System Run */
/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         ((uint32_t)0x00000001U)     /*!< Wakeup Flag                                      */
#define  PWR_CSR_SBF                         ((uint32_t)0x00000002U)     /*!< Standby Flag                                     */
#define  PWR_CSR_PVDO                        ((uint32_t)0x00000004U)     /*!< PVD Output                                       */
#define  PWR_CSR_BRR                         ((uint32_t)0x00000008U)     /*!< Backup regulator ready                           */
#define  PWR_CSR_EWUP                        ((uint32_t)0x00000100U)     /*!< Enable WKUP pin                                  */
#define  PWR_CSR_BRE                         ((uint32_t)0x00000200U)     /*!< Backup regulator enable                          */
#define  PWR_CSR_VOSRDY                      ((uint32_t)0x00004000U)     /*!< Regulator voltage scaling output selection ready */

/* Legacy define */
#define  PWR_CSR_REGRDY                      PWR_CSR_VOSRDY

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001U)
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002U)

#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8U)
#define  RCC_CR_HSITRIM_0                    ((uint32_t)0x00000008U)/*!<Bit 0 */
#define  RCC_CR_HSITRIM_1                    ((uint32_t)0x00000010U)/*!<Bit 1 */
#define  RCC_CR_HSITRIM_2                    ((uint32_t)0x00000020U)/*!<Bit 2 */
#define  RCC_CR_HSITRIM_3                    ((uint32_t)0x00000040U)/*!<Bit 3 */
#define  RCC_CR_HSITRIM_4                    ((uint32_t)0x00000080U)/*!<Bit 4 */

#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00U)
#define  RCC_CR_HSICAL_0                     ((uint32_t)0x00000100U)/*!<Bit 0 */
#define  RCC_CR_HSICAL_1                     ((uint32_t)0x00000200U)/*!<Bit 1 */
#define  RCC_CR_HSICAL_2                     ((uint32_t)0x00000400U)/*!<Bit 2 */
#define  RCC_CR_HSICAL_3                     ((uint32_t)0x00000800U)/*!<Bit 3 */
#define  RCC_CR_HSICAL_4                     ((uint32_t)0x00001000U)/*!<Bit 4 */
#define  RCC_CR_HSICAL_5                     ((uint32_t)0x00002000U)/*!<Bit 5 */
#define  RCC_CR_HSICAL_6                     ((uint32_t)0x00004000U)/*!<Bit 6 */
#define  RCC_CR_HSICAL_7                     ((uint32_t)0x00008000U)/*!<Bit 7 */

#define  RCC_CR_HSEON                        ((uint32_t)0x00010000U)
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000U)
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000U)
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000U)
#define  RCC_CR_PLLON                        ((uint32_t)0x01000000U)
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000U)

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define  RCC_PLLCFGR_PLLM                    ((uint32_t)0x0000003FU)
#define  RCC_PLLCFGR_PLLM_0                  ((uint32_t)0x00000001U)
#define  RCC_PLLCFGR_PLLM_1                  ((uint32_t)0x00000002U)
#define  RCC_PLLCFGR_PLLM_2                  ((uint32_t)0x00000004U)
#define  RCC_PLLCFGR_PLLM_3                  ((uint32_t)0x00000008U)
#define  RCC_PLLCFGR_PLLM_4                  ((uint32_t)0x00000010U)
#define  RCC_PLLCFGR_PLLM_5                  ((uint32_t)0x00000020U)

#define  RCC_PLLCFGR_PLLN                     ((uint32_t)0x00007FC0U)
#define  RCC_PLLCFGR_PLLN_0                   ((uint32_t)0x00000040U)
#define  RCC_PLLCFGR_PLLN_1                   ((uint32_t)0x00000080U)
#define  RCC_PLLCFGR_PLLN_2                   ((uint32_t)0x00000100U)
#define  RCC_PLLCFGR_PLLN_3                   ((uint32_t)0x00000200U)
#define  RCC_PLLCFGR_PLLN_4                   ((uint32_t)0x00000400U)
#define  RCC_PLLCFGR_PLLN_5                   ((uint32_t)0x00000800U)
#define  RCC_PLLCFGR_PLLN_6                   ((uint32_t)0x00001000U)
#define  RCC_PLLCFGR_PLLN_7                   ((uint32_t)0x00002000U)
#define  RCC_PLLCFGR_PLLN_8                   ((uint32_t)0x00004000U)

#define  RCC_PLLCFGR_PLLP                    ((uint32_t)0x00030000U)
#define  RCC_PLLCFGR_PLLP_0                  ((uint32_t)0x00010000U)
#define  RCC_PLLCFGR_PLLP_1                  ((uint32_t)0x00020000U)

#define  RCC_PLLCFGR_PLLSRC                  ((uint32_t)0x00400000U)
#define  RCC_PLLCFGR_PLLSRC_HSE              ((uint32_t)0x00400000U)
#define  RCC_PLLCFGR_PLLSRC_HSI              ((uint32_t)0x00000000U)

#define  RCC_PLLCFGR_PLLQ                    ((uint32_t)0x0F000000U)
#define  RCC_PLLCFGR_PLLQ_0                  ((uint32_t)0x01000000U)
#define  RCC_PLLCFGR_PLLQ_1                  ((uint32_t)0x02000000U)
#define  RCC_PLLCFGR_PLLQ_2                  ((uint32_t)0x04000000U)
#define  RCC_PLLCFGR_PLLQ_3                  ((uint32_t)0x08000000U)

#define  RCC_PLLCFGR_PLLR                    ((uint32_t)0x70000000U)
#define  RCC_PLLCFGR_PLLR_0                  ((uint32_t)0x10000000U)
#define  RCC_PLLCFGR_PLLR_1                  ((uint32_t)0x20000000U)
#define  RCC_PLLCFGR_PLLR_2                  ((uint32_t)0x40000000U)
/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003U)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001U)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002U)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000U)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001U)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002U)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000CU)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004U)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008U)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000U)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004U)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008U)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0U)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010U)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020U)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040U)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080U)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000U)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080U)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090U)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0U)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0U)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0U)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0U)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0U)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0U)        /*!< SYSCLK divided by 512 */

/*!< MCO1EN configuration */
#define  RCC_CFGR_MCO1EN                     ((uint32_t)0x00000100U)        /*!< MCO1EN bit */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00001C00U)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00001000U)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000U)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00001000U)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00001400U)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00001800U)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00001C00U)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x0000E000U)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00002000U)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00004000U)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00008000U)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000U)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00008000U)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x0000A000U)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x0000C000U)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x0000E000U)        /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define  RCC_CFGR_RTCPRE                     ((uint32_t)0x001F0000U)
#define  RCC_CFGR_RTCPRE_0                   ((uint32_t)0x00010000U)
#define  RCC_CFGR_RTCPRE_1                   ((uint32_t)0x00020000U)
#define  RCC_CFGR_RTCPRE_2                   ((uint32_t)0x00040000U)
#define  RCC_CFGR_RTCPRE_3                   ((uint32_t)0x00080000U)
#define  RCC_CFGR_RTCPRE_4                   ((uint32_t)0x00100000U)

/*!< MCO1 configuration */
#define  RCC_CFGR_MCO1                       ((uint32_t)0x00600000U)
#define  RCC_CFGR_MCO1_0                     ((uint32_t)0x00200000U)
#define  RCC_CFGR_MCO1_1                     ((uint32_t)0x00400000U)

#define  RCC_CFGR_MCO1PRE                    ((uint32_t)0x07000000U)
#define  RCC_CFGR_MCO1PRE_0                  ((uint32_t)0x01000000U)
#define  RCC_CFGR_MCO1PRE_1                  ((uint32_t)0x02000000U)
#define  RCC_CFGR_MCO1PRE_2                  ((uint32_t)0x04000000U)

#define  RCC_CFGR_MCO2PRE                    ((uint32_t)0x38000000U)
#define  RCC_CFGR_MCO2PRE_0                  ((uint32_t)0x08000000U)
#define  RCC_CFGR_MCO2PRE_1                  ((uint32_t)0x10000000U)
#define  RCC_CFGR_MCO2PRE_2                  ((uint32_t)0x20000000U)

#define  RCC_CFGR_MCO2                       ((uint32_t)0xC0000000U)
#define  RCC_CFGR_MCO2_0                     ((uint32_t)0x40000000U)
#define  RCC_CFGR_MCO2_1                     ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_CIR register  *******************/
#define  RCC_CIR_LSIRDYF                     ((uint32_t)0x00000001U)
#define  RCC_CIR_LSERDYF                     ((uint32_t)0x00000002U)
#define  RCC_CIR_HSIRDYF                     ((uint32_t)0x00000004U)
#define  RCC_CIR_HSERDYF                     ((uint32_t)0x00000008U)
#define  RCC_CIR_PLLRDYF                     ((uint32_t)0x00000010U)

#define  RCC_CIR_CSSF                        ((uint32_t)0x00000080U)
#define  RCC_CIR_LSIRDYIE                    ((uint32_t)0x00000100U)
#define  RCC_CIR_LSERDYIE                    ((uint32_t)0x00000200U)
#define  RCC_CIR_HSIRDYIE                    ((uint32_t)0x00000400U)
#define  RCC_CIR_HSERDYIE                    ((uint32_t)0x00000800U)
#define  RCC_CIR_PLLRDYIE                    ((uint32_t)0x00001000U)

#define  RCC_CIR_LSIRDYC                     ((uint32_t)0x00010000U)
#define  RCC_CIR_LSERDYC                     ((uint32_t)0x00020000U)
#define  RCC_CIR_HSIRDYC                     ((uint32_t)0x00040000U)
#define  RCC_CIR_HSERDYC                     ((uint32_t)0x00080000U)
#define  RCC_CIR_PLLRDYC                     ((uint32_t)0x00100000U)

#define  RCC_CIR_CSSC                        ((uint32_t)0x00800000U)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_GPIOARST               ((uint32_t)0x00000001U)
#define  RCC_AHB1RSTR_GPIOBRST               ((uint32_t)0x00000002U)
#define  RCC_AHB1RSTR_GPIOCRST               ((uint32_t)0x00000004U)
#define  RCC_AHB1RSTR_GPIOHRST               ((uint32_t)0x00000080U)
#define  RCC_AHB1RSTR_CRCRST                 ((uint32_t)0x00001000U)
#define  RCC_AHB1RSTR_DMA1RST                ((uint32_t)0x00200000U)
#define  RCC_AHB1RSTR_DMA2RST                ((uint32_t)0x00400000U)
#define  RCC_AHB1RSTR_RNGRST                 ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define  RCC_APB1RSTR_TIM5RST                ((uint32_t)0x00000008U)
#define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010U)
#define  RCC_APB1RSTR_LPTIM1RST              ((uint32_t)0x00000200U)
#define  RCC_APB1RSTR_WWDGRST                ((uint32_t)0x00000800U)
#define  RCC_APB1RSTR_USART2RST              ((uint32_t)0x00020000U)
#define  RCC_APB1RSTR_I2C1RST                ((uint32_t)0x00200000U)
#define  RCC_APB1RSTR_I2C2RST                ((uint32_t)0x00400000U)
#define  RCC_APB1RSTR_FMPI2C1RST             ((uint32_t)0x01000000U)
#define  RCC_APB1RSTR_PWRRST                 ((uint32_t)0x10000000U)
#define  RCC_APB1RSTR_DACRST                 ((uint32_t)0x20000000U)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000001U)
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00000010U)
#define  RCC_APB2RSTR_ADCRST                 ((uint32_t)0x00000100U)
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000U)
#define  RCC_APB2RSTR_SYSCFGRST              ((uint32_t)0x00004000U)
#define  RCC_APB2RSTR_TIM9RST                ((uint32_t)0x00010000U)
#define  RCC_APB2RSTR_TIM11RST               ((uint32_t)0x00040000U)

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_GPIOAEN                 ((uint32_t)0x00000001U)
#define  RCC_AHB1ENR_GPIOBEN                 ((uint32_t)0x00000002U)
#define  RCC_AHB1ENR_GPIOCEN                 ((uint32_t)0x00000004U)
#define  RCC_AHB1ENR_GPIOHEN                 ((uint32_t)0x00000080U)
#define  RCC_AHB1ENR_CRCEN                   ((uint32_t)0x00001000U)
#define  RCC_AHB1ENR_DMA1EN                  ((uint32_t)0x00200000U)
#define  RCC_AHB1ENR_DMA2EN                  ((uint32_t)0x00400000U)
#define  RCC_AHB1ENR_RNGEN                   ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define  RCC_APB1ENR_TIM5EN                  ((uint32_t)0x00000008U)
#define  RCC_APB1ENR_TIM6EN                  ((uint32_t)0x00000010U)
#define  RCC_APB1ENR_LPTIM1EN                ((uint32_t)0x00000200U)
#define  RCC_APB1ENR_RTCAPBEN                ((uint32_t)0x00000400U)
#define  RCC_APB1ENR_WWDGEN                  ((uint32_t)0x00000800U)
#define  RCC_APB1ENR_USART2EN                ((uint32_t)0x00020000U)
#define  RCC_APB1ENR_I2C1EN                  ((uint32_t)0x00200000U)
#define  RCC_APB1ENR_I2C2EN                  ((uint32_t)0x00400000U)
#define  RCC_APB1ENR_FMPI2C1EN               ((uint32_t)0x01000000U)
#define  RCC_APB1ENR_PWREN                   ((uint32_t)0x10000000U)
#define  RCC_APB1ENR_DACEN                   ((uint32_t)0x20000000U)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000001U)
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00000010U)
#define  RCC_APB2ENR_ADC1EN                  ((uint32_t)0x00000100U)
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000U)
#define  RCC_APB2ENR_SYSCFGEN                ((uint32_t)0x00004000U)
#define  RCC_APB2ENR_EXTITEN                 ((uint32_t)0x00008000U)
#define  RCC_APB2ENR_TIM9EN                  ((uint32_t)0x00010000U)
#define  RCC_APB2ENR_TIM11EN                 ((uint32_t)0x00040000U)

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define  RCC_AHB1LPENR_GPIOALPEN             ((uint32_t)0x00000001U)
#define  RCC_AHB1LPENR_GPIOBLPEN             ((uint32_t)0x00000002U)
#define  RCC_AHB1LPENR_GPIOCLPEN             ((uint32_t)0x00000004U)
#define  RCC_AHB1LPENR_GPIOHLPEN             ((uint32_t)0x00000080U)
#define  RCC_AHB1LPENR_CRCLPEN               ((uint32_t)0x00001000U)
#define  RCC_AHB1LPENR_FLITFLPEN             ((uint32_t)0x00008000U)
#define  RCC_AHB1LPENR_SRAM1LPEN             ((uint32_t)0x00010000U)
#define  RCC_AHB1LPENR_DMA1LPEN              ((uint32_t)0x00200000U)
#define  RCC_AHB1LPENR_DMA2LPEN              ((uint32_t)0x00400000U)
#define  RCC_AHB1LPENR_RNGLPEN               ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define  RCC_APB1LPENR_TIM5LPEN              ((uint32_t)0x00000008U)
#define  RCC_APB1LPENR_TIM6LPEN              ((uint32_t)0x00000010U)
#define  RCC_APB1LPENR_LPTIM1LPEN            ((uint32_t)0x00000200U)
#define  RCC_APB1LPENR_RTCAPBLPEN            ((uint32_t)0x00000400U)
#define  RCC_APB1LPENR_WWDGLPEN              ((uint32_t)0x00000800U)
#define  RCC_APB1LPENR_USART2LPEN            ((uint32_t)0x00020000U)
#define  RCC_APB1LPENR_I2C1LPEN              ((uint32_t)0x00200000U)
#define  RCC_APB1LPENR_I2C2LPEN              ((uint32_t)0x00400000U)
#define  RCC_APB1LPENR_FMPI2C1LPEN           ((uint32_t)0x01000000U)
#define  RCC_APB1LPENR_PWRLPEN               ((uint32_t)0x10000000U)
#define  RCC_APB1LPENR_DACLPEN               ((uint32_t)0x20000000U)

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define  RCC_APB2LPENR_TIM1LPEN              ((uint32_t)0x00000001U)
#define  RCC_APB2LPENR_USART1LPEN            ((uint32_t)0x00000010U)
#define  RCC_APB2LPENR_ADC1LPEN              ((uint32_t)0x00000100U)
#define  RCC_APB2LPENR_SPI1LPEN              ((uint32_t)0x00001000U)
#define  RCC_APB2LPENR_SYSCFGLPEN            ((uint32_t)0x00004000U)
#define  RCC_APB2LPENR_EXTITLPEN             ((uint32_t)0x00008000U)
#define  RCC_APB2LPENR_TIM9LPEN              ((uint32_t)0x00010000U)
#define  RCC_APB2LPENR_TIM11LPEN             ((uint32_t)0x00040000U)

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001U)
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002U)
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004U)
#define  RCC_BDCR_LSEMOD                     ((uint32_t)0x00000008U)

#define  RCC_BDCR_RTCSEL                    ((uint32_t)0x00000300U)
#define  RCC_BDCR_RTCSEL_0                  ((uint32_t)0x00000100U)
#define  RCC_BDCR_RTCSEL_1                  ((uint32_t)0x00000200U)

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000U)
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000U)

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001U)
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002U)
#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000U)
#define  RCC_CSR_BORRSTF                     ((uint32_t)0x02000000U)
#define  RCC_CSR_PADRSTF                     ((uint32_t)0x04000000U)
#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000U)
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000U)
#define  RCC_CSR_WDGRSTF                     ((uint32_t)0x20000000U)
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000U)
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_SSCGR register  *****************/
#define  RCC_SSCGR_MODPER                    ((uint32_t)0x00001FFFU)
#define  RCC_SSCGR_INCSTEP                   ((uint32_t)0x0FFFE000U)
#define  RCC_SSCGR_SPREADSEL                 ((uint32_t)0x40000000U)
#define  RCC_SSCGR_SSCGEN                    ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_DCKCFGR register  ***************/
#define  RCC_DCKCFGR_TIMPRE                  ((uint32_t)0x01000000U)
#define  RCC_DCKCFGR_I2SSRC                  ((uint32_t)0x06000000U)
#define  RCC_DCKCFGR_I2SSRC_0                ((uint32_t)0x02000000U)
#define  RCC_DCKCFGR_I2SSRC_1                ((uint32_t)0x04000000U)

/********************  Bit definition for RCC_CKGATENR register  **************/
#define  RCC_CKGATENR_AHB2APB1_CKEN          ((uint32_t)0x00000001U)
#define  RCC_CKGATENR_AHB2APB2_CKEN          ((uint32_t)0x00000002U)
#define  RCC_CKGATENR_CM4DBG_CKEN            ((uint32_t)0x00000004U)
#define  RCC_CKGATENR_SPARE_CKEN             ((uint32_t)0x00000008U)
#define  RCC_CKGATENR_SRAM_CKEN              ((uint32_t)0x00000010U)
#define  RCC_CKGATENR_FLITF_CKEN             ((uint32_t)0x00000020U)
#define  RCC_CKGATENR_RCC_CKEN               ((uint32_t)0x00000040U)

/********************  Bit definition for RCC_DCKCFGR2 register  **************/
#define  RCC_DCKCFGR2_FMPI2C1SEL             ((uint32_t)0x00C00000U)
#define  RCC_DCKCFGR2_FMPI2C1SEL_0           ((uint32_t)0x00400000U)
#define  RCC_DCKCFGR2_FMPI2C1SEL_1           ((uint32_t)0x00800000U)
#define  RCC_DCKCFGR2_LPTIM1SEL              ((uint32_t)0xC0000000U)
#define  RCC_DCKCFGR2_LPTIM1SEL_0            ((uint32_t)0x40000000U)
#define  RCC_DCKCFGR2_LPTIM1SEL_1            ((uint32_t)0x80000000U)

/******************************************************************************/
/*                                                                            */
/*                                    RNG                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RNG_CR register  *******************/
#define RNG_CR_RNGEN                         ((uint32_t)0x00000004U)
#define RNG_CR_IE                            ((uint32_t)0x00000008U)

/********************  Bits definition for RNG_SR register  *******************/
#define RNG_SR_DRDY                          ((uint32_t)0x00000001U)
#define RNG_SR_CECS                          ((uint32_t)0x00000002U)
#define RNG_SR_SECS                          ((uint32_t)0x00000004U)
#define RNG_SR_CEIS                          ((uint32_t)0x00000020U)
#define RNG_SR_SEIS                          ((uint32_t)0x00000040U)

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_PM                            ((uint32_t)0x00400000U)
#define RTC_TR_HT                            ((uint32_t)0x00300000U)
#define RTC_TR_HT_0                          ((uint32_t)0x00100000U)
#define RTC_TR_HT_1                          ((uint32_t)0x00200000U)
#define RTC_TR_HU                            ((uint32_t)0x000F0000U)
#define RTC_TR_HU_0                          ((uint32_t)0x00010000U)
#define RTC_TR_HU_1                          ((uint32_t)0x00020000U)
#define RTC_TR_HU_2                          ((uint32_t)0x00040000U)
#define RTC_TR_HU_3                          ((uint32_t)0x00080000U)
#define RTC_TR_MNT                           ((uint32_t)0x00007000U)
#define RTC_TR_MNT_0                         ((uint32_t)0x00001000U)
#define RTC_TR_MNT_1                         ((uint32_t)0x00002000U)
#define RTC_TR_MNT_2                         ((uint32_t)0x00004000U)
#define RTC_TR_MNU                           ((uint32_t)0x00000F00U)
#define RTC_TR_MNU_0                         ((uint32_t)0x00000100U)
#define RTC_TR_MNU_1                         ((uint32_t)0x00000200U)
#define RTC_TR_MNU_2                         ((uint32_t)0x00000400U)
#define RTC_TR_MNU_3                         ((uint32_t)0x00000800U)
#define RTC_TR_ST                            ((uint32_t)0x00000070U)
#define RTC_TR_ST_0                          ((uint32_t)0x00000010U)
#define RTC_TR_ST_1                          ((uint32_t)0x00000020U)
#define RTC_TR_ST_2                          ((uint32_t)0x00000040U)
#define RTC_TR_SU                            ((uint32_t)0x0000000FU)
#define RTC_TR_SU_0                          ((uint32_t)0x00000001U)
#define RTC_TR_SU_1                          ((uint32_t)0x00000002U)
#define RTC_TR_SU_2                          ((uint32_t)0x00000004U)
#define RTC_TR_SU_3                          ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_YT                            ((uint32_t)0x00F00000U)
#define RTC_DR_YT_0                          ((uint32_t)0x00100000U)
#define RTC_DR_YT_1                          ((uint32_t)0x00200000U)
#define RTC_DR_YT_2                          ((uint32_t)0x00400000U)
#define RTC_DR_YT_3                          ((uint32_t)0x00800000U)
#define RTC_DR_YU                            ((uint32_t)0x000F0000U)
#define RTC_DR_YU_0                          ((uint32_t)0x00010000U)
#define RTC_DR_YU_1                          ((uint32_t)0x00020000U)
#define RTC_DR_YU_2                          ((uint32_t)0x00040000U)
#define RTC_DR_YU_3                          ((uint32_t)0x00080000U)
#define RTC_DR_WDU                           ((uint32_t)0x0000E000U)
#define RTC_DR_WDU_0                         ((uint32_t)0x00002000U)
#define RTC_DR_WDU_1                         ((uint32_t)0x00004000U)
#define RTC_DR_WDU_2                         ((uint32_t)0x00008000U)
#define RTC_DR_MT                            ((uint32_t)0x00001000U)
#define RTC_DR_MU                            ((uint32_t)0x00000F00U)
#define RTC_DR_MU_0                          ((uint32_t)0x00000100U)
#define RTC_DR_MU_1                          ((uint32_t)0x00000200U)
#define RTC_DR_MU_2                          ((uint32_t)0x00000400U)
#define RTC_DR_MU_3                          ((uint32_t)0x00000800U)
#define RTC_DR_DT                            ((uint32_t)0x00000030U)
#define RTC_DR_DT_0                          ((uint32_t)0x00000010U)
#define RTC_DR_DT_1                          ((uint32_t)0x00000020U)
#define RTC_DR_DU                            ((uint32_t)0x0000000FU)
#define RTC_DR_DU_0                          ((uint32_t)0x00000001U)
#define RTC_DR_DU_1                          ((uint32_t)0x00000002U)
#define RTC_DR_DU_2                          ((uint32_t)0x00000004U)
#define RTC_DR_DU_3                          ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_COE                           ((uint32_t)0x00800000U)
#define RTC_CR_OSEL                          ((uint32_t)0x00600000U)
#define RTC_CR_OSEL_0                        ((uint32_t)0x00200000U)
#define RTC_CR_OSEL_1                        ((uint32_t)0x00400000U)
#define RTC_CR_POL                           ((uint32_t)0x00100000U)
#define RTC_CR_COSEL                         ((uint32_t)0x00080000U)
#define RTC_CR_BCK                           ((uint32_t)0x00040000U)
#define RTC_CR_SUB1H                         ((uint32_t)0x00020000U)
#define RTC_CR_ADD1H                         ((uint32_t)0x00010000U)
#define RTC_CR_TSIE                          ((uint32_t)0x00008000U)
#define RTC_CR_WUTIE                         ((uint32_t)0x00004000U)
#define RTC_CR_ALRBIE                        ((uint32_t)0x00002000U)
#define RTC_CR_ALRAIE                        ((uint32_t)0x00001000U)
#define RTC_CR_TSE                           ((uint32_t)0x00000800U)
#define RTC_CR_WUTE                          ((uint32_t)0x00000400U)
#define RTC_CR_ALRBE                         ((uint32_t)0x00000200U)
#define RTC_CR_ALRAE                         ((uint32_t)0x00000100U)
#define RTC_CR_DCE                           ((uint32_t)0x00000080U)
#define RTC_CR_FMT                           ((uint32_t)0x00000040U)
#define RTC_CR_BYPSHAD                       ((uint32_t)0x00000020U)
#define RTC_CR_REFCKON                       ((uint32_t)0x00000010U)
#define RTC_CR_TSEDGE                        ((uint32_t)0x00000008U)
#define RTC_CR_WUCKSEL                       ((uint32_t)0x00000007U)
#define RTC_CR_WUCKSEL_0                     ((uint32_t)0x00000001U)
#define RTC_CR_WUCKSEL_1                     ((uint32_t)0x00000002U)
#define RTC_CR_WUCKSEL_2                     ((uint32_t)0x00000004U)

/********************  Bits definition for RTC_ISR register  ******************/
#define RTC_ISR_RECALPF                      ((uint32_t)0x00010000U)
#define RTC_ISR_TAMP1F                       ((uint32_t)0x00002000U)
#define RTC_ISR_TAMP2F                       ((uint32_t)0x00004000U)
#define RTC_ISR_TSOVF                        ((uint32_t)0x00001000U)
#define RTC_ISR_TSF                          ((uint32_t)0x00000800U)
#define RTC_ISR_WUTF                         ((uint32_t)0x00000400U)
#define RTC_ISR_ALRBF                        ((uint32_t)0x00000200U)
#define RTC_ISR_ALRAF                        ((uint32_t)0x00000100U)
#define RTC_ISR_INIT                         ((uint32_t)0x00000080U)
#define RTC_ISR_INITF                        ((uint32_t)0x00000040U)
#define RTC_ISR_RSF                          ((uint32_t)0x00000020U)
#define RTC_ISR_INITS                        ((uint32_t)0x00000010U)
#define RTC_ISR_SHPF                         ((uint32_t)0x00000008U)
#define RTC_ISR_WUTWF                        ((uint32_t)0x00000004U)
#define RTC_ISR_ALRBWF                       ((uint32_t)0x00000002U)
#define RTC_ISR_ALRAWF                       ((uint32_t)0x00000001U)

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_A                    ((uint32_t)0x007F0000U)
#define RTC_PRER_PREDIV_S                    ((uint32_t)0x00007FFFU)

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT                         ((uint32_t)0x0000FFFFU)

/********************  Bits definition for RTC_CALIBR register  ***************/
#define RTC_CALIBR_DCS                       ((uint32_t)0x00000080U)
#define RTC_CALIBR_DC                        ((uint32_t)0x0000001FU)

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_MSK4                      ((uint32_t)0x80000000U)
#define RTC_ALRMAR_WDSEL                     ((uint32_t)0x40000000U)
#define RTC_ALRMAR_DT                        ((uint32_t)0x30000000U)
#define RTC_ALRMAR_DT_0                      ((uint32_t)0x10000000U)
#define RTC_ALRMAR_DT_1                      ((uint32_t)0x20000000U)
#define RTC_ALRMAR_DU                        ((uint32_t)0x0F000000U)
#define RTC_ALRMAR_DU_0                      ((uint32_t)0x01000000U)
#define RTC_ALRMAR_DU_1                      ((uint32_t)0x02000000U)
#define RTC_ALRMAR_DU_2                      ((uint32_t)0x04000000U)
#define RTC_ALRMAR_DU_3                      ((uint32_t)0x08000000U)
#define RTC_ALRMAR_MSK3                      ((uint32_t)0x00800000U)
#define RTC_ALRMAR_PM                        ((uint32_t)0x00400000U)
#define RTC_ALRMAR_HT                        ((uint32_t)0x00300000U)
#define RTC_ALRMAR_HT_0                      ((uint32_t)0x00100000U)
#define RTC_ALRMAR_HT_1                      ((uint32_t)0x00200000U)
#define RTC_ALRMAR_HU                        ((uint32_t)0x000F0000U)
#define RTC_ALRMAR_HU_0                      ((uint32_t)0x00010000U)
#define RTC_ALRMAR_HU_1                      ((uint32_t)0x00020000U)
#define RTC_ALRMAR_HU_2                      ((uint32_t)0x00040000U)
#define RTC_ALRMAR_HU_3                      ((uint32_t)0x00080000U)
#define RTC_ALRMAR_MSK2                      ((uint32_t)0x00008000U)
#define RTC_ALRMAR_MNT                       ((uint32_t)0x00007000U)
#define RTC_ALRMAR_MNT_0                     ((uint32_t)0x00001000U)
#define RTC_ALRMAR_MNT_1                     ((uint32_t)0x00002000U)
#define RTC_ALRMAR_MNT_2                     ((uint32_t)0x00004000U)
#define RTC_ALRMAR_MNU                       ((uint32_t)0x00000F00U)
#define RTC_ALRMAR_MNU_0                     ((uint32_t)0x00000100U)
#define RTC_ALRMAR_MNU_1                     ((uint32_t)0x00000200U)
#define RTC_ALRMAR_MNU_2                     ((uint32_t)0x00000400U)
#define RTC_ALRMAR_MNU_3                     ((uint32_t)0x00000800U)
#define RTC_ALRMAR_MSK1                      ((uint32_t)0x00000080U)
#define RTC_ALRMAR_ST                        ((uint32_t)0x00000070U)
#define RTC_ALRMAR_ST_0                      ((uint32_t)0x00000010U)
#define RTC_ALRMAR_ST_1                      ((uint32_t)0x00000020U)
#define RTC_ALRMAR_ST_2                      ((uint32_t)0x00000040U)
#define RTC_ALRMAR_SU                        ((uint32_t)0x0000000FU)
#define RTC_ALRMAR_SU_0                      ((uint32_t)0x00000001U)
#define RTC_ALRMAR_SU_1                      ((uint32_t)0x00000002U)
#define RTC_ALRMAR_SU_2                      ((uint32_t)0x00000004U)
#define RTC_ALRMAR_SU_3                      ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_MSK4                      ((uint32_t)0x80000000U)
#define RTC_ALRMBR_WDSEL                     ((uint32_t)0x40000000U)
#define RTC_ALRMBR_DT                        ((uint32_t)0x30000000U)
#define RTC_ALRMBR_DT_0                      ((uint32_t)0x10000000U)
#define RTC_ALRMBR_DT_1                      ((uint32_t)0x20000000U)
#define RTC_ALRMBR_DU                        ((uint32_t)0x0F000000U)
#define RTC_ALRMBR_DU_0                      ((uint32_t)0x01000000U)
#define RTC_ALRMBR_DU_1                      ((uint32_t)0x02000000U)
#define RTC_ALRMBR_DU_2                      ((uint32_t)0x04000000U)
#define RTC_ALRMBR_DU_3                      ((uint32_t)0x08000000U)
#define RTC_ALRMBR_MSK3                      ((uint32_t)0x00800000U)
#define RTC_ALRMBR_PM                        ((uint32_t)0x00400000U)
#define RTC_ALRMBR_HT                        ((uint32_t)0x00300000U)
#define RTC_ALRMBR_HT_0                      ((uint32_t)0x00100000U)
#define RTC_ALRMBR_HT_1                      ((uint32_t)0x00200000U)
#define RTC_ALRMBR_HU                        ((uint32_t)0x000F0000U)
#define RTC_ALRMBR_HU_0                      ((uint32_t)0x00010000U)
#define RTC_ALRMBR_HU_1                      ((uint32_t)0x00020000U)
#define RTC_ALRMBR_HU_2                      ((uint32_t)0x00040000U)
#define RTC_ALRMBR_HU_3                      ((uint32_t)0x00080000U)
#define RTC_ALRMBR_MSK2                      ((uint32_t)0x00008000U)
#define RTC_ALRMBR_MNT                       ((uint32_t)0x00007000U)
#define RTC_ALRMBR_MNT_0                     ((uint32_t)0x00001000U)
#define RTC_ALRMBR_MNT_1                     ((uint32_t)0x00002000U)
#define RTC_ALRMBR_MNT_2                     ((uint32_t)0x00004000U)
#define RTC_ALRMBR_MNU                       ((uint32_t)0x00000F00U)
#define RTC_ALRMBR_MNU_0                     ((uint32_t)0x00000100U)
#define RTC_ALRMBR_MNU_1                     ((uint32_t)0x00000200U)
#define RTC_ALRMBR_MNU_2                     ((uint32_t)0x00000400U)
#define RTC_ALRMBR_MNU_3                     ((uint32_t)0x00000800U)
#define RTC_ALRMBR_MSK1                      ((uint32_t)0x00000080U)
#define RTC_ALRMBR_ST                        ((uint32_t)0x00000070U)
#define RTC_ALRMBR_ST_0                      ((uint32_t)0x00000010U)
#define RTC_ALRMBR_ST_1                      ((uint32_t)0x00000020U)
#define RTC_ALRMBR_ST_2                      ((uint32_t)0x00000040U)
#define RTC_ALRMBR_SU                        ((uint32_t)0x0000000FU)
#define RTC_ALRMBR_SU_0                      ((uint32_t)0x00000001U)
#define RTC_ALRMBR_SU_1                      ((uint32_t)0x00000002U)
#define RTC_ALRMBR_SU_2                      ((uint32_t)0x00000004U)
#define RTC_ALRMBR_SU_3                      ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY                          ((uint32_t)0x000000FFU)

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS                           ((uint32_t)0x0000FFFFU)

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS                     ((uint32_t)0x00007FFFU)
#define RTC_SHIFTR_ADD1S                     ((uint32_t)0x80000000U)

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_PM                          ((uint32_t)0x00400000U)
#define RTC_TSTR_HT                          ((uint32_t)0x00300000U)
#define RTC_TSTR_HT_0                        ((uint32_t)0x00100000U)
#define RTC_TSTR_HT_1                        ((uint32_t)0x00200000U)
#define RTC_TSTR_HU                          ((uint32_t)0x000F0000U)
#define RTC_TSTR_HU_0                        ((uint32_t)0x00010000U)
#define RTC_TSTR_HU_1                        ((uint32_t)0x00020000U)
#define RTC_TSTR_HU_2                        ((uint32_t)0x00040000U)
#define RTC_TSTR_HU_3                        ((uint32_t)0x00080000U)
#define RTC_TSTR_MNT                         ((uint32_t)0x00007000U)
#define RTC_TSTR_MNT_0                       ((uint32_t)0x00001000U)
#define RTC_TSTR_MNT_1                       ((uint32_t)0x00002000U)
#define RTC_TSTR_MNT_2                       ((uint32_t)0x00004000U)
#define RTC_TSTR_MNU                         ((uint32_t)0x00000F00U)
#define RTC_TSTR_MNU_0                       ((uint32_t)0x00000100U)
#define RTC_TSTR_MNU_1                       ((uint32_t)0x00000200U)
#define RTC_TSTR_MNU_2                       ((uint32_t)0x00000400U)
#define RTC_TSTR_MNU_3                       ((uint32_t)0x00000800U)
#define RTC_TSTR_ST                          ((uint32_t)0x00000070U)
#define RTC_TSTR_ST_0                        ((uint32_t)0x00000010U)
#define RTC_TSTR_ST_1                        ((uint32_t)0x00000020U)
#define RTC_TSTR_ST_2                        ((uint32_t)0x00000040U)
#define RTC_TSTR_SU                          ((uint32_t)0x0000000FU)
#define RTC_TSTR_SU_0                        ((uint32_t)0x00000001U)
#define RTC_TSTR_SU_1                        ((uint32_t)0x00000002U)
#define RTC_TSTR_SU_2                        ((uint32_t)0x00000004U)
#define RTC_TSTR_SU_3                        ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_WDU                         ((uint32_t)0x0000E000U)
#define RTC_TSDR_WDU_0                       ((uint32_t)0x00002000U)
#define RTC_TSDR_WDU_1                       ((uint32_t)0x00004000U)
#define RTC_TSDR_WDU_2                       ((uint32_t)0x00008000U)
#define RTC_TSDR_MT                          ((uint32_t)0x00001000U)
#define RTC_TSDR_MU                          ((uint32_t)0x00000F00U)
#define RTC_TSDR_MU_0                        ((uint32_t)0x00000100U)
#define RTC_TSDR_MU_1                        ((uint32_t)0x00000200U)
#define RTC_TSDR_MU_2                        ((uint32_t)0x00000400U)
#define RTC_TSDR_MU_3                        ((uint32_t)0x00000800U)
#define RTC_TSDR_DT                          ((uint32_t)0x00000030U)
#define RTC_TSDR_DT_0                        ((uint32_t)0x00000010U)
#define RTC_TSDR_DT_1                        ((uint32_t)0x00000020U)
#define RTC_TSDR_DU                          ((uint32_t)0x0000000FU)
#define RTC_TSDR_DU_0                        ((uint32_t)0x00000001U)
#define RTC_TSDR_DU_1                        ((uint32_t)0x00000002U)
#define RTC_TSDR_DU_2                        ((uint32_t)0x00000004U)
#define RTC_TSDR_DU_3                        ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS                         ((uint32_t)0x0000FFFFU)

/********************  Bits definition for RTC_CAL register  *****************/
#define RTC_CALR_CALP                        ((uint32_t)0x00008000U)
#define RTC_CALR_CALW8                       ((uint32_t)0x00004000U)
#define RTC_CALR_CALW16                      ((uint32_t)0x00002000U)
#define RTC_CALR_CALM                        ((uint32_t)0x000001FFU)
#define RTC_CALR_CALM_0                      ((uint32_t)0x00000001U)
#define RTC_CALR_CALM_1                      ((uint32_t)0x00000002U)
#define RTC_CALR_CALM_2                      ((uint32_t)0x00000004U)
#define RTC_CALR_CALM_3                      ((uint32_t)0x00000008U)
#define RTC_CALR_CALM_4                      ((uint32_t)0x00000010U)
#define RTC_CALR_CALM_5                      ((uint32_t)0x00000020U)
#define RTC_CALR_CALM_6                      ((uint32_t)0x00000040U)
#define RTC_CALR_CALM_7                      ((uint32_t)0x00000080U)
#define RTC_CALR_CALM_8                      ((uint32_t)0x00000100U)

/********************  Bits definition for RTC_TAFCR register  ****************/
#define RTC_TAFCR_ALARMOUTTYPE               ((uint32_t)0x00040000U)
#define RTC_TAFCR_TSINSEL                    ((uint32_t)0x00020000U)
#define RTC_TAFCR_TAMPINSEL                  ((uint32_t)0x00010000U)
#define RTC_TAFCR_TAMPPUDIS                  ((uint32_t)0x00008000U)
#define RTC_TAFCR_TAMPPRCH                   ((uint32_t)0x00006000U)
#define RTC_TAFCR_TAMPPRCH_0                 ((uint32_t)0x00002000U)
#define RTC_TAFCR_TAMPPRCH_1                 ((uint32_t)0x00004000U)
#define RTC_TAFCR_TAMPFLT                    ((uint32_t)0x00001800U)
#define RTC_TAFCR_TAMPFLT_0                  ((uint32_t)0x00000800U)
#define RTC_TAFCR_TAMPFLT_1                  ((uint32_t)0x00001000U)
#define RTC_TAFCR_TAMPFREQ                   ((uint32_t)0x00000700U)
#define RTC_TAFCR_TAMPFREQ_0                 ((uint32_t)0x00000100U)
#define RTC_TAFCR_TAMPFREQ_1                 ((uint32_t)0x00000200U)
#define RTC_TAFCR_TAMPFREQ_2                 ((uint32_t)0x00000400U)
#define RTC_TAFCR_TAMPTS                     ((uint32_t)0x00000080U)
#define RTC_TAFCR_TAMP2TRG                   ((uint32_t)0x00000010U)
#define RTC_TAFCR_TAMP2E                     ((uint32_t)0x00000008U)
#define RTC_TAFCR_TAMPIE                     ((uint32_t)0x00000004U)
#define RTC_TAFCR_TAMP1TRG                   ((uint32_t)0x00000002U)
#define RTC_TAFCR_TAMP1E                     ((uint32_t)0x00000001U)

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_MASKSS                  ((uint32_t)0x0F000000U)
#define RTC_ALRMASSR_MASKSS_0                ((uint32_t)0x01000000U)
#define RTC_ALRMASSR_MASKSS_1                ((uint32_t)0x02000000U)
#define RTC_ALRMASSR_MASKSS_2                ((uint32_t)0x04000000U)
#define RTC_ALRMASSR_MASKSS_3                ((uint32_t)0x08000000U)
#define RTC_ALRMASSR_SS                      ((uint32_t)0x00007FFFU)

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_MASKSS                  ((uint32_t)0x0F000000U)
#define RTC_ALRMBSSR_MASKSS_0                ((uint32_t)0x01000000U)
#define RTC_ALRMBSSR_MASKSS_1                ((uint32_t)0x02000000U)
#define RTC_ALRMBSSR_MASKSS_2                ((uint32_t)0x04000000U)
#define RTC_ALRMBSSR_MASKSS_3                ((uint32_t)0x08000000U)
#define RTC_ALRMBSSR_SS                      ((uint32_t)0x00007FFFU)

/********************  Bits definition for RTC_BKP0R register  ****************/
#define RTC_BKP0R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP1R register  ****************/
#define RTC_BKP1R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP2R register  ****************/
#define RTC_BKP2R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP3R register  ****************/
#define RTC_BKP3R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP4R register  ****************/
#define RTC_BKP4R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP5R register  ****************/
#define RTC_BKP5R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP6R register  ****************/
#define RTC_BKP6R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP7R register  ****************/
#define RTC_BKP7R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP8R register  ****************/
#define RTC_BKP8R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP9R register  ****************/
#define RTC_BKP9R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP10R register  ***************/
#define RTC_BKP10R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP11R register  ***************/
#define RTC_BKP11R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP12R register  ***************/
#define RTC_BKP12R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP13R register  ***************/
#define RTC_BKP13R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP14R register  ***************/
#define RTC_BKP14R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP15R register  ***************/
#define RTC_BKP15R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP16R register  ***************/
#define RTC_BKP16R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP17R register  ***************/
#define RTC_BKP17R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP18R register  ***************/
#define RTC_BKP18R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP19R register  ***************/
#define RTC_BKP19R                           ((uint32_t)0xFFFFFFFFU)

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((uint32_t)0x00000001U)            /*!<Clock Phase      */
#define  SPI_CR1_CPOL                        ((uint32_t)0x00000002U)            /*!<Clock Polarity   */
#define  SPI_CR1_MSTR                        ((uint32_t)0x00000004U)            /*!<Master Selection */

#define  SPI_CR1_BR                          ((uint32_t)0x00000038U)            /*!<BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((uint32_t)0x00000008U)            /*!<Bit 0 */
#define  SPI_CR1_BR_1                        ((uint32_t)0x00000010U)            /*!<Bit 1 */
#define  SPI_CR1_BR_2                        ((uint32_t)0x00000020U)            /*!<Bit 2 */

#define  SPI_CR1_SPE                         ((uint32_t)0x00000040U)            /*!<SPI Enable                          */
#define  SPI_CR1_LSBFIRST                    ((uint32_t)0x00000080U)            /*!<Frame Format                        */
#define  SPI_CR1_SSI                         ((uint32_t)0x00000100U)            /*!<Internal slave select               */
#define  SPI_CR1_SSM                         ((uint32_t)0x00000200U)            /*!<Software slave management           */
#define  SPI_CR1_RXONLY                      ((uint32_t)0x00000400U)            /*!<Receive only                        */
#define  SPI_CR1_DFF                         ((uint32_t)0x00000800U)            /*!<Data Frame Format                   */
#define  SPI_CR1_CRCNEXT                     ((uint32_t)0x00001000U)            /*!<Transmit CRC next                   */
#define  SPI_CR1_CRCEN                       ((uint32_t)0x00002000U)            /*!<Hardware CRC calculation enable     */
#define  SPI_CR1_BIDIOE                      ((uint32_t)0x00004000U)            /*!<Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((uint32_t)0x00008000U)            /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((uint32_t)0x00000001U)               /*!<Rx Buffer DMA Enable                 */
#define  SPI_CR2_TXDMAEN                     ((uint32_t)0x00000002U)               /*!<Tx Buffer DMA Enable                 */
#define  SPI_CR2_SSOE                        ((uint32_t)0x00000004U)               /*!<SS Output Enable                     */
#define  SPI_CR2_FRF                         ((uint32_t)0x00000010U)               /*!<Frame Format                         */
#define  SPI_CR2_ERRIE                       ((uint32_t)0x00000020U)               /*!<Error Interrupt Enable               */
#define  SPI_CR2_RXNEIE                      ((uint32_t)0x00000040U)               /*!<RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((uint32_t)0x00000080U)               /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((uint32_t)0x00000001U)               /*!<Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((uint32_t)0x00000002U)               /*!<Transmit buffer Empty    */
#define  SPI_SR_CHSIDE                       ((uint32_t)0x00000004U)               /*!<Channel side             */
#define  SPI_SR_UDR                          ((uint32_t)0x00000008U)               /*!<Underrun flag            */
#define  SPI_SR_CRCERR                       ((uint32_t)0x00000010U)               /*!<CRC Error flag           */
#define  SPI_SR_MODF                         ((uint32_t)0x00000020U)               /*!<Mode fault               */
#define  SPI_SR_OVR                          ((uint32_t)0x00000040U)               /*!<Overrun flag             */
#define  SPI_SR_BSY                          ((uint32_t)0x00000080U)               /*!<Busy flag                */
#define  SPI_SR_FRE                          ((uint32_t)0x00000100U)               /*!<Frame format error flag  */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           ((uint32_t)0x0000FFFFU)            /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   ((uint32_t)0x0000FFFFU)            /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    ((uint32_t)0x0000FFFFU)            /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    ((uint32_t)0x0000FFFFU)            /*!<Tx CRC Register         */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   ((uint32_t)0x00000001U)            /*!<Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                  ((uint32_t)0x00000006U)            /*!<DATLEN[1:0] bits (Data length to be transferred)  */
#define  SPI_I2SCFGR_DATLEN_0                ((uint32_t)0x00000002U)            /*!<Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                ((uint32_t)0x00000004U)            /*!<Bit 1 */

#define  SPI_I2SCFGR_CKPOL                   ((uint32_t)0x00000008U)            /*!<steady state clock polarity               */

#define  SPI_I2SCFGR_I2SSTD                  ((uint32_t)0x00000030U)            /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                ((uint32_t)0x00000010U)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                ((uint32_t)0x00000020U)            /*!<Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                 ((uint32_t)0x00000080U)            /*!<PCM frame synchronization                 */

#define  SPI_I2SCFGR_I2SCFG                  ((uint32_t)0x00000300U)            /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                ((uint32_t)0x00000100U)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                ((uint32_t)0x00000200U)            /*!<Bit 1 */

#define  SPI_I2SCFGR_I2SE                    ((uint32_t)0x00000400U)            /*!<I2S Enable         */
#define  SPI_I2SCFGR_I2SMOD                  ((uint32_t)0x00000800U)            /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    ((uint32_t)0x000000FFU)            /*!<I2S Linear prescaler         */
#define  SPI_I2SPR_ODD                       ((uint32_t)0x00000100U)            /*!<Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     ((uint32_t)0x00000200U)            /*!<Master Clock Output Enable   */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/  
#define SYSCFG_MEMRMP_MEM_MODE          ((uint32_t)0x00000007U) /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0        ((uint32_t)0x00000001U)
#define SYSCFG_MEMRMP_MEM_MODE_1        ((uint32_t)0x00000002U)
#define SYSCFG_MEMRMP_MEM_MODE_2        ((uint32_t)0x00000004U)

/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_ADC1DC2              ((uint32_t)0x00010000U) /*!< Refer to AN4073 on how to use this bit  */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0            ((uint32_t)0x000FU) /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1            ((uint32_t)0x00F0U) /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2            ((uint32_t)0x0F00U) /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3            ((uint32_t)0xF000U) /*!<EXTI 3 configuration */
/** 
  * @brief   EXTI0 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI0_PA         ((uint32_t)0x0000U) /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB         ((uint32_t)0x0001U) /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC         ((uint32_t)0x0002U) /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH         ((uint32_t)0x0007U) /*!<PH[0] pin */

/** 
  * @brief   EXTI1 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI1_PA         ((uint32_t)0x0000U) /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB         ((uint32_t)0x0010U) /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC         ((uint32_t)0x0020U) /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH         ((uint32_t)0x0070U) /*!<PH[1] pin */

/** 
  * @brief   EXTI2 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI2_PA         ((uint32_t)0x0000U) /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB         ((uint32_t)0x0100U) /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC         ((uint32_t)0x0200U) /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH         ((uint32_t)0x0700U) /*!<PH[2] pin */

/** 
  * @brief   EXTI3 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI3_PA         ((uint32_t)0x0000U) /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB         ((uint32_t)0x1000U) /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC         ((uint32_t)0x2000U) /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH         ((uint32_t)0x7000U) /*!<PH[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4            ((uint32_t)0x000FU) /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5            ((uint32_t)0x00F0U) /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6            ((uint32_t)0x0F00U) /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7            ((uint32_t)0xF000U) /*!<EXTI 7 configuration */
/** 
  * @brief   EXTI4 configuration  
  */ 
#define SYSCFG_EXTICR2_EXTI4_PA         ((uint32_t)0x0000U) /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB         ((uint32_t)0x0001U) /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC         ((uint32_t)0x0002U) /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH         ((uint32_t)0x0007U) /*!<PH[4] pin */

/** 
  * @brief   EXTI5 configuration  
  */ 
#define SYSCFG_EXTICR2_EXTI5_PA         ((uint32_t)0x0000U) /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB         ((uint32_t)0x0010U) /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC         ((uint32_t)0x0020U) /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH         ((uint32_t)0x0070U) /*!<PH[5] pin */

/** 
  * @brief   EXTI6 configuration  
  */ 
#define SYSCFG_EXTICR2_EXTI6_PA         ((uint32_t)0x0000U) /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB         ((uint32_t)0x0100U) /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC         ((uint32_t)0x0200U) /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH         ((uint32_t)0x0700U) /*!<PH[6] pin */

/** 
  * @brief   EXTI7 configuration  
  */ 
#define SYSCFG_EXTICR2_EXTI7_PA         ((uint32_t)0x0000U) /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB         ((uint32_t)0x1000U) /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC         ((uint32_t)0x2000U) /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH         ((uint32_t)0x7000U) /*!<PH[7] pin */


/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8            ((uint32_t)0x000FU) /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9            ((uint32_t)0x00F0U) /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10           ((uint32_t)0x0F00U) /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11           ((uint32_t)0xF000U) /*!<EXTI 11 configuration */
           
/** 
  * @brief   EXTI8 configuration  
  */ 
#define SYSCFG_EXTICR3_EXTI8_PA         ((uint32_t)0x0000U) /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB         ((uint32_t)0x0001U) /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC         ((uint32_t)0x0002U) /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH         ((uint32_t)0x0007U) /*!<PH[8] pin */

/** 
  * @brief   EXTI9 configuration  
  */ 
#define SYSCFG_EXTICR3_EXTI9_PA         ((uint32_t)0x0000U) /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB         ((uint32_t)0x0010U) /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC         ((uint32_t)0x0020U) /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH         ((uint32_t)0x0070U) /*!<PH[9] pin */

/** 
  * @brief   EXTI10 configuration  
  */ 
#define SYSCFG_EXTICR3_EXTI10_PA        ((uint32_t)0x0000U) /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB        ((uint32_t)0x0100U) /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC        ((uint32_t)0x0200U) /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH        ((uint32_t)0x0700U) /*!<PH[10] pin */

/** 
  * @brief   EXTI11 configuration  
  */ 
#define SYSCFG_EXTICR3_EXTI11_PA        ((uint32_t)0x0000U) /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB        ((uint32_t)0x1000U) /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC        ((uint32_t)0x2000U) /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH        ((uint32_t)0x7000U) /*!<PH[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12           ((uint32_t)0x000FU) /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13           ((uint32_t)0x00F0U) /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14           ((uint32_t)0x0F00U) /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15           ((uint32_t)0xF000U) /*!<EXTI 15 configuration */
/** 
  * @brief   EXTI12 configuration  
  */ 
#define SYSCFG_EXTICR4_EXTI12_PA        ((uint32_t)0x0000U) /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB        ((uint32_t)0x0001U) /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC        ((uint32_t)0x0002U) /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH        ((uint32_t)0x0007U) /*!<PH[12] pin */

/** 
  * @brief   EXTI13 configuration  
  */ 
#define SYSCFG_EXTICR4_EXTI13_PA        ((uint32_t)0x0000U) /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB        ((uint32_t)0x0010U) /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC        ((uint32_t)0x0020U) /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH        ((uint32_t)0x0070U) /*!<PH[13] pin */

/** 
  * @brief   EXTI14 configuration  
  */ 
#define SYSCFG_EXTICR4_EXTI14_PA        ((uint32_t)0x0000U) /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB        ((uint32_t)0x0100U) /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC        ((uint32_t)0x0200U) /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH        ((uint32_t)0x0700U) /*!<PH[14] pin */

/** 
  * @brief   EXTI15 configuration  
  */ 
#define SYSCFG_EXTICR4_EXTI15_PA        ((uint32_t)0x0000U) /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB        ((uint32_t)0x1000U) /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC        ((uint32_t)0x2000U) /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH        ((uint32_t)0x7000U) /*!<PH[15] pin */

/******************  Bit definition for SYSCFG_CMPCR register  ****************/
#define SYSCFG_CMPCR_CMP_PD             ((uint32_t)0x00000001U) /*!<Compensation cell ready flag */
#define SYSCFG_CMPCR_READY              ((uint32_t)0x00000100U) /*!<Compensation cell power-down */

/******************  Bit definition for SYSCFG_CFGR register  *****************/
#define SYSCFG_CFGR_FMPI2C1_SCL         ((uint32_t)0x00000001U) /*!<FM+ drive capability for FMPI2C1_SCL pin */
#define SYSCFG_CFGR_FMPI2C1_SDA         ((uint32_t)0x00000002U) /*!<FM+ drive capability for FMPI2C1_SDA pin */

/******************  Bit definition for SYSCFG_CFGR2 register  *****************/
#define SYSCFG_CFGR2_LOCKUP_LOCK        ((uint32_t)0x00000001U) /*!<Core Lockup lock */
#define SYSCFG_CFGR2_PVD_LOCK           ((uint32_t)0x00000004U) /*!<PVD Lock         */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         ((uint32_t)0x0001U)            /*!<Counter enable        */
#define  TIM_CR1_UDIS                        ((uint32_t)0x0002U)            /*!<Update disable        */
#define  TIM_CR1_URS                         ((uint32_t)0x0004U)            /*!<Update request source */
#define  TIM_CR1_OPM                         ((uint32_t)0x0008U)            /*!<One pulse mode        */
#define  TIM_CR1_DIR                         ((uint32_t)0x0010U)            /*!<Direction             */

#define  TIM_CR1_CMS                         ((uint32_t)0x0060U)            /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       ((uint32_t)0x0020U)            /*!<Bit 0 */
#define  TIM_CR1_CMS_1                       ((uint32_t)0x0040U)            /*!<Bit 1 */

#define  TIM_CR1_ARPE                        ((uint32_t)0x0080U)            /*!<Auto-reload preload enable     */

#define  TIM_CR1_CKD                         ((uint32_t)0x0300U)            /*!<CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       ((uint32_t)0x0100U)            /*!<Bit 0 */
#define  TIM_CR1_CKD_1                       ((uint32_t)0x0200U)            /*!<Bit 1 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        ((uint32_t)0x0001U)            /*!<Capture/Compare Preloaded Control        */
#define  TIM_CR2_CCUS                        ((uint32_t)0x0004U)            /*!<Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        ((uint32_t)0x0008U)            /*!<Capture/Compare DMA Selection            */

#define  TIM_CR2_MMS                         ((uint32_t)0x0070U)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       ((uint32_t)0x0010U)            /*!<Bit 0 */
#define  TIM_CR2_MMS_1                       ((uint32_t)0x0020U)            /*!<Bit 1 */
#define  TIM_CR2_MMS_2                       ((uint32_t)0x0040U)            /*!<Bit 2 */

#define  TIM_CR2_TI1S                        ((uint32_t)0x0080U)            /*!<TI1 Selection */
#define  TIM_CR2_OIS1                        ((uint32_t)0x0100U)            /*!<Output Idle state 1 (OC1 output)  */
#define  TIM_CR2_OIS1N                       ((uint32_t)0x0200U)            /*!<Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        ((uint32_t)0x0400U)            /*!<Output Idle state 2 (OC2 output)  */
#define  TIM_CR2_OIS2N                       ((uint32_t)0x0800U)            /*!<Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        ((uint32_t)0x1000U)            /*!<Output Idle state 3 (OC3 output)  */
#define  TIM_CR2_OIS3N                       ((uint32_t)0x2000U)            /*!<Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        ((uint32_t)0x4000U)            /*!<Output Idle state 4 (OC4 output)  */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        ((uint32_t)0x0007U)            /*!<SMS[2:0] bits (Slave mode selection)    */
#define  TIM_SMCR_SMS_0                      ((uint32_t)0x0001U)            /*!<Bit 0 */
#define  TIM_SMCR_SMS_1                      ((uint32_t)0x0002U)            /*!<Bit 1 */
#define  TIM_SMCR_SMS_2                      ((uint32_t)0x0004U)            /*!<Bit 2 */

#define  TIM_SMCR_TS                         ((uint32_t)0x0070U)            /*!<TS[2:0] bits (Trigger selection)        */
#define  TIM_SMCR_TS_0                       ((uint32_t)0x0010U)            /*!<Bit 0 */
#define  TIM_SMCR_TS_1                       ((uint32_t)0x0020U)            /*!<Bit 1 */
#define  TIM_SMCR_TS_2                       ((uint32_t)0x0040U)            /*!<Bit 2 */

#define  TIM_SMCR_MSM                        ((uint32_t)0x0080U)            /*!<Master/slave mode                       */

#define  TIM_SMCR_ETF                        ((uint32_t)0x0F00U)            /*!<ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      ((uint32_t)0x0100U)            /*!<Bit 0 */
#define  TIM_SMCR_ETF_1                      ((uint32_t)0x0200U)            /*!<Bit 1 */
#define  TIM_SMCR_ETF_2                      ((uint32_t)0x0400U)            /*!<Bit 2 */
#define  TIM_SMCR_ETF_3                      ((uint32_t)0x0800U)            /*!<Bit 3 */

#define  TIM_SMCR_ETPS                       ((uint32_t)0x3000U)            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     ((uint32_t)0x1000U)            /*!<Bit 0 */
#define  TIM_SMCR_ETPS_1                     ((uint32_t)0x2000U)            /*!<Bit 1 */

#define  TIM_SMCR_ECE                        ((uint32_t)0x4000U)            /*!<External clock enable     */
#define  TIM_SMCR_ETP                        ((uint32_t)0x8000U)            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        ((uint32_t)0x0001U)            /*!<Update interrupt enable */
#define  TIM_DIER_CC1IE                      ((uint32_t)0x0002U)            /*!<Capture/Compare 1 interrupt enable   */
#define  TIM_DIER_CC2IE                      ((uint32_t)0x0004U)            /*!<Capture/Compare 2 interrupt enable   */
#define  TIM_DIER_CC3IE                      ((uint32_t)0x0008U)            /*!<Capture/Compare 3 interrupt enable   */
#define  TIM_DIER_CC4IE                      ((uint32_t)0x0010U)            /*!<Capture/Compare 4 interrupt enable   */
#define  TIM_DIER_COMIE                      ((uint32_t)0x0020U)            /*!<COM interrupt enable                 */
#define  TIM_DIER_TIE                        ((uint32_t)0x0040U)            /*!<Trigger interrupt enable             */
#define  TIM_DIER_BIE                        ((uint32_t)0x0080U)            /*!<Break interrupt enable               */
#define  TIM_DIER_UDE                        ((uint32_t)0x0100U)            /*!<Update DMA request enable            */
#define  TIM_DIER_CC1DE                      ((uint32_t)0x0200U)            /*!<Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      ((uint32_t)0x0400U)            /*!<Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      ((uint32_t)0x0800U)            /*!<Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      ((uint32_t)0x1000U)            /*!<Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      ((uint32_t)0x2000U)            /*!<COM DMA request enable               */
#define  TIM_DIER_TDE                        ((uint32_t)0x4000U)            /*!<Trigger DMA request enable           */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          ((uint32_t)0x0001U)            /*!<Update interrupt Flag              */
#define  TIM_SR_CC1IF                        ((uint32_t)0x0002U)            /*!<Capture/Compare 1 interrupt Flag   */
#define  TIM_SR_CC2IF                        ((uint32_t)0x0004U)            /*!<Capture/Compare 2 interrupt Flag   */
#define  TIM_SR_CC3IF                        ((uint32_t)0x0008U)            /*!<Capture/Compare 3 interrupt Flag   */
#define  TIM_SR_CC4IF                        ((uint32_t)0x0010U)            /*!<Capture/Compare 4 interrupt Flag   */
#define  TIM_SR_COMIF                        ((uint32_t)0x0020U)            /*!<COM interrupt Flag                 */
#define  TIM_SR_TIF                          ((uint32_t)0x0040U)            /*!<Trigger interrupt Flag             */
#define  TIM_SR_BIF                          ((uint32_t)0x0080U)            /*!<Break interrupt Flag               */
#define  TIM_SR_CC1OF                        ((uint32_t)0x0200U)            /*!<Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        ((uint32_t)0x0400U)            /*!<Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        ((uint32_t)0x0800U)            /*!<Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        ((uint32_t)0x1000U)            /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          ((uint32_t)0x01U)               /*!<Update Generation                         */
#define  TIM_EGR_CC1G                        ((uint32_t)0x02U)               /*!<Capture/Compare 1 Generation              */
#define  TIM_EGR_CC2G                        ((uint32_t)0x04U)               /*!<Capture/Compare 2 Generation              */
#define  TIM_EGR_CC3G                        ((uint32_t)0x08U)               /*!<Capture/Compare 3 Generation              */
#define  TIM_EGR_CC4G                        ((uint32_t)0x10U)               /*!<Capture/Compare 4 Generation              */
#define  TIM_EGR_COMG                        ((uint32_t)0x20U)               /*!<Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          ((uint32_t)0x40U)               /*!<Trigger Generation                        */
#define  TIM_EGR_BG                          ((uint32_t)0x80U)               /*!<Break Generation                          */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      ((uint32_t)0x0003U)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    ((uint32_t)0x0001U)            /*!<Bit 0 */
#define  TIM_CCMR1_CC1S_1                    ((uint32_t)0x0002U)            /*!<Bit 1 */

#define  TIM_CCMR1_OC1FE                     ((uint32_t)0x0004U)            /*!<Output Compare 1 Fast enable                 */
#define  TIM_CCMR1_OC1PE                     ((uint32_t)0x0008U)            /*!<Output Compare 1 Preload enable              */

#define  TIM_CCMR1_OC1M                      ((uint32_t)0x0070U)            /*!<OC1M[2:0] bits (Output Compare 1 Mode)       */
#define  TIM_CCMR1_OC1M_0                    ((uint32_t)0x0010U)            /*!<Bit 0 */
#define  TIM_CCMR1_OC1M_1                    ((uint32_t)0x0020U)            /*!<Bit 1 */
#define  TIM_CCMR1_OC1M_2                    ((uint32_t)0x0040U)            /*!<Bit 2 */

#define  TIM_CCMR1_OC1CE                     ((uint32_t)0x0080U)            /*!<Output Compare 1Clear Enable                 */

#define  TIM_CCMR1_CC2S                      ((uint32_t)0x0300U)            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    ((uint32_t)0x0100U)            /*!<Bit 0 */
#define  TIM_CCMR1_CC2S_1                    ((uint32_t)0x0200U)            /*!<Bit 1 */

#define  TIM_CCMR1_OC2FE                     ((uint32_t)0x0400U)            /*!<Output Compare 2 Fast enable                 */
#define  TIM_CCMR1_OC2PE                     ((uint32_t)0x0800U)            /*!<Output Compare 2 Preload enable              */

#define  TIM_CCMR1_OC2M                      ((uint32_t)0x7000U)            /*!<OC2M[2:0] bits (Output Compare 2 Mode)       */
#define  TIM_CCMR1_OC2M_0                    ((uint32_t)0x1000U)            /*!<Bit 0 */
#define  TIM_CCMR1_OC2M_1                    ((uint32_t)0x2000U)            /*!<Bit 1 */
#define  TIM_CCMR1_OC2M_2                    ((uint32_t)0x4000U)            /*!<Bit 2 */

#define  TIM_CCMR1_OC2CE                     ((uint32_t)0x8000U)            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    ((uint32_t)0x000CU)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  ((uint32_t)0x0004U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  ((uint32_t)0x0008U)            /*!<Bit 1 */

#define  TIM_CCMR1_IC1F                      ((uint32_t)0x00F0U)            /*!<IC1F[3:0] bits (Input Capture 1 Filter)      */
#define  TIM_CCMR1_IC1F_0                    ((uint32_t)0x0010U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1F_1                    ((uint32_t)0x0020U)            /*!<Bit 1 */
#define  TIM_CCMR1_IC1F_2                    ((uint32_t)0x0040U)            /*!<Bit 2 */
#define  TIM_CCMR1_IC1F_3                    ((uint32_t)0x0080U)            /*!<Bit 3 */

#define  TIM_CCMR1_IC2PSC                    ((uint32_t)0x0C00U)            /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler)  */
#define  TIM_CCMR1_IC2PSC_0                  ((uint32_t)0x0400U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  ((uint32_t)0x0800U)            /*!<Bit 1 */

#define  TIM_CCMR1_IC2F                      ((uint32_t)0xF000U)            /*!<IC2F[3:0] bits (Input Capture 2 Filter)       */
#define  TIM_CCMR1_IC2F_0                    ((uint32_t)0x1000U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2F_1                    ((uint32_t)0x2000U)            /*!<Bit 1 */
#define  TIM_CCMR1_IC2F_2                    ((uint32_t)0x4000U)            /*!<Bit 2 */
#define  TIM_CCMR1_IC2F_3                    ((uint32_t)0x8000U)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      ((uint32_t)0x0003U)            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection)  */
#define  TIM_CCMR2_CC3S_0                    ((uint32_t)0x0001U)            /*!<Bit 0 */
#define  TIM_CCMR2_CC3S_1                    ((uint32_t)0x0002U)            /*!<Bit 1 */

#define  TIM_CCMR2_OC3FE                     ((uint32_t)0x0004U)            /*!<Output Compare 3 Fast enable           */
#define  TIM_CCMR2_OC3PE                     ((uint32_t)0x0008U)            /*!<Output Compare 3 Preload enable        */

#define  TIM_CCMR2_OC3M                      ((uint32_t)0x0070U)            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    ((uint32_t)0x0010U)            /*!<Bit 0 */
#define  TIM_CCMR2_OC3M_1                    ((uint32_t)0x0020U)            /*!<Bit 1 */
#define  TIM_CCMR2_OC3M_2                    ((uint32_t)0x0040U)            /*!<Bit 2 */

#define  TIM_CCMR2_OC3CE                     ((uint32_t)0x0080U)            /*!<Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      ((uint32_t)0x0300U)            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    ((uint32_t)0x0100U)            /*!<Bit 0 */
#define  TIM_CCMR2_CC4S_1                    ((uint32_t)0x0200U)            /*!<Bit 1 */

#define  TIM_CCMR2_OC4FE                     ((uint32_t)0x0400U)            /*!<Output Compare 4 Fast enable    */
#define  TIM_CCMR2_OC4PE                     ((uint32_t)0x0800U)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      ((uint32_t)0x7000U)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    ((uint32_t)0x1000U)            /*!<Bit 0 */
#define  TIM_CCMR2_OC4M_1                    ((uint32_t)0x2000U)            /*!<Bit 1 */
#define  TIM_CCMR2_OC4M_2                    ((uint32_t)0x4000U)            /*!<Bit 2 */

#define  TIM_CCMR2_OC4CE                     ((uint32_t)0x8000U)            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    ((uint32_t)0x000CU)            /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  ((uint32_t)0x0004U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  ((uint32_t)0x0008U)            /*!<Bit 1 */

#define  TIM_CCMR2_IC3F                      ((uint32_t)0x00F0U)            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    ((uint32_t)0x0010U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3F_1                    ((uint32_t)0x0020U)            /*!<Bit 1 */
#define  TIM_CCMR2_IC3F_2                    ((uint32_t)0x0040U)            /*!<Bit 2 */
#define  TIM_CCMR2_IC3F_3                    ((uint32_t)0x0080U)            /*!<Bit 3 */

#define  TIM_CCMR2_IC4PSC                    ((uint32_t)0x0C00U)            /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  ((uint32_t)0x0400U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  ((uint32_t)0x0800U)            /*!<Bit 1 */

#define  TIM_CCMR2_IC4F                      ((uint32_t)0xF000U)            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    ((uint32_t)0x1000U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4F_1                    ((uint32_t)0x2000U)            /*!<Bit 1 */
#define  TIM_CCMR2_IC4F_2                    ((uint32_t)0x4000U)            /*!<Bit 2 */
#define  TIM_CCMR2_IC4F_3                    ((uint32_t)0x8000U)            /*!<Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       ((uint32_t)0x0001U)            /*!<Capture/Compare 1 output enable                 */
#define  TIM_CCER_CC1P                       ((uint32_t)0x0002U)            /*!<Capture/Compare 1 output Polarity               */
#define  TIM_CCER_CC1NE                      ((uint32_t)0x0004U)            /*!<Capture/Compare 1 Complementary output enable   */
#define  TIM_CCER_CC1NP                      ((uint32_t)0x0008U)            /*!<Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       ((uint32_t)0x0010U)            /*!<Capture/Compare 2 output enable                 */
#define  TIM_CCER_CC2P                       ((uint32_t)0x0020U)            /*!<Capture/Compare 2 output Polarity               */
#define  TIM_CCER_CC2NE                      ((uint32_t)0x0040U)            /*!<Capture/Compare 2 Complementary output enable   */
#define  TIM_CCER_CC2NP                      ((uint32_t)0x0080U)            /*!<Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       ((uint32_t)0x0100U)            /*!<Capture/Compare 3 output enable                 */
#define  TIM_CCER_CC3P                       ((uint32_t)0x0200U)            /*!<Capture/Compare 3 output Polarity               */
#define  TIM_CCER_CC3NE                      ((uint32_t)0x0400U)            /*!<Capture/Compare 3 Complementary output enable   */
#define  TIM_CCER_CC3NP                      ((uint32_t)0x0800U)            /*!<Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       ((uint32_t)0x1000U)            /*!<Capture/Compare 4 output enable                 */
#define  TIM_CCER_CC4P                       ((uint32_t)0x2000U)            /*!<Capture/Compare 4 output Polarity               */
#define  TIM_CCER_CC4NP                      ((uint32_t)0x8000U)            /*!<Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         ((uint32_t)0xFFFFU)            /*!<Counter Value            */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         ((uint32_t)0xFFFFU)            /*!<Prescaler Value          */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         ((uint32_t)0xFFFFU)            /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         ((uint32_t)0xFFU)               /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       ((uint32_t)0xFFFFU)            /*!<Capture/Compare 1 Value  */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       ((uint32_t)0xFFFFU)            /*!<Capture/Compare 2 Value  */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       ((uint32_t)0xFFFFU)            /*!<Capture/Compare 3 Value  */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       ((uint32_t)0xFFFFU)            /*!<Capture/Compare 4 Value  */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        ((uint32_t)0x00FFU)            /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      ((uint32_t)0x0001U)            /*!<Bit 0 */
#define  TIM_BDTR_DTG_1                      ((uint32_t)0x0002U)            /*!<Bit 1 */
#define  TIM_BDTR_DTG_2                      ((uint32_t)0x0004U)            /*!<Bit 2 */
#define  TIM_BDTR_DTG_3                      ((uint32_t)0x0008U)            /*!<Bit 3 */
#define  TIM_BDTR_DTG_4                      ((uint32_t)0x0010U)            /*!<Bit 4 */
#define  TIM_BDTR_DTG_5                      ((uint32_t)0x0020U)            /*!<Bit 5 */
#define  TIM_BDTR_DTG_6                      ((uint32_t)0x0040U)            /*!<Bit 6 */
#define  TIM_BDTR_DTG_7                      ((uint32_t)0x0080U)            /*!<Bit 7 */

#define  TIM_BDTR_LOCK                       ((uint32_t)0x0300U)            /*!<LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     ((uint32_t)0x0100U)            /*!<Bit 0 */
#define  TIM_BDTR_LOCK_1                     ((uint32_t)0x0200U)            /*!<Bit 1 */

#define  TIM_BDTR_OSSI                       ((uint32_t)0x0400U)            /*!<Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       ((uint32_t)0x0800U)            /*!<Off-State Selection for Run mode  */
#define  TIM_BDTR_BKE                        ((uint32_t)0x1000U)            /*!<Break enable                      */
#define  TIM_BDTR_BKP                        ((uint32_t)0x2000U)            /*!<Break Polarity                    */
#define  TIM_BDTR_AOE                        ((uint32_t)0x4000U)            /*!<Automatic Output enable           */
#define  TIM_BDTR_MOE                        ((uint32_t)0x8000U)            /*!<Main Output enable                */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         ((uint32_t)0x001FU)            /*!<DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       ((uint32_t)0x0001U)            /*!<Bit 0 */
#define  TIM_DCR_DBA_1                       ((uint32_t)0x0002U)            /*!<Bit 1 */
#define  TIM_DCR_DBA_2                       ((uint32_t)0x0004U)            /*!<Bit 2 */
#define  TIM_DCR_DBA_3                       ((uint32_t)0x0008U)            /*!<Bit 3 */
#define  TIM_DCR_DBA_4                       ((uint32_t)0x0010U)            /*!<Bit 4 */

#define  TIM_DCR_DBL                         ((uint32_t)0x1F00U)            /*!<DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       ((uint32_t)0x0100U)            /*!<Bit 0 */
#define  TIM_DCR_DBL_1                       ((uint32_t)0x0200U)            /*!<Bit 1 */
#define  TIM_DCR_DBL_2                       ((uint32_t)0x0400U)            /*!<Bit 2 */
#define  TIM_DCR_DBL_3                       ((uint32_t)0x0800U)            /*!<Bit 3 */
#define  TIM_DCR_DBL_4                       ((uint32_t)0x1000U)            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       ((uint32_t)0xFFFFU)            /*!<DMA register for burst accesses                    */

/*******************  Bit definition for TIM_OR register  *********************/
#define TIM_OR_TI4_RMP                       ((uint32_t)0x00C0U)            /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
#define TIM_OR_TI4_RMP_0                     ((uint32_t)0x0040U)            /*!<Bit 0 */
#define TIM_OR_TI4_RMP_1                     ((uint32_t)0x0080U)            /*!<Bit 1 */

/******************************************************************************/
/*                                                                            */
/*                         Low Power Timer (LPTIM)                            */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for LPTIM_ISR register  *******************/
#define  LPTIM_ISR_CMPM                         ((uint32_t)0x00000001U)            /*!< Compare match                       */
#define  LPTIM_ISR_ARRM                         ((uint32_t)0x00000002U)            /*!< Autoreload match                    */
#define  LPTIM_ISR_EXTTRIG                      ((uint32_t)0x00000004U)            /*!< External trigger edge event         */
#define  LPTIM_ISR_CMPOK                        ((uint32_t)0x00000008U)            /*!< Compare register update OK          */
#define  LPTIM_ISR_ARROK                        ((uint32_t)0x00000010U)            /*!< Autoreload register update OK       */
#define  LPTIM_ISR_UP                           ((uint32_t)0x00000020U)            /*!< Counter direction change down to up */
#define  LPTIM_ISR_DOWN                         ((uint32_t)0x00000040U)            /*!< Counter direction change up to down */

/******************  Bit definition for LPTIM_ICR register  *******************/
#define  LPTIM_ICR_CMPMCF                       ((uint32_t)0x00000001U)            /*!< Compare match Clear Flag                       */
#define  LPTIM_ICR_ARRMCF                       ((uint32_t)0x00000002U)            /*!< Autoreload match Clear Flag                    */
#define  LPTIM_ICR_EXTTRIGCF                    ((uint32_t)0x00000004U)            /*!< External trigger edge event Clear Flag         */
#define  LPTIM_ICR_CMPOKCF                      ((uint32_t)0x00000008U)            /*!< Compare register update OK Clear Flag          */
#define  LPTIM_ICR_ARROKCF                      ((uint32_t)0x00000010U)            /*!< Autoreload register update OK Clear Flag       */
#define  LPTIM_ICR_UPCF                         ((uint32_t)0x00000020U)            /*!< Counter direction change down to up Clear Flag */
#define  LPTIM_ICR_DOWNCF                       ((uint32_t)0x00000040U)            /*!< Counter direction change up to down Clear Flag */

/******************  Bit definition for LPTIM_IER register ********************/
#define  LPTIM_IER_CMPMIE                       ((uint32_t)0x00000001U)            /*!< Compare match Interrupt Enable                       */
#define  LPTIM_IER_ARRMIE                       ((uint32_t)0x00000002U)            /*!< Autoreload match Interrupt Enable                    */
#define  LPTIM_IER_EXTTRIGIE                    ((uint32_t)0x00000004U)            /*!< External trigger edge event Interrupt Enable         */
#define  LPTIM_IER_CMPOKIE                      ((uint32_t)0x00000008U)            /*!< Compare register update OK Interrupt Enable          */
#define  LPTIM_IER_ARROKIE                      ((uint32_t)0x00000010U)            /*!< Autoreload register update OK Interrupt Enable       */
#define  LPTIM_IER_UPIE                         ((uint32_t)0x00000020U)            /*!< Counter direction change down to up Interrupt Enable */
#define  LPTIM_IER_DOWNIE                       ((uint32_t)0x00000040U)            /*!< Counter direction change up to down Interrupt Enable */

/******************  Bit definition for LPTIM_CFGR register *******************/
#define  LPTIM_CFGR_CKSEL                       ((uint32_t)0x00000001U)             /*!< Clock selector */

#define  LPTIM_CFGR_CKPOL                       ((uint32_t)0x00000006U)             /*!< CKPOL[1:0] bits (Clock polarity) */
#define  LPTIM_CFGR_CKPOL_0                     ((uint32_t)0x00000002U)             /*!< Bit 0 */
#define  LPTIM_CFGR_CKPOL_1                     ((uint32_t)0x00000004U)             /*!< Bit 1 */

#define  LPTIM_CFGR_CKFLT                       ((uint32_t)0x00000018U)             /*!< CKFLT[1:0] bits (Configurable digital filter for external clock) */
#define  LPTIM_CFGR_CKFLT_0                     ((uint32_t)0x00000008U)             /*!< Bit 0 */
#define  LPTIM_CFGR_CKFLT_1                     ((uint32_t)0x00000010U)             /*!< Bit 1 */

#define  LPTIM_CFGR_TRGFLT                      ((uint32_t)0x000000C0U)             /*!< TRGFLT[1:0] bits (Configurable digital filter for trigger) */
#define  LPTIM_CFGR_TRGFLT_0                    ((uint32_t)0x00000040U)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRGFLT_1                    ((uint32_t)0x00000080U)             /*!< Bit 1 */

#define  LPTIM_CFGR_PRESC                       ((uint32_t)0x00000E00U)             /*!< PRESC[2:0] bits (Clock prescaler) */
#define  LPTIM_CFGR_PRESC_0                     ((uint32_t)0x00000200U)             /*!< Bit 0 */
#define  LPTIM_CFGR_PRESC_1                     ((uint32_t)0x00000400U)             /*!< Bit 1 */
#define  LPTIM_CFGR_PRESC_2                     ((uint32_t)0x00000800U)             /*!< Bit 2 */

#define  LPTIM_CFGR_TRIGSEL                     ((uint32_t)0x0000E000U)             /*!< TRIGSEL[2:0]] bits (Trigger selector) */
#define  LPTIM_CFGR_TRIGSEL_0                   ((uint32_t)0x00002000U)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRIGSEL_1                   ((uint32_t)0x00004000U)             /*!< Bit 1 */
#define  LPTIM_CFGR_TRIGSEL_2                   ((uint32_t)0x00008000U)             /*!< Bit 2 */

#define  LPTIM_CFGR_TRIGEN                      ((uint32_t)0x00060000U)             /*!< TRIGEN[1:0] bits (Trigger enable and polarity) */
#define  LPTIM_CFGR_TRIGEN_0                    ((uint32_t)0x00020000U)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRIGEN_1                    ((uint32_t)0x00040000U)             /*!< Bit 1 */

#define  LPTIM_CFGR_TIMOUT                      ((uint32_t)0x00080000U)             /*!< Timout enable           */
#define  LPTIM_CFGR_WAVE                        ((uint32_t)0x00100000U)             /*!< Waveform shape          */
#define  LPTIM_CFGR_WAVPOL                      ((uint32_t)0x00200000U)             /*!< Waveform shape polarity */
#define  LPTIM_CFGR_PRELOAD                     ((uint32_t)0x00400000U)             /*!< Reg update mode         */
#define  LPTIM_CFGR_COUNTMODE                   ((uint32_t)0x00800000U)             /*!< Counter mode enable     */
#define  LPTIM_CFGR_ENC                         ((uint32_t)0x01000000U)             /*!< Encoder mode enable     */

/******************  Bit definition for LPTIM_CR register  ********************/
#define  LPTIM_CR_ENABLE                        ((uint32_t)0x00000001U)             /*!< LPTIMer enable                 */
#define  LPTIM_CR_SNGSTRT                       ((uint32_t)0x00000002U)             /*!< Timer start in single mode     */
#define  LPTIM_CR_CNTSTRT                       ((uint32_t)0x00000004U)             /*!< Timer start in continuous mode */

/******************  Bit definition for LPTIM_CMP register  *******************/
#define  LPTIM_CMP_CMP                          ((uint32_t)0x0000FFFFU)             /*!< Compare register     */

/******************  Bit definition for LPTIM_ARR register  *******************/
#define  LPTIM_ARR_ARR                          ((uint32_t)0x0000FFFFU)             /*!< Auto reload register */

/******************  Bit definition for LPTIM_CNT register  *******************/
#define  LPTIM_CNT_CNT                          ((uint32_t)0x0000FFFFU)             /*!< Counter register     */

/******************  Bit definition for LPTIM_OR register  *******************/
#define  LPTIM_OR_OR                           ((uint32_t)0x00000003U)               /*!< LPTIMER[1:0] bits (Remap selection) */
#define  LPTIM_OR_OR_0                         ((uint32_t)0x00000001U)               /*!< Bit 0 */
#define  LPTIM_OR_OR_1                         ((uint32_t)0x00000002U)               /*!< Bit 1 */

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         ((uint32_t)0x0001U)            /*!<Parity Error                 */
#define  USART_SR_FE                         ((uint32_t)0x0002U)            /*!<Framing Error                */
#define  USART_SR_NE                         ((uint32_t)0x0004U)            /*!<Noise Error Flag             */
#define  USART_SR_ORE                        ((uint32_t)0x0008U)            /*!<OverRun Error                */
#define  USART_SR_IDLE                       ((uint32_t)0x0010U)            /*!<IDLE line detected           */
#define  USART_SR_RXNE                       ((uint32_t)0x0020U)            /*!<Read Data Register Not Empty */
#define  USART_SR_TC                         ((uint32_t)0x0040U)            /*!<Transmission Complete        */
#define  USART_SR_TXE                        ((uint32_t)0x0080U)            /*!<Transmit Data Register Empty */
#define  USART_SR_LBD                        ((uint32_t)0x0100U)            /*!<LIN Break Detection Flag     */
#define  USART_SR_CTS                        ((uint32_t)0x0200U)            /*!<CTS Flag                     */

/*******************  Bit definition for USART_DR register  *******************/
#define  USART_DR_DR                         ((uint32_t)0x01FFU)            /*!<Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction              ((uint32_t)0x000FU)            /*!<Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa              ((uint32_t)0xFFF0U)            /*!<Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       ((uint32_t)0x0001U)            /*!<Send Break                             */
#define  USART_CR1_RWU                       ((uint32_t)0x0002U)            /*!<Receiver wakeup                        */
#define  USART_CR1_RE                        ((uint32_t)0x0004U)            /*!<Receiver Enable                        */
#define  USART_CR1_TE                        ((uint32_t)0x0008U)            /*!<Transmitter Enable                     */
#define  USART_CR1_IDLEIE                    ((uint32_t)0x0010U)            /*!<IDLE Interrupt Enable                  */
#define  USART_CR1_RXNEIE                    ((uint32_t)0x0020U)            /*!<RXNE Interrupt Enable                  */
#define  USART_CR1_TCIE                      ((uint32_t)0x0040U)            /*!<Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((uint32_t)0x0080U)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PEIE                      ((uint32_t)0x0100U)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PS                        ((uint32_t)0x0200U)            /*!<Parity Selection                       */
#define  USART_CR1_PCE                       ((uint32_t)0x0400U)            /*!<Parity Control Enable                  */
#define  USART_CR1_WAKE                      ((uint32_t)0x0800U)            /*!<Wakeup method                          */
#define  USART_CR1_M                         ((uint32_t)0x1000U)            /*!<Word length                            */
#define  USART_CR1_UE                        ((uint32_t)0x2000U)            /*!<USART Enable                           */
#define  USART_CR1_OVER8                     ((uint32_t)0x8000U)            /*!<USART Oversampling by 8 enable         */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       ((uint32_t)0x000FU)            /*!<Address of the USART node            */
#define  USART_CR2_LBDL                      ((uint32_t)0x0020U)            /*!<LIN Break Detection Length           */
#define  USART_CR2_LBDIE                     ((uint32_t)0x0040U)            /*!<LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((uint32_t)0x0100U)            /*!<Last Bit Clock pulse                 */
#define  USART_CR2_CPHA                      ((uint32_t)0x0200U)            /*!<Clock Phase                          */
#define  USART_CR2_CPOL                      ((uint32_t)0x0400U)            /*!<Clock Polarity                       */
#define  USART_CR2_CLKEN                     ((uint32_t)0x0800U)            /*!<Clock Enable                         */

#define  USART_CR2_STOP                      ((uint32_t)0x3000U)            /*!<STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((uint32_t)0x1000U)            /*!<Bit 0 */
#define  USART_CR2_STOP_1                    ((uint32_t)0x2000U)            /*!<Bit 1 */

#define  USART_CR2_LINEN                     ((uint32_t)0x4000U)            /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((uint32_t)0x0001U)            /*!<Error Interrupt Enable      */
#define  USART_CR3_IREN                      ((uint32_t)0x0002U)            /*!<IrDA mode Enable            */
#define  USART_CR3_IRLP                      ((uint32_t)0x0004U)            /*!<IrDA Low-Power              */
#define  USART_CR3_HDSEL                     ((uint32_t)0x0008U)            /*!<Half-Duplex Selection       */
#define  USART_CR3_NACK                      ((uint32_t)0x0010U)            /*!<Smartcard NACK enable       */
#define  USART_CR3_SCEN                      ((uint32_t)0x0020U)            /*!<Smartcard mode enable       */
#define  USART_CR3_DMAR                      ((uint32_t)0x0040U)            /*!<DMA Enable Receiver         */
#define  USART_CR3_DMAT                      ((uint32_t)0x0080U)            /*!<DMA Enable Transmitter      */
#define  USART_CR3_RTSE                      ((uint32_t)0x0100U)            /*!<RTS Enable                  */
#define  USART_CR3_CTSE                      ((uint32_t)0x0200U)            /*!<CTS Enable                  */
#define  USART_CR3_CTSIE                     ((uint32_t)0x0400U)            /*!<CTS Interrupt Enable        */
#define  USART_CR3_ONEBIT                    ((uint32_t)0x0800U)            /*!<USART One bit method enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((uint32_t)0x00FFU)            /*!<PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_PSC_0                    ((uint32_t)0x0001U)            /*!<Bit 0 */
#define  USART_GTPR_PSC_1                    ((uint32_t)0x0002U)            /*!<Bit 1 */
#define  USART_GTPR_PSC_2                    ((uint32_t)0x0004U)            /*!<Bit 2 */
#define  USART_GTPR_PSC_3                    ((uint32_t)0x0008U)            /*!<Bit 3 */
#define  USART_GTPR_PSC_4                    ((uint32_t)0x0010U)            /*!<Bit 4 */
#define  USART_GTPR_PSC_5                    ((uint32_t)0x0020U)            /*!<Bit 5 */
#define  USART_GTPR_PSC_6                    ((uint32_t)0x0040U)            /*!<Bit 6 */
#define  USART_GTPR_PSC_7                    ((uint32_t)0x0080U)            /*!<Bit 7 */

#define  USART_GTPR_GT                       ((uint32_t)0xFF00U)            /*!<Guard time value */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           ((uint32_t)0x7FU)               /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T0                          ((uint32_t)0x01U)               /*!<Bit 0 */
#define  WWDG_CR_T1                          ((uint32_t)0x02U)               /*!<Bit 1 */
#define  WWDG_CR_T2                          ((uint32_t)0x04U)               /*!<Bit 2 */
#define  WWDG_CR_T3                          ((uint32_t)0x08U)               /*!<Bit 3 */
#define  WWDG_CR_T4                          ((uint32_t)0x10U)               /*!<Bit 4 */
#define  WWDG_CR_T5                          ((uint32_t)0x20U)               /*!<Bit 5 */
#define  WWDG_CR_T6                          ((uint32_t)0x40U)               /*!<Bit 6 */

#define  WWDG_CR_WDGA                        ((uint32_t)0x80U)               /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          ((uint32_t)0x007FU)            /*!<W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W0                         ((uint32_t)0x0001U)            /*!<Bit 0 */
#define  WWDG_CFR_W1                         ((uint32_t)0x0002U)            /*!<Bit 1 */
#define  WWDG_CFR_W2                         ((uint32_t)0x0004U)            /*!<Bit 2 */
#define  WWDG_CFR_W3                         ((uint32_t)0x0008U)            /*!<Bit 3 */
#define  WWDG_CFR_W4                         ((uint32_t)0x0010U)            /*!<Bit 4 */
#define  WWDG_CFR_W5                         ((uint32_t)0x0020U)            /*!<Bit 5 */
#define  WWDG_CFR_W6                         ((uint32_t)0x0040U)            /*!<Bit 6 */

#define  WWDG_CFR_WDGTB                      ((uint32_t)0x0180U)            /*!<WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB0                     ((uint32_t)0x0080U)            /*!<Bit 0 */
#define  WWDG_CFR_WDGTB1                     ((uint32_t)0x0100U)            /*!<Bit 1 */

#define  WWDG_CFR_EWI                        ((uint32_t)0x0200U)            /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        ((uint32_t)0x01U)               /*!<Early Wakeup Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((uint32_t)0x00000001U)        /*!<DAC channel1 enable */
#define  DAC_CR_BOFF1                        ((uint32_t)0x00000002U)        /*!<DAC channel1 output buffer disable */
#define  DAC_CR_TEN1                         ((uint32_t)0x00000004U)        /*!<DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((uint32_t)0x00000038U)        /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((uint32_t)0x00000008U)        /*!<Bit 0 */
#define  DAC_CR_TSEL1_1                      ((uint32_t)0x00000010U)        /*!<Bit 1 */
#define  DAC_CR_TSEL1_2                      ((uint32_t)0x00000020U)        /*!<Bit 2 */

#define  DAC_CR_WAVE1                        ((uint32_t)0x000000C0U)        /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((uint32_t)0x00000040U)        /*!<Bit 0 */
#define  DAC_CR_WAVE1_1                      ((uint32_t)0x00000080U)        /*!<Bit 1 */

#define  DAC_CR_MAMP1                        ((uint32_t)0x00000F00U)        /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((uint32_t)0x00000100U)        /*!<Bit 0 */
#define  DAC_CR_MAMP1_1                      ((uint32_t)0x00000200U)        /*!<Bit 1 */
#define  DAC_CR_MAMP1_2                      ((uint32_t)0x00000400U)        /*!<Bit 2 */
#define  DAC_CR_MAMP1_3                      ((uint32_t)0x00000800U)        /*!<Bit 3 */

#define  DAC_CR_DMAEN1                       ((uint32_t)0x00001000U)        /*!<DAC channel1 DMA enable */
#define  DAC_CR_DMAUDRIE1                    ((uint32_t)0x00002000U)        /*!<DAC channel1 DMA underrun interrupt enable*/
#define  DAC_CR_EN2                          ((uint32_t)0x00010000U)        /*!<DAC channel2 enable */
#define  DAC_CR_BOFF2                        ((uint32_t)0x00020000U)        /*!<DAC channel2 output buffer disable */
#define  DAC_CR_TEN2                         ((uint32_t)0x00040000U)        /*!<DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        ((uint32_t)0x00380000U)        /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      ((uint32_t)0x00080000U)        /*!<Bit 0 */
#define  DAC_CR_TSEL2_1                      ((uint32_t)0x00100000U)        /*!<Bit 1 */
#define  DAC_CR_TSEL2_2                      ((uint32_t)0x00200000U)        /*!<Bit 2 */

#define  DAC_CR_WAVE2                        ((uint32_t)0x00C00000U)        /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      ((uint32_t)0x00400000U)        /*!<Bit 0 */
#define  DAC_CR_WAVE2_1                      ((uint32_t)0x00800000U)        /*!<Bit 1 */

#define  DAC_CR_MAMP2                        ((uint32_t)0x0F000000U)        /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      ((uint32_t)0x01000000U)        /*!<Bit 0 */
#define  DAC_CR_MAMP2_1                      ((uint32_t)0x02000000U)        /*!<Bit 1 */
#define  DAC_CR_MAMP2_2                      ((uint32_t)0x04000000U)        /*!<Bit 2 */
#define  DAC_CR_MAMP2_3                      ((uint32_t)0x08000000U)        /*!<Bit 3 */

#define  DAC_CR_DMAEN2                       ((uint32_t)0x10000000U)        /*!<DAC channel2 DMA enabled */
#define  DAC_CR_DMAUDRIE2                    ((uint32_t)0x20000000U)        /*!<DAC channel2 DMA underrun interrupt enable*/

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((uint32_t)0x01U)               /*!<DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 ((uint32_t)0x02U)               /*!<DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((uint32_t)0x0FFFU)            /*!<DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((uint32_t)0xFFF0U)            /*!<DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((uint32_t)0xFFU)               /*!<DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                ((uint32_t)0x0FFFU)            /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                ((uint32_t)0xFFF0U)            /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 ((uint32_t)0xFFU)               /*!<DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((uint32_t)0x00000FFFU)        /*!<DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                ((uint32_t)0x0FFF0000U)        /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((uint32_t)0x0000FFF0U)        /*!<DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                ((uint32_t)0xFFF00000U)        /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((uint32_t)0x00FFU)            /*!<DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 ((uint32_t)0xFF00U)            /*!<DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((uint32_t)0x0FFFU)            /*!<DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   ((uint32_t)0x0FFFU)            /*!<DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      ((uint32_t)0x00002000U)        /*!<DAC channel1 DMA underrun flag */
#define  DAC_SR_DMAUDR2                      ((uint32_t)0x20000000U)        /*!<DAC channel2 DMA underrun flag */
/******************************************************************************/
/*                                                                            */
/*                                DBG                                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define  DBGMCU_IDCODE_DEV_ID                ((uint32_t)0x00000FFFU)
#define  DBGMCU_IDCODE_REV_ID                ((uint32_t)0xFFFF0000U)

/********************  Bit definition for DBGMCU_CR register  *****************/
#define  DBGMCU_CR_DBG_SLEEP                 ((uint32_t)0x00000001U)
#define  DBGMCU_CR_DBG_STOP                  ((uint32_t)0x00000002U)
#define  DBGMCU_CR_DBG_STANDBY               ((uint32_t)0x00000004U)
#define  DBGMCU_CR_TRACE_IOEN                ((uint32_t)0x00000020U)

#define  DBGMCU_CR_TRACE_MODE                ((uint32_t)0x000000C0U)
#define  DBGMCU_CR_TRACE_MODE_0              ((uint32_t)0x00000040U)/*!<Bit 0 */
#define  DBGMCU_CR_TRACE_MODE_1              ((uint32_t)0x00000080U)/*!<Bit 1 */

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
#define  DBGMCU_APB1_FZ_DBG_TIM5_STOP            ((uint32_t)0x00000008U)
#define  DBGMCU_APB1_FZ_DBG_TIM6_STOP            ((uint32_t)0x00000010U)
#define  DBGMCU_APB1_FZ_DBG_RTC_STOP             ((uint32_t)0x00000400U)
#define  DBGMCU_APB1_FZ_DBG_WWDG_STOP            ((uint32_t)0x00000800U)
#define  DBGMCU_APB1_FZ_DBG_IWDG_STOP            ((uint32_t)0x00001000U)
#define  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT   ((uint32_t)0x00200000U)
#define  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT   ((uint32_t)0x00400000U)
#define  DBGMCU_APB1_FZ_DBG_CAN1_STOP            ((uint32_t)0x02000000U)
#define  DBGMCU_APB1_FZ_DBG_CAN2_STOP            ((uint32_t)0x04000000U)

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
#define  DBGMCU_APB2_FZ_DBG_TIM1_STOP        ((uint32_t)0x00000001U)
#define  DBGMCU_APB2_FZ_DBG_TIM9_STOP        ((uint32_t)0x00010000U)
#define  DBGMCU_APB2_FZ_DBG_TIM11_STOP       ((uint32_t)0x00040000U)

/**
  * @}
  */ 

/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */
 
/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)
 
/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/******************************* DAC Instances ********************************/
#define IS_DAC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DAC)

/******************************** DMA Instances *******************************/
#define IS_DMA_STREAM_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Stream0) || \
                                              ((INSTANCE) == DMA1_Stream1) || \
                                              ((INSTANCE) == DMA1_Stream2) || \
                                              ((INSTANCE) == DMA1_Stream3) || \
                                              ((INSTANCE) == DMA1_Stream4) || \
                                              ((INSTANCE) == DMA1_Stream5) || \
                                              ((INSTANCE) == DMA1_Stream6) || \
                                              ((INSTANCE) == DMA1_Stream7) || \
                                              ((INSTANCE) == DMA2_Stream0) || \
                                              ((INSTANCE) == DMA2_Stream1) || \
                                              ((INSTANCE) == DMA2_Stream2) || \
                                              ((INSTANCE) == DMA2_Stream3) || \
                                              ((INSTANCE) == DMA2_Stream4) || \
                                              ((INSTANCE) == DMA2_Stream5) || \
                                              ((INSTANCE) == DMA2_Stream6) || \
                                              ((INSTANCE) == DMA2_Stream7))

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOC) || \
                                        ((INSTANCE) == GPIOH))

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2)) 
/******************************** I2S Instances *******************************/
#define IS_I2S_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == SPI1)

/******************************* LPTIM Instances ******************************/
#define IS_LPTIM_INSTANCE(__INSTANCE__) ((__INSTANCE__) == LPTIM1)

/******************************* RNG Instances ********************************/
#define IS_RNG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RNG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SPI1)
/*************************** SPI Extended Instances ***************************/
#define IS_SPI_ALL_INSTANCE_EXT(INSTANCE) ((INSTANCE) == SPI1)

/****************** TIM Instances : All supported instances *******************/
#define IS_TIM_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                   ((INSTANCE) == TIM5)   || \
                                   ((INSTANCE) == TIM6)   || \
                                   ((INSTANCE) == TIM9)   || \
                                   ((INSTANCE) == TIM11))

/************* TIM Instances : at least 1 capture/compare channel *************/
#define IS_TIM_CC1_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)  || \
                                         ((INSTANCE) == TIM5)  || \
                                         ((INSTANCE) == TIM9)  || \
                                         ((INSTANCE) == TIM11))

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_TIM_CC2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM5) || \
                                       ((INSTANCE) == TIM9))

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_TIM_CC3_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM5))

/************ TIM Instances : at least 4 capture/compare channels *************/
#define IS_TIM_CC4_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM5))

/******************** TIM Instances : Advanced-control timers *****************/
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)

/******************* TIM Instances : Timer input XOR function *****************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM5))

/****************** TIM Instances : DMA requests generation (UDE) *************/
#define IS_TIM_DMA_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM5) || \
                                       ((INSTANCE) == TIM6))

/************ TIM Instances : DMA requests generation (CCxDE) *****************/
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM5))

/************ TIM Instances : DMA requests generation (COMDE) *****************/
#define IS_TIM_CCDMA_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM5)) 

/******************** TIM Instances : DMA burst feature ***********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                             ((INSTANCE) == TIM5))

/****** TIM Instances : master mode available (TIMx_CR2.MMS available )********/
#define IS_TIM_MASTER_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM5) || \
                                          ((INSTANCE) == TIM6) || \
                                          ((INSTANCE) == TIM9))

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM5) || \
                                         ((INSTANCE) == TIM9))

/********************** TIM Instances : 32 bit Counter ************************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE)(((INSTANCE) == TIM5))

/***************** TIM Instances : external trigger input availabe ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                        ((INSTANCE) == TIM5))

/****************** TIM Instances : remapping capability **********************/
#define IS_TIM_REMAP_INSTANCE(INSTANCE) (((INSTANCE) == TIM5)  || \
                                         ((INSTANCE) == TIM11))

/******************* TIM Instances : output(s) available **********************/
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
    ((((INSTANCE) == TIM1) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM5) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM9) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2)))           \
    ||                                         \
    (((INSTANCE) == TIM11) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1))))

/************ TIM Instances : complementary output(s) available ***************/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3))))

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                     ((INSTANCE) == USART2))

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2))

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (0)

/********************* UART Instances : Smard card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2))

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2))     

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/***************************** FMPI2C Instances *******************************/
#define IS_FMPI2C_ALL_INSTANCE(__INSTANCE__) ((__INSTANCE__) == FMPI2C1)

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F410Tx_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
