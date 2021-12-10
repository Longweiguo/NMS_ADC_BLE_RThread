//*****************************************************************************
//
//  am_bsp_pins.h
//! @file
//!
//! @brief BSP pin configuration definitions.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @addtogroup apollo3_bsp BSP for the Apollo3 EVB.
//! @ingroup BSP
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.4.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_BSP_PINS_H
#define AM_BSP_PINS_H

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

#ifdef __cplusplus
extern "C"
{
#endif


#define LITTLE_BOARD

#define AM_BSP_HRM_I2C_MODULE               3

#define AM_BSP_GPIO_ACC_EN                  2
#define AM_BSP_GPIO_ACC_INT                 3
#define AM_BSP_GPIO_TP_PWR_EN               12
#define AM_BSP_GPIO_NAND_PWR_EN             28
#define AM_BSP_GPIO_BAT_ADC                 32
#define AM_BSP_GPIO_MOTOR_PWM               33
#define AM_BSP_GPIO_GH3011_INT              38
#define AM_BSP_GPIO_CHR_INT                 46
#define AM_BSP_GPIO_LCD_EN                  50

#ifdef LITTLE_BOARD
    #define AM_BSP_GPIO_HOME_BUTTON         15
    #define AM_BSP_GPIO_NTC2_EN             25
    #define AM_BSP_GPIO_NTC1_EN             29
    #define AM_BSP_GPIO_LCD_LED_PWM         30
    #define AM_BSP_GPIO_NTC1_ADC            31
    #define AM_BSP_GPIO_NTC2_ADC            34
    #define AM_BSP_GPIO_USB_ADC             35
    #define AM_BSP_GPIO_HRM_EN              44
    #define AM_BSP_GPIO_TP_INT              48
    #define AM_BSP_GPIO_PSRAM_EN            49
    #define AM_BSP_GPIO_LCD_1V8_EN          71
    #define AM_BSP_GPIO_LDO1_EN             72
    #define AM_BSP_GPIO_LDO2_EN             73
#else
    #define AM_BSP_GPIO_TP_INT              11
    #define AM_BSP_GPIO_HOME_BUTTON         35
    #define AM_BSP_GPIO_HRM_EN              36
    #define AM_BSP_GH3011_3V3_EN            36
    #define AM_BSP_GH3011_VLED_EN           34
    #define AM_BSP_GPIO_PSRAM_EN            69
    #define AM_BSP_SATURN_BUTTON            35
#endif


extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_PSRAM_EN;
//*****************************************************************************
//
//  COM_UART_TX pin: This pin is the COM_UART transmit pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_TX         16
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_COM_UART_TX;

//*****************************************************************************
//
//  COM_UART_RX pin: This pin is the COM_UART receive pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_RX         17
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_COM_UART_RX;


//*****************************************************************************
//
//  IOM0_CS pin: I/O Master 0 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_CS             1
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM0_CS;
#define AM_BSP_IOM0_CS_CHNL             2

//*****************************************************************************
//
//  IOM0_CS3 pin: I/O Master 0 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_CS3            15
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM0_CS3;
#define AM_BSP_IOM0_CS3_CHNL            3

//*****************************************************************************
//
//  IOM0_MISO pin: I/O Master 0 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_MISO           6
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM0_MISO;

//*****************************************************************************
//
//  IOM0_MOSI pin: I/O Master 0 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_MOSI           7
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM0_MOSI;

//*****************************************************************************
//
//  IOM0_SCK pin: I/O Master 0 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SCK            5
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM0_SCK;

//*****************************************************************************
//
//  IOM0_SCL pin: I/O Master 0 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SCL            5
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM0_SCL;

//*****************************************************************************
//
//  IOM0_SDA pin: I/O Master 0 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SDA            6
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM0_SDA;

//*****************************************************************************
//
//  IOM1_CS pin: I/O Master 1 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_CS             11
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM1_CS;
#define AM_BSP_IOM1_CS_CHNL             2

//*****************************************************************************
//
//  IOM1_MISO pin: I/O Master 1 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_MISO           9
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM1_MISO;

//*****************************************************************************
//
//  IOM1_MOSI pin: I/O Master 1 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_MOSI           10
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM1_MOSI;

//*****************************************************************************
//
//  IOM1_SCK pin: I/O Master 1 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SCK            8
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM1_SCK;

//*****************************************************************************
//
//  IOM1_SCL pin: I/O Master 1 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SCL            8
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM1_SCL;

//*****************************************************************************
//
//  IOM1_SDA pin: I/O Master 1 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SDA            9
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM1_SDA;

//*****************************************************************************
//
//  TP_RESET pin: tp reset pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_TP_RESET            10

//*****************************************************************************
//
//  TP_INT pin: tp int pin.
//
//*****************************************************************************

extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_TP_INT;

//*****************************************************************************
//
//  TP_PWR_EN pin: tp i2c power enable.
//
//*****************************************************************************


//*****************************************************************************
//
//  IOM2_CS pin: I/O Master 2 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_CS             15
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM2_CS;
#define AM_BSP_IOM2_CS_CHNL             3

//*****************************************************************************
//
//  IOM2_MISO pin: I/O Master 2 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_MISO           25
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM2_MISO;

//*****************************************************************************
//
//  IOM2_MOSI pin: I/O Master 2 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_MOSI           28
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM2_MOSI;

//*****************************************************************************
//
//  IOM2_SCK pin: I/O Master 2 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SCK            27
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM2_SCK;

//*****************************************************************************
//
//  IOM2_SCL pin: I/O Master 2 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SCL            27
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM2_SCL;

//*****************************************************************************
//
//  IOM2_SDA pin: I/O Master 2 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SDA            25
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM2_SDA;

//*****************************************************************************
//
//  IOM3_CS pin: I/O Master 3 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_CS             29
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM3_CS;
#define AM_BSP_IOM3_CS_CHNL             0

//*****************************************************************************
//
//  IOM3_MISO pin: I/O Master 3 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_MISO           43
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM3_MISO;

//*****************************************************************************
//
//  IOM3_MOSI pin: I/O Master 3 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_MOSI           38
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM3_MOSI;

//*****************************************************************************
//
//  IOM3_SCK pin: I/O Master 3 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SCK            42
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM3_SCK;

//*****************************************************************************
//
//  IOM3_SCL pin: I/O Master 3 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SCL            42
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM3_SCL;

//*****************************************************************************
//
//  IOM3_SDA pin: I/O Master 3 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SDA            43
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM3_SDA;

//*****************************************************************************
//
//  IOM4_CS pin: I/O Master 4 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_CS             13
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM4_CS;
#define AM_BSP_IOM4_CS_CHNL             1

//*****************************************************************************
//
//  IOM4_MISO pin: I/O Master 4 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_MISO           40
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM4_MISO;

//*****************************************************************************
//
//  IOM4_MOSI pin: I/O Master 4 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_MOSI           44
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM4_MOSI;

//*****************************************************************************
//
//  IOM4_SCK pin: I/O Master 4 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_SCK            39
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM4_SCK;

//*****************************************************************************
//
//  IOM4_SCL pin: I/O Master 4 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_SCL            39
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM4_SCL;

//*****************************************************************************
//
//  IOM4_SDA pin: I/O Master 4 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_SDA            40
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM4_SDA;

//*****************************************************************************
//
//  IOM5_CS pin: I/O Master 5 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_CS             16
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM5_CS;
#define AM_BSP_IOM5_CS_CHNL             0

//*****************************************************************************
//
//  IOM5_MISO pin: I/O Master 5 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_MISO           49
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM5_MISO;

//*****************************************************************************
//
//  IOM5_MOSI pin: I/O Master 5 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_MOSI           47
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM5_MOSI;

//*****************************************************************************
//
//  IOM5_SCK pin: I/O Master 5 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SCK            48
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM5_SCK;

//*****************************************************************************
//
//  IOM5_SCL pin: I/O Master 5 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SCL            48
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM5_SCL;

//*****************************************************************************
//
//  IOM5_SDA pin: I/O Master 5 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SDA            49
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOM5_SDA;

//*****************************************************************************
//
// Nand flash Power enable
//
//*****************************************************************************

extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_NAND_PWR_EN;

//*****************************************************************************
//
//  MSPI0_CE0 pin: MSPI0 chip select 0.  Note: CE1 must be disabled or tristated when using this pin for CE.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_CE0           27
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI0_CE0;
#define AM_BSP_MSPI0_CE0_CHNL           0

//*****************************************************************************
//
//  MSPI0_CE1 pin: MSPI0 chip select 1.  Note: CE0 must be disabled or tristated when using this pin for CE.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_CE1           12
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI0_CE1;
#define AM_BSP_MSPI0_CE1_CHNL           0

//*****************************************************************************
//
//  MSPI0_D0 pin: MSPI0 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D0            22
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI0_D0;

//*****************************************************************************
//
//  MSPI0_D1 pin: MSPI0 data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D1            26
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI0_D1;

//*****************************************************************************
//
//  MSPI0_D2 pin: MSPI0 data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D2            4
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI0_D2;

//*****************************************************************************
//
//  MSPI0_D3 pin: MSPI0 data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D3            23
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI0_D3;

//*****************************************************************************
//
//  MSPI0_SCK pin: MSPI0 clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_SCK           24
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI0_SCK;

//*****************************************************************************
//
//  MSPI1_CE0 pin: MSPI1 chip select 0.  Note: CE1 must be disabled or tristated when using this pin for CE.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_CE0           61
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_CE0;
#define AM_BSP_MSPI1_CE0_CHNL           0

//*****************************************************************************
//
//  MSPI1_CE1 pin: MSPI1 chip select 1.  Note: CE0 must be disabled or tristated when using this pin for CE.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_CE1           62
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_CE1;
#define AM_BSP_MSPI1_CE1_CHNL           0

//*****************************************************************************
//
//  MSPI1_D0 pin: MSPI1 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D0            51
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_D0;

//*****************************************************************************
//
//  MSPI1_D1 pin: MSPI1 data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D1            52
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_D1;

//*****************************************************************************
//
//  MSPI1_D2 pin: MSPI1 data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D2            53
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_D2;

//*****************************************************************************
//
//  MSPI1_D3 pin: MSPI1 data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D3            54
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_D3;

//*****************************************************************************
//
//  MSPI1_D4 pin: MSPI1 data 4.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D4            55
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_D4;

//*****************************************************************************
//
//  MSPI1_D5 pin: MSPI1 data 5.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D5            56
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_D5;

//*****************************************************************************
//
//  MSPI1_D6 pin: MSPI1 data 6.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D6            57
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_D6;

//*****************************************************************************
//
//  MSPI1_D7 pin: MSPI1 data 7.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D7            58
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_D7;

//*****************************************************************************
//
//  MSPI1_SCK pin: MSPI1 clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_SCK           59
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_SCK;

//*****************************************************************************
//
//  MSPI1_DMDQS pin: MSPI1 DDR Data Strobe.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_DMDQS         60
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI1_DMDQS;

//*****************************************************************************
//
//  MSPI2_CE0 pin: MSPI2 chip select 0.  Note: CE2 must be disabled or tristated when using this pin for CE.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_CE0           70//46  // 70 is HEY_HDK,  46 is demo board
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI2_CE0;
#define AM_BSP_MSPI2_CE0_CHNL           0

//*****************************************************************************
//
//  MSPI2_CE1 pin: MSPI2 chip select 1.  Note: CE0 must be disabled or tristated when using this pin for CE.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_CE1           61
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI2_CE1;
#define AM_BSP_MSPI2_CE1_CHNL           0

//*****************************************************************************
//
//  MSPI2_D0 pin: MSPI2 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D0            64
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI2_D0;

//*****************************************************************************
//
//  MSPI2_D1 pin: MSPI2 data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D1            65
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI2_D1;

//*****************************************************************************
//
//  MSPI2_D2 pin: MSPI2 data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D2            66
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI2_D2;

//*****************************************************************************
//
//  MSPI2_D3 pin: MSPI2 data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D3            67
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI2_D3;

//*****************************************************************************
//
//  MSPI2_SCK pin: MSPI2 clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_SCK           68
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_MSPI2_SCK;



//*****************************************************************************
//
//  DISPLAY_TE pin: Display TE signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISPLAY_TE          63
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_DISPLAY_TE;

//*****************************************************************************
//
//  DISPLAY_RESET pin: Display reset control.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISPLAY_RESET       62
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_DISPLAY_RESET;

//*****************************************************************************
//
//  DISPLAY_DCX pin: Display DCX control.
//
//*****************************************************************************
#define AM_BSP_GPIO_DISPLAY_DCX         60

//*****************************************************************************
//
//  DISPLAY_EN pin: Display power enable control.
//
//*****************************************************************************


//*****************************************************************************
//
//  DISPLAY_LED_PWM pin: Display backlight led pwm out.
//
//*****************************************************************************

// for final board
#define AM_BSP_GPIO_DISPLAY_LED_TIMER         2
#define AM_BSP_GPIO_DISPLAY_LED_TIMER_SEG     AM_HAL_CTIMER_TIMERB



//*****************************************************************************
//
//  IOS_CE pin: I/O Slave chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_CE              3
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOS_CE;
#define AM_BSP_IOS_CE_CHNL              0

//*****************************************************************************
//
//  IOS_MISO pin: I/O Slave SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_MISO            2
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOS_MISO;

//*****************************************************************************
//
//  IOS_MOSI pin: I/O Slave SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_MOSI            1
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOS_MOSI;

//*****************************************************************************
//
//  IOS_SCK pin: I/O Slave SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SCK             0
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOS_SCK;

//*****************************************************************************
//
//  IOS_SCL pin: I/O Slave I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SCL             0
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOS_SCL;

//*****************************************************************************
//
//  IOS_SDA pin: I/O Slave I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SDA             1
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_IOS_SDA;

//*****************************************************************************
//
//  PDMCLK pin: PDM CLK.
//
//*****************************************************************************
#define AM_BSP_GPIO_PDMCLK              12
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_PDMCLK;

//*****************************************************************************
//
//  PDM_DATA pin: PDM DATA.
//
//*****************************************************************************
#define AM_BSP_GPIO_PDM_DATA            11
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_PDM_DATA;

//*****************************************************************************
//
//  SCARD_SCCCLK pin: SCARD SCCCLK.
//
//*****************************************************************************
#define AM_BSP_GPIO_SCARD_SCCCLK        8
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_SCARD_SCCCLK;

//*****************************************************************************
//
//  SCARD_SCCIO pin: Fireball device test board chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_SCARD_SCCIO         9
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_SCARD_SCCIO;

//*****************************************************************************
//
//  SCARD_SCCRST pin: SCARD SCCRST.
//
//*****************************************************************************
#define AM_BSP_GPIO_SCARD_SCCRST        46
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_SCARD_SCCRST;

//*****************************************************************************
//
//  ITM_SWO pin: ITM Serial Wire Output.
//
//*****************************************************************************
#define AM_BSP_GPIO_ITM_SWO             41
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_ITM_SWO;

//*****************************************************************************
//
//  SWDCK pin: Cortex Serial Wire DCK.
//
//*****************************************************************************
#define AM_BSP_GPIO_SWDCK               20
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_SWDCK;

//*****************************************************************************
//
//  SWDIO pin: Cortex Serial Wire DIO.
//
//*****************************************************************************
#define AM_BSP_GPIO_SWDIO               21
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_SWDIO;

//*****************************************************************************
//
//  FIREBALL_CE pin: Fireball device test board chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_FIREBALL_CE         30
extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_FIREBALL_CE;
#define AM_BSP_FIREBALL_CE_CHNL         3

//================================    ADC samples input pin @ eason  ==============================
#define LARGE_BOARD
#ifdef LARGE_BOARD
    typedef enum{
        ADC_GPIO_TEMP_SATURN_LARGE_MAIN_BOARD       =   31  ,
        ADC_GPIO_VOLT_SATURN_LARGE_BATTERY          =   32  ,
        ADC_GPIO_TEMP_SATURN_LARGE_FPC_BOARD        =   34  ,

        ADC_GPIO_VOLT_SATURN_LARGE_USB              =   35  ,
        ADC_GPIO_VOLT_SATURN_LARGE_BUTTON           =   35  ,

        ADC_GPIO_VOLT_DEMO_BUTTON0                  =   16  ,

        ADC_GPIO_DEMO_INTERNAL_VOLTAGE              =   100 ,   //invalid pin for interval circuit measurement
    }ADC_GPIO;
#else
     #define GPIO_ADC_VOLT_SATURN_LARGE_BATTERY      31
    #define GPIO_ADC_TEMP_SATURN_LARGE_MAIN_BOARD   32
    #define GPIO_ADC_TEMP_SATURN_LARGE_FPC_BOARD    34
    #define GPIO_ADC_VOLT_SATURN_LARGE_USB          100
#endif

#ifdef __cplusplus
}
#endif

#endif // AM_BSP_PINS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
