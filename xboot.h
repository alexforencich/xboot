/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* tested with ATXMEGA64A3                                              */
/*                                                                      */
/* xboot.h                                                              */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2010 Alex Forencich                                    */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#ifndef __XBOOT_H
#define __XBOOT_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "sp_driver.h"
#include "eeprom_driver.h"

// token pasting
#define token_paste2_int(x, y) x ## y
#define token_paste2(x, y) token_paste2_int(x, y)
#define token_paste3_int(x, y, z) x ## y ## z
#define token_paste3(x, y, z) token_paste3_int(x, y, z)

// Configuration

// clock config
#define USE_DFLL
// use 32MHz osc if makefile calls for it
#if (F_CPU == 32000000L)
// defaults to 2MHz RC oscillator
// define USE_32MHZ_RC to override
#define USE_32MHZ_RC
#endif

// AVR1008 fixes
// Really only applicable to 256a3 rev A and B devices
//#define USE_AVR1008_EEPROM

// bootloader entrace
//#define USE_ENTER_DELAY
//#define USE_ENTER_PIN
//#define USE_ENTER_UART
//#define USE_ENTER_I2C

// bootloader exit
//#define LOCK_SPM_ON_EXIT

// bootloader communication
#define USE_LED
#define USE_UART
#define USE_I2C
#define USE_I2C_ADDRESS_NEGOTIATION
#define USE_ATTACH_LED

//#define USE_INTERRUPTS
//#define USE_WATCHDOG

// communication modes
// (please leave as-is)
#define MODE_UNDEF              0
#define MODE_UART               1
#define MODE_I2C                2

// bootloader features
#define ENABLE_BLOCK_SUPPORT
#define ENABLE_FLASH_BYTE_SUPPORT
#define ENABLE_EEPROM_BYTE_SUPPORT
#define ENABLE_LOCK_BITS
#define ENABLE_FUSE_BITS

// ENTER_PIN
#define ENTER_PORT              PORTC
#define ENTER_PIN               0
#define ENTER_PIN_CTRL          token_paste3(ENTER_PORT.PIN, ENTER_PIN, CTRL)
#define ENTER_PIN_STATE         0
#define ENTER_PIN_PUEN          1

// ENTER_DELAY
#define ENTER_BLINK_COUNT       3
#define ENTER_BLINK_WAIT        30000

// WATCHDOG
#define WATCHDOG_TIMEOUT 1024

// LED
#define LED_PORT                PORTD
#define LED_PIN                 0
#define LED_INV                 1

// UART
#define UART_BAUD_RATE                  19200
#define UART_PORT                       PORTD
#define UART_DEVICE_PORT                D1
#define UART_TX_PIN                     PIN7_bm
#define UART_DEVICE                     token_paste2(USART, UART_DEVICE_PORT)
#define UART_DEVICE_RXC_ISR             token_paste3(USART, UART_DEVICE_PORT, _RXC_vect)
#define UART_DEVICE_DRE_ISR             token_paste3(USART, UART_DEVICE_PORT, _DRE_vect)
#define UART_DEVICE_TXC_ISR             token_paste3(USART, UART_DEVICE_PORT, _TXC_vect)

#ifdef __AVR_XMEGA__

// BAUD Rate Values
// Known good at 2MHz
#if (F_CPU == 2000000L) && (UART_BAUD_RATE == 19200)
#define UART_BSEL_VALUE         12
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              1
#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 38400)
#define UART_BSEL_VALUE         23
#define UART_BSCALE_VALUE       -2
#define UART_CLK2X              1
#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 57600)
#define UART_BSEL_VALUE         27
#define UART_BSCALE_VALUE       -3
#define UART_CLK2X              1
#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 115200)
#define UART_BSEL_VALUE         19
#define UART_BSCALE_VALUE       -4
#define UART_CLK2X              1
// Known good at 32MHz
#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 19200)
#define UART_BSEL_VALUE         103
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 38400)
#define UART_BSEL_VALUE         51
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 57600)
#define UART_BSEL_VALUE         34
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 115200)
#define UART_BSEL_VALUE         16
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
// None of the above, so calculate something
#else
#warning Not using predefined BAUD rate, possible BAUD rate error!
#if (F_CPU == 2000000L)
#define UART_BSEL_VALUE         ((F_CPU) / ((uint32_t)UART_BAUD_RATE * 8) - 1)
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              1
#else
#define UART_BSEL_VALUE         ((F_CPU) / ((uint32_t)UART_BAUD_RATE * 16) - 1)
#define UART_BSCALE_VALUE       0
#define UART_CLK2X              0
#endif
#endif

#define UART_BAUD_RATE_LOW_REG          UART_DEVICE.BAUDCTRLA
#define UART_BAUD_RATE_HIGH_REG         UART_DEVICE.BAUDCTRLB
#define UART_CONTROL_REG                UART_DEVICE.CTRLB
#define UART_ENABLE_TRANSMITTER_BIT     USART_TXEN_bp
#define UART_ENABLE_RECEIVER_BIT        USART_RXEN_bp
#define UART_STATUS_REG                 UART_DEVICE.STATUS
#define UART_TRANSMIT_COMPLETE_BIT      USART_TXCIF_bp
#define UART_DATA_REG_EMPTY_BIT         USART_DREIF_bp
#define UART_RECEIVE_COMPLETE_BIT       USART_RXCIF_bp
#define UART_CLK2X_BIT                  USART_CLK2X_bp
#define UART_DATA_REG                   UART_DEVICE.DATA

#endif // __AVR_XMEGA__

// I2C
#ifdef __AVR_XMEGA__

#define I2C_DEVICE_PORT                 E
#define I2C_DEVICE                      token_paste2(TWI, I2C_DEVICE_PORT)
#define I2C_DEVICE_ISR                  token_paste3(TWI, I2C_DEVICE_PORT, _TWIS_vect)

#define I2C_MATCH_ANY                   1
#define I2C_ADDRESS                     0x10
#define I2C_GC_ENABLE                   1

#endif // __AVR_XMEGA__

// I2C Address Autonegotiation
// Note: only works on XMega chips for the time being
// There is no easy way to get this to work on regular
// ATMega chips as they have no unique part ID number
#define I2C_AUTONEG_DIS_PROMISC         1
#define I2C_AUTONEG_DIS_GC              0
#define I2C_AUTONEG_PORT                PORTA
#define I2C_AUTONEG_PIN                 2

// Attach LED
#define ATTACH_LED_PORT                 PORTA
#define ATTACH_LED_PIN                  1
#define ATTACH_LED_INV                  1

#ifndef EEPROM_BYTE_ADDRESS_MASK
#if EEPROM_PAGE_SIZE == 32
#define EEPROM_BYTE_ADDRESS_MASK 0x1f
#else
#error Unknown EEPROM page size!  Please add new byte address value!
#endif
#endif

#ifdef USE_INTERRUPTS
#ifndef NEED_INTERRUPTS
#define NEED_INTERRUPTS
#endif // NEED_INTERRUPTS
#endif // USE_INTERRUPTS

#ifdef USE_AVR1008_EEPROM
#ifndef NEED_INTERRUPTS
#define NEED_INTERRUPTS
#endif // NEED_INTERRUPTS
#endif // USE_AVR1008_EEPROM

#ifdef USE_WATCHDOG
/*! \brief Check if Synchronization busy flag is set. */
#define WDT_IsSyncBusy() ( WDT.STATUS & WDT_SYNCBUSY_bm )
#define WDT_Reset()	asm("wdr")
#endif // USE_WATCHDOG

typedef uint32_t ADDR_T;

// Functions
unsigned char __attribute__ ((noinline)) ow_slave_read_bit(void);
void __attribute__ ((noinline)) ow_slave_write_bit(unsigned char b);
void ow_slave_wait_bit(void);

unsigned char __attribute__ ((noinline)) get_char(void);
void __attribute__ ((noinline)) send_char(unsigned char c);

unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address);
void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address);


#endif // __XBOOT_H
