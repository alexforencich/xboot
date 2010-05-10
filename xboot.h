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
#include "sp_driver.h"
#include "eeprom_driver.h"

// Configuration

// bootloader entrace
#define USE_ENTER_PIN
#define USE_ENTER_DELAY
#define USE_ENTER_UART
#define USE_ENTER_I2C

// bootloader exit
//#define LOCK_SPM_ON_EXIT

// bootloader communication
#define USE_LED
#define USE_UART
#define USE_I2C
#define USE_I2C_ADDRESS_NEGOTIATION
#define USE_ATTACH_LED

// communication modes
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
#define ENTER_PIN_CTRL          ENTER_PORT.PIN0CTRL
#define ENTER_PIN_STATE         0
#define ENTER_PIN_PUEN          1

// ENTER_DELAY
#define ENTER_BLINK_COUNT       3
#define ENTER_BLINK_WAIT        30000

// LED
#define LED_PORT                PORTA
#define LED_PIN                 0
#define LED_INV                 1

// UART
#define UART_BAUD_RATE                  19200
#define UART_PORT                       PORTD
#define UART_DEVICE                     USARTD1
#define UART_TX_PIN                     PIN7_bm

#ifdef __AVR_XMEGA__

#if (F_CPU == 2000000L) && (UART_BAUD_RATE == 19200)
#define UART_BSEL_VALUE         12
#define UART_BSCALE_VALUE       0
#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 38400)
#warning BAUD rate uses BSCALE != 0
#define UART_BSEL_VALUE         12
#define UART_BSCALE_VALUE       -1
#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 115200)
#warning BAUD rate uses BSCALE != 0
#define UART_BSEL_VALUE         16
#define UART_BSCALE_VALUE       -3
#else
#warning Not using predefined BAUD rate, possible BAUD rate error!
#define UART_BSEL_VALUE         ((F_CPU + UART_BAUD_RATE * 4) / (UART_BAUD_RATE * 8) - 1)
#define UART_BSCALE_VALUE       0
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

#define I2C_MATCH_ANY                   1
#define I2C_ADDRESS                     0x10
#define I2C_GC_ENABLE                   1

#define I2C_DEVICE                      TWIE


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
