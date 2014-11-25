/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* Watchdog Module                                                      */
/*                                                                      */
/* watchdog.c                                                           */
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

#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include "xboot.h"

#ifdef __AVR_XMEGA__
// Globals


// Defines

/*! \brief Check if Synchronization busy flag is set. */
#define WDT_IsSyncBusy() ( WDT.STATUS & WDT_SYNCBUSY_bm )
#define WDT_Reset()     asm("wdr")

// Prototypes
extern void WDT_EnableAndSetTimeout( void );
extern void WDT_Disable( void );

#else // __AVR_XMEGA__

// ATMEGA -- no sense calling a function for these.
#include <avr/wdt.h>

#define WDT_IsSyncBusy() (0)
#define WDT_Reset() wdt_reset()
#define WDT_EnableAndSetTimeout() wdt_enable(WATCHDOG_TIMEOUT)
#define WDT_Disable() wdt_disable()

#endif // __AVR_XMEGA__

#endif // __WATCHDOG_H
