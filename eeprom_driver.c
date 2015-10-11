/************************************************************************/
/* XMEGA EEPROM Driver                                                  */
/*                                                                      */
/* eeprom.c                                                             */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2011 Alex Forencich                                    */
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

#include "eeprom_driver.h"
#include "string.h"

#ifdef __AVR_XMEGA__
#ifdef USE_AVR1008_EEPROM

#include <avr/sleep.h>

#endif // USE_AVR1008_EEPROM
#else
#include <avr/boot.h>
#include <avr/io.h>
#endif // __AVR_XMEGA__

#ifdef __AVR_XMEGA__

// NVM call
static inline void NVM_EXEC(void)
{
        void *z = (void *)&NVM_CTRLA;
        
        __asm__ volatile("out %[ccp], %[ioreg]"  "\n\t"
        "st z, %[cmdex]"
        :
        : [ccp] "I" (_SFR_IO_ADDR(CCP)),
        [ioreg] "d" (CCP_IOREG_gc),
                     [cmdex] "r" (NVM_CMDEX_bm),
                     [z] "z" (z)
                     );
}

#ifdef USE_AVR1008_EEPROM

// Interrupt handler for the EEPROM write "done" interrupt
ISR(NVM_EE_vect)
{
        // Disable the EEPROM interrupt
        NVM.INTCTRL = (NVM.INTCTRL & ~NVM_EELVL_gm);
}

// AVR1008 fix
static inline void NVM_EXEC_WRAPPER(void)
{
        // Save the Sleep register
        uint8_t sleepCtr = SLEEP.CTRL;
        // Set sleep mode to IDLE
        SLEEP.CTRL = (SLEEP.CTRL & ~SLEEP.CTRL) | SLEEP_SMODE_IDLE_gc;
        // Save the PMIC Status and control registers
        uint8_t statusStore = PMIC.STATUS;
        uint8_t pmicStore = PMIC.CTRL;
        
        // Enable only the highest level of interrupts
        PMIC.CTRL = (PMIC.CTRL & ~(PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm)) | PMIC_HILVLEN_bm;
        // Save SREG for later use
        uint8_t globalInt = SREG;
        // Enable global interrupts
        sei();
        // Set sleep enabled
        SLEEP.CTRL |= SLEEP_SEN_bm;
        // Save eeprom interrupt settings for later
        uint8_t eepromintStore = NVM.INTCTRL;
        NVM_EXEC();
        // Enable EEPROM interrupt
        NVM.INTCTRL =  NVM_EELVL0_bm | NVM_EELVL1_bm;
        // Sleep before 2.5uS has passed
        sleep_cpu();
        // Restore sleep settings
        SLEEP.CTRL = sleepCtr;
        // Restore PMIC status and control registers
        PMIC.STATUS = statusStore;
        PMIC.CTRL = pmicStore;
        // Restore EEPROM interruptsettings
        NVM.INTCTRL = eepromintStore;
        // Restore global interrupt settings
        SREG = globalInt;
}

#else

#define NVM_EXEC_WRAPPER NVM_EXEC

#endif // USE_AVR1008_EEPROM


void wait_for_nvm(void)
{
        while (NVM.STATUS & NVM_NVMBUSY_bm) { };
}

void EEPROM_erase_all(void)
{
        wait_for_nvm();
        
        NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;
        NVM_EXEC_WRAPPER();
}

#else // __AVR_XMEGA__

void EEPROM_erase_all(void)
{
    uint8_t hfuse = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
    if ((hfuse & (1 << 3)) != 0) {
        for (uint16_t i = 0; i < E2END; i++)
        {
            eeprom_update_byte((uint8_t *)i, 0xff);
        }
    }
}

#ifdef USE_ENTER_EEPROM

// Parts taken from Atmega's AVR103 App Note.

void eeprom_overwrite_byte(unsigned int addr, char value)
{
#ifdef USE_INTERRUPTS
    cli();
#endif // USE_INTERRUPTS

    do {} while(EECR & (1<<EEPE));  // Wait for completion of previous write.
    do {} while(SPMCSR & (1<<SELFPRGEN));   // Wait for SPM completion.

    EEAR = addr;        // Set EEPROM address register.
    EECR = (1<<EERE);   // Start EEPROM read operation.

    EEDR = value;       // Set EEPROM data register.
    EECR = (1<<EEMPE) | // Set Master Write Enable bit...
           (1<<EEPM1);  // ...and Write-only mode.
    EECR |= (1<<EEPE);  // Start Write-only operation.

#ifdef USE_INTERRUPTS
    sei();
#endif // USE_INTERRUPTS
}


uint8_t enter_eeprom_index;
uint8_t enter_eeprom_value;
uint8_t enter_eeprom_count;

/**
 * Check whether to enter the bootloader based on a 32-bit EEPROM value.
 * Enter the bootloader if the number of set bits is odd.  Fully erased
 * there are 32-set bits.  The program resets the highest rank set bit
 * first, and the bootloader resets the next highest order bit.  Each
 * bootloader entry requires two bits be reset.
 *
 * It is the program's, not the bootloader's responsibility to erase the
 * values when 0 is reached.
 *
 * @return 1 if an odd number of bits is set, otherwise 0.
 */
uint8_t enter_eeprom_check(void)
{
    enter_eeprom_count = 0;
    for (uint8_t i = 0; i != 4; i++)
    {
        enter_eeprom_value = eeprom_read_byte((uint8_t*) ENTER_EEPROM_ADDR + i);
        if (enter_eeprom_value != 0)
        {
            enter_eeprom_index = i;
            break;
        }
    }

    uint8_t c = enter_eeprom_value;

    // Now check bit at a time.
    while (c & 1)
    {
        enter_eeprom_count += 1;
        c >>= 1;
    }

    return enter_eeprom_count & 1;
}

void enter_eeprom_reset()
{
    // Set bits to an even value.

    if ((enter_eeprom_count & 1) == 0) return;  // Already even.
    enter_eeprom_value &= ~_BV(enter_eeprom_count - 1);
    eeprom_overwrite_byte(
        ENTER_EEPROM_ADDR + enter_eeprom_index, enter_eeprom_value);
}

#endif // USE_ENTER_EEPROM

#endif // __AVR_XMEGA__
