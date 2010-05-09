/*******************************************************************/
/* uBoot Extensible AVR Bootloader                                 */
/*                                                                 */
/* tested with ATXMEGA64A3                                         */
/*                                                                 */
/* uboot.c                                                         */
/*                                                                 */
/* Alex Forencich <alex@alexforencich.com>                         */
/*                                                                 */
/*******************************************************************/

#include "uboot.h"

unsigned char comm_mode;

int main(void)
{
        ADDR_T address = 0;
        unsigned char in_bootloader = 0;
        unsigned char val = 0;
        int i, j, k;
        void (*reset_vect)( void ) = 0x000000;
        
        comm_mode = MODE_UNDEF;
        
        // Initialization section
        // Entry point and communication methods are initialized here
        // --------------------------------------------------
        
        #ifdef USE_LED
        // Initialize LED pin
        LED_PORT.DIR = (1 << LED_PIN);
        LED_PORT.OUT = (1 << LED_PIN);
        #endif // USE_LED
        
        #ifdef USE_ENTER_PIN
        #if ENTER_PIN_PUEN
        // Enable bootloader entry pin pullup
        ENTER_PIN_CTRL = 0x18;
        #endif
        #endif
        
        #ifdef USE_UART
        // Initialize UART
        #ifdef __AVR_XMEGA__
        UART_PORT.DIRSET |= UART_TX_PIN;
        UART_DEVICE.BAUDCTRLA = (UART_BSEL_VALUE & USART_BSEL_gm);
        UART_DEVICE.BAUDCTRLB = ((UART_BSCALE_VALUE << USART_BSCALE_gp) & USART_BSCALE_gm);
        UART_DEVICE.CTRLB = USART_RXEN_bm | USART_CLK2X_bm | USART_TXEN_bm;
        #endif // __AVR_XMEGA__

        #endif // USE_UART
        
        #ifdef USE_I2C
        // Initialize I2C interface
        // TODO
        #endif // USE_I2C
        
        #ifdef USE_SPI
        // Initialize SPI interface
        // TODO
        #endif // USE_SPI
        
        
        // --------------------------------------------------
        // End initialization section
        
        // One time trigger section
        // Triggers that are checked once, regardless of
        // whether or not USE_ENTER_DELAY is selected
        // --------------------------------------------------
        
        
        
        // --------------------------------------------------
        // End one time trigger section
        
#ifdef USE_ENTER_DELAY
        k = ENTER_BLINK_COUNT*2;
        j = ENTER_BLINK_WAIT;
        while (!in_bootloader && k > 0)
        {
                if (j-- <= 0)
                {
                        #ifdef USE_LED
                        LED_PORT.OUT ^= (1 << LED_PIN);
                        #endif // USE_LED
                        j = ENTER_BLINK_WAIT;
                        k--;
                }
#else // USE_ENTER_DELAY
                // Need a small delay when not running loop
                // so we don't accidentally enter the bootloader
                // on power-up with USE_ENTER_PIN selected
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
#endif // USE_ENTER_DELAY
                
                // Main trigger section
                // Set in_bootloader here to enter the bootloader
                // Checked when USE_ENTER_DELAY is selected
                // --------------------------------------------------
                
                #ifdef USE_ENTER_PIN
                // Check entry pin state
                if ((ENTER_PORT.IN & (1 << ENTER_PIN)) == (ENTER_PIN_STATE ? (1 << ENTER_PIN) : 0))
                        in_bootloader = 1;
                #endif // USE_ENTER_PIN
                
                #ifdef USE_ENTER_UART
                // Check for received character
                #ifdef __AVR_XMEGA__
                if (UART_DEVICE.STATUS & USART_RXCIF_bm)
                {
                        in_bootloader = 1;
                        comm_mode = MODE_UART;
                }
                #endif // __AVR_XMEGA__
                
                #endif // USE_ENTER_UART
                
                #ifdef USE_ENTER_I2C
                // Check for received character
                // TODO
                #endif // USE_ENTER_I2C
                
                #ifdef USE_ENTER_SPI
                // Check for received character
                // TODO
                #endif // USE_ENTER_SPI
                
                // --------------------------------------------------
                // End main trigger section
                
        #ifdef USE_ENTER_DELAY
        }
        #endif // USE_ENTER_DELAY
        
        // Main bootloader
        
        while (in_bootloader) {
                #ifdef USE_LED
                LED_PORT.OUT ^= (1 << LED_PIN);
                #endif // USE_LED
                
                val = get_char();
                
                // Main bootloader parser
                // check autoincrement status
                if (val == 'a')
                {
                        // yes, it is supported
                        send_char('Y');
                }
                // Set address
                else if (val == 'A')
                {
                        // Read address high then low
                        address = (get_char() << 8) | get_char();
                        // acknowledge
                        send_char('\r');
                }
                // Extended address
                else if (val == 'H')
                {
                        // Read address high then low
                        address = (get_char() << 16) | (get_char() << 8) | get_char();
                        // acknowledge
                        send_char('\r');
                }
                // Chip erase
                else if (val == 'e')
                {
                        for (address = 0; address < APP_SECTION_SIZE; address += APP_SECTION_PAGE_SIZE)
                        {
                                // wait for SPM instruction to complete
                                SP_WaitForSPM();
                                // erase page
                                SP_EraseApplicationPage(address);
                        }
                        
                        // Randomize page buffer
                        EEPROM_LoadPage(&val);
                        // Erase EEPROM
                        EEPROM_EraseAll();
                        
                        // acknowledge
                        send_char('\r');
                }
                // Check block load support
                else if (val == 'b')
                {
                        // yes, it is supported
                        send_char('Y');
                        // Send block size (page size)
                        send_char((APP_SECTION_PAGE_SIZE >> 8) & 0xFF);
                        send_char(APP_SECTION_PAGE_SIZE & 0xFF);
                }
                // Block load
                else if (val == 'B')
                {
                        // Block size
                        i = (get_char() << 8) | get_char();
                        // Memory type
                        val = get_char();
                        // Load it
                        send_char(BlockLoad(i, val, &address));
                }
                // Block read
                else if (val == 'g')
                {
                        // Block size
                        i = (get_char() << 8) | get_char();
                        // Memory type
                        val = get_char();
                        // Read it
                        BlockRead(i, val, &address);
                }
                // Read program memory byte
                else if (val == 'R')
                {
                        SP_WaitForSPM();
                        send_char(SP_ReadByte((address << 1)+1));
                        send_char(SP_ReadByte((address << 1)+0));
                        
                        address++;
                }
                // Write program memory low byte
                else if (val == 'c')
                {
                        // get low byte
                        i = get_char();
                        send_char('\r');
                }
                // Write program memory high byte
                else if (val == 'C')
                {
                        // get high byte; combine
                        i |= (get_char() << 8);
                        SP_WaitForSPM();
                        SP_LoadFlashWord((address << 1), i);
                        address++;
                        send_char('\r');
                }
                // Write page
                else if (val == 'm')
                {
                        if (address >= (APP_SECTION_SIZE>>1))
                        {
                                // don't allow bootloader overwrite
                                send_char('?');
                        }
                        else
                        {
                                SP_WaitForSPM();
                                SP_WriteApplicationPage( address << 1);
                                send_char('\r');
                        }
                }
                // Write EEPROM memory
                else if (val == 'D')
                {
                        EEPROM_WriteByte( (unsigned char)(address / EEPROM_PAGE_SIZE), (unsigned char) (address & EEPROM_BYTE_ADDRESS_MASK), get_char() );
                        address++;
                }
                // Read EEPROM memory
                else if (val == 'd')
                {
                        send_char( EEPROM_ReadByte( (unsigned char)(address / EEPROM_PAGE_SIZE), (unsigned char) (address & EEPROM_BYTE_ADDRESS_MASK) ) );
                        address++;
                }
                // Write lockbits
                else if (val == 'l')
                {
                        SP_WaitForSPM();
                        SP_WriteLockBits( get_char() );
                        send_char('\r');
                }
                // Read lockbits
                else if (val == 'r')
                {
                        SP_WaitForSPM();
                        send_char(SP_ReadLockBits());
                }
                // Read low fuse bits
                else if (val == 'F')
                {
                        SP_WaitForSPM();
                        send_char(SP_ReadFuseByte(0));
                }
                // Read high fuse bits
                else if (val == 'N')
                {
                        SP_WaitForSPM();
                        send_char(SP_ReadFuseByte(1));
                }
                // Read extended fuse bits
                else if (val == 'Q')
                {
                        SP_WaitForSPM();
                        send_char(SP_ReadFuseByte(2));
                }
                // Enter and leave programming mode
                else if ((val == 'P') || (val == 'L'))
                {
                        // just acknowledge
                        send_char('\r');
                }
                // Exit bootloader
                else if (val == 'E')
                {
                        in_bootloader = 0;
                        send_char('\r');
                }
                // Get programmer type
                else if (val == 'p')
                {
                        // serial
                        send_char('S');
                }
                // Return supported device codes
                else if (val == 't')
                {
                        // send only this device
                        send_char(123); // TODO
                        // terminator
                        send_char(0);
                }
                // Set LED, clear LED, and set device type
                else if ((val == 'x') || (val == 'y') || (val == 'T'))
                {
                        // discard parameter
                        get_char();
                        send_char('\r');
                }
                // Return program identifier
                else if (val == 'S')
                {
                        send_char('A');
                        send_char('V');
                        send_char('R');
                        send_char('B');
                        send_char('O');
                        send_char('O');
                        send_char('T');
                }
                // Read software version
                else if (val == 'V')
                {
                        send_char('1');
                        send_char('6');
                }
                // Read signature bytes
                else if (val == 's')
                {
                        send_char(SIGNATURE_2);
                        send_char(SIGNATURE_1);
                        send_char(SIGNATURE_0);
                }
                // ESC to exit bootloader
                else if (val == 0x1b)
                {
                        // synchronize (do nothing)
                }
                // otherwise, error
                else
                {
                        send_char('?');
                }
        }
        
        // Wait for any lingering SPM instructions to finish
        SP_WaitForSPM();
        
        #ifdef LOCK_SPM_ON_EXIT
        // Lock SPM writes
        SP_LockSPM();
        #endif // LOCK_SPM_ON_EXIT
        
        #ifdef USE_LED
        // Turn off LED on exit
        LED_PORT.DIR &= ~(1 << LED_PIN);
        #endif // USE_LED
        
        // Jump into main code
        EIND = 0x00;
        reset_vect();
}

unsigned char __attribute__ ((noinline)) get_char(void)
{
        while (1)
        {
                #ifdef USE_UART
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_UART)
                {
                        #ifdef __AVR_XMEGA__
                        if (UART_DEVICE.STATUS & USART_RXCIF_bm)
                        {
                                comm_mode = MODE_UART;
                                return UART_DEVICE.DATA;
                        }
                        #endif // __AVR_XMEGA__
                }
                #endif // USE_UART
                
                #ifdef USE_I2C
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_UART)
                {
                        #ifdef __AVR_XMEGA__
                        // TODO
                        #endif // __AVR_XMEGA__
                }
                #endif // USE_I2C
                
                #ifdef USE_SPI
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_UART)
                {
                        #ifdef __AVR_XMEGA__
                        if (UART_DEVICE.STATUS & USART_RXCIF_bm)
                        // TODO
                        #endif // __AVR_XMEGA__
                }
                #endif // USE_SPI
        }
        
        return 0;
}

void __attribute__ ((noinline)) send_char(unsigned char c)
{
        #ifdef USE_UART
        // Send character
        if (comm_mode == MODE_UNDEF || comm_mode == MODE_UART)
        {
                #ifdef __AVR_XMEGA__
                UART_DEVICE.DATA = c;
                while (!(UART_DEVICE.STATUS & USART_TXCIF_bm)) { }
                UART_DEVICE.STATUS |= USART_TXCIF_bm; // clear flag
                #endif // __AVR_XMEGA__
                
        }
        #endif // USE_UART
        
        #ifdef USE_I2C
        // Send character
        if (comm_mode == MODE_UNDEF || comm_mode == MODE_I2C)
        {
                // TODO
        }
        #endif // USE_I2C
        
        #ifdef USE_SPI
        // Send character
        if (comm_mode == MODE_UNDEF || comm_mode == MODE_SPI)
        {
                // TODO
        }
        #endif // USE_SPI
}

unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address)
{
        unsigned int data;
        ADDR_T tempaddress;
        
        // EEPROM memory type.
        if(mem == 'E')
        {
                unsigned char pageAddr, byteAddr, value;
                unsigned char buffer[APP_SECTION_PAGE_SIZE];
                
                EEPROM_FlushBuffer();
                // disable mapping of EEPROM into data space (enable IO mapped access)
                EEPROM_DisableMapping();
                
                // Fill buffer first, as EEPROM is too slow to copy with UART speed 
                for(tempaddress=0;tempaddress<size;tempaddress++){
                        buffer[tempaddress] = get_char();
                }
                
                // Then program the EEPROM
                for( tempaddress=0; tempaddress < size; tempaddress++)
                {
                        // void EEPROM_WriteByte( uint8_t pageAddr, uint8_t byteAddr, uint8_t value )
                        pageAddr = (unsigned char)( (*address) / EEPROM_PAGE_SIZE);
                        byteAddr = (unsigned char)( (*address) & EEPROM_BYTE_ADDRESS_MASK);
                        value = buffer[tempaddress];
                        
                        EEPROM_WriteByte(pageAddr, byteAddr, value);
                        
                        (*address)++; // Select next EEPROM byte
                }
                
                return '\r'; // Report programming OK
        } 
        
        // Flash memory type
        else if (mem == 'F')
        {
                // NOTE: For flash programming, 'address' is given in words.
                (*address) <<= 1; // Convert address to bytes temporarily.
                tempaddress = (*address);  // Store address in page.
                
                do
                {
                        data = get_char();
                        data |= (get_char() << 8);
                        SP_LoadFlashWord(*address, data);
                        (*address)+=2; // Select next word in memory.
                        size -= 2; // Reduce number of bytes to write by two.
                } while(size); // Loop until all bytes written.
                
                SP_WriteApplicationPage(tempaddress);
                
                SP_WaitForSPM();
                (*address) >>= 1; // Convert address back to Flash words again.
                return '\r'; // Report programming OK
        }

        // Invalid memory type?
        else
        {
                return '?';
        }
}



void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address)
{
        // EEPROM memory type.
        
        if (mem == 'E') // Read EEPROM
        {
                unsigned char byteAddr, pageAddr;
                
                EEPROM_DisableMapping();
                EEPROM_FlushBuffer();
                
                do
                {
                        pageAddr = (unsigned char)(*address / EEPROM_PAGE_SIZE);
                        byteAddr = (unsigned char)(*address & EEPROM_BYTE_ADDRESS_MASK);
                        
                        send_char( EEPROM_ReadByte( pageAddr, byteAddr ) );
                        // Select next EEPROM byte
                        (*address)++;
                        size--; // Decrease number of bytes to read
                } while (size); // Repeat until all block has been read
        }
        
        // Flash memory type.
        else if (mem == 'F')
        {
                (*address) <<= 1; // Convert address to bytes temporarily.
                
                do
                {
                        send_char( SP_ReadByte( *address) );
                        send_char( SP_ReadByte( (*address)+1) );
                        SP_WaitForSPM();
                        
                        (*address) += 2;    // Select next word in memory.
                        size -= 2;          // Subtract two bytes from number of bytes to read
                } while (size);         // Repeat until all block has been read
                
                (*address) >>= 1;       // Convert address back to Flash words again.
        }
}


