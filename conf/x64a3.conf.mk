# xmega64a3 configuration

# use config.h
USE_CONFIG_H = yes

# MCU
MCU = atxmega64a3

# Clock Speed
# Use 2 MHz internal RC oscillator
F_CPU = 2000000

# Programmer settings
OVERRIDE_AVRDUDE_PROGRAMMER = yes
AVRDUDE_PROGRAMMER = jtag2pdi
AVRDUDE_PORT = usb

# XBoot settings

# AVR1008 fixes
# Really only applicable to XMEGA 256a3 rev A and B devices
#USE_AVR1008_EEPROM = yes

# Entry
USE_ENTER_DELAY = yes
#USE_ENTER_PIN = yes
USE_ENTER_UART = yes
#USE_ENTER_I2C = yes
#USE_ENTER_FIFO = yes

# Exit
#LOCK_SPM_ON_EXIT = yes

# Communication
USE_LED = yes
USE_UART = yes
#USE_UART_EN_PIN = yes
#USE_I2C = yes
#USE_I2C_ADDRESS_NEGOTIATION = yes
#USE_ATTACH_LED = yes
#USE_FIFO = yes

# General Options
#USE_INTERRUPTS = yes
#USE_WATCHDOG = yes

# Bootloader Features
ENABLE_BLOCK_SUPPORT = yes
ENABLE_FLASH_BYTE_SUPPORT = yes
ENABLE_EEPROM_BYTE_SUPPORT = yes
ENABLE_LOCK_BITS = yes
ENABLE_FUSE_BITS = yes
ENABLE_FLASH_ERASE_WRITE = yes

# API
ENABLE_API = yes
USE_API_VERSION = 1
ENABLE_API_LOW_LEVEL_FLASH = yes
ENABLE_API_SPM_WRAPPER = yes
ENABLE_API_FIRMWARE_UPDATE = yes

# ENTER_PIN
ENTER_PORT_NAME       = C
ENTER_PIN             = 0
ENTER_PIN_STATE       = 0
ENTER_PIN_PUEN        = 1

# ENTER_DELAY
ENTER_BLINK_COUNT     = 3
ENTER_BLINK_WAIT      = 30000

# ENTER_UART
#ENTER_UART_NEED_SYNC = yes

# ENTER_FIFO
#ENTER_FIFO_NEED_SYNC = yes

# USE_WATCHDOG
# Select only one
#WATCHDOG_TIMEOUT      = WDT_PER_8CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_16CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_32CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_64CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_128CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_256CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_512CLK_gc
WATCHDOG_TIMEOUT      = WDT_PER_1KCLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_2KCLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_4KCLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_8KCLK_gc

# LED
LED_PORT_NAME         = A
LED_PIN               = 0
LED_INV               = 1

# UART
# Select BAUD rate, port name, and UART number
UART_BAUD_RATE        = 115200
UART_PORT_NAME        = C
UART_NUMBER           = 0

# UART RS485 Enable Output
UART_EN_PORT_NAME     = C
UART_EN_PIN           = 4
UART_EN_PIN_INV       = 0

# FIFO
FIFO_DATA_PORT_NAME   = C
FIFO_CTL_PORT_NAME    = D
FIFO_RXF_N_bm         = (1<<3)
FIFO_TXE_N_bm         = (1<<2)
FIFO_RD_N_bm          = (1<<1)
FIFO_WR_N_bm          = (1<<0)
FIFO_BIT_REVERSE = yes

# I2C
I2C_DEVICE_PORT       = C

I2C_MATCH_ANY         = 1
I2C_ADDRESS           = 0x10
I2C_GC_ENABLE         = 1

# I2C Address Autonegotiation
# Note: only works on XMega chips for the time being
# There is no easy way to get this to work on regular
# ATMega chips as they have no unique part ID number
I2C_AUTONEG_DIS_PROMISC       = 1
I2C_AUTONEG_DIS_GC            = 0
I2C_AUTONEG_PORT_NAME         = A
I2C_AUTONEG_PIN               = 2

# Attach LED
ATTACH_LED_PORT_NAME          = A
ATTACH_LED_PIN                = 1
ATTACH_LED_INV                = 1



