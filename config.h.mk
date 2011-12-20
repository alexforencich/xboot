
config.h: config.mk
	@echo "Generating config.h for $(MCU)"
	@echo "// XBoot config header file" > config.h
	@echo "// MCU: $(MCU)" >> config.h
	@echo "// F_CPU: $(F_CPU)" >> config.h
	@echo >> config.h
	@echo "// AVR1008" >> config.h
ifneq ($(USE_AVR1008_EEPROM), )
	@echo "#define USE_AVR1008_EEPROM" >> config.h
endif
	@echo >> config.h
	@echo "// Entry" >> config.h
ifneq ($(USE_ENTER_DELAY), )
	@echo "#define USE_ENTER_DELAY" >> config.h
endif
ifneq ($(USE_ENTER_PIN), )
	@echo "#define USE_ENTER_PIN" >> config.h
endif
ifneq ($(USE_ENTER_UART), )
	@echo "#define USE_ENTER_UART" >> config.h
endif
ifneq ($(USE_ENTER_I2C), )
	@echo "#define USE_ENTER_I2C" >> config.h
endif
ifneq ($(USE_ENTER_FIFO), )
	@echo "#define USE_ENTER_FIFO" >> config.h
endif
	@echo >> config.h
	@echo "// Exit" >> config.h
ifneq ($(LOCK_SPM_ON_EXIT), )
	@echo "#define LOCK_SPM_ON_EXIT" >> config.h
endif
	@echo >> config.h
	@echo "// Communication" >> config.h
ifneq ($(USE_LED), )
	@echo "#define USE_LED" >> config.h
endif
ifneq ($(USE_UART), )
	@echo "#define USE_UART" >> config.h
endif
ifneq ($(USE_UART_EN_PIN), )
	@echo "#define USE_UART_EN_PIN" >> config.h
endif
ifneq ($(USE_I2C), )
	@echo "#define USE_I2C" >> config.h
endif
ifneq ($(USE_I2C_ADDRESS_NEGOTIATION), )
	@echo "#define USE_I2C_ADDRESS_NEGOTIATION" >> config.h
endif
ifneq ($(USE_ATTACH_LED), )
	@echo "#define USE_ATTACH_LED" >> config.h
endif
ifneq ($(USE_FIFO), )
	@echo "#define USE_FIFO" >> config.h
endif
	@echo >> config.h
	@echo "// General Options" >> config.h
ifneq ($(USE_INTERRUPTS), )
	@echo "#define USE_INTERRUPTS" >> config.h
endif
ifneq ($(USE_WATCHDOG), )
	@echo "#define USE_WATCHDOG" >> config.h
endif
	@echo >> config.h
	@echo "// Bootloader Features" >> config.h
ifneq ($(ENABLE_BLOCK_SUPPORT), )
	@echo "#define ENABLE_BLOCK_SUPPORT" >> config.h
endif
ifneq ($(ENABLE_FLASH_BYTE_SUPPORT), )
	@echo "#define ENABLE_FLASH_BYTE_SUPPORT" >> config.h
endif
ifneq ($(ENABLE_EEPROM_BYTE_SUPPORT), )
	@echo "#define ENABLE_EEPROM_BYTE_SUPPORT" >> config.h
endif
ifneq ($(ENABLE_LOCK_BITS), )
	@echo "#define ENABLE_LOCK_BITS" >> config.h
endif
ifneq ($(ENABLE_FUSE_BITS), )
	@echo "#define ENABLE_FUSE_BITS" >> config.h
endif
ifneq ($(ENABLE_FLASH_ERASE_WRITE), )
	@echo "#define ENABLE_FLASH_ERASE_WRITE" >> config.h
endif
	@echo >> config.h
	@echo "// API" >> config.h
ifneq ($(ENABLE_API), )
	@echo "#define ENABLE_API" >> config.h
endif
ifneq ($(USE_API_VERSION), )
	@echo "#define USE_API_VERSION $(USE_API_VERSION)" >> config.h
endif
ifneq ($(ENABLE_API_LOW_LEVEL_FLASH), )
	@echo "#define ENABLE_API_LOW_LEVEL_FLASH" >> config.h
endif
ifneq ($(ENABLE_API_SPM_WRAPPER), )
	@echo "#define ENABLE_API_SPM_WRAPPER" >> config.h
endif
ifneq ($(ENABLE_API_FIRMWARE_UPDATE), )
	@echo "#define ENABLE_API_FIRMWARE_UPDATE" >> config.h
endif
	@echo >> config.h
	@echo "// ENTER_PIN" >> config.h
ifneq ($(ENTER_PORT_NAME), )
	@echo "#define ENTER_PORT_NAME $(ENTER_PORT_NAME)" >> config.h
endif
ifneq ($(ENTER_PIN), )
	@echo "#define ENTER_PIN $(ENTER_PIN)" >> config.h
endif
ifneq ($(ENTER_PIN_STATE), )
	@echo "#define ENTER_PIN_STATE $(ENTER_PIN_STATE)" >> config.h
endif
ifneq ($(ENTER_PIN_PUEN), )
	@echo "#define ENTER_PIN_PUEN $(ENTER_PIN_PUEN)" >> config.h
endif
	@echo >> config.h
	@echo "// ENTER_DELAY" >> config.h
ifneq ($(ENTER_BLINK_COUNT), )
	@echo "#define ENTER_BLINK_COUNT $(ENTER_BLINK_COUNT)" >> config.h
endif
ifneq ($(ENTER_BLINK_WAIT), )
	@echo "#define ENTER_BLINK_WAIT $(ENTER_BLINK_WAIT)" >> config.h
endif
	@echo >> config.h
	@echo "// ENTER_UART" >> config.h
ifneq ($(ENTER_UART_NEED_SYNC), )
	@echo "#define ENTER_UART_NEED_SYNC" >> config.h
endif
	@echo >> config.h
	@echo "// ENTER_FIFO" >> config.h
ifneq ($(ENTER_FIFO_NEED_SYNC), )
	@echo "#define ENTER_FIFO_NEED_SYNC" >> config.h
endif
	@echo >> config.h
	@echo "// USE_WATCHDOG" >> config.h
ifneq ($(WATCHDOG_TIMEOUT), )
	@echo "#define WATCHDOG_TIMEOUT $(WATCHDOG_TIMEOUT)" >> config.h
endif
	@echo >> config.h
	@echo "// LED" >> config.h
ifneq ($(LED_PORT_NAME), )
	@echo "#define LED_PORT_NAME $(LED_PORT_NAME)" >> config.h
endif
ifneq ($(LED_PIN), )
	@echo "#define LED_PIN $(LED_PIN)" >> config.h
endif
ifneq ($(LED_INV), )
	@echo "#define LED_INV $(LED_INV)" >> config.h
endif
	@echo >> config.h
	@echo "// UART" >> config.h
ifneq ($(UART_BAUD_RATE), )
	@echo "#define UART_BAUD_RATE $(UART_BAUD_RATE)" >> config.h
endif
ifneq ($(UART_PORT_NAME), )
	@echo "#define UART_PORT_NAME $(UART_PORT_NAME)" >> config.h
endif
ifneq ($(UART_NUMBER), )
	@echo "#define UART_NUMBER $(UART_NUMBER)" >> config.h
endif
ifneq ($(UART_U2X), )
	@echo "#define UART_U2X" >> config.h
endif
	@echo >> config.h
	@echo "// UART RS485 Enable Output" >> config.h
ifneq ($(UART_EN_PORT_NAME), )
	@echo "#define UART_EN_PORT_NAME $(UART_EN_PORT_NAME)" >> config.h
endif
ifneq ($(UART_EN_PIN), )
	@echo "#define UART_EN_PIN $(UART_EN_PIN)" >> config.h
endif
ifneq ($(UART_EN_PIN_INV), )
	@echo "#define UART_EN_PIN_INV $(UART_EN_PIN_INV)" >> config.h
endif
	@echo >> config.h
	@echo "// FIFO" >> config.h
ifneq ($(FIFO_DATA_PORT_NAME), )
	@echo "#define FIFO_DATA_PORT_NAME $(FIFO_DATA_PORT_NAME)" >> config.h
endif
ifneq ($(FIFO_CTL_PORT_NAME), )
	@echo "#define FIFO_CTL_PORT_NAME $(FIFO_CTL_PORT_NAME)" >> config.h
endif
ifneq ($(FIFO_RXF_N_bm), )
	@echo "#define FIFO_RXF_N_bm $(FIFO_RXF_N_bm)" >> config.h
endif
ifneq ($(FIFO_TXE_N_bm), )
	@echo "#define FIFO_TXE_N_bm $(FIFO_TXE_N_bm)" >> config.h
endif
ifneq ($(FIFO_RD_N_bm), )
	@echo "#define FIFO_RD_N_bm $(FIFO_RD_N_bm)" >> config.h
endif
ifneq ($(FIFO_WR_N_bm), )
	@echo "#define FIFO_WR_N_bm $(FIFO_WR_N_bm)" >> config.h
endif
ifneq ($(FIFO_BIT_REVERSE), )
	@echo "#define FIFO_BIT_REVERSE" >> config.h
endif
	@echo >> config.h
	@echo "// I2C" >> config.h
ifneq ($(I2C_DEVICE_PORT), )
	@echo "#define I2C_DEVICE_PORT $(I2C_DEVICE_PORT)" >> config.h
endif
ifneq ($(I2C_MATCH_ANY), )
	@echo "#define I2C_MATCH_ANY $(I2C_MATCH_ANY)" >> config.h
endif
ifneq ($(I2C_ADDRESS), )
	@echo "#define I2C_ADDRESS $(I2C_ADDRESS)" >> config.h
endif
ifneq ($(I2C_GC_ENABLE), )
	@echo "#define I2C_GC_ENABLE $(I2C_GC_ENABLE)" >> config.h
endif
	@echo >> config.h
	@echo "// I2C Address Autonegotiation" >> config.h
ifneq ($(I2C_AUTONEG_DIS_PROMISC), )
	@echo "#define I2C_AUTONEG_DIS_PROMISC $(I2C_AUTONEG_DIS_PROMISC)" >> config.h
endif
ifneq ($(I2C_AUTONEG_DIS_GC), )
	@echo "#define I2C_AUTONEG_DIS_GC $(I2C_AUTONEG_DIS_GC)" >> config.h
endif
ifneq ($(I2C_AUTONEG_PORT_NAME), )
	@echo "#define I2C_AUTONEG_PORT_NAME $(I2C_AUTONEG_PORT_NAME)" >> config.h
endif
ifneq ($(I2C_AUTONEG_PIN), )
	@echo "#define I2C_AUTONEG_PIN $(I2C_AUTONEG_PIN)" >> config.h
endif
	@echo >> config.h
	@echo "// Attach LED" >> config.h
ifneq ($(ATTACH_LED_PORT_NAME), )
	@echo "#define ATTACH_LED_PORT_NAME $(ATTACH_LED_PORT_NAME)" >> config.h
endif
ifneq ($(ATTACH_LED_PIN), )
	@echo "#define ATTACH_LED_PIN $(ATTACH_LED_PIN)" >> config.h
endif
ifneq ($(ATTACH_LED_PORT), )
	@echo "#define ATTACH_LED_INV $(ATTACH_LED_INV)" >> config.h
endif


