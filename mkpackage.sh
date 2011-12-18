#!/bin/bash

files="README
Makefile
xboot.c
xboot.h
flash.c
flash.h
eeprom_driver.c
eeprom_driver.h
protocol.h
sp_driver.S
sp_driver.h
uart.c
uart.h
i2c.c
i2c.h
fifo.c
fifo.h
watchdog.c
watchdog.h
api.c
api.h
xbootapi.c
xbootapi.h"

name=xboot

output=xboot-$(date +%Y%m%d).tar.gz

mkdir -p pkg/$name
rm -rf pkg/$name/*
cp -r $files pkg/$name

cd pkg
tar -cvzf ../$output xboot

