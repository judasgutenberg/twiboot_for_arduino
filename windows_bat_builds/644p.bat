@echo off
REM === Compile AVR bootloader ===

REM Stop script if any command fails
setlocal enabledelayedexpansion
set ERR=0

echo Compiling main.c to object file...
avr-gcc.exe -mmcu=atmega644p -Os -c main.c -o twiboot.o
if errorlevel 1 (
    echo ERROR: Compilation failed!
    set ERR=1
)

if !ERR! neq 0 exit /b !ERR!

echo Linking object file to ELF at 0xF000...
avr-gcc.exe -mmcu=atmega644p  -Wl,-Ttext=0xF000 twiboot.o -o twiboot.elf
if errorlevel 1 (
    echo ERROR: Linking failed!
    set ERR=1
)

if !ERR! neq 0 exit /b !ERR!

echo Generating HEX file...
avr-objcopy.exe -O ihex -R .eeprom twiboot.elf twiboot.hex
if errorlevel 1 (
    echo ERROR: HEX generation failed!
    set ERR=1
)

if !ERR! neq 0 exit /b !ERR!

echo SUCCESS: twiboot.hex generated!
