# twiboot - a TWI / I2C bootloader for AVR MCUs ##
twiboot is a simple/small bootloader for AVR MCUs written in C. It uses the integrated TWI or USI peripheral of the controller to implement a I2C slave.
It was originally created to update I2C controlled BLMCs (Brushless Motor Controller) without an AVR ISP adapter.

twiboot acts as a slave device on a TWI/I2C bus and allows reading/writing of the internal flash memory.
As a compile time option (EEPROM_SUPPORT) twiboot also allows reading/writing of the whole internal EEPROM memory.
The bootloader is not able to update itself (only application flash memory region accessible).

## Modifications by Gus Mueller (February 5, 2026) ##
This version only requires an I2C connection to a master to reflash the sketch area of the slave's flash. The idea here is to be able to have an Arduino sketch running on some fairly powerful internet-capable microcontroller (such as an ESP8266) that can, while also logging data and controlling relays, re-flash the sketch running on an AVR Arduino slave being used as an I2C port expander using I2C alone.  In, for example, <a href=https://github.com/judasgutenberg/Esp8266_RemoteControl>my ESP8266 Remote Control system</a>, this allows me to not only trigger remote OTA updates of my distant microcontrollers, but also any connected I2C slaves.  My I2C slaves started out as simple port expanders but now do all sorts of complicated tasks and it's good to be able to fix bugs on them not only in-circuit but also from anywhere on the internet without being forced to use an interface they don't ordinarily use.

Support is provided in my <a href=https://github.com/judasgutenberg/Arduino_I2C_Slave_With_Commands  target=Arduino>Arduino Slave With Commands sketch</a> to jump into this bootloader directly so that the master can then send the data necessary to reflash it.
This code uses two bytes beginning at EEPROM address 510 (decimal) to pass a "stay in bootloader"
state from the sketch to the bootloader so that the master can then send the new flash image.  When that is finished, the slave will boot back into the sketch if it can be run. This all happens entirely over I2C.  There is a risk that if power should fail during reflashing you may have to rescue the slave with a programmer such as a USBTiny, so keep this in mind in your mission-critical applications.  Obviously, any new firmware flashed this way will have to be twiboot-aware for the slave to be able to switch back into the bootloader at will for further updates.

The files slaveupdate.cpp and slaveupdate.h contain a library of functions to run on an ESP8266 master, which will allow that master to stream a chunked hex image file from a web server to the slave while this bootloader is running on it.  

The original twiboot didn't have built-in support for chunked data (that is, data in packets significantly smaller than the 128 byte page size of an Atmega328p).  Such chunking is essential if one is using most Arduino I2C libraries, which impose a 32 byte limit on I2C transfers.  Chunking is configurable from the master end, and, after much trial and error, I settled on 16 byte chunks.  The 32 byte Arduino I2C packet limitation cannot all be used for data, as there is some overhead with those packets. I would not attempt to use chunks larger than 24 bytes.

For now this version is bulky and requires at least a 2k bootloader partition in the flash. If you set UART_DEBUG to 1, you will require a 4k boot partition. But the UART is really only good for debugging; if it is enabled, the bootloader fails about half the time. Otherwise the bootloader is extremely reliable. I have tested it by going back and forth between two different firmware versions dozens of times and the correct firmware boots up every time without failure. Using my ESP8266 Remote Master to flash a firmware hosted on an Apache server, it typically takes 30 seconds to flash a 12kilobyte Atmega328p firmware.

I have only tested this new version on Atmega328ps and Atmega644ps and am skeptical that AVRs with I2C emulated by bit-banging will work, though apparently the original twiboot supported that.

## Devices Supported in This Version ##
Currently the following AVR MCUs are supported:

AVR MCU | Flash bytes used (.text + .data) | Bootloader region size
--- | --- | ---
atmega328p | 1616 (0x650) | 1024 words
atmeg644p | 1644 (0x66C) | 1024 words

(Compiled on Windows 10 with EEPROM and LED support)


## Operation ##
twiboot is installed in the bootloader section and executed directly after reset (BOOTRST fuse is programmed). Normally control is immediately handed to the sketch (application). But if a magic value is found in the two bytes starting at EEPROM location 510 (decimal), then the bootloader waits to receive bytes to flash from the master.  Eventually this will timeout if no such data is forthcoming and control will be handed back to the sketch.

For MCUs without bootloader section see [Virtual bootloader section](#virtual-bootloader-section) below.

While running, twiboot configures the TWI/USI peripheral as slave device and waits for valid protocol messages
directed to its address on the TWI/I2C bus. The slave address is configured during compile time of twiboot.
When receiving no messages for 1000ms after reset, the bootloader exits and executes the main application at address 0x0000.

A TWI/I2C master can use the protocol to
- abort the boot timeout
- query information about the device (bootloader version, AVR signature bytes, flash/eeprom size, flash page size)
- read internal flash / eeprom memory (byte wise)
- write the internal flash (page wise)
- write the internal eeprom (byte wise)
- exit the bootloader and start the application

As a compile time option (LED_SUPPORT) twiboot can output its state with two LEDs.
One LED will flash with a frequency of 20Hz while twiboot is active (including boot wait time).
A second LED will flash when the bootloader is addressed on the TWI/I2C bus.


### Virtual Bootloader Section ###
For MCUs without bootloader section twiboot will patch the vector table on the fly during flash programming to stay active.
The reset vector is patched to execute twiboot instead of the application code.

Another vector entry will be patched to store the original entry point of the application.
This vector entry is overridden and MUST NOT be used by the application.
twiboot uses this vector to start the application after the initial timeout.

This live patching changes the content of the vector table, which would result in a verification error after programming.
To counter this kind of error, twiboot caches the original vector table entries in RAM and return those on a read command.
The real content of the vector table is only returned after a reset.


## Build and install twiboot ##
twiboot uses gcc, avr-libc and GNU Make for building, avrdude is used for flashing the MCU.
The build and install procedures are only tested under linux.

The selection of the target MCU and the programming interface can be found in the Makefile,
TWI/I2C slave address and optional components (EEPROM / LED support) are configured
in the main.c source.

To build twiboot for the selected target:
``` shell
$ make
```

To install (flash download) twiboot with avrdude on the target:
``` shell
$ make install
```

Set AVR fuses with avrdude on the target (internal RC-Osz, enable BOD, enable BOOTRST):
``` shell
$ make fuses
```


## TWI/I2C Protocol ##
A TWI/I2C master can use the following protocol for accessing the bootloader.

Function | TWI/I2C data | Comment
--- | --- | ---
Abort boot timeout | **SLA+W**, 0x00, **STO** |
Show bootloader version | **SLA+W**, 0x01, **SLA+R**, {16 bytes}, **STO** | ASCII, not null terminated
Start application | **SLA+W**, 0x01, 0x80, **STO** |
Read chip info | **SLA+W**, 0x02, 0x00, 0x00, 0x00, **SLA+R**, {8 bytes}, **STO** | 3byte signature, 1byte page size, 2byte flash size, 2byte eeprom size
Read 1+ flash bytes | **SLA+W**, 0x02, 0x01, addrh, addrl, **SLA+R**, {* bytes}, **STO** |
Read 1+ eeprom bytes | **SLA+W**, 0x02, 0x02, addrh, addrl, **SLA+R**, {* bytes}, **STO** |
Write one flash page | **SLA+W**, 0x02, 0x01, addrh, addrl, {* bytes}, **STO** | page size as indicated in chip info
Write 1+ eeprom bytes | **SLA+W**, 0x02, 0x02, addrh, addrl, {* bytes}, **STO** | write 0 < n < page size bytes at once

**SLA+R** means Start Condition, Slave Address, Read Access

**SLA+W** means Start Condition, Slave Address, Write Access

**STO** means Stop Condition

A flash page / eeprom write is only triggered after the Stop Condition.
During the write process twiboot will NOT acknowledge its slave address.

The multiboot_tool repository contains a simple linux application that uses
this protocol to access the bootloader over linux i2c device.

The ispprog programming adapter can also be used as a avr910/butterfly to twiboot protocol bridge.


## TWI/I2C Clockstretching ##
While a write is in progress twiboot will not respond on the TWI/I2C bus and the
TWI/I2C master needs to retry/poll the slave address until the write has completed.

As a compile time option (USE_CLOCKSTRETCH) the previous behavior of twiboot can be restored:
TWI/I2C Clockstretching is then used to inform the master of the duration of the write.
Please note that there are some TWI/I2C masters that do not support clockstretching.


## Development ##
Issue reports, feature requests, patches or simply success stories are much appreciated.
