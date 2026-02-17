CC	:= avr-gcc
LD	:= avr-ld
OBJCOPY	:= avr-objcopy
OBJDUMP	:= avr-objdump
SIZE	:= avr-size

TARGET = twiboot
SOURCE = $(wildcard *.c)

# select MCU
MCU = atmega2560

AVRDUDE_PROG := -c usbtiny -b 115200 -P /dev/ttyUSB0
#AVRDUDE_PROG := -c dragon_isp -P usb



# ---------------------------------------------------------------------------

ifeq ($(MCU), atmega8)
# atmega8:
# Fuse L: 0x84 (8Mhz internal RC-Osz., 2.7V BOD)
# Fuse H: 0xda (512 words bootloader)
AVRDUDE_MCU=m8
AVRDUDE_FUSES=lfuse:w:0x84:m hfuse:w:0xda:m

BOOTLOADER_START=0x1C00
endif

ifeq ($(MCU), atmega88)
# atmega88:
# Fuse L: 0xc2 (8Mhz internal RC-Osz.)
# Fuse H: 0xdd (2.7V BOD)
# Fuse E: 0xfa (512 words bootloader)
AVRDUDE_MCU=m88
AVRDUDE_FUSES=lfuse:w:0xc2:m hfuse:w:0xdd:m efuse:w:0xfa:m

BOOTLOADER_START=0x1C00
endif

ifeq ($(MCU), atmega168)
# atmega168:
# Fuse L: 0xE2 (external crystal 8–16 MHz, CKDIV8 off)
# Fuse H: 0xD9 (BOOTRST enabled, 2 KB bootloader)
# Fuse E: 0xF9 (BOD unchanged)
AVRDUDE_MCU=m168 -F
AVRDUDE_FUSES=lfuse:w:0xE2:m hfuse:w:0xD9:m efuse:w:0xF9:m

BOOTLOADER_START=0x3800
endif


ifeq ($(MCU), atmega328p)
# atmega328p:
# Fuse L: 0xc2 (8Mhz internal RC-Osz.)
# Fuse H: 0xdc (512 words bootloader)
# Fuse E: 0xfd (2.7V BOD)
AVRDUDE_MCU=m328p
AVRDUDE_FUSES=lfuse:w:0xc2:m hfuse:w:0xdc:m efuse:w:0xfd:m

BOOTLOADER_START=0x7C00
endif

ifeq ($(MCU), atmega644p)
# atmega644p:
# Fuse L: 0xFB (8MHz external crystal)
# Fuse H: 0xD8 (BOOTRST, 8KB bootloader, SPI enabled)
# Fuse E: 0xFD (BOD disabled)
AVRDUDE_MCU=m644p
AVRDUDE_FUSES=lfuse:w:0xFB:m hfuse:w:0xD8:m efuse:w:0xFD:m

BOOTLOADER_START=0xF000
endif

ifeq ($(MCU), atmega1284p)
# atmega644p:
# Fuse L: 0xFB (8MHz external crystal)
# Fuse H: 0xD8 (BOOTRST, 8KB bootloader, SPI enabled)
# Fuse E: 0xFD (BOD disabled)
AVRDUDE_MCU=m644p
AVRDUDE_FUSES=lfuse:w:0xFF:m hfuse:w:0xD8:m efuse:w:0xFD:m

BOOTLOADER_START=0x1F000
endif

ifeq ($(MCU), atmega2560)
# atmega2560:
# Fuse L: 0xFF (external crystal, 16 MHz)
# Fuse H: 0xDA (BOOTRST, 4KB bootloader, SPI enabled)
# Fuse E: 0xFD (BOD enabled, 2.7V)
AVRDUDE_MCU=m2560
AVRDUDE_FUSES=lfuse:w:0xFF:m hfuse:w:0xDA:m efuse:w:0xFD:m
BOOTLOADER_START=0x3F800
endif



ifeq ($(MCU), attiny85)
# attiny85:
# Fuse L: 0xe2 (8Mhz internal RC-Osz.)
# Fuse H: 0xdd (2.7V BOD)
# Fuse E: 0xfe (self programming enable)
AVRDUDE_MCU=t85
AVRDUDE_FUSES=lfuse:w:0xe2:m hfuse:w:0xdd:m efuse:w:0xfe:m

BOOTLOADER_START=0x1C00
CFLAGS_TARGET=-DUSE_CLOCKSTRETCH=1 -DVIRTUAL_BOOT_SECTION=1
endif

# ---------------------------------------------------------------------------

CFLAGS = -pipe -g -Os -mmcu=$(MCU) -Wall -fdata-sections -ffunction-sections
CFLAGS += -Wa,-adhlns=$(*F).lst -DBOOTLOADER_START=$(BOOTLOADER_START) $(CFLAGS_TARGET)
LDFLAGS = -Wl,-Map,$(@:.elf=.map),--cref,--relax,--gc-sections,--section-start=.text=$(BOOTLOADER_START)
LDFLAGS += -nostartfiles

# ---------------------------------------------------------------------------

$(TARGET): $(TARGET).elf
	@$(SIZE) -B -x --mcu=$(MCU) $<

$(TARGET).elf: $(SOURCE:.c=.o)
	@echo " Linking file:  $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	@$(OBJDUMP) -h -S $@ > $(@:.elf=.lss)
	@$(OBJCOPY) -j .text -j .data -O ihex $@ $(@:.elf=.hex)
	@$(OBJCOPY) -j .text -j .data -O binary $@ $(@:.elf=.bin)

%.o: %.c $(MAKEFILE_LIST)
	@echo " Building file: $<"
	@$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -rf $(SOURCE:.c=.o) $(SOURCE:.c=.lst) $(addprefix $(TARGET), .elf .map .lss .hex .bin)

install: $(TARGET).elf
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U flash:w:$(<:.elf=.hex)

fuses:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(patsubst %,-U %, $(AVRDUDE_FUSES))
