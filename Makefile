/***************************************************************************
 *   Copyright (C) 10/2020 by Olaf Rempel                                  *
 *   razzor@kopf-tisch.de                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License,               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             +
 +                                                                         +
 +    This version was modified by Gus Mueller Jan 2026-Feb 2026           +
 +    And has been successfully tested on an Atmega328p                    +
 +    and several other AVRs                                               +
 +    The necessary connection required to do the master-coordinated       +
 +    reflash is the I2C signals and ground.  State is passed to the       +
 +    bootloader through a byte set at 510 (decimal) in the AVR's EEPROM   +
 +    Contains serial functionality that helped me debug it.               +
 +                                                                         +
 +                                                                         +
 +                                                                         +
 +                                                                         +
 ***************************************************************************/
 
//make sure you provide F_CPU in the build

#define BAUD 115200
#define UBRR_VALUE ((F_CPU / (8UL * BAUD)) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <util/twi.h>
 
#define LED_PIN PB5

//use 7000 for atmega328p, f000 for atmega644p, for an atmega168
#ifndef BOOTLOADER_START
#define BOOTLOADER_START 0x3800
#endif

#ifndef UART_DEBUG
#define UART_DEBUG 0
#endif

#ifndef CLOCKED_SERIAL_DEBUG
#define CLOCKED_SERIAL_DEBUG 0
#endif

#define VERSION_STRING          "TWIBOOT v3.3 NR"
#define EEPROM_SUPPORT          0
#define LED_SUPPORT             0

#ifndef USE_CLOCKSTRETCH
#define USE_CLOCKSTRETCH        0
#endif

#ifndef VIRTUAL_BOOT_SECTION
#define VIRTUAL_BOOT_SECTION    0
#endif

#ifndef TWI_ADDRESS
#define TWI_ADDRESS             20
#endif


#define TIMER_DIVISOR        256
#define TIMER_IRQFREQ_MS     25

#define TIMER_MSEC2TICKS(x)  ((uint8_t)(((x) * F_CPU) / (TIMER_DIVISOR * 1000ULL)))
#define TIMER_MSEC2IRQCNT(x) ((x) / TIMER_IRQFREQ_MS)


#if (LED_SUPPORT)
#define LED_INIT()              DDRB = ((1<<PORTB4) | (1<<PORTB5))
#define LED_RT_ON()             PORTB |= (1<<PORTB4)
#define LED_RT_OFF()            PORTB &= ~(1<<PORTB4)
#define LED_GN_ON()             PORTB |= (1<<PORTB5)
#define LED_GN_OFF()            PORTB &= ~(1<<PORTB5)
#define LED_GN_TOGGLE()         PORTB ^= (1<<PORTB5)
#define LED_OFF()               PORTB = 0x00
#else
#define LED_INIT()
#define LED_RT_ON()
#define LED_RT_OFF()
#define LED_GN_ON()
#define LED_GN_OFF()
#define LED_GN_TOGGLE()
#define LED_OFF()
#endif /* LED_SUPPORT */

#if !defined(TWCR) && defined(USICR)
#define USI_PIN_INIT()          { PORTB |= ((1<<PORTB0) | (1<<PORTB2)); \
                                  DDRB |= (1<<PORTB2); \
                                }
#define USI_PIN_SDA_INPUT()     DDRB &= ~(1<<PORTB0)
#define USI_PIN_SDA_OUTPUT()    DDRB |= (1<<PORTB0)
#define USI_PIN_SCL()           (PINB & (1<<PINB2))

#if (USE_CLOCKSTRETCH == 0)
#error "USI peripheral requires enabled USE_CLOCKSTRETCH"
#endif

#define USI_STATE_MASK          0x0F
#define USI_STATE_IDLE          0x00    /* wait for Start Condition */
#define USI_STATE_SLA           0x01    /* wait for Slave Address */
#define USI_STATE_SLAW_ACK      0x02    /* ACK Slave Address + Write (Master writes) */
#define USI_STATE_SLAR_ACK      0x03    /* ACK Slave Address + Read (Master reads) */
#define USI_STATE_NAK           0x04    /* send NAK */
#define USI_STATE_DATW          0x05    /* receive Data */
#define USI_STATE_DATW_ACK      0x06    /* transmit ACK for received Data */
#define USI_STATE_DATR          0x07    /* transmit Data */
#define USI_STATE_DATR_ACK      0x08    /* receive ACK for transmitted Data */
#define USI_WAIT_FOR_ACK        0x10    /* wait for ACK bit (2 SCL clock edges) */
#define USI_ENABLE_SDA_OUTPUT   0x20    /* SDA is output (slave transmitting) */
#define USI_ENABLE_SCL_HOLD     0x40    /* Hold SCL low after clock overflow */
#endif /* !defined(TWCR) && defined(USICR) */

#if (VIRTUAL_BOOT_SECTION)
/* unused vector to store application start address */
#define APPVECT_NUM             EE_RDY_vect_num

/* each vector table entry is a 2byte RJMP opcode */
#define RSTVECT_ADDR            0x0000
#define APPVECT_ADDR            (APPVECT_NUM * 2)
#define RSTVECT_PAGE_OFFSET     (RSTVECT_ADDR % SPM_PAGESIZE)
#define APPVECT_PAGE_OFFSET     (APPVECT_ADDR % SPM_PAGESIZE)

/* create RJMP opcode for the vector table */
#define OPCODE_RJMP(addr)       (((addr) & 0x0FFF) | 0xC000)

#elif (!defined(ASRE) && !defined (RWWSRE))
#error "Device without bootloader section requires VIRTUAL_BOOT_SECTION"
#endif

/* SLA+R */
#define CMD_WAIT                0x00
#define CMD_READ_VERSION        0x01
#define CMD_ACCESS_MEMORY       0x02
/* internal mappings */
#define CMD_ACCESS_CHIPINFO     (0x10 | CMD_ACCESS_MEMORY)
#define CMD_ACCESS_FLASH        (0x20 | CMD_ACCESS_MEMORY)
#define CMD_ACCESS_EEPROM       (0x30 | CMD_ACCESS_MEMORY)
#define CMD_WRITE_FLASH_PAGE    (0x40 | CMD_ACCESS_MEMORY)
#define CMD_WRITE_EEPROM_PAGE   (0x50 | CMD_ACCESS_MEMORY)

/* SLA+W */
#define CMD_SWITCH_APPLICATION  CMD_READ_VERSION
/* internal mappings */
#define CMD_BOOT_BOOTLOADER     (0x10 | CMD_SWITCH_APPLICATION) /* only in APP */
#define CMD_BOOT_APPLICATION    (0x20 | CMD_SWITCH_APPLICATION)

/* CMD_SWITCH_APPLICATION parameter */
#define BOOTTYPE_BOOTLOADER     0x00    /* only in APP */
#define BOOTTYPE_APPLICATION    0x80

/* CMD_{READ|WRITE}_* parameter */
#define MEMTYPE_CHIPINFO        0x00
#define MEMTYPE_FLASH           0x01
#define MEMTYPE_EEPROM          0x02

/*
 * LED_GN flashes with 20Hz (while bootloader is running)
 * LED_RT flashes on TWI activity
 *
 * bootloader twi-protocol:
 * - abort boot timeout:
 *   SLA+W, 0x00, STO
 *
 * - show bootloader version
 *   SLA+W, 0x01, SLA+R, {16 bytes}, STO
 *
 * - start application
 *   SLA+W, 0x01, 0x80, STO
 *
 * - read chip info: 3byte signature, 1byte page size, 2byte flash size, 2byte eeprom size
 *   SLA+W, 0x02, 0x00, 0x00, 0x00, SLA+R, {8 bytes}, STO
 *
 * - read one (or more) flash bytes
 *   SLA+W, 0x02, 0x01, addrh, addrl, SLA+R, {* bytes}, STO
 *
 * - read one (or more) eeprom bytes
 *   SLA+W, 0x02, 0x02, addrh, addrl, SLA+R, {* bytes}, STO
 *
 * - write one flash page
 *   SLA+W, 0x02, 0x01, addrh, addrl, {* bytes}, STO
 *
 * - write one (or more) eeprom bytes
 *   SLA+W, 0x02, 0x02, addrh, addrl, {* bytes}, STO
 */

uint16_t boot_magic = 0;

const static uint8_t info[16] = VERSION_STRING;
const static uint16_t chipinfo[8] = {
    SIGNATURE_0, SIGNATURE_1, SIGNATURE_2,
    SPM_PAGESIZE,

    (BOOTLOADER_START >> 8) & 0xFF,
    BOOTLOADER_START & 0xFF,

#if (EEPROM_SUPPORT)
    ((E2END +1) >> 8 & 0xFF),
    (E2END +1) & 0xFF
#else
    0x00, 0x00
#endif
};


 
//static uint8_t boot_timeout = TIMER_MSEC2IRQCNT(TIMEOUT_MS);
//static volatile uint16_t boot_timeout = 1;
static volatile uint16_t boot_timeout = 1000;

static uint8_t cmd = CMD_WAIT;

/* flash buffer */
static uint8_t buf[SPM_PAGESIZE];
static uint16_t addr;

#if (VIRTUAL_BOOT_SECTION)
/* reset/application vectors received from host, needed for verify read */
static uint8_t rstvect_save[2];
static uint8_t appvect_save[2];
#endif /* (VIRTUAL_BOOT_SECTION) */

//for Gus's no-reset I2C bootloader:
#define BOOT_MAGIC_ADDR ((uint16_t*)510)
#define BOOT_MAGIC_VALUE 0xB007

const static uint16_t pageInitializedValue = 0xFFFF;
static uint16_t current_page = pageInitializedValue; //had been 0xFFFF
static uint8_t page_dirty = 0;
static uint8_t page_pos = 0;
static uint16_t page_dirty_bytes = 0; // NEW: number of bytes buffered in current page
 
static uint16_t current_page_word;
volatile uint8_t flash_write_pending = 0;
volatile uint8_t bootloadLoopCount = 0;

/////////////////////////////////////////////////////////////
//debug bitbanger functions. the plan was to use this when serial was unreliable

#if CLOCKED_SERIAL_DEBUG
#define TX_CLOCK_PIN 1  // A0 -> PC0
#define TX_DATA_PIN  0  // A1 -> PC1

#define TX_DDR   DDRC
#define TX_PORT  PORTC

void init_tx_pins() {
    TX_DDR |= (1 << TX_CLOCK_PIN) | (1 << TX_DATA_PIN); // set both as output
    TX_PORT &= ~((1 << TX_CLOCK_PIN) | (1 << TX_DATA_PIN)); // drive low
}

// Send one byte MSB-first
void tx_byte(uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        if (b & (1 << i)) TX_PORT |=  (1 << TX_DATA_PIN);
        else              TX_PORT &= ~(1 << TX_DATA_PIN);

        TX_PORT |=  (1 << TX_CLOCK_PIN);  // clock HIGH
        _delay_us(50);
        TX_PORT &= ~(1 << TX_CLOCK_PIN);  // clock LOW
        _delay_us(50);
    }
 
}

#endif

/////////////////////////////////////////////////////////////
//debug serial, because i eventually got serial to work
#if UART_DEBUG
// initialize UART
static void uart_init(void)
{
#if defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__)
    /* ---------- classic UART ---------- */
    UCSRA = (1 << U2X);   // double speed

    UBRRH = (uint8_t)(UBRR_VALUE >> 8);
    UBRRL = (uint8_t)(UBRR_VALUE & 0xFF);

    UCSRB = (1 << TXEN);  // TX only
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0); // 8N1
#else

    /* ---------- USART0 ---------- */
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);

    UCSR0A |= (1 << U2X0);        // double speed
    UCSR0B = (1 << TXEN0);        // TX only
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1

#endif
}

static void uart_putc(char c)
{
#if defined(__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
    while (!(UCSRA & (1 << UDRE))) {
        ; // wait
    }
    UDR = c;
#else
    while (!(UCSR0A & (1 << UDRE0))) {
        ; // wait
    }
    UDR0 = c;
#endif
}

static void uart_puts(const char *s)
{
    while (*s) {
        uart_putc(*s++);
    }
}

void uart_putint(uint32_t value)
{
    char buf[11];   // max uint32_t: 4294967295 + '\0'
    itoa(value, buf, 10);
    uart_puts(buf);
}
 
void uart_puthex(int value)
{
    char buf[8];              // enough for -32768\0
    itoa(value, buf, 16);
    uart_puts(buf);
}

static inline char hex_digit(uint8_t v)
{
    v &= 0x0F;
    return (v < 10) ? ('0' + v) : ('A' + (v - 10));
}
 
void u32_to_hex(uint32_t val, char *out)
{
    for (int i = 0; i < 8; i++) {
        out[7 - i] = hex_digit(val);
        val >>= 4;
    }
    out[8] = '\0';
}

void uart_put_bytes_hex(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        uart_putc(hex_digit(b >> 4)); // high nibble
        uart_putc(hex_digit(b));      // low nibble
        uart_putc(' ');               // optional space between bytes
    }
    //uart_putc('\n');                  // optional newline at end
}

void uart_dump_buf(void)
{
    for (uint16_t i = 0; i < sizeof(buf); i += 16) {

        // print offset (hex)
        char off[9];
        u32_to_hex(i, off);
        uart_puts(off);
        uart_putc(' ');

        // print up to 16 bytes
        for (uint8_t j = 0; j < 16; j++) {
            uint16_t idx = i + j;
            if (idx >= sizeof(buf)) {
              break;
            }
            uint8_t b = buf[idx];
            uart_putc(hex_digit(b >> 4));
            uart_putc(hex_digit(b));
            uart_putc(' ');
        }

        uart_putc('\n');
    }
}

#endif // UART_DEBUG

/////////////////////////////////////////////////////////////////////////////
//hardware debugging, for the stage when i was just using LEDs to debug

void setArduinoPin(uint8_t pin, uint8_t value)
{
    if (pin >= 2 && pin <= 7) {
        // PORTD: D2..D7 ? PD2..PD7
        uint8_t bit = pin;  // direct mapping

        DDRD |=  (1 << bit);          // set as output
        if (value)
            PORTD |=  (1 << bit);
        else
            PORTD &= ~(1 << bit);
    }
    else if (pin >= 8 && pin <= 10) {
        // PORTB: D8..D10 ? PB0..PB2
        uint8_t bit = pin - 8;

        DDRB |=  (1 << bit);          // set as output
        if (value)
            PORTB |=  (1 << bit);
        else
            PORTB &= ~(1 << bit);
    }
}

//////////////////////////////////////////////////////////////////////////////////
//new interrupt stuff we were missing

void TWI_SlaveInit(void) {
    TWAR = (TWI_ADDRESS << 1);   // set 7-bit address
    TWCR = (1 << TWEN)  |          // enable TWI
           (1 << TWEA)  |          // enable ACK
           //(1 << TWIE)  |          // enable TWI interrupt
           (1 << TWINT);           // clear any pending TWINT
           
    #if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__)
        TWCR |= (1 << TWIE);
    #endif
 
}

static void twi_handler(void);

 
/* *************************************************************************
 * write_flash_page - safe flash page write for ATmega328P
 * ************************************************************************* */
static void write_flash_page(void)
{
    bootloadLoopCount = 0;
    if (!page_dirty) return;  // nothing to write

    // Save SREG and disable interrupts
    uint8_t sreg = SREG;
    cli();

    uint16_t pagestart = current_page;
    uint16_t fill_addr = pagestart;
    uint8_t *p = buf;

    // Determine bytes to write (either full page or partially filled)
    uint16_t bytes_to_write = page_dirty_bytes ? page_dirty_bytes : SPM_PAGESIZE;

    // Prevent bootloader overwrite
    if (pagestart >= BOOTLOADER_START)
    {
        sei();
        return;
    }

#if (VIRTUAL_BOOT_SECTION)
    if (pagestart == (RSTVECT_ADDR & ~(SPM_PAGESIZE - 1)))
    {
        rstvect_save[0] = buf[RSTVECT_PAGE_OFFSET];
        rstvect_save[1] = buf[RSTVECT_PAGE_OFFSET + 1];
        appvect_save[0] = buf[APPVECT_PAGE_OFFSET];
        appvect_save[1] = buf[APPVECT_PAGE_OFFSET + 1];

        uint16_t rst_vector = OPCODE_RJMP(BOOTLOADER_START - 1);
        buf[RSTVECT_PAGE_OFFSET]     = (rst_vector & 0xFF);
        buf[RSTVECT_PAGE_OFFSET + 1] = (rst_vector >> 8);

        uint16_t app_vector = rstvect_save[0] | (rstvect_save[1] << 8);
        app_vector = OPCODE_RJMP(app_vector - APPVECT_NUM);
        buf[APPVECT_PAGE_OFFSET]     = (app_vector & 0xFF);
        buf[APPVECT_PAGE_OFFSET + 1] = (app_vector >> 8);
    }
#endif

#if UART_DEBUG
    uart_puts("pagestart:");
    uart_putint(pagestart);
    uart_puts(" bytes_to_write:");
    uart_putint(bytes_to_write);
    uart_puts("\n");
    uart_dump_buf();
    uart_puts("\n");
#endif
    wdt_disable();  // disable watchdog
    boot_spm_busy_wait();

    // -------------------------------
    // Step 1: Erase the target flash page
    // -------------------------------
    boot_page_erase(pagestart);
    boot_spm_busy_wait();

    // -------------------------------
    // Step 2: Clear the internal page buffer
    // -------------------------------
    for (uint16_t a = pagestart; a < pagestart + SPM_PAGESIZE; a += 2)
    {
        boot_page_fill(a, 0xFFFF);
    }

    // -------------------------------
    // Step 3: Fill the buffer with actual data
    // -------------------------------
    while (bytes_to_write >= 2)
    {
        uint16_t data = *p++;
        data |= (*p++) << 8;
        boot_page_fill(fill_addr, data);
        fill_addr += 2;
        bytes_to_write -= 2;
        
        if(bytes_to_write % 64 == 0) {
          wdt_reset();
        }
    }

    // Handle odd remaining byte (if page_dirty_bytes is odd)
    if (bytes_to_write == 1)
    {
        uint16_t data = *p++;
        data |= 0x00 << 8;
        boot_page_fill(fill_addr, data);
    }

    // -------------------------------
    // Step 4: Commit buffer to flash
    // -------------------------------
    boot_page_write(pagestart);
    boot_spm_busy_wait();

    // Step 5: Re-enable RWW section
    boot_rww_enable();

    // Step 6: Clear dirty flags
    page_dirty = 0;
    page_dirty_bytes = 0;

    // Restore interrupts
    SREG = sreg;
}



#if (EEPROM_SUPPORT)
/* *************************************************************************
 * read_eeprom_byte
 * ************************************************************************* */
static uint8_t read_eeprom_byte(uint16_t address)
{
    EEARL = address;
    EEARH = (address >> 8);
    EECR |= (1<<EERE);

    return EEDR;
} /* read_eeprom_byte */


/* *************************************************************************
 * write_eeprom_byte
 * ************************************************************************* */
static void write_eeprom_byte(uint8_t val)
{
    EEARL = addr;
    EEARH = (addr >> 8);
    EEDR = val;
    addr++;

#if defined (EEWE)
    EECR |= (1<<EEMWE);
    EECR |= (1<<EEWE);
#elif defined (EEPE)
    EECR |= (1<<EEMPE);
    EECR |= (1<<EEPE);
#else
#error "EEWE/EEPE not defined"
#endif

    eeprom_busy_wait();
} /* write_eeprom_byte */


#if (USE_CLOCKSTRETCH == 0)
/* *************************************************************************
 * write_eeprom_buffer
 * ************************************************************************* */
static void write_eeprom_buffer(uint8_t size)
{
    uint8_t *p = buf;

    while (size--)
    {
        write_eeprom_byte(*p++);
    }
} /* write_eeprom_buffer */
#endif /* (USE_CLOCKSTRETCH == 0) */
#endif /* EEPROM_SUPPORT */

static inline void flash_stream_byte(uint8_t data)
{
    uint8_t page_offset = addr % SPM_PAGESIZE;
    uint16_t page_start = addr - page_offset;
    current_page = page_start;
    buf[page_offset] = data;
    addr++;
    page_dirty = 1;
    page_dirty_bytes = page_offset + 1;
    //uart_puts("..................................\n");
    /*
    uart_puts("current page:");
    uart_putint(current_page);
    uart_puts("   page_start:");
    uart_putint(page_start);
    uart_puts("   addr:");
    uart_putint(addr);
    uart_puts("\n");
    */
}

 /* *************************************************************************
 * TWI_data_write - I2C Slave Data Handler
 * ************************************************************************* */
static uint8_t TWI_data_write(uint8_t bcnt, uint8_t data)
{
    uint8_t ack = 0x01;

    switch (bcnt)
    {
        case 0:
            switch (data)
            {
                case CMD_SWITCH_APPLICATION:
                case CMD_ACCESS_MEMORY:
                case CMD_WAIT:
                    boot_timeout = 0;
                    cmd = data;
                    break;
                default:
                    cmd = CMD_BOOT_APPLICATION;
                    ack = 0x00;
                    break;
            }
            break;

        case 1:
            switch (cmd)
            {
                case CMD_SWITCH_APPLICATION:
                    if (data == BOOTTYPE_APPLICATION)
                    {
                        cmd = CMD_BOOT_APPLICATION;
                    }
                    else if (data == BOOTTYPE_BOOTLOADER)
                    {
                        boot_magic = BOOT_MAGIC_VALUE;
                        eeprom_write_word(BOOT_MAGIC_ADDR, boot_magic);
                        #if UART_DEBUG
                        uart_puts("ABOUT TO DO THE TWI_DATA RESET\n");
                        #endif
                        wdt_enable(WDTO_15MS);
                        while (1);
                    }
                    ack = 0x00;
                    break;

                case CMD_ACCESS_MEMORY:
                    //uart_puts("got a CMD_ACCESS_MEMORY\n");
                    if (data == MEMTYPE_CHIPINFO)
                    {
                        cmd = CMD_ACCESS_CHIPINFO;
                    }
                    else if (data == MEMTYPE_FLASH)
                    {
                        //uart_puts("got a MEMTYPE_FLASH\n");
                        cmd = CMD_ACCESS_FLASH;

                        page_dirty = 0;               // nothing yet
                        page_dirty_bytes = 0;
                        //current_page = pageInitializedValue; // this was causing problems
                    }
#if (EEPROM_SUPPORT)
                    else if (data == MEMTYPE_EEPROM)
                    {
                        cmd = CMD_ACCESS_EEPROM;
                    }
#endif
                    else
                    {
                        ack = 0x00;
                    }
                    break;

                default:
                    ack = 0x00;
                    break;
            }
            break;

        case 2:
            addr = ((uint16_t)data) << 8;
            break;

        case 3:
            addr |= data;
            /*
            uart_puts("\nfrom address mode: ");
            uart_putint(addr);
            uart_puts("\n");

            // Initialize current_page on first byte received
            uart_puts("Initial conditional, pageinitializedValue: ");
            
            uart_putint(pageInitializedValue);
            uart_puts("current page: ");
            uart_putint(current_page);
            uart_puts("\n");
            */
            if (current_page == pageInitializedValue)
            {
                current_page = 0; // start at first page
                /*
                uart_puts("Initializing current_page to: ");
                uart_putint(current_page);
                uart_puts("\n");
                */
            }
            break;

        default:
            //uart_puts("*********** COMMAND:");
            //uart_putint(cmd);
            //uart_puts("\n");
            switch (cmd)
            {
#if (EEPROM_SUPPORT)
#if (USE_CLOCKSTRETCH)
                case CMD_ACCESS_EEPROM:
                    write_eeprom_byte(data);
                    break;
#else
                case CMD_ACCESS_EEPROM:
                    cmd = CMD_WRITE_EEPROM_PAGE;
                    /* fall through */
                case CMD_WRITE_EEPROM_PAGE:
#endif
#endif
                case CMD_ACCESS_FLASH:
                {
                    cmd = CMD_WRITE_FLASH_PAGE;
                    flash_stream_byte(data);
                    break;

   
                    
                    //THIS CONDITION WAS NEVER TRUE:
                    /*
                    // Flush previous page if we crossed into a new one
                    if (page_offset == 0 && page_dirty_bytes > 0  && addr > 0)
                    {
                        uart_puts("WRITE_FLASH-newpage\n");
                        uart_puts("-+-------------------------\n");
                        uart_puts("_fill_addr:");
                        uart_putint(current_page);
                        uart_puts("\n_pagestart:");
                        uart_putint(current_page);
                        uart_puts("\n");

#if (USE_CLOCKSTRETCH)
                        TWCR &= ~(1 << TWINT);
                        write_flash_page();
                        TWCR |= (1 << TWINT);
#else
                        write_flash_page();
#endif
                        page_dirty = 0;
                        page_dirty_bytes = 0;
                    }
                    */
             
 
                    // Store byte in page buffer
                    //uart_puts("FILLING BUFFER. offset: ");
                    //uart_putint(page_offset);
                    //uart_puts("=>");
                    //uart_putint(data);
                    //uart_puts("\n");
          
                    /*
                    //THIS CONDITION WAS NEVER TRUE:
                    // Flush page immediately if full
                    if (page_offset == (SPM_PAGESIZE - 1))
                    {
                        uart_puts("WRITE_FLASH--pagefull\n");
                        uart_puts("-!-------------------------\n");
                        uart_puts(" fill_addr:");
                        uart_putint(current_page);
                        uart_puts("\n pagestart:");
                        uart_putint(current_page);
                        uart_puts("\n");

#if (USE_CLOCKSTRETCH)
                        TWCR &= ~(1 << TWINT);
                        write_flash_page();
                        TWCR |= (1 << TWINT);
#else
                        write_flash_page();
#endif
                        page_dirty = 0;
                        page_dirty_bytes = 0;
                    }
                    */
                    break;
                }
                case CMD_WRITE_FLASH_PAGE:
                {
                  //also doing this here, since mostly the cmd value is 66, or CMD_WRITE_FLASH_PAGE
                  flash_stream_byte(data);

                  break;
                }
                default:
                    ack = 0x00;
                    break;
            }
            break;
    }

    return ack;
} /* TWI_data_write */



/* *************************************************************************
 * TWI_data_read
 * ************************************************************************* */
static uint8_t TWI_data_read(uint8_t bcnt)
{
    uint8_t data;

    switch (cmd)
    {
        case CMD_READ_VERSION:
            bcnt %= sizeof(info);
            data = info[bcnt];
            break;

        case CMD_ACCESS_CHIPINFO:
            bcnt %= sizeof(chipinfo);
            data = chipinfo[bcnt];
            break;

        case CMD_ACCESS_FLASH:
            switch (addr)
            {
/* return cached values for verify read */
#if (VIRTUAL_BOOT_SECTION)
                case RSTVECT_ADDR:
                    data = rstvect_save[0];
                    break;

                case (RSTVECT_ADDR + 1):
                    data = rstvect_save[1];
                    break;

                case APPVECT_ADDR:
                    data = appvect_save[0];
                    break;

                case (APPVECT_ADDR + 1):
                    data = appvect_save[1];
                    break;
#endif /* (VIRTUAL_BOOT_SECTION) */

                default:
                    data = pgm_read_byte_near(addr);
                    break;
            }

            addr++;
            break;

#if (EEPROM_SUPPORT)
        case CMD_ACCESS_EEPROM:
            data = read_eeprom_byte(addr++);
            break;
#endif /* (EEPROM_SUPPORT) */

        default:
            data = 0xFF;
            break;
    }

    return data;
} /* TWI_data_read */


#if defined (TWCR)
/* *************************************************************************
 * TWI_vect
 * ************************************************************************* */
static void twi_handler(void)
{
    static uint8_t bcnt;       // buffer counter

    // Step 1: read TWSR before touching TWCR
    uint8_t status = TWSR & 0xF8;

    // Debug: print status
    /*
    char buf[9];
    uart_puts("TWSR & 0xF8: ");
    u32_to_hex((uint32_t)status, buf);
    uart_puts(buf);
    uart_puts("\n");  
    */
    // Step 2: handle TWI state
    switch (status)
    {
        // SLA+W received, ACK returned -> receive data
        case 0x60:
            bcnt = 0;
            LED_RT_ON();
            break;

        // Previous SLA+W, data received, ACK returned -> store data
        case 0x80:
            if (TWI_data_write(bcnt++, TWDR) == 0x00)
            {
 
                // disable ACK for next byte if TWI_data_write says so
                // will update TWCR at the end
            }
            break;

        // SLA+R received, ACK returned -> send data
        case 0xA8:
            bcnt = 0;
            LED_RT_ON();
            // fall through

        // Previous SLA+R, data sent, ACK returned -> send next byte
        case 0xB8:
 
            TWDR = TWI_data_read(bcnt++);
            break;

        // SLA+W, data received, NACK returned -> last byte
        case 0x88:
            TWI_data_write(bcnt++, TWDR);
            break;

        // STOP or repeated START -> reset state
        case 0xA0:
        #if (USE_CLOCKSTRETCH == 0)
            if ((cmd == CMD_WRITE_FLASH_PAGE)
        #if (EEPROM_SUPPORT)
                || (cmd == CMD_WRITE_EEPROM_PAGE)
        #endif
               )
            {
                //uart_puts("ADDR AT 0xA0: ");
                //uart_putint(addr);
                //uart_puts("\n");
                if ((addr) % SPM_PAGESIZE == 0) {
                  //uart_puts("EVEN STEVEN!! woooooot!");
                  flash_write_pending = 1; // flag main loop to commit page
                }
            }
        #endif
            //uart_puts("in the good part of 0xA0\n");
            bcnt = 0;
            break;

        // SLA+R, data sent, NACK returned -> idle
        case 0xC0:
            LED_RT_OFF();
            break;

        // illegal/unknown state -> reset TWI hardware
        default:
            TWCR |= (1<<TWSTO);
            break;
    }

    // Step 3: re-arm TWI for next event
    // always leave TWEN, TWIE, TWEA set; clear TWINT to acknowledge current event
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
}

#endif /* defined (TWCR) */

#if defined (USICR)
/* *************************************************************************
 * usi_statemachine
 * ************************************************************************* */
static void usi_statemachine(uint8_t usisr)
{
    static uint8_t usi_state;
    static uint8_t bcnt;

    uint8_t data = USIDR;
    uint8_t state = usi_state & USI_STATE_MASK;
    
    /* Start Condition detected */
    if (usisr & (1<<USISIF))
    {
        /* wait until SCL goes low */
        while (USI_PIN_SCL());
        

        usi_state = USI_STATE_SLA | USI_ENABLE_SCL_HOLD;
        state = USI_STATE_IDLE;
    }
    
    /* Stop Condition detected */
    if (usisr & (1<<USIPF))
    {
        LED_RT_OFF();

        if (page_dirty) {
  
    #if (USE_CLOCKSTRETCH)
            /* USI does not truly stretch SCL, but keep symmetry */
            write_flash_page();
    #else
            write_flash_page();
    #endif
            page_dirty = 0;
            flash_write_pending = 0;
            _delay_ms(2);
        }

        usi_state = USI_STATE_IDLE;
        state = USI_STATE_IDLE;
    }

    if (state == USI_STATE_IDLE)
    {
        /* do nothing */
    }
    /* Slave Address received => prepare ACK/NAK */
    else if (state == USI_STATE_SLA)
    {
        bcnt = 0;

        /* SLA+W received -> send ACK */
        if (data == ((TWI_ADDRESS<<1) | 0x00))
        {
            LED_RT_ON();
            usi_state = USI_STATE_SLAW_ACK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x00;
        }
        /* SLA+R received -> send ACK */
        else if (data == ((TWI_ADDRESS<<1) | 0x01))
        {
            LED_RT_ON();
            usi_state = USI_STATE_SLAR_ACK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x00;
        }
        /* not addressed -> send NAK */
        else
        {
            usi_state = USI_STATE_NAK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x80;
        }
    }
    /* sent NAK -> go to idle */
    else if (state == USI_STATE_NAK)
    {
        usi_state = USI_STATE_IDLE;
    }
    /* sent ACK after SLA+W -> wait for data */
    /* sent ACK after DAT+W -> wait for more data */
    else if ((state == USI_STATE_SLAW_ACK) ||
             (state == USI_STATE_DATW_ACK)
            )
    {
        usi_state = USI_STATE_DATW | USI_ENABLE_SCL_HOLD;
    }
    /* data received -> send ACK/NAK */
    else if (state == USI_STATE_DATW)
    {
        if (TWI_data_write(bcnt++, data))
        {
            usi_state = USI_STATE_DATW_ACK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x00;
        }
        else
        {
            usi_state = USI_STATE_NAK | USI_WAIT_FOR_ACK | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
            USIDR = 0x80;
        }
    }
    /* sent ACK after SLA+R -> send data */
    /* received ACK after DAT+R -> send more data */
    else if ((state == USI_STATE_SLAR_ACK) ||
             ((state == USI_STATE_DATR_ACK) && !(data & 0x01))
            )
    {
        USIDR = TWI_data_read(bcnt++);
        usi_state = USI_STATE_DATR | USI_ENABLE_SDA_OUTPUT | USI_ENABLE_SCL_HOLD;
    }
    /* sent data after SLA+R -> receive ACK/NAK */
    else if (state == USI_STATE_DATR)
    {
        usi_state = USI_STATE_DATR_ACK | USI_WAIT_FOR_ACK | USI_ENABLE_SCL_HOLD;
        USIDR = 0x80;
    }
    /* received NAK after DAT+R -> go to idle */
    else if ((state == USI_STATE_DATR_ACK) && (data & 0x01))
    {
        usi_state = USI_STATE_IDLE;
    }
    /* default -> go to idle */
    else
    {
        usi_state = USI_STATE_IDLE;
    }

    /* set SDA direction according to current state */
    if (usi_state & USI_ENABLE_SDA_OUTPUT)
    {
        USI_PIN_SDA_OUTPUT();
    }
    else
    {
        USI_PIN_SDA_INPUT();
    }

    if (usi_state & USI_ENABLE_SCL_HOLD)
    {
        /* Enable TWI Mode, hold SCL low after counter overflow, count both SCL edges */
        USICR = (1<<USIWM1) | (1<<USIWM0) | (1<<USICS1);
    }
    else
    {
        /* Enable TWI, hold SCL low only after start condition, count both SCL edges */
        USICR = (1<<USIWM1) | (1<<USICS1);
    }

    /* clear start/overflow/stop condition flags */
    usisr &= ((1<<USISIF) | (1<<USIOIF) | (1<<USIPF));
    if (usi_state & USI_WAIT_FOR_ACK)
    {
        /* count 2 SCL edges (ACK/NAK bit) */
        USISR = usisr | ((16 -2)<<USICNT0);
    }
    else
    {
        /* count 16 SCL edges (8bit data) */
        USISR = usisr | ((16 -16)<<USICNT0);
    }
} /* usi_statemachine */
#endif /* defined (USICR) */


/* *************************************************************************
 * TIMER0_OVF_vect
 * ************************************************************************* */
static void TIMER0_OVF_vect(void)
{
    /* restart timer */
    TCNT0 = 0xFF - TIMER_MSEC2TICKS(TIMER_IRQFREQ_MS);

    /* blink LED while running */
    LED_GN_TOGGLE();

    /* count down for app-boot */
    if (boot_timeout > 1)
    {
        boot_timeout--;
    }
    else if (boot_timeout == 1)
    {
        /* trigger app-boot */
        cmd = CMD_BOOT_APPLICATION;
    }
} /* TIMER0_OVF_vect */


#if (VIRTUAL_BOOT_SECTION)
static void (*jump_to_app)(void) __attribute__ ((noreturn)) = (void*)APPVECT_ADDR;
#else
static void (*jump_to_app)(void) __attribute__ ((noreturn)) = (void*)0x0000;
#endif


#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega2560__)
void init3(void) __attribute__((naked, section(".init3")));
void init3(void)
{
    cli();

    MCUSR = 0;      // preserve this only if you do not need reset cause
    wdt_disable(); // canonical, safe, portable

    asm volatile ("clr __zero_reg__");
}


void __attribute__((naked, section(".init0"))) kill_wdt(void)
{
    MCUSR = 0;
    WDTCSR = (1<<WDCE) | (1<<WDE);
    WDTCSR = 0;
}
#endif

 

/* *************************************************************************
 * init1
 * ************************************************************************* */
void init1(void) __attribute__((naked, section(".init1")));
void init1(void)
{
  /* make sure r1 is 0x00 */
  asm volatile ("clr __zero_reg__");

  /* on some MCUs the stack pointer defaults NOT to RAMEND */
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8515__) || \
    defined(__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || \
    defined (__AVR_ATmega32__) || defined (__AVR_ATmega64__)  || \
    defined (__AVR_ATmega128__) || defined (__AVR_ATmega162__) || defined(__AVR_ATmega168__)
  SP = RAMEND;
#endif
#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega2560__)
  SP = RAMEND;
#endif
} /* init1 */


/*
 * For newer devices the watchdog timer remains active even after a
 * system reset. So disable it as soon as possible.
 * automagically called on startup
 */
#if defined (__AVR_ATmega88__) || defined (__AVR_ATmega168__) ||  defined (__AVR_ATmega328P__)
/* *************************************************************************
 * disable_wdt_timer
 * ************************************************************************* */

void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
    MCUSR = 0;
    WDTCSR = (1<<WDCE) | (1<<WDE);
    WDTCSR = 0;
} /* disable_wdt_timer */
#elif defined(__AVR_ATmega32A__) || defined(__AVR_ATmega32__)

void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
    MCUCSR = 0;                       // clear reset flags
    WDTCR = (1<<WDTOE) | (1<<WDE);    // enable timed sequence
    WDTCR = 0;                       // disable watchdog
}

#endif


/* *************************************************************************
 * main
 * ************************************************************************* */
int main(void) __attribute__ ((OS_main, section (".init9")));
int main(void)
{
    //eventually get rid of this block:
    MCUSR = 0;          // clear reset flags
    wdt_disable();      // disable watchdog immediately
    cli();              // disable interrupts while blinking

    // --- basic MCU init ---
    #if defined (__AVR_ATmega64__)  
    MCUCR = (1 << IVCE);
    MCUCR = 0;          // IVSEL = 0 ? application vectors
    
    #elif defined (__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
        MCUCSR = 0;
    #else
        MCUSR = 0;
    #endif
    
    wdt_disable();
    #if defined (__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
        MCUCSR = 0;
    #else
        MCUSR = 0;
    #endif
    #if defined (__AVR_ATmega64__)
    SP = RAMEND;
    
    #endif
    #if UART_DEBUG
      uart_puts("Bootloader triggered...");
    #endif
     
    //if you want to make sure you are entering the bootloader if serial isn't working:
    for(int i =0; i<10; i++) {
      setArduinoPin(5, 1);
      _delay_ms(50);
      setArduinoPin(5, 0);
      _delay_ms(20);
    }
   
    
 
 
    
    TWI_SlaveInit(); 

    uint8_t stay_in_bootloader = 0;
 
    
    
    #if defined (__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
        uint8_t mcusr = MCUCSR;  // save reset flags
        MCUCSR = 0;
    #else
        uint8_t mcusr = MCUSR;  // save reset flags
        MCUSR = 0;
    #endif
    

    // --- read boot magic from EEPROM ---
    uint16_t boot_magic = eeprom_read_word(BOOT_MAGIC_ADDR);

    // --- default: jump to sketch ---
    cmd = CMD_BOOT_APPLICATION;

    if (boot_magic == BOOT_MAGIC_VALUE) {
        #if UART_DEBUG
        uart_puts("Now in the bootloader...\n");
        #endif
        stay_in_bootloader = 1;           // force bootloader loop
        cmd = CMD_WAIT;                    // reset TWI command state
        boot_timeout = 0xFFFF;

        // clear boot magic in EEPROM so next reset is normal
        eeprom_write_word(BOOT_MAGIC_ADDR, 0);

        // small delay to let I2C bus stabilize
        _delay_ms(200);
    }

    // --- LED init ---
    LED_INIT();
    LED_GN_ON();

#if (VIRTUAL_BOOT_SECTION)
    /* load current values (for reading flash) */
    rstvect_save[0] = pgm_read_byte_near(RSTVECT_ADDR);
    rstvect_save[1] = pgm_read_byte_near(RSTVECT_ADDR + 1);
    appvect_save[0] = pgm_read_byte_near(APPVECT_ADDR);
    appvect_save[1] = pgm_read_byte_near(APPVECT_ADDR + 1);
#endif /* (VIRTUAL_BOOT_SECTION) */

    /* timer0: running with F_CPU/1024 */
#if defined (TCCR0)
    TCCR0 = (1<<CS02) | (1<<CS00);
#elif defined (TCCR0B)
    TCCR0B = (1<<CS02) | (1<<CS00);
#else
#error "TCCR0(B) not defined"
#endif
#if UART_DEBUG
    uart_init();
#endif
#if CLOCKED_SERIAL_DEBUG
    init_tx_pins();
#endif
#if defined (TWCR)
    /* TWI init: set address, auto ACKs */
    TWAR = (TWI_ADDRESS<<1);
    TWCR = (1<<TWEA) | (1<<TWEN);
#elif defined (USICR)
    USI_PIN_INIT()
    usi_statemachine(0x00);
#else
#error "No TWI/USI peripheral found"
#endif

    sei();  // enable interrupts

    // --- bootloader loop: only if forced or page write pending ---
    
    while (stay_in_bootloader && (cmd != CMD_BOOT_APPLICATION || page_dirty || flash_write_pending)) {
        
        static uint16_t heartbeat = 0;
        heartbeat++;
        
        //TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA); //new code gus installed
        #if defined (TWCR)
        TWCR = (1<<TWEN) | (1<<TWEA); // NO TWIE
        #endif
 
        if (heartbeat == 0) {
            bootloadLoopCount++;
            #if UART_DEBUG
            uart_puts("BOOTLOADER LOOPING...\n");
            //uart_putint(cmd);
            //uart_puts("\n");
            #endif
        }
        
        if (flash_write_pending) {
            //uart_puts("======from the main loop\n");
            write_flash_page();
            flash_write_pending = 0;
        }

        // handle TWI / USI events
        #if defined (TWCR)
            if (TWCR & (1<<TWINT)) {  //POLLING THE I2C interrupt flag to fake an interrupt
              //uart_puts("+++++++++++++++++++ MAIN TWI_VECT thingie\n");
              twi_handler(); 
            }
        #elif defined (USICR)
            if (USISR & ((1<<USISIF)|(1<<USIOIF)|(1<<USIPF))) { 
              usi_statemachine(USISR); 
            }
        #endif

        // handle timer overflow
        #if defined (TIFR)
            if (TIFR & (1<<TOV0)) { 
              TIMER0_OVF_vect(); 
              TIFR = (1<<TOV0); 
             }
        #elif defined (TIFR0)
            if (TIFR0 & (1<<TOV0)) { 
              TIMER0_OVF_vect(); 
              TIFR0 = (1<<TOV0);
            }
        #endif
        if(bootloadLoopCount > 20) {
          #if UART_DEBUG
            uart_puts("Too many loops..\n");
            //uart_putint(cmd);
            //uart_puts("\n");
          #endif
          stay_in_bootloader = false;
        }
    }

    //the why we left section
    //uart_puts("Stay in Bootloader: ");
    //uart_putint((int)stay_in_bootloader);
    //uart_puts("\n");
    //uart_puts("CMD: ");
    //uart_putint((int)cmd);
    //uart_puts("\n");
    //uart_puts("page dirty: ");
    //uart_putint((int)page_dirty);
    //uart_puts("\n");   
    //uart_puts("flash write pending: ");
    //uart_putint((int)flash_write_pending);
    //uart_puts("\n"); 
    // --- disable peripherals before jumping ---
#if defined (TWCR)
    TWCR = 0x00;  // disable TWI but keep address?
#elif defined (USICR)
    USICR = 0x00;
#endif

#if defined (TCCR0)
    TCCR0 = 0x00;  // disable timer0
#elif defined (TCCR0B)
    TCCR0B = 0x00;
#endif

    LED_OFF();

    // --- optional LED pause ---
#if (LED_SUPPORT)
    uint16_t wait = 0x0000;
    do { __asm volatile ("nop"); } while (--wait);
#endif

    // --- final jump to sketch ---
    ;
    //this code never runs:
    if (page_dirty) {
        //uart_puts("PAGE DIRTY, writing now\n");
        write_flash_page();
        page_dirty = 0;
    }
#if UART_DEBUG
    uart_puts("Returning to slave app...\n");
#endif
    jump_to_app();
} /* main */
