// ArduinoISP version 201508
// Copyright (c) 2008-2011 Randall Bohn, 2015 Ralph Doncaster
// If you require a license, see 
//     http://www.opensource.org/licenses/bsd-license.php
//
// This sketch turns the Arduino into a AVRISP
// using the following arduino pro mini pins:
//
// pin name:
// slave reset: 5:
// MOSI:        11:
// MISO:        12:
// SCK:         13:
//
// This allows piggy-backing a pro mini on top of the 8-pin tiny AVRs
// such as the tinyx5 and tiny13
//
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat   - shows the programmer is running
// 8: Error       - Lights up if something goes wrong (use red if that makes sense)
// SCK is the LED pin, so the LED will flicker during programming
// 
// November 2015 Ralph Doncaster
// added clock output on pin 6 (PD6) for AVRs with bad clock fuse settings 
//
// August 2015 Ralph Doncaster
// adapted Weihong Guan's fork to work with a pro mini as piggy-prog
// http://nerdralph.blogspot.ca/2015/08/pigggy-prog-project-ideas.html
//
// 23 July 2011 Randall Bohn
// -Address Arduino issue 509 :: Portability of ArduinoISP
// http://code.google.com/p/arduino/issues/detail?id=509
//
// October 2010 by Randall Bohn
// - Write to EEPROM > 256 bytes
// - Better use of LEDs:
// -- Flash LED_PMODE on each flash commit
// -- Flash LED_PMODE while writing EEPROM (both give visual feedback of writing progress)
// - Light LED_ERR whenever we hit a STK_NOSYNC. Turn it off when back in sync.
// - Use pins_arduino.h (should also work on Arduino Mega)
//
// October 2009 by David A. Mellis
// - Added support for the read signature command
// 
// February 2009 by Randall Bohn
// - Added support for writing to EEPROM (what took so long?)
// Windows users should consider WinAVR's avrdude instead of the
// avrdude included with Arduino software.
//
// January 2008 by Randall Bohn
// - Thanks to Amplificar for helping me with the STK500 protocol
// - The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
// - The SPI functions herein were developed for the AVR910_ARD programmer 
// - More information at http://code.google.com/p/mega-isp

#include "SPI.h"
#include <util/delay.h>

#define RESET     5
#define CLOCK     6
#define POWER     14
#define GROUND    8

#define LED_HEARTBEAT    9
#define LED_ERROR    	2

#define HARDWARE_VERSION    2
#define FIRMWARE_MAJOR_VERSION    1
#define FIRMWARE_MINOR_VERSION    42

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20

#define BUFF_LENGTH 256

static bool _error = false;
static bool _programming = false;
uint8_t _buff[BUFF_LENGTH]; // global block storage

typedef struct param
{
    uint8_t signature;
    uint8_t revision;
    uint8_t progtype;
    uint8_t parmode;
    uint8_t polling;
    uint8_t selftimed;
    uint8_t lock_bytes;
    uint8_t fuse_bytes;
    uint8_t flash_poll;
    uint16_t eeprom_poll;
    uint16_t flash_pagesize; // in bytes
    uint16_t eeprom_size;
    uint32_t flash_size;
} parameter;

parameter _param;

void reply(bool has_byte = false, byte val = 0x00, bool send_ok = true);
void pulse(uint8_t pin, uint8_t times);
void avrisp();

// Arduino delay uses timer0 millis timer, which we are using for
// clock output.  We define our own delay instead
void msDelay(uint8_t ms)
{
    while ( ms-- ) _delay_ms(1);
}

void heartbeat()
{
    static bool state;
    static uint16_t timer;
    if (timer > 0x4000)
    {
    	digitalWrite(LED_HEARTBEAT, state);
    	state = !state;
    	timer = 0;
    }
    timer++;

    digitalWrite(LED_ERROR, _error);
}

void setup()
{
    Serial.begin(115200);

    SPI.setDataMode(0);
    SPI.setBitOrder(MSBFIRST);

    // Clock Div 128 == 125kHz SCK clock frequency, slow enough for
    // 1Mhz AVRs
    SPI.setClockDivider(SPI_CLOCK_DIV128);

    pulse(SCK, 2);

    pinMode(LED_ERROR, OUTPUT);
    pulse(LED_ERROR, 2);

    pinMode(LED_HEARTBEAT, OUTPUT);
    pulse(LED_HEARTBEAT, 2);
}

void loop(void)
{
    heartbeat();

    if (Serial.available())
    	avrisp();
}

uint8_t getch()
{
    while (!Serial.available())
    	;
    return Serial.read();
}

void fill(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
    	_buff[i] = getch();
    }
}

void pulse(uint8_t pin, uint8_t times)
{
    while (times--)
    {
    	digitalWrite(pin, HIGH);
    	msDelay(30);
    	digitalWrite(pin, LOW);
    	msDelay(70);
    }
}

uint8_t spiTransfer(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    SPI.transfer(a);
    SPI.transfer(b);
    SPI.transfer(c);
    return SPI.transfer(d);
}

uint8_t spiTransfer(uint8_t a, uint16_t b, uint8_t c)
{
    return spiTransfer(a, highByte(b), lowByte(b), c);
}

bool receiveEop()
{
    bool eop = getch() == CRC_EOP;

    if (eop)
    	Serial.write(STK_INSYNC);
    else
    {
    	_error = true;
    	Serial.write(STK_NOSYNC);
    }

    return eop;
}

void reply(bool has_byte, byte val, bool send_ok)
{
    if (!receiveEop())
    	return;

    if (has_byte)
    	Serial.write(val);

    if (send_ok)
    	Serial.write(STK_OK);
}

void replyVersion(uint8_t c)
{
    switch (c)
    {
    case 0x80:
    	reply(true, HARDWARE_VERSION);
    	break;
    case 0x81:
    	reply(true, FIRMWARE_MAJOR_VERSION);
    	break;
    case 0x82:
    	reply(true, FIRMWARE_MINOR_VERSION);
    	break;
    case 0x93:
    	reply(true, 'S'); // serial programmer
    	break;
    default:
    	reply(true);
    	break;
    }
}

void setParameters()
{
    // call this after reading paramter packet into buff[]
    _param.signature = _buff[0];
    _param.revision = _buff[1];
    _param.progtype = _buff[2];
    _param.parmode = _buff[3];
    _param.polling = _buff[4];
    _param.selftimed = _buff[5];
    _param.lock_bytes = _buff[6];
    _param.fuse_bytes = _buff[7];
    _param.flash_poll = _buff[8];

    _param.eeprom_poll = makeWord(_buff[10], _buff[11]);
    _param.flash_pagesize = makeWord(_buff[12], _buff[13]);
    _param.eeprom_size = makeWord(_buff[14], _buff[15]);

    // 32 bits flashsize (big endian)
    _param.flash_size = _buff[16] * 0x01000000 + _buff[17] * 0x00010000
    		+ _buff[18] * 0x00000100 + _buff[19];
}

void beginProgramming()
{
    
    // power up target
    digitalWrite(GROUND, LOW);
    pinMode(GROUND, OUTPUT);
    digitalWrite(POWER, HIGH);
    pinMode(POWER, OUTPUT);

    // power-on reset delay can be up to 64ms + 14 clk
    msDelay(65);

    pinMode(CLOCK, OUTPUT);
    // Arduino core uses timer/counter0 for millis interrupt, so disable
    // it since we are using it for pin 6 clock output
    TIMSK0 = 0;

    // setup PWM on pin 6 at half CPU clock frequency
    OCR0A = 0;
    // OC0A output, CTC mode 
    TCCR0A = (1<<WGM01) | (1<<COM0A0)| (1<<WGM01);
    TCCR0B = (1<<CS00); // no clock prescale

    SPI.begin();
    // SPI.begin will set SCK to output, which defaults low

    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, LOW);

    msDelay(20);

    spiTransfer(0xAC, 0x53, 0x00, 0x00);
    _programming = true;
}

void endProgramming()
{
    SPI.end();

    // power down target
    //  remove these lines to leave targed powered after programming
    pinMode(GROUND, INPUT);
    pinMode(POWER, INPUT);
    pinMode(CLOCK, INPUT);
    digitalWrite(POWER, LOW);

    TCCR0A = 0;
    TCCR0B = 0;

    pinMode(RESET, INPUT);
    _programming = false;
}

void universal()
{
    uint8_t ch;

    fill(4);
    ch = spiTransfer(_buff[0], _buff[1], _buff[2], _buff[3]);
    reply(true, ch);
}

uint16_t getPage(uint16_t addr)
{
    return addr & ~((_param.flash_pagesize >> 1) - 1);
}

void writeFlash(uint16_t address, uint16_t length)
{
    if (length > _param.flash_pagesize || length > BUFF_LENGTH)
    {
    	_error = true;
    	Serial.write(STK_FAILED);
    	return;
    }

    fill(length);

    if (!receiveEop())
    	return;

    for (uint8_t *p = _buff, word_length = length >> 1, i = 0; i < word_length;
    		i++)
    {
    	spiTransfer(0x40, i, *p++);
    	spiTransfer(0x48, i, *p++);
    }
    spiTransfer(0x4C, getPage(address), 0);

    Serial.write(STK_OK);
}

void writeEeprom(uint16_t address, uint16_t length)
{
    if (length > _param.eeprom_size || length > BUFF_LENGTH)
    {
    	_error = true;
    	Serial.write(STK_FAILED);
    	return;
    }

    fill(length);

    if (!receiveEop())
    	return;

    uint8_t * p = _buff;
    for (uint16_t i = length, addr = address << 1; i--;)
    {
    	spiTransfer(0xC0, addr++, *p++);
    	msDelay(4);
    }
    Serial.write(STK_OK);
}

void programPage(uint16_t address)
{
    uint16_t length = (getch() << 8) | getch(); // It is weird that makeWord does not work here
    uint8_t memtype = getch();

    switch (memtype)
    {
    case 'F':
    	writeFlash(address, length);
    	break;
    case 'E':
    	writeEeprom(address, length);
    	break;
    default:
    	Serial.write(STK_FAILED);
    	break;
    }
}

void readFlashPage(uint16_t address, uint16_t length)
{
    for (uint16_t i = 0; i < length; i += 2)
    {
    	Serial.write(spiTransfer(0x20, address, 0));
    	Serial.write(spiTransfer(0x28, address, 0));
    	address++;
    }
    Serial.write(STK_OK);
}

void readEepromPage(uint16_t address, uint16_t length)
{
    // here again we have a word address
    for (uint16_t addr = address * 2, i = 0; i < length; i++)
    {
    	uint8_t ee = spiTransfer(0xA0, addr++, 0xFF);
    	Serial.write(ee);
    }
    Serial.write(STK_OK);
}

void readPage(uint16_t address)
{
    uint16_t length = (getch() << 8) | getch();

    uint8_t memtype = getch();

    if (!receiveEop())
    	return;

    switch (memtype)
    {
    case 'F':
    	readFlashPage(address, length);
    	break;
    case 'E':
    	readEepromPage(address, length);
    	break;
    default:
    	Serial.write(STK_FAILED);
    	break;
    }
}

void readSignature()
{
    if (!receiveEop())
    	return;

    Serial.write(spiTransfer(0x30, 0x00, 0x00, 0x00));
    Serial.write(spiTransfer(0x30, 0x00, 0x01, 0x00));
    Serial.write(spiTransfer(0x30, 0x00, 0x02, 0x00));

    Serial.write(STK_OK);
}

void avrisp()
{
    static uint16_t address = 0;
    uint8_t ch = getch();
    switch (ch)
    {
    case '0': // signon
    	_error = 0;
    	reply();
    	break;
    case '1':
    	if (receiveEop())
    	{
    		Serial.print("AVR ISP");
    		Serial.write(STK_OK);
    	}
    	break;
    case 'A':
    	replyVersion(getch());
    	break;
    case 'B':
    	fill(20);
    	setParameters();
    	reply();
    	break;
    case 'E': // extended parameters - ignore for now
    	fill(5);
    	reply();
    	break;
    case 'P':
    	_programming ? pulse(LED_ERROR, 3) : beginProgramming();
    	reply();
    	break;
    case 'U': // set address (word)
    	address = getch() | (getch() << 8);
    	reply();
    	break;
    case 0x60: //STK_PROG_FLASH
    	getch();
    	getch();
    	reply();
    	break;
    case 0x61: //STK_PROG_DATA
    	getch();
    	reply();
    	break;
    case 0x64: //STK_PROG_PAGE
    	programPage(address);
    	break;
    case 0x74: //STK_READ_PAGE 't'
    	readPage(address);
    	break;
    case 'V': //0x56
    	universal();
    	break;
    case 'Q': //0x51
    	_error = 0;
    	endProgramming();
    	reply();
    	break;
    case 0x75: //STK_READ_SIGN 'u'
    	readSignature();
    	break;
    	// expecting a command, not CRC_EOP
    	// this is how we can get back in sync
    case CRC_EOP:
    	_error = true;
    	Serial.write(STK_NOSYNC);
    	break;
    default: // anything else we will return STK_UNKNOWN
    	_error = true;
    	if (receiveEop())
    		Serial.write(STK_UNKNOWN);
    	break;
    }
}

