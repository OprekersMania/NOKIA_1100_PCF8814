/*
 * PCF8814 - Interface with Philips PCF8814 (or compatible) LCDs.
 *
 * Copyright (c) 2014 Catalin Vasiliu <vasiliu.catalin.mihai@gmail.com>
 
 * Originaly made by Carlos Rodrigues <cefrodrigues@gmail.com> for PCD8544(Nokia 3310 LCD)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.

 //  Remod by TONGBAJIEL 2017
 //  Tanks to igorox for the Pict function with PROGMEM
 //  https://github.com/igorok107/PCF8814
 */
 
#include "PCF8814.h"

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#include <avr/pgmspace.h>

/*
	flags used to understand by lcd if the bits is taken as command or store in ram
*/
#define PCF8814_CMD  LOW
#define PCF8814_DATA HIGH

#include "charset.cpp"

#define LCD_CONTRAST 0x10

#define SCREEN_HEIGHT 68
#define SCREEN_WIDTH 96

/*
  some constants that will be ored with command to effect
    ON : turn on the command
    OFF : turn off the command
    DISPLAY: turn display on/of used with LCD_MODE only, (LCD_MODE|DISPLAY|ON/OFF)
    ALL : turn on all , only used with LCD_MODE , (LCD_MODE|ALL|ON/OFF) use off for normal display
    INVERT : invert pixels, only used with LCD_MODE , (LCD_MODE|INVERT|ON/OFF) , it bring lcd into normal form use off
  *note: you can use (LCD_MODE|ALL/INVERT|OFF)  to bring lcd into normal mode
*/
#define ON 0x01
#define OFF 0x00
#define ALL 0x04
#define INVERT 0x06
#define DISPLAY 0x0E
/*
	Command list of list
        LCD_NOP                                 : no operation
	LCD_MODE				: lcd  mode, LCD_MODE|(ALL/INVERT/DISPLAY|ON/OFF)
	LCD_VOB_MSB				: use LCD_VOB_MSB|0x04 ,the value after | is a mystry,dont mess(previos notice)
	LCD_VOB_LSB				: use LCD_VOB_LSB|(contrast value,0x00 to 0x1F)
	LCD_CHARGE_PUMP_ON 		: read the datasheet , i could nt understand
								voltage muliplication          value
								     X2							0x00
									 X3							0x01
									 X4							0x02
									 X5							0x03
	LCD_RAM_ADDR_MODE		: use LCD_RAM_ADDR_MODE|(conditon ,OFF/ON),write in RAM,
								 OFF : write horizontally (by default)
								 ON : write vertically
	LCD_CHANGE_ROW_LSB				: accessed by LCD_ROW_LSB|(b3 b2 b1 b0), last four bits of the address
	LCD_CHANGE_ROW_MSB				: accessed by LCD_ROW_MSB|(b6 b5 b4),first 3 bits of the address; alias is 0x18
	LCD_CHANGE_COL					: move to col,LCD_COL|(b2 b1 b0)
	LCD_MIRROR_Y			: mirror on y axis , use(LCD_MIRROR_Y| condition 0x08 or OFF)
								turn on/enable mirroring, conditon->0x08 , dont use ON because its 0x01
								turn off/disable mirroring, conditon->OFF
	LCD_MIRROR_X			: turn on mirroring on x axis . this is a speical instruction & 
                                          i couldt found|dont exists reset counter; its alias is 0xA0,didnt worked,
                                          and datasheet says , NOP: MX is pad selected?
	LCD_EXT_OSC				: use a external oscillator (LCD_EXT_OSC|ON / OFF)
	LCD_SOFT_RESET			: internal or software reset
	LCD_FRAME_FREQ			: frame frequency | see datasheet page 29 table 13
	LCD_PARTIAL_DISPLAY		: set partial display MUX RATE | see datasheet page 29 table 14 and 15
	
 * special instruction: use 1 not ON for enabling LCD_MIRROR_X
*/
#define LCD_NOP 0xE3
#define LCD_MODE 0xA0
#define LCD_VOB_MSB 0x20
#define LCD_VOB_LSB 0x80
#define LCD_CHARGE_PUMP_ON 0x2F
#define LCD_RAM_ADDR_MODE 0xAA
#define LCD_CHANGE_ROW_LSB 0x00
#define LCD_CHANGE_ROW_MSB 0x10
#define LCD_CHANGE_COL 0xB0
#define LCD_MIRROR_Y 0xC0
#define LCD_MIRROR_X 0xA0
#define LCD_EXT_OSC 0x3A
#define LCD_SOFT_RESET 0xE2
#define LCD_FRAME_FREQ 0xEC				// (for this i use 80Hz ) 
#define LCD_PARTIAL_DISPLAY 0xD0		// partial display with MUX RATE 1:65

//#define LCD_DATA_DISPLAY_LEN , read page no 27
//#define LCD_FACTORY_DEFAULT ,read page no 27 of datasheet
//#define LCD_REFRESH_RATE ,read page 27,29 of datasheet
/*----------------------------------------------------------------------------------------------*/
#define CHAR_WIDTH 6
#define CHAR_HEIGHT 8


#define lcd_gotoyx(ROW,COL)\ 
    lcd_row(ROW);\
	lcd_col(COL);

static uint8_t lcd_memory[SCREEN_WIDTH][(SCREEN_HEIGHT/8)+1];
static uint8_t lcd_xcurr, lcd_ycurr;

PCF8814::PCF8814(unsigned char sclk, unsigned char sdin, unsigned char sce, unsigned char reset):
    pin_sclk(sclk),
    pin_sdin(sdin),
    pin_sce(sce),
    pin_reset(reset)
{}


void PCF8814::begin(unsigned char width, unsigned char height)
{
    this->width = width;
    this->height = height;

    this->column = 0;
    this->line = 0;

    // Sanitize the custom glyphs...
    memset(this->custom, 0, sizeof(this->custom));

    pinMode(this->pin_sclk,  OUTPUT);
    pinMode(this->pin_sdin,  OUTPUT);
    pinMode(this->pin_reset, OUTPUT);
    pinMode(this->pin_sce,   OUTPUT);

	digitalWrite(this->pin_reset, HIGH);
	digitalWrite(this->pin_sce,   HIGH);
	digitalWrite(this->pin_sclk,   LOW);
	
    this->send(PCF8814_CMD, LCD_SOFT_RESET); 
    this->send(PCF8814_CMD, LCD_EXT_OSC); 
    this->send(PCF8814_CMD, LCD_FRAME_FREQ); 
    this->send(PCF8814_CMD, LCD_PARTIAL_DISPLAY); 
	this->send(PCF8814_CMD, LCD_CHARGE_PUMP_ON);
	
	this->setContrast(LCD_CONTRAST);
	rotate(false);
	this->lcd_on();
	delay(200);
	this->clear();
}

void PCF8814::send(unsigned char type, unsigned char data)
{
    digitalWrite(this->pin_sdin, type);
    digitalWrite(this->pin_sce, LOW);
	digitalWrite(this->pin_sclk, HIGH);
	digitalWrite(this->pin_sclk, LOW);
    shiftOut(this->pin_sdin, this->pin_sclk, MSBFIRST, data);
    digitalWrite(this->pin_sce, HIGH);
}

void PCF8814::setCursor(unsigned char column, unsigned char line)
{
    this->column = (column % this->width);
    this->line = (line % (this->height/9 + 1));

	this->send(PCF8814_CMD, LCD_CHANGE_ROW_LSB | ( this->column & 0x0F));
	this->send(PCF8814_CMD, LCD_CHANGE_ROW_MSB | ( (this->column >> 4) & 0x07 ));
    this->send(PCF8814_CMD, LCD_CHANGE_COL | ( this->line & 0x0F ));
}

void PCF8814::home()
{
	this->setCursor(0,0);
}

void PCF8814::refresh()
{
    this->clear();
	this->setContrast(LCD_CONTRAST);   // CONTRAST default 0x10
	rotate(false);
	this->lcd_on();
}

void PCF8814::clear()
{
	this->setCursor(0, 0);
	int index;
	for(index=0; index < 864; index++)
		this->send(PCF8814_DATA, 0x00);
	
	delay(200);
}

void PCF8814::clearLine()
{
    this->setCursor(0, this->line);

    for (unsigned char i = 0; i < this->width; i++) {
        this->send(PCF8814_DATA, 0x00);
    }

    this->setCursor(0, this->line);
}

void PCF8814::setContrast(byte value)
{
	this->send(PCF8814_CMD, LCD_VOB_MSB|0x04);
	this->send(PCF8814_CMD, LCD_VOB_LSB|(value & 0x1F));
}

void PCF8814::rotate(bool value)
{
	if (value == true)
	{
		this->send(PCF8814_CMD, LCD_MIRROR_Y|OFF);
		this->send(PCF8814_CMD, LCD_MIRROR_X|ON);
	}
	else 
	{
		this->send(PCF8814_CMD, LCD_MIRROR_Y|0x08);
		this->send(PCF8814_CMD, LCD_MIRROR_X|OFF);
	}
}

void PCF8814::setInverse(bool invert)
{
	if (invert == true) {
	this->send(PCF8814_CMD, LCD_MODE|INVERT|ON);
	this->send(PCF8814_CMD, LCD_MODE|ALL|OFF);
	this->send(PCF8814_CMD, LCD_MODE|DISPLAY|ON);
	}
	else {
		lcd_on();
	}
}

void PCF8814::lcd_on()
{
	this->send(PCF8814_CMD, LCD_MODE|DISPLAY|ON);
	this->send(PCF8814_CMD, LCD_MODE|ALL|OFF);
	this->send(PCF8814_CMD, LCD_MODE|INVERT|OFF);
}

void PCF8814::createChar(unsigned char chr, const unsigned char *glyph)
{
    // ASCII 0-31 only...
    if (chr >= ' ') {
        return;
    }
    this->custom[chr] = glyph;
}

void PCF8814::drawColumn(unsigned char lines, unsigned char value)
{
    unsigned char scolumn = this->column;
    unsigned char sline = this->line;

    // Keep "value" within range...
    if (value > lines*8) {
        value = lines*8;
    }

    // Find the line where "value" resides...
    unsigned char mark = (lines*8 - 1 - value)/8;
    
    // Clear the lines above the mark...
    for (unsigned char line = 0; line < mark; line++) {
        this->setCursor(scolumn, sline + line);
        this->send(PCF8814_DATA, 0x00);
    }

    // Compute the byte to draw at the "mark" line...
    unsigned char b = 0xff;
    for (unsigned char i = 0; i < lines*8 - mark*8 - value; i++) {
        b <<= 1;
    }

    this->setCursor(scolumn, sline + mark);
    this->send(PCF8814_DATA, b);

    // Fill the lines below the mark...
    for (unsigned char line = mark + 1; line < lines; line++) {
        this->setCursor(scolumn, sline + line);
        this->send(PCF8814_DATA, 0xff);
    }
  
    // Leave the cursor in a consistent position...
    this->setCursor(scolumn + 1, sline); 
}

void PCF8814::drawPict (uint8_t x, uint8_t y, const uint8_t * picture)
{
	uint8_t pict_width = pgm_read_byte(&picture[0]);  // ширина спрайта в пикселах
	uint8_t pict_height = pgm_read_byte(&picture[1]); // высота спрайта в пикселах
	uint8_t pict_height_bank=pict_height / 8+((pict_height%8)>0?1:0); // высота спрайта в банках
	uint8_t y_pos_in_bank = y/8 + ((y%8)>0?1:0);		// позиция по y в банках (строках по 8 пикс.)

	int adr = 2; // индекс текущего байта в массиве с картинкой
	uint8_t i;
	for (i=0; i< pict_height_bank; i++)
	{ // проход построчно (по банкам)

		if (i<((SCREEN_HEIGHT/8)+1)) // не выводить картинку за пределами экрана
		{
			//позиционирование на новую строку
			lcd_xcurr=x;
			lcd_ycurr=y_pos_in_bank + i;

			this->send(PCF8814_CMD,(0xB0|((y_pos_in_bank+i)&0x0F))); // установка адреса по Y: 0100 yyyy
			this->send(PCF8814_CMD,(0x00|(x&0x0F)));      // установка адреса по X: 0000 xxxx - биты (x3 x2 x1 x0)
			this->send(PCF8814_CMD,(0x10|((x>>4)&0x07))); // установка адреса по X: 0010 0xxx - биты (x6 x5 x4)

			//вывод строки
			uint8_t j;
			for ( j = 0; j < pict_width; j++ )
			{
				if ((x+j) < SCREEN_WIDTH) this->send(PCF8814_DATA,pgm_read_byte(&picture[adr])); // не выводить картинку за пределами экрана
				adr++;
			}
		}
	}
}


#if ARDUINO < 100
void PCF8814::write(uint8_t chr)
#else
size_t PCF8814::write(uint8_t chr)
#endif
{
    // ASCII 7-bit only...
    if (chr >= 0x80) {
#if ARDUINO < 100
        return;
#else
        return 0;
#endif
    }

    const unsigned char *glyph;
    unsigned char pgm_buffer[5];

    if (chr >= ' ') {
        // Regular ASCII characters are kept in flash to save RAM...
        memcpy_P(pgm_buffer, &charset[chr - ' '], sizeof(pgm_buffer));
        glyph = pgm_buffer;
    } else {
        // Custom glyphs, on the other hand, are stored in RAM...
        if (this->custom[chr]) {
            glyph = this->custom[chr];
        } else {
            // Default to a space character if unset...
            memcpy_P(pgm_buffer, &charset[0], sizeof(pgm_buffer));
            glyph = pgm_buffer;
        }
    }

    // Output one column at a time...
    for (unsigned char i = 0; i < 5; i++) {
        this->send(PCF8814_DATA, glyph[i]);
    }

    // One column between characters...
    this->send(PCF8814_DATA, 0x00);

    // Update the cursor position...
    this->column = (this->column + 6) % this->width;

    if (this->column == 0) {
        this->line = (this->line + 1) % (this->height/9 + 1);
    }

#if ARDUINO >= 100
    return 1;
#endif
}

// ************************
//    USE IF U NEED, DONT FORGET TO UNCOMENT CALLING FUNCTION IN PCF8814.h 
// ************************

/*
void PCF8814::stop()
{
    this->clear();
    this->send(PCF8814_CMD, LCD_MODE|DISPLAY|OFF);   //
	this->send(PCF8814_CMD, LCD_MODE|ALL|ON);        //
}

void PCF8814::setPower(bool on)
{
	if (on == false){
		this->send(PCF8814_CMD, LCD_MODE|DISPLAY|OFF );//
		this->send(PCF8814_CMD, LCD_MODE|ALL|ON);
	}
	else {
		this->send(PCF8814_CMD, LCD_MODE|DISPLAY|ON );//
		this->send(PCF8814_CMD, LCD_MODE|ALL|OFF);
		this->send(PCF8814_CMD, LCD_MODE|INVERT|OFF);
	}
}

inline void PCF8814::display()
{
    this->setPower(true);
}

inline void PCF8814::noDisplay()
{
    this->setPower(false);
}


void PCF8814::home()
{
    this->setCursor(0, this->line);
}

void PCF8814::bitmap(const unsigned char *data, unsigned char columns, unsigned char lines)
{
    unsigned char scolumn = this->column;
    unsigned char sline = this->line;

    // The bitmap will be clipped at the right/bottom edge of the display...
    unsigned char mx = (scolumn + columns > this->width) ? (this->width - scolumn) : columns;
    unsigned char my = (sline + lines > this->height/8) ? (this->height/8 - sline) : lines;

    for (unsigned char y = 0; y < my; y++) {
        this->setCursor(scolumn, sline + y);

        for (unsigned char x = 0; x < mx; x++) {
            this->send(PCF8814_DATA, data[y * columns + x]);
        }
    }

    // Leave the cursor in a consistent position...
    this->setCursor(scolumn + columns, sline);
}
*/

/* vim: set expandtab ts=4 sw=4: */
