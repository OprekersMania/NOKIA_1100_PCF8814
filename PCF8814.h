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
 */
 
 
 
#ifndef PCF8814_H
#define PCF8814_H


#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

class PCF8814: public Print {
    public:
        // All the pins can be changed from the default values...
        PCF8814(unsigned char sclk		= 13,   /* clock       (display pin 2) */
                unsigned char sdin		= 11,   /* data-in     (display pin 3) */
                unsigned char sce		= 10,   /* reset       (display pin 8) */
                unsigned char reset		= 9);  /* enable      (display pin 5) */

		// Display initialization
        void begin(unsigned char width=96, unsigned char height=65);
		
        // Place the cursor at position (column, line)...
        void setCursor(unsigned char column, unsigned char line);
		
        // Place the cursor at the zero position (x = 0, y = 0)
        void home();
		
		// Refres the display contrast and rotating to default if changed
		void refresh();
		
        // Erase everything on the display...
        void clear();
        void clearLine();  // ...or just the current line
		
		//set contrast values 0x00 to 0x1F....default, set by lib 0x05
		void setContrast(byte value);
		
		//rotate display 180 ... in fact mirror x and mirror y
		void rotate(bool value); 
		
        // Activate white-on-black mode (whole display)...[true/false]
        void setInverse(bool invert);
		
        // Assign a user-defined glyph (5x8) to an ASCII character (0-31)...
        void createChar(unsigned char chr, const unsigned char *glyph);
		
        // Draw a chart element at the current cursor position...
        void drawColumn(unsigned char lines, unsigned char value);
		
        // Draw a bitmap at the current cursor position...
		void drawPict (uint8_t x, uint8_t y, const uint8_t * picture);   
		
				//*************** FUNCTION IN CPP IS DISABLE, UNCOMMENT TO ACTIVATE *****************
        // void bitmap(const unsigned char *data, unsigned char columns, unsigned char lines);
        // void home();
        // void stop();
        // void setPower(bool on);
        // void display();
        // void noDisplay();

        // Write an ASCII character at the current cursor position (7-bit)...
#if ARDUINO < 100
        virtual void write(uint8_t chr);
#else        
        virtual size_t write(uint8_t chr);
#endif


    private:
        unsigned char pin_sclk;
        unsigned char pin_sdin;
        unsigned char pin_reset;
        unsigned char pin_sce;

        // The size of the display, in pixels...
        unsigned char width;
        unsigned char height;

        // Current cursor position...
        unsigned char column;
        unsigned char line;

        // User-defined glyphs (below the ASCII space character)...
        const unsigned char *custom[' '];

        // Send a command or data to the display...
        void send(unsigned char type, unsigned char data);
		
		void lcd_on();
};


#endif  /* PCF8814_H */

