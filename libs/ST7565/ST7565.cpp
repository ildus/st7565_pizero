/*
$Id:$

ST7565 LCD library!

Copyright (C) 2010 Limor Fried, Adafruit Industries

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

// some of this code was written by <cstone@pobox.com> originally; it is in the
public domain.
*/

//#include <Wire.h>
#ifdef __AVR__
#    include <avr/pgmspace.h>
#    include <util/delay.h>
#endif

#ifndef _delay_ms
#    define _delay_ms(t) delay(t)
#endif

#ifndef _BV
#    define _BV(bit) (1 << (bit))
#endif

#include <stdlib.h>

#include "ST7565.h"

uint8_t is_reversed = 1;

// a handy reference to where the pages are on the screen
const uint8_t pagemap[] = {0, 1, 2, 3, 4, 5, 6, 7};

// a 5x7 font table
const extern uint8_t PROGMEM font[];

// reduces how much is refreshed, which speeds it up!
// originally derived from Steve Evans/JCW's mod but cleaned up and
// optimized
#define enablePartialUpdate

#ifdef enablePartialUpdate
static uint8_t xUpdateMin, xUpdateMax, yUpdateMin, yUpdateMax;
#endif

static unsigned char lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };

static uint8_t reverse(uint8_t n) {
   // Reverse the top and bottom nibble then swap them.
   return (lookup[n&0b1111] << 4) | lookup[n>>4];
}


static void updateBoundingBox(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax)
{
#ifdef enablePartialUpdate
    if (xmin < xUpdateMin)
        xUpdateMin = xmin;
    if (xmax > xUpdateMax)
        xUpdateMax = xmax;
    if (ymin < yUpdateMin)
        yUpdateMin = ymin;
    if (ymax > yUpdateMax)
        yUpdateMax = ymax;
#endif
}

void ST7565::drawbitmap(uint8_t x, uint8_t y, const uint8_t * bitmap, uint8_t w, uint8_t h, uint8_t color)
{
    for (uint8_t j = 0; j < h; j++)
    {
        for (uint8_t i = 0; i < w; i++)
        {
            if (pgm_read_byte(bitmap + i + (j / 8) * w) & _BV(j % 8))
            {
                setPixelInternal(x + i, y + j, color);
            }
        }
    }

    updateBoundingBox(x, y, x + w, y + h);
}

void ST7565::drawString(uint8_t x, uint8_t line, char * c)
{
    while (*c != 0)
    {
        drawChar(x, line, *c);
        c++;
        x += 6; // 6 pixels wide
        if (x + 6 >= LCDWIDTH)
        {
            x = 0; // ran out of this line
            line++;
        }
        if (line >= (LCDHEIGHT / 8))
            break; // ran out of space :(
    }
}

void ST7565::drawStringP(uint8_t x, uint8_t line, const char * str)
{
    while (true)
    {
        char c = pgm_read_byte(str++);
        if (!c)
            break;

        drawChar(x, line, c);
        x += 6; // 6 pixels wide
        if (x + 6 >= LCDWIDTH)
        {
            x = 0; // ran out of this line
            line++;
        }
        if (line >= (LCDHEIGHT / 8))
            break; // ran out of space :(
    }
}

void ST7565::drawChar(uint8_t x, uint8_t line, char c)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        lcd_buffer[x + (line * 128)] = pgm_read_byte(font + (c * 5) + i);
        x++;
    }

    updateBoundingBox(x, line * 8, x + 5, line * 8 + 8);
}

// bresenham's algorithm - thx wikpedia
void ST7565::drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
    uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        swap(x0, y0);
        swap(x1, y1);
    }

    if (x0 > x1)
    {
        swap(x0, x1);
        swap(y0, y1);
    }

    // much faster to put the test here, since we've already sorted the points
    updateBoundingBox(x0, y0, x1, y1);

    uint8_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int8_t err = dx / 2;
    int8_t ystep;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    for (; x0 <= x1; x0++)
    {
        if (steep)
        {
            setPixelInternal(y0, x0, color);
        }
        else
        {
            setPixelInternal(x0, y0, color);
        }
        err -= dy;
        if (err < 0)
        {
            y0 += ystep;
            err += dx;
        }
    }
}

// filled rectangle
void ST7565::fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    // stupidest version - just pixels - but fast with internal buffer!
    for (uint8_t i = x; i < x + w; i++)
    {
        for (uint8_t j = y; j < y + h; j++)
        {
            setPixelInternal(i, j, color);
        }
    }

    updateBoundingBox(x, y, x + w, y + h);
}

// draw a rectangle
void ST7565::drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    // stupidest version - just pixels - but fast with internal buffer!
    for (uint8_t i = x; i < x + w; i++)
    {
        setPixelInternal(i, y, color);
        setPixelInternal(i, y + h - 1, color);
    }
    for (uint8_t i = y; i < y + h; i++)
    {
        setPixelInternal(x, i, color);
        setPixelInternal(x + w - 1, i, color);
    }

    updateBoundingBox(x, y, x + w, y + h);
}

// draw a circle outline
void ST7565::drawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
    updateBoundingBox(x0 - r, y0 - r, x0 + r, y0 + r);

    int8_t f = 1 - r;
    int8_t ddf_x = 1;
    int8_t ddf_y = -2 * r;
    int8_t x = 0;
    int8_t y = r;

    setPixelInternal(x0, y0 + r, color);
    setPixelInternal(x0, y0 - r, color);
    setPixelInternal(x0 + r, y0, color);
    setPixelInternal(x0 - r, y0, color);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddf_y += 2;
            f += ddf_y;
        }
        x++;
        ddf_x += 2;
        f += ddf_x;

        setPixelInternal(x0 + x, y0 + y, color);
        setPixelInternal(x0 - x, y0 + y, color);
        setPixelInternal(x0 + x, y0 - y, color);
        setPixelInternal(x0 - x, y0 - y, color);

        setPixelInternal(x0 + y, y0 + x, color);
        setPixelInternal(x0 - y, y0 + x, color);
        setPixelInternal(x0 + y, y0 - x, color);
        setPixelInternal(x0 - y, y0 - x, color);
    }
}

void ST7565::fillcircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
    updateBoundingBox(x0 - r, y0 - r, x0 + r, y0 + r);

    int8_t f = 1 - r;
    int8_t ddf_x = 1;
    int8_t ddf_y = -2 * r;
    int8_t x = 0;
    int8_t y = r;

    for (uint8_t i = y0 - r; i <= y0 + r; i++)
    {
        setPixelInternal(x0, i, color);
    }

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddf_y += 2;
            f += ddf_y;
        }
        x++;
        ddf_x += 2;
        f += ddf_x;

        for (uint8_t i = y0 - y; i <= y0 + y; i++)
        {
            setPixelInternal(x0 + x, i, color);
            setPixelInternal(x0 - x, i, color);
        }
        for (uint8_t i = y0 - x; i <= y0 + x; i++)
        {
            setPixelInternal(x0 + y, i, color);
            setPixelInternal(x0 - y, i, color);
        }
    }
}

void ST7565::setPixelInternal(uint8_t x, uint8_t y, uint8_t color)
{
    setPixel(x, y, color, true);
}

// the most basic function, set a single pixel
void ST7565::setPixel(uint8_t x, uint8_t y, uint8_t color, bool updateBox)
{
    if ((x >= LCDWIDTH) || (y >= LCDHEIGHT))
        return;

    // x is which column
    if (color)
        lcd_buffer[x + (y / 8) * 128] |= _BV(7 - (y % 8));
    else
        lcd_buffer[x + (y / 8) * 128] &= ~_BV(7 - (y % 8));

    if (updateBox)
        updateBoundingBox(x, y, x, y);
}

// the most basic function, get a single pixel
uint8_t ST7565::getPixel(uint8_t x, uint8_t y)
{
    if ((x >= LCDWIDTH) || (y >= LCDHEIGHT))
        return 0;

    return (lcd_buffer[x + (y / 8) * 128] >> (7 - (y % 8))) & 0x1;
}

void ST7565::begin(uint8_t contrast)
{
    clear();
    init();
    sendCommand(CMD_DISPLAY_ON);
    sendCommand(CMD_SET_ALLPTS_NORMAL);
    setBrightness(contrast);
}

void ST7565::init()
{
    // set pin directions
    pinMode(sid, OUTPUT);
    pinMode(sclk, OUTPUT);
    pinMode(a0, OUTPUT);
    pinMode(rst, OUTPUT);
    pinMode(cs1, OUTPUT);
    pinMode(cs2, OUTPUT);

    // toggle RST low to reset; CS low so it'll listen to us
    if (cs1 > 0)
        digitalWrite(cs1, LOW);

    if (cs2 > 0)
        digitalWrite(cs2, HIGH);

    digitalWrite(rst, LOW);
    _delay_ms(500);
    digitalWrite(rst, HIGH);

    // LCD bias select
    sendCommand(CMD_SET_BIAS_7); // 0xA3
    // ADC select
    sendCommand(CMD_SET_ADC_NORMAL); // 0xA0
    // SHL select
    sendCommand(CMD_SET_COM_NORMAL); // 0xC0
    // ReadModifyWrite clear
    sendCommand(CMD_RMW_CLEAR); // 0xEE *
    // Display normal
    sendCommand(CMD_SET_DISP_NORMAL); // 0xA6 *

    // Initial display line
    sendCommand(CMD_SET_DISP_START_LINE); // 0x40

    // turn on voltage converter (VC=1, VR=0, VF=0)
    sendCommand(CMD_SET_POWER_CONTROL | 0x4); // 0x28 | 0x4
    // wait for 50% rising
    _delay_ms(50);

    // turn on voltage regulator (VC=1, VR=1, VF=0)
    sendCommand(CMD_SET_POWER_CONTROL | 0x6); // 0x28 | 0x6
    // wait >=50ms
    _delay_ms(50);

    // turn on voltage follower (VC=1, VR=1, VF=1)
    sendCommand(CMD_SET_POWER_CONTROL | 0x7); // 0x28 | 0x7
    // wait
    _delay_ms(10);

    // set lcd operating voltage (regulator resistor, ref voltage resistor)
    sendCommand(CMD_SET_RESISTOR_RATIO | 0b110); // 0x20 | 0x6

    //                end  disp            bright   power              bright
    // 0xA3 0xA0 0xC0 0xEE 0xA6 0xEC 0x37 0x81 0x3B 0x2F --- pause -- 0x81 0x2C
    // initial display line
    // set page address
    // set column address
    // write display data

    // set up a bounding box for screen updates

    clearDisplay();
    updateBoundingBox(0, 0, LCDWIDTH - 1, LCDHEIGHT - 1);
}

inline void ST7565::spiWrite(uint8_t c)
{
#if not defined(_VARIANT_ARDUINO_DUE_X_) && not defined(_VARIANT_ARDUINO_ZERO_)
    shiftOut(sid, sclk, MSBFIRST, c);
#else
    int8_t i;
    for (i = 7; i >= 0; i--)
    {
        digitalWrite(sclk, LOW);
        delayMicroseconds(5); // need to slow down the data rate for Due and Zero
        if (c & _BV(i))
            digitalWrite(sid, HIGH);
        else
            digitalWrite(sid, LOW);
        //      delayMicroseconds(5);      //need to slow down the data rate for Due
        //      and Zero
        digitalWrite(sclk, HIGH);
    }
#endif
}
void ST7565::sendCommand(uint8_t c)
{
    digitalWrite(a0, LOW);

    spiWrite(c);
}

void ST7565::sendData(uint8_t c)
{
    digitalWrite(a0, HIGH);

    spiWrite(c);
}

void ST7565::setBrightness(uint8_t val)
{
    sendCommand(CMD_SET_VOLUME_FIRST);
    sendCommand(CMD_SET_VOLUME_SECOND | (val & 0x3f));
}

void ST7565::display()
{
    uint8_t col, maxcol, p;

    /*
  Serial.print("Refresh ("); Serial.print(xUpdateMin, DEC);
  Serial.print(", "); Serial.print(xUpdateMax, DEC);
  Serial.print(","); Serial.print(yUpdateMin, DEC);
  Serial.print(", "); Serial.print(yUpdateMax, DEC); Serial.println(")");
  */

    for (p = 0; p < 8; p++)
    {
#ifdef enablePartialUpdate
        // check if this page is part of update
        if (yUpdateMin >= ((p + 1) * 8))
        {
            continue; // nope, skip it!
        }
        if (yUpdateMax < p * 8)
        {
            break;
        }
#endif

        sendCommand(CMD_SET_PAGE | pagemap[p]);

#ifdef enablePartialUpdate
        col = xUpdateMin;
        maxcol = xUpdateMax;
#else
        // start at the beginning of the row
        col = 0;
        maxcol = LCDWIDTH - 1;
#endif

        sendCommand(CMD_SET_COLUMN_LOWER | (col & 0xf));
        sendCommand(CMD_SET_COLUMN_UPPER | ((col >> 4) & 0x0F));
        sendCommand(CMD_RMW);

        for (; col <= maxcol; col++)
        {
            if (is_reversed)
                sendData(reverse(lcd_buffer[(128 * p) + col]));
            else
                sendData(lcd_buffer[(128 * p) + col]);
        }
    }

#ifdef enablePartialUpdate
    xUpdateMin = LCDWIDTH - 1;
    xUpdateMax = 0;
    yUpdateMin = LCDHEIGHT - 1;
    yUpdateMax = 0;
#endif
}

// clear everything
void ST7565::clear()
{
    memset(lcd_buffer, 0, 1024);
    updateBoundingBox(0, 0, LCDWIDTH - 1, LCDHEIGHT - 1);
}

// this doesnt touch the buffer, just clears the display RAM - might be handy
void ST7565::clearDisplay()
{
    uint8_t p, c;

    for (p = 0; p < 8; p++)
    {
        sendCommand(CMD_SET_PAGE | p);
        for (c = 0; c < 129; c++)
        {
            sendCommand(CMD_SET_COLUMN_LOWER | (c & 0xf));
            sendCommand(CMD_SET_COLUMN_UPPER | ((c >> 4) & 0xf));
            sendData(0x0);
        }
    }
}
