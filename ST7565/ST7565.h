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


#include <stdexcept>
#include <stdlib.h>
#include <unistd.h>
#include <bits/stdint-uintn.h>
#include "bcm2835.h"

#define swap(a, b) \
    { \
        uint8_t t = a; \
        a = b; \
        b = t; \
    }

#define BLACK 1
#define WHITE 0

#define LCDWIDTH 128
#define LCDHEIGHT 64

#define CMD_DISPLAY_OFF 0xAE
#define CMD_DISPLAY_ON 0xAF

#define CMD_SET_DISP_START_LINE 0x40
#define CMD_SET_PAGE 0xB0

#define CMD_SET_COLUMN_UPPER 0x10
#define CMD_SET_COLUMN_LOWER 0x00

#define CMD_SET_ADC_NORMAL 0xA0
#define CMD_SET_ADC_REVERSE 0xA1

#define CMD_SET_DISP_NORMAL 0xA6
#define CMD_SET_DISP_REVERSE 0xA7

#define CMD_SET_ALLPTS_NORMAL 0xA4
#define CMD_SET_ALLPTS_ON 0xA5
#define CMD_SET_BIAS_9 0xA2
#define CMD_SET_BIAS_7 0xA3

#define CMD_RMW 0xE0
#define CMD_RMW_CLEAR 0xEE
#define CMD_INTERNAL_RESET 0xE2
#define CMD_SET_COM_NORMAL 0xC0
#define CMD_SET_COM_REVERSE 0xC8
#define CMD_SET_POWER_CONTROL 0x28
#define CMD_SET_RESISTOR_RATIO 0x20
#define CMD_SET_VOLUME_FIRST 0x81
#define CMD_SET_VOLUME_SECOND 0
#define CMD_SET_STATIC_OFF 0xAC
#define CMD_SET_STATIC_ON 0xAD
#define CMD_SET_STATIC_REG 0x0
#define CMD_SET_BOOSTER_FIRST 0xF8
#define CMD_SET_BOOSTER_234 0
#define CMD_SET_BOOSTER_5 1
#define CMD_SET_BOOSTER_6 3
#define CMD_NOP 0xE3
#define CMD_TEST 0xF0

#define sleep_ms(ms) usleep((ms) * 1000)

struct LCDGPIO
{
    int8_t a0, rst;

public:
    explicit LCDGPIO(int8_t A0, int8_t RST) : a0(A0), rst(RST)
    {
        if (!bcm2835_init())
            throw std::runtime_error("bcm2835_init failed. Are you running as root?");

        bcm2835_gpio_set_pud(a0, BCM2835_GPIO_PUD_OFF);
        bcm2835_gpio_set_pud(rst, BCM2835_GPIO_PUD_OFF);
        bcm2835_gpio_fsel(a0, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(rst, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_clr(a0);
        bcm2835_gpio_clr(rst);

        if (!bcm2835_spi_begin())
        {
            throw std::runtime_error("bcm2835_spi_begin failed. Are you running as root?");
        }
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32768);

        lcdReset();
    }

    ~LCDGPIO()
    {
        bcm2835_spi_end();
        bcm2835_close();
    }

    void lcdReset() const
    {
        chipSelect(0);
        bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
        chipSelect(1);
        bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, HIGH);

        // toggle RST low to reset; CS low so it'll listen to us
        bcm2835_gpio_set(rst);
        sleep_ms(500);
        bcm2835_gpio_clr(rst);
    }

    static void chipSelect(int8_t c)
    {
        if (c == 0)
        {
            bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
        }
        else if (c == 1)
        {
            bcm2835_spi_chipSelect(BCM2835_SPI_CS1);
        }
        else
            throw std::runtime_error("invalid chip number " + std::to_string(c));
    }

    inline static void spiWrite(uint8_t c)
    {
        bcm2835_spi_transfer(c);
    }


    inline void sendCommand(uint8_t c) const
    {
        bcm2835_gpio_set(a0);
        spiWrite(c);
    }

    inline void sendData(uint8_t c) const
    {
        bcm2835_gpio_clr(a0);
        spiWrite(c);
    }
};

class ST7565
{
    LCDGPIO lcd_io;

public:
    explicit ST7565(int8_t A0 = 22, int8_t RST = 27) : lcd_io(A0, RST) { clear(); }

    void init();
    void begin(uint8_t contrast);
    void setBrightness(uint8_t val);
    void clearDisplay();
    void clear();
    void display();

    void setPixel(uint8_t x, uint8_t y, uint8_t color, bool updateBox = true);
    uint8_t getPixel(uint8_t x, uint8_t y);
    void fillcircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
    void drawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
    void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
    void fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
    void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
    void drawChar(uint8_t, uint8_t, char);
    void drawString(uint8_t, uint8_t, char *);
    void drawStringP(uint8_t, uint8_t, const char *);

    void drawbitmap(uint8_t x, uint8_t y, const uint8_t * bitmap, uint8_t w, uint8_t h, uint8_t color);

private:
    void setPixelInternal(uint8_t x, uint8_t y, uint8_t color);

    // the memory buffer for the LCD
    uint8_t lcd_buffer[1024];
};
