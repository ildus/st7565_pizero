#include "ST7565.h"

// the LCD backlight is connected up to a pin so you can turn it on & off
int freeRam();
void testdrawbitmap(const uint8_t * bitmap, uint8_t w, uint8_t h);

// pin 22 - Serial data out (SID)
// pin 24 - Serial clock out (SCLK)
// pin 26 - Data/Command select (RS or A0)
// pin 28 - LCD reset (RST)
// pin 30 - LCD chip select (CS1)
// pin 23 - LCD chip select (CS2)
ST7565 glcd(22, 24, 26, 28, 30, 23);

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH 16

// a bitmap of a 16x16 fruit icon
const static unsigned char __attribute__((progmem)) logo16_glcd_bmp[] = {
    0x30, 0xf0, 0xf0, 0xf0, 0xf0, 0x30, 0xf8, 0xbe, 0x9f, 0xff, 0xf8, 0xc0, 0xc0, 0xc0, 0x80, 0x00,
    0x20, 0x3c, 0x3f, 0x3f, 0x1f, 0x19, 0x1f, 0x7b, 0xfb, 0xfe, 0xfe, 0x07, 0x07, 0x07, 0x03, 0x00,
};

// The setup() method runs once, when the sketch starts
void setup()
{
    Serial.begin(9600);

#ifdef __AVR__
    Serial.print(freeRam());
#endif

    // initialize and set the contrast to 0x3B (almost max)
    glcd.begin(0x3B);
    delay(1);
    // after a little delay decrease to 0x2C
    glcd.setBrightness(0x2C);
    glcd.clear();

    // draw a single pixel
    glcd.setPixel(0, 0, BLACK);
    glcd.setPixel(127, 0, BLACK);
    glcd.setPixel(0, 63, BLACK);
    glcd.setPixel(127, 63, BLACK);
    glcd.drawRect(5, 5, 10, 10, BLACK);
    glcd.fillRect(7, 7, 6, 6, BLACK);
    glcd.drawCircle(25, 8, 5, BLACK);
    glcd.drawLine(15, 15, 20, 20, BLACK);
    glcd.drawString(1, 3, const_cast<char*>("hey!!!!"));
    glcd.display(); // show the changes to the buffer
    delay(5000);
	testdrawbitmap(logo16_glcd_bmp, 16, 16);
    glcd.display(); // show the changes to the buffer
    delay(2000);
}

void loop()
{
}

#ifdef __AVR__
// this handy function will return the number of bytes currently free in RAM,
// great for debugging!
int freeRam(void)
{
    extern int __bss_end;
    extern int * __brkval;
    int free_memory;
    if ((int)__brkval == 0)
    {
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    }
    else
    {
        free_memory = ((int)&free_memory) - ((int)__brkval);
    }
    return free_memory;
}
#endif

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

void testdrawbitmap(const uint8_t * bitmap, uint8_t w, uint8_t h)
{
    uint8_t icons[NUMFLAKES][3];
    randomSeed(666); // whatever seed

    // initialize
    for (uint8_t f = 0; f < NUMFLAKES; f++)
    {
        icons[f][XPOS] = random(128);
        icons[f][YPOS] = 0;
        icons[f][DELTAY] = random(5) + 1;
    }

    while (true)
    {
        // draw each icon
        for (uint8_t f = 0; f < NUMFLAKES; f++)
        {
            glcd.drawbitmap(icons[f][XPOS], icons[f][YPOS], logo16_glcd_bmp, w, h, BLACK);
        }
        glcd.display();
        delay(200);

        // then erase it + move it
        for (uint8_t f = 0; f < NUMFLAKES; f++)
        {
            glcd.drawbitmap(icons[f][XPOS], icons[f][YPOS], logo16_glcd_bmp, w, h, 0);
            // move it
            icons[f][YPOS] += icons[f][DELTAY];
            // if its gone, reinit
            if (icons[f][YPOS] > 64)
            {
                icons[f][XPOS] = random(128);
                icons[f][YPOS] = 0;
                icons[f][DELTAY] = random(5) + 1;
            }
        }
    }
}
