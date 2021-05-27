#include <random>
#include "ST7565.h"

// the LCD backlight is connected up to a pin so you can turn it on & off
int freeRam();
void testdrawbitmap(const uint8_t * bitmap, uint8_t w, uint8_t h);

ST7565 glcd;

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH 16

// a bitmap of a 16x16 fruit icon
const static unsigned char logo16_glcd_bmp[] = {
    0x30, 0xf0, 0xf0, 0xf0, 0xf0, 0x30, 0xf8, 0xbe, 0x9f, 0xff, 0xf8, 0xc0, 0xc0, 0xc0, 0x80, 0x00,
    0x20, 0x3c, 0x3f, 0x3f, 0x1f, 0x19, 0x1f, 0x7b, 0xfb, 0xfe, 0xfe, 0x07, 0x07, 0x07, 0x03, 0x00,
};

// The setup() method runs once, when the sketch starts
void setup()
{
    // initialize and set the contrast to 0x3B (almost max)
    glcd.begin(0x3B);
    sleep_ms(1);
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
    sleep_ms(5000);
	testdrawbitmap(logo16_glcd_bmp, 16, 16);
    glcd.display(); // show the changes to the buffer
    sleep_ms(2000);
}

int main()
{
    setup();
    pause();
    return 0;
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
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> rand_xpos(0, 127);
    std::uniform_int_distribution<int> rand_deltay(1, 5);

    // initialize
    for (auto & icon : icons)
    {
        icon[XPOS] = rand_xpos(mt);
        icon[YPOS] = 0;
        icon[DELTAY] = rand_deltay(mt);
    }

    while (true)
    {
        // draw each icon
        for (auto & icon : icons)
        {
            glcd.drawbitmap(icon[XPOS], icon[YPOS], logo16_glcd_bmp, w, h, BLACK);
        }
        glcd.display();
        sleep_ms(200);

        // then erase it + move it
        for (auto & icon : icons)
        {
            glcd.drawbitmap(icon[XPOS], icon[YPOS], logo16_glcd_bmp, w, h, 0);
            // move it
            icon[YPOS] += icon[DELTAY];
            // if its gone, reinit
            if (icon[YPOS] > 64)
            {
                icon[XPOS] = rand_xpos(mt);
                icon[YPOS] = 0;
                icon[DELTAY] = rand_deltay(mt);
            }
        }
    }
}
