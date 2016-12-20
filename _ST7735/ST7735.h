/***************************************************
  Code has been ported from Arduino Adafruit library.
  Copyrigths below:

  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ST7735H_
#define _ST7735H_


#include <stdint.h>
#include "DefaultFonts.h"

// some flags for initR() :(
#define INITR_GREENTAB	0x0
#define INITR_REDTAB	0x1
#define INITR_BLACKTAB  0x2

#define ST7735_TFTWIDTH  128
#define ST7735_TFTHEIGHT 160


// 5-6-5 bits colors samples 
#define	ST7735_BLACK   0x0000
#define	ST7735_BLUE    0x001F
#define	ST7735_RED     0xF800
#define	ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0  
#define ST7735_WHITE   0xFFFF

#define INVERT_ON		1
#define INVERT_OFF		0

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

#define PORTRAIT		0
#define LANDSAPE		1
#define PORTRAIT_FLIP	2
#define LANDSAPE_FLIP	3

#define CENTER	-1
#define RIGHT	-2

#define bitmapdatatype uint16_t *

// Initialization for ST7735B screens
void lcd7735_initB(void);
// Initialization for ST7735R screens (green or red tabs)
void lcd7735_initR(uint8_t options);

uint8_t lcd7735_getWidth(void);
uint8_t lcd7735_getHeight(void);

void lcd7735_invertDisplay(const uint8_t mode);
void lcd7735_setRotation(uint8_t m);
void lcd7735_fillScreen(uint16_t color);
// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t lcd7735_Color565(uint8_t r, uint8_t g, uint8_t b);
void lcd7735_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void lcd7735_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void lcd7735_drawPixel(int16_t x, int16_t y, uint16_t color);
void lcd7735_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void lcd7735_setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void lcd7735_pushColor(uint16_t color); // CAUTION!! can't be used separately
void lcd7735_drawFastLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
void lcd7735_drawRect(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2, uint16_t color);
void lcd7735_drawCircle(int16_t x, int16_t y, int radius, uint16_t color);
void lcd7735_fillCircle(int16_t x, int16_t y, int radius, uint16_t color);
void lcd7735_drawBitmap(int x, int y, int sx, int sy, bitmapdatatype data, int scale);
void lcd7735_drawBitmapRotate(int x, int y, int sx, int sy, bitmapdatatype data, int deg, int rox, int roy);
void lcd7735_setFont(uint8_t* font);
void lcd7735_setTransparent(uint8_t s);
void lcd7735_setForeground(uint16_t s);
void lcd7735_setBackground(uint16_t s);
void lcd7735_print(char *st, int x, int y, int deg);

void lcd7735_init_screen(void *font,uint16_t fg, uint16_t bg, uint8_t orientation);
void lcd7735_puts(char *str);
void lcd7735_putc(char c);

#endif /* _ST7735H_ */
