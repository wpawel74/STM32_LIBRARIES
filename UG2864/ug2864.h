/*
  GOFi2cOLED.h - SSD1306 OLED Driver Library
  2012 Copyright (c) GOF Electronics Co. Ltd.  All right reserved.
 
  Original Author: Limor Fried/Ladyada£¨Adafruit Industries£©
  Modified by: Jimbo.we(www.geekonfire.com)
  
  GeekOnFire invests time and resources providing this open source code, 
	please support GeekOnFire and open-source hardware by purchasing 
	products from GeekOnFire!
	
  This library is derived from Adafruit_GFX library, only for SSD1306 in I2C Mode.
  It is a free software; you can redistribute it and/or modify it 
  under the terms of BSD license, check license.txt for more information.
	All text above must be included in any redistribution.
*/
/**
 * 2016.04.26 - modifications and other improvements for STM32_LIBRARIES: <w_pawel74@tlen.pl>
 */
#ifndef UG2864_H
#define UG2864_H
#include <stm32fxxx_hal.h>
#include <os_compat.h>

/* defined colors *:( */
#define BLACK                0
#define WHITE                1

/**
 * initialize display controller
 */
void UG2864_init(void);

/**
 * set normal mode
 */
void UG2864_setNormalDisplay(void);

/**
 * set inverse mode, content will be automatically changed
 */
void UG2864_setInverseDisplay(void);

/**
 * set page addressing mode
 */
void UG2864_setPageMode(void);

/**
 * set brightness
 * @param Brightness      new value
 */
void UG2864_setBrightness(unsigned char Brightness);

/**
 * set addressing in horizontal orientation
 */
void UG2864_setHorizontalMode(void);

/**
 * set addressing in vertical orientation
 */
void UG2864_setVerticalMode(void);

/**
 * horizontal scrolling properties
 * @param
 */
void UG2864_setHorizontalScrollProperties(BOOLEAN direction,unsigned char startPage, unsigned char endPage, unsigned char scrollSpeed);

/**
 * activate scrolling
 */
void UG2864_activateScroll(void);

/**
 * deactivate scrolling
 */
void UG2864_deactivateScroll(void);

/**
 * get surface height in pixels
 * @return height of surface
 */
uint8_t UG2864_height(void);

/**
 * get surface width in pixels
 * @return width of surface
 */
uint8_t UG2864_width(void);

/**
 * set rotation for surface
 * @param                range 0 ... 3
 */
void UG2864_setRotation(uint8_t r);

/**
 * get current surface rotation
 * @return               range 0 ... 3
 */
uint8_t UG2864_getRotation(void);

/**-------------------------------------------------------------------
 * @drawing methods on local memory
 *                         TEXT METHODS
 *-------------------------------------------------------------------*/

/**
 * set new default text size
 * @param s           text size
 */
void UG2864_setTextSize(uint8_t s);

/**
 * set cursor at position
 * @param x           x coordinate
 * @param y           y coordinate
 */
void UG2864_setCursor(uint8_t x, uint8_t y);

/**
 * set text color and background
 * @param c           font color
 * @param bg          background color
 */
void UG2864_setTextColor(uint8_t c, uint8_t bg);

/**
 * default behaviour for text wrapping
 * @param w           text will be wrapped or not
 */
void UG2864_setTextWrap(BOOLEAN w);

/**
 * write char on surface
 * @param c           char in ascii
 */
void UG2864_write(uint8_t c);

/**
 * draw char on chosen position
 * @param x           x coordinate
 * @param y           y coordinate
 * @param color       color (WHITE or BLACK)
 * @param bg          ????
 * @param size        font size
 */
void UG2864_drawChar(int8_t x, int8_t y, unsigned char c, int8_t color, int8_t bg, uint8_t size);

/**-------------------------------------------------------------------
 * @drawing methods on local memory
 *                       GRAPHICS METHODS
 *-------------------------------------------------------------------*/
/**
 * clear display surface
 */
void UG2864_clearDisplay(void);

/**
 * clear selected area
 * @param x           x coordinate
 * @param y           y coordinate
 * @param w           width
 * @param h           height
 */
void UG2864_clearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/**
 * draw pixel at coordinates on surface
 * @param x           pixel x coordinate
 * @param y           pixel y coordinate
 * @param color       color (WHITE or BLACK)
 */
void UG2864_drawPixel(uint8_t x, uint8_t y, uint8_t color);

/**
 * put bitmap on coordinates
 * @param x           x coordinate
 * @param y           y coordinate
 * @param bitmap      handler to bitmap
 * @param w           bitmap width
 * @param h           bitmap height
 * @param color       color
 */
void UG2864_drawBitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color);

/**
 * draw rectangle at position
 * @param x           x coordinate
 * @param y           y coordinate
 * @param w           bitmap width
 * @param h           bitmap height
 * @param color       color for boundaries
 */
void UG2864_drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);

/**
 * draw rectangle and fill with chosen color
 * @param x           x coordinate
 * @param y           y coordinate
 * @param w           bitmap width
 * @param h           bitmap height
 * @param color       background color
 */
void UG2864_fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);

/**
 * draw line with coordinates
 * @param x0          begin x coordinate
 * @param y0          begin y coordinate
 * @param x1          end x coordinate
 * @param y1          end y coordinate
 * @param color       color for boundaries
 */
void UG2864_drawFastHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
void UG2864_drawFastVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color);
void UG2864_drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);

/**
 * draw triangle on surface
 * @param x0          first x coordinate
 * @param y0          first y coordinate
 * @param x1          second x coordinate
 * @param y1          second y coordinate
 * @param x2          third x coordinate
 * @param y2          third y coordinate
 * @param color       color for boundaries
 */
void UG2864_drawTriangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

/**
 * draw triangle on surface
 * @param x0          first x coordinate
 * @param y0          first y coordinate
 * @param x1          second x coordinate
 * @param y1          second y coordinate
 * @param x2          third x coordinate
 * @param y2          third y coordinate
 * @param color       background color
 */
void UG2864_fillTriangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

void UG2864_drawRoundRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t radius, uint8_t color);
void UG2864_fillRoundRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t radius, uint8_t color);


/**
 * draw circle on surface
 * @param x0          first x coordinate
 * @param y0          first y coordinate
 * @param r           radius
 * @param color       color for boundaries
 */
void UG2864_drawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
void UG2864_drawCircleHelper(uint8_t x0, uint8_t y0, uint8_t r, uint8_t cornername, uint8_t color);

/**
 * draw circle on surface
 * @param x0          first x coordinate
 * @param y0          first y coordinate
 * @param r           radius
 * @param color       background color
 */
void UG2864_fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
void UG2864_fillCircleHelper(uint8_t x0, uint8_t y0, uint8_t r, uint8_t cornername, int16_t delta, uint8_t color);

#endif // UG8264_H
