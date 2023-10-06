/*
 * This file is part of AR project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdio.h>
#include <stdlib.h>
#include <zephyr.h>
#include <device.h>
#include <display/cfb.h>
#include <drivers/display.h>
#include <kernel.h>

#include "oled.h"
#include "log.h"

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#define POINT_SIZE 8
#define POINT_HALF_SIZE ((int16_t) POINT_SIZE / 2)

uint8_t oled_buf[1024] = {0};

const int16_t WIDTH = 128;        ///< This is the 'raw' oled width - never changes
const int16_t HEIGHT = 64;       ///< This is the 'raw' oled height - never changes

static const struct device *oled = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
uint16_t color = 1;

const struct display_buffer_descriptor buf_desc = {
    .buf_size = 128 * 64,
    .width = 128,
    .height = 64,
    .pitch = 128
};

uint8_t buf[1024] = {0};

static uint8_t count_spaces(char* text, uint8_t size);

static uint8_t count_spaces(char* text, uint8_t size)
{
  uint8_t spaces = 0;
  uint8_t i = 0;

  for (i = 0; i < size; i++) {
    if (text[i] == ' ')
      spaces++;
  }

  return spaces;
}

void oled_write_pixel(int16_t x, int16_t y)
{
  if (oled_buf) {
    if ((x < 0) || (y < 0) || (x >= WIDTH) || (y >= HEIGHT))
      return;

    uint8_t rotation = 0;
    y = (HEIGHT - 1) - y;

    int16_t t;
    switch (rotation) {
    case 1:
      t = x;
      x = WIDTH - 1 - y;
      y = t;
      break;
    case 2:
      x = WIDTH - 1 - x;
      y = HEIGHT - 1 - y;
      break;
    case 3:
      t = x;
      x = y;
      y = HEIGHT - 1 - t;
      break;
    case 4:
      break;
    }

    uint8_t *ptr = &oled_buf[((x % 128) + (y / 8) * 128)];
#ifdef __AVR__
    if (color)
      *ptr |= pgm_read_byte(&GFXsetBit[x & 7]);
    else
      *ptr &= pgm_read_byte(&GFXclrBit[x & 7]);
#else

    y = y % 8;

    if (color)
      *ptr |= 0x1 << (y & 7);
    else
      *ptr &= ~(0x1 << (y & 7));
#endif
  }
}

void oled_write_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
#if defined(ESP8266)
  yield();
#endif
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      oled_write_pixel(y0, x0);
    } else {
      oled_write_pixel(x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

uint8_t oled_font_size = 8;
uint8_t oled_font_width = 5;
enum TEXT_ALIGNMENT oled_font_alignment;

void oled_set_font(uint8_t font_size, enum TEXT_ALIGNMENT alignment)
{
  if (font_size == 8 || // font max size should be 16
      font_size == 8) {
    oled_font_size = font_size;
  } else {
    oled_font_size = 8;
  }

  if (alignment >= TEXT_ALIGNMENT_MAX)
    oled_font_alignment = LEFT;
  else
    oled_font_alignment = alignment;
}

uint16_t a_char[5] = {0xFE, 0x05, 0x05, 0x05, 0xFE};

void oled_write_char(int16_t x, int16_t y, char letter)
{
  uint8_t i = 0;

  int16_t buf_pos;
  int16_t pos;
  int16_t pos_start = (63 - y) / 8;
  if (y > 63)
    pos_start--;
  int16_t pos_end = pos_start + ((oled_font_size + 7) / 8); // here is mistake
  int16_t pos_offset = y % 8;
  int16_t pos_mask;

  buf_pos = x + pos * WIDTH;
  LOGI("pos = %d, pos_start = %d, pos_end = %d, x = %d, y = %d, buf_pos = %d", pos, pos_start, pos_end, x, y, buf_pos);

  for (i = 0; i < oled_font_width; i++) {
    if (x < 0) {
      x++;
      continue;
    }
    if (x >= WIDTH)
      break;

    pos = pos_start;
    LOGI("pos = %d, pos_start = %d, pos_end = %d, x = %d, y = %d, buf_pos = %d", pos, pos_start, pos_end, x, y, buf_pos);
    while(pos <= pos_end) { //process single pixel row in Y column
      if (pos == pos_start) {
        pos_mask = ((uint8_t)(a_char[i] << (8 - pos_offset)));
      } else if (pos == pos_end) {
        pos_mask = ((uint8_t)(a_char[i] >> pos_offset));
      } else {
        // TBD - right now only 8 pix font used
      }

      buf_pos = x + pos * WIDTH;
      if (buf_pos < 0 || buf_pos >= 1024) {
        LOGI("pos = %d, pos_start = %d, pos_end = %d, x = %d, y = %d, buf_pos = %d", pos, pos_start, pos_end, x, y, buf_pos);
      } else {
        oled_buf[buf_pos] |= pos_mask;
      }

      pos++;
    }

    x++;
  }
}


void oled_write_text(int16_t x, int16_t y, char* text, uint8_t text_size)
{
  uint8_t spaces = count_spaces(text, text_size);

  uint8_t i, j;

  for (i = 0; i < 3; i++) {

  }
}

void oled_draw_diamond(int16_t x, int16_t y)
{
  oled_write_line(x - POINT_HALF_SIZE, y                  , x                  , y + POINT_HALF_SIZE);
  oled_write_line(x                  , y + POINT_HALF_SIZE, x + POINT_HALF_SIZE, y                  );
  oled_write_line(x + POINT_HALF_SIZE, y                  , x                  , y - POINT_HALF_SIZE);
  oled_write_line(x                  , y - POINT_HALF_SIZE, x - POINT_HALF_SIZE, y                  );
}

void oled_draw_square(int16_t x, int16_t y)
{
  oled_write_line(x - POINT_HALF_SIZE, y - POINT_HALF_SIZE, x - POINT_HALF_SIZE, y + POINT_HALF_SIZE);
  oled_write_line(x - POINT_HALF_SIZE, y + POINT_HALF_SIZE, x + POINT_HALF_SIZE, y + POINT_HALF_SIZE);
  oled_write_line(x + POINT_HALF_SIZE, y + POINT_HALF_SIZE, x + POINT_HALF_SIZE, y - POINT_HALF_SIZE);
  oled_write_line(x + POINT_HALF_SIZE, y - POINT_HALF_SIZE, x - POINT_HALF_SIZE, y - POINT_HALF_SIZE);
}

void oled_draw_triangle(int16_t x, int16_t y)
{
  oled_write_line(x + POINT_HALF_SIZE, y + POINT_HALF_SIZE, x                  , y - POINT_HALF_SIZE);
  oled_write_line(x                  , y - POINT_HALF_SIZE, x - POINT_HALF_SIZE, y + POINT_HALF_SIZE);
  oled_write_line(x + POINT_HALF_SIZE, y + POINT_HALF_SIZE, x - POINT_HALF_SIZE, y + POINT_HALF_SIZE);
}

void oled_draw_x_shape(int16_t x, int16_t y)
{
  oled_write_line(x - POINT_HALF_SIZE, y + POINT_HALF_SIZE, x + POINT_HALF_SIZE, y - POINT_HALF_SIZE);
  oled_write_line(x - POINT_HALF_SIZE, y - POINT_HALF_SIZE, x + POINT_HALF_SIZE, y + POINT_HALF_SIZE);
}

void oled_draw_circle(int16_t x0, int16_t y0)
 {
  int16_t r = POINT_HALF_SIZE;
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  oled_write_pixel(x0, y0 + r);
  oled_write_pixel(x0, y0 - r);
  oled_write_pixel(x0 + r, y0);
  oled_write_pixel(x0 - r, y0);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    oled_write_pixel(x0 + x, y0 + y);
    oled_write_pixel(x0 - x, y0 + y);
    oled_write_pixel(x0 + x, y0 - y);
    oled_write_pixel(x0 - x, y0 - y);
    oled_write_pixel(x0 + y, y0 + x);
    oled_write_pixel(x0 - y, y0 + x);
    oled_write_pixel(x0 + y, y0 - x);
    oled_write_pixel(x0 - y, y0 - x);
  }
}

void oled_write_N(int16_t x, int16_t y, uint8_t size)
{
  oled_write_line(x - size, y - size, x - size, y + size);
  oled_write_line(x - size, y + size, x + size, y - size);
  oled_write_line(x + size, y - size, x + size, y + size);

}

void oled_write_E(int16_t x, int16_t y, uint8_t size)
{
  oled_write_line(x - size, y - size, x - size, y + size);
  oled_write_line(x - size, y + size, x + size, y + size);
  oled_write_line(x - size, y       , x + size, y       );
  oled_write_line(x - size, y - size, x + size, y - size);
}

void oled_write_S(int16_t x, int16_t y, uint8_t size)
{
  oled_write_line(x - size, y - size, x + size, y - size);
  oled_write_line(x + size, y - size, x + size, y       );
  oled_write_line(x + size, y       , x - size, y       );
  oled_write_line(x - size, y       , x - size, y + size);
  oled_write_line(x - size, y + size, x + size, y + size);
}

void oled_write_W(int16_t x, int16_t y, uint8_t size)
{
  oled_write_line(x - size, y - size, x - size, y + size);
  oled_write_line(x - size, y - size, x       , y       );
  oled_write_line(x       , y       , x + size, y - size);
  oled_write_line(x + size, y - size, x + size, y + size);
}

void oled_update()
{
  display_write(oled, 0, 0, &buf_desc, oled_buf);
}

void oled_clean()
{
  for(uint16_t i = 0; i < 1024; i++) {
    oled_buf[i] = 0;
  }
}

void oled_init()
{
  if (oled == NULL) {
    LOGI("oled pointer is NULL");
  } else {
    LOGI("oled pointer is OK");
  }

  if (!device_is_ready(oled)) {
    LOGI("oled device is not ready");
    return;
  } else {
    LOGI("oled device is ready");
  }

  if (display_write(oled, 0, 0, &buf_desc, oled_buf) != 0) {
    LOGI("could not write to oled");
  } else {
    LOGI("Written to oled");
  }

  if (display_set_contrast(oled, 128) != 0) {
    LOGI("could not set oled contrast");
  } else {
    LOGI("Contrast set");
  }

  oled_write_line(0,0,127,63);
  oled_write_line(0,63,127,0);
  oled_write_line(0,0,127,0);
  oled_write_line(0,0,0,63);
  oled_write_line(127,0,127,63);
  oled_write_line(0,63,127,63);

  display_write(oled, 0, 0, &buf_desc, oled_buf);
  rt_sleep_ms(5000);
}