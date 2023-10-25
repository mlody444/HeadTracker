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

enum TEXT_ALIGNMENT oled_font_alignment;

uint16_t a_char[5] = {0xFE, 0x05, 0x05, 0x05, 0xFE};
extern const uint16_t *const* pix5;
extern const uint16_t *const* pix7;
extern const uint16_t *const* pix11;
extern const uint16_t *const* pix14;


static const uint16_t *const*get_font_param(uint8_t height, uint8_t *width)
{
  const uint16_t *const*test_font;

  switch (height) {
  case 5:
    if (width != nullptr)
      *width = 4;
    test_font = pix5;
    break;
  case 7:
    if (width != nullptr)
      *width = 4;
    test_font = pix7;
    break;
  case 11:
    if (width != nullptr)
      *width = 7;
    test_font = pix11;
    break;
  case 14:
    if (width != nullptr)
      *width = 8;
    test_font = pix14;
    break;
  default:
    if (width != nullptr)
      *width = 4;
    test_font = pix5;
    break;
  }

  return test_font;
}

union test_MASK {
  uint32_t value;
  uint8_t values[4];
};

void oled_write_char(int16_t x, int16_t y, char letter, uint8_t font_size)
{
  const uint16_t *x_char;
  const uint16_t *const*font;
  union test_MASK mask;
  int16_t y_buf_mask;
  int16_t y_buf_end;
  int16_t buf_index;
  int16_t y_cur_pos;
  uint8_t i = 0;
  uint8_t oled_font_width = 0;
  uint8_t oled_font_size = 8;

  font = get_font_param(font_size, &oled_font_width);

  uint8_t test_y_height = ((font_size + (8 - (y % 8))) + 7) / 8;
  LOGI("test_y_height = %d, y = %d", test_y_height, y);

  x_char = font[letter - 0x20];

/* process multiple Y column of pixels to display char*/

  for (i = 0; i < oled_font_width; i++) {
    if (x < 0) {
      x++;
      continue;
    }
    if (x >= WIDTH)
      break;

    mask.value = (x_char[i] << (8 - (y % 8)));
    y_cur_pos = 7 - (y / 8);
/* process single Y column of pixels */
    uint8_t temp_y;
    uint8_t j = 0;
    for (j = 0; j < test_y_height; j++) {

      y_buf_mask = mask.values[j];

      buf_index = x + y_cur_pos * WIDTH;
      if (buf_index < 0 || buf_index >= 1024) {
        LOGI("y_cur_pos = %d, y_buf_end = %d, x = %d, y = %d, buf_index = %d", y_cur_pos, y_buf_end, x, y, buf_index);
      } else {
        oled_buf[buf_index] |= y_buf_mask;
      }

      y_cur_pos++;
    }

    x++;
  }
}


void oled_write_text(int16_t x, int16_t y, char* text, uint8_t text_size, bool center)
{
  uint8_t text_length = strlen(text);
  uint8_t spaces = count_spaces(text, text_size);

  uint8_t x_shift = 0;

  const uint16_t *const*font;
  uint8_t width;
  font = get_font_param(text_size, &width);
  uint8_t i, j;

  if (center) {
    x -= (text_length * width + text_length - 1) / 2;
  }

  while (*text != NULL) {
    oled_write_char(x, y, *text, text_size);
    x += 1 + width;
    text++;
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

void oled_init(uint32_t delay)
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
  rt_sleep_ms(delay);
}