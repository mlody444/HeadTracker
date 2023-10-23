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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <string.h> // temporary for strlen ONLY
#include <stdlib.h> // utoa ONLY

#include "position.h"
#include "log.h"
#include "oled.h"

#define POINTS_MAX 32

#define POS_TOL 40 // TBD - it's probably to high value
#define FOV_CALIBRATION 5.0

#define ROLL_ADJUST 1
#define ROLL_IGNORE 0

#define COMPASS_ELEMENTS      24    // TBD maybe more elements would be helpfull?
#define COMPASS_PITCH         21.0
#define COMPASS_PITCH_ZERO    COMPASS_PITCH + 1.5
#define COMPASS_PITCH_SHORT   COMPASS_PITCH + 3.0
#define COMPASS_PITCH_LONG    COMPASS_PITCH + 4.0
#define COMPASS_LETTER_LARGE  5
#define COMPASS_LETTER_SMALL  3

#define PI 3.14159265359

struct Compass_Data_T {
  float azimuth;
  bool large;
  char text[2];
};

struct Head_Track_T {
  float azimuth;
  float pitch;
  float roll;
};

struct Relative_Position_T {
  float radius;
  float angle;
};

struct Point_T {
  int16_t x;
  int16_t y;
};

struct Compass_Data_T compass_array[COMPASS_ELEMENTS] = {
    {0.0,   true, 'N', 0},
    {15.0,  false, 0, 0},
    {30.0,  false, 0, 0},
    {45.0,  false, 'N', 'E'},
    {60.0,  false, 0, 0},
    {75.0,  false, 0, 0},
    {90.0,  true,  'E', 0},
    {105.0, false, 0, 0},
    {120.0, false, 0, 0},
    {135.0, false, 'E', 'S'},
    {150.0, false, 0, 0},
    {165.0, false, 0, 0},
    {180.0, true,  'S', 0},
    {195.0, false, 0, 0},
    {210.0, false, 0, 0},
    {225.0, false, 'S', 'W'},
    {240.0, false, 0, 0},
    {255.0, false, 0, 0},
    {270.0, true,  'W', 0},
    {285.0, false, 0, 0},
    {300.0, false, 0, 0},
    {315.0, false, 'W', 'N'},
    {330.0, false, 0, 0},
    {345.0, false, 0, 0}
};

struct Position_Data_T positions_memory[POINTS_MAX];

struct Head_Track_T head_track;

static bool cordinates_within_frame(struct Head_Track_T head, float azimuth, float pitch);
static struct Point_T calculate_cordinates(struct Head_Track_T head, float azimuth, float pitch, bool adjust_roll);
static void process_point(struct Head_Track_T head, struct Position_Data_T position, bool adjust_roll);
static void process_all_points(struct Head_Track_T head, struct Position_Data_T positions[], uint32_t size, bool adjust_roll);
static void process_compass(struct Head_Track_T head, struct Compass_Data_T compass_array[], uint8_t size, bool adjust_roll);

static bool cordinates_within_frame(struct Head_Track_T head, float azimuth, float pitch)
{
  float difference;

  if (azimuth > head.azimuth) {
    difference = azimuth - head.azimuth;
  } else {
    difference = head.azimuth - azimuth;
  }

  if (difference > POS_TOL) {   // if point is wrapping 360 => 0 - calculate real dif
    difference = difference - (360 - POS_TOL);
  }

  if (difference > POS_TOL || difference < 0) {
    return false;
  }

  if (pitch > head.pitch) {
    difference = pitch - head.pitch;
  } else {
    difference = head.pitch - pitch;
  }

  if (difference > POS_TOL) {
    return false;
  }

  return true;
}

static struct Point_T calculate_cordinates(struct Head_Track_T head, float azimuth, float pitch, bool adjust_roll)
{
  struct Point_T point;

  float new_azimuth = head.azimuth - azimuth;
  if (new_azimuth < -180) {
    new_azimuth += 360.0;
  } else if (new_azimuth > 180) {
    new_azimuth -= 360.0;
  }
  float new_pitch = head.pitch - pitch;

  if (adjust_roll){
    float radius = sqrt(new_azimuth * new_azimuth + new_pitch * new_pitch);
    float angle = asin(new_pitch/radius);

    if (new_azimuth < 0) {
      angle = PI - angle;
    }

    angle -= (head.roll / 57.2957795);  // degrees to radians

    new_azimuth = radius * cos(angle);
    new_pitch = radius * sin(angle);
  }

  point.x = (int16_t)(new_azimuth * FOV_CALIBRATION) + X_CENTER;
  point.y = (int16_t)(new_pitch   * FOV_CALIBRATION) + Y_CENTER;
  point.y = 63 - point.y;

  return point;
}

static void distance_to_text(uint32_t distance, char *buf, uint8_t size)
{
  uint32_t x;
  if (size < 4) {
    return;
  }

  if (distance >= 10000) {
    buf[0] = '9';
    buf[1] = '.';
    buf[2] = '9';
    buf[3] = 0;
    return;
  }

  if (distance >= 1000) {
    x = distance / 1000;
    buf[0] = x + '0';
    buf[1] = '.';
    distance -= x * 1000;
    buf[2] = (distance / 100) + '0';
    buf[3] = 0;
    return;
  }

  if (distance >= 100) {
    x = distance / 100;
    buf[0] = x + '0';
    distance -= x * 100;
    x = distance / 10;
    buf[1] = x + '0';
    distance -= x * 10;
    x = distance;
    buf[2] = x + '0';
    buf[3] = 0;
    return;
  }

  if (distance >= 10) {
    x = distance / 10;
    buf[0] = x + '0';
    distance -= x * 10;
    x = distance;
    buf[1] = x + '0';
    buf[2] = 0;
    return;
  }

  if (distance >= 1) {
    buf[0] = distance + '0';
    buf[1] = 0;
    return;
  }
}

static void process_point(struct Head_Track_T head, struct Position_Data_T position, bool adjust_roll)
{
  struct Point_T point = calculate_cordinates(head, position.azimuth, position.pitch, adjust_roll);

  switch(position.point_type) {
  case DIAMOND:
    oled_draw_diamond(point.x, point.y);
    // oled_write_text(point.x - 7, point.y-11, "103", true);
    break;
  case SQUARE:
    oled_draw_square(point.x, point.y);
    break;
  case TRIANGLE:
    oled_draw_triangle(point.x, point.y);
    // oled_write_text(point.x - 7, point.y-11, "4.7", false);
    break;
  case X_SHAPE:
    oled_draw_x_shape(point.x, point.y);
    break;
  case CIRCLE:
    oled_draw_circle(point.x, point.y);
    // oled_write_text(point.x - 7, point.y-11, "451", false);
    break;
  default:
    oled_draw_square(point.x, point.y);
    oled_draw_x_shape(point.x, point.y);
    break;
  }
  // position.name[0] = 'R';
  // position.name[1] = 'O';
  // oled_write_char(point.x-3, point.y-5, position.name[0], 5);
  // oled_write_char(point.x+3, point.y-5, position.name[1], 5);
  oled_write_text(point.x, point.y-5, position.name, 5, true);
  char number[4] = {0};
  distance_to_text(position.distance, number, sizeof(number));
  // oled_write_text(point.x, point.y-11, number, true);
  oled_write_text(point.x, point.y-11, position.name, 14, true);
}

static void process_all_points(struct Head_Track_T head, struct Position_Data_T positions[], uint32_t size, bool adjust_roll)
{
  uint32_t i = 0;

  for (i = 0; i < size; i++) {
    if(cordinates_within_frame(head, positions[i].azimuth, positions[i].pitch)) {
      process_point(head, positions[i], adjust_roll);
    }
  }
}

static void process_compass(struct Head_Track_T head, struct Compass_Data_T compass_array[], uint8_t size, bool adjust_roll)
{
  struct Point_T point_compass;
  struct Point_T point_start;
  struct Point_T point_end;
  uint8_t i = 0;

  for (i = 0; i < size; i++) {
    if (cordinates_within_frame(head, compass_array[i].azimuth, COMPASS_PITCH)) {
      if (compass_array[i].text[0] != 0) {
        point_compass = calculate_cordinates(head, compass_array[i].azimuth, COMPASS_PITCH, adjust_roll);
      }
      point_start   = calculate_cordinates(head, compass_array[i].azimuth, COMPASS_PITCH_ZERO, adjust_roll);

      if (compass_array[i].large) {
        point_end = calculate_cordinates(head, compass_array[i].azimuth, COMPASS_PITCH_LONG, adjust_roll);
      } else {
        point_end = calculate_cordinates(head, compass_array[i].azimuth, COMPASS_PITCH_SHORT, adjust_roll);
      }
      oled_write_line(point_start.x, point_start.y, point_end.x, point_end.y);

      if(compass_array[i].large) {
        switch(compass_array[i].text[0]) { // process large main letter direction
        case 'N':
          oled_write_N(point_start.x, point_start.y - 6, COMPASS_LETTER_LARGE);
          break;
        case 'S':
          oled_write_S(point_start.x, point_start.y - 6, COMPASS_LETTER_LARGE);
          break;
        case 'W':
          oled_write_W(point_start.x, point_start.y - 6, COMPASS_LETTER_LARGE);
          break;
        case 'E':
          oled_write_E(point_start.x, point_start.y - 6, COMPASS_LETTER_LARGE);
          break;
        }
      } else if(compass_array[i].text[1] != 0) {  // process small double letter direction
        switch(compass_array[i].text[0]) {
        case 'N':
          oled_write_N(point_start.x - 4, point_start.y - 5, COMPASS_LETTER_SMALL);
          break;
        case 'S':
          oled_write_S(point_start.x - 4, point_start.y - 5, COMPASS_LETTER_SMALL);
          break;
        case 'W':
          oled_write_W(point_start.x - 4, point_start.y - 5, COMPASS_LETTER_SMALL);
          break;
        case 'E':
          oled_write_E(point_start.x - 4, point_start.y - 5, COMPASS_LETTER_SMALL);
          break;
        }
        switch(compass_array[i].text[1]) {
        case 'N':
          oled_write_N(point_start.x + 4, point_start.y - 5, COMPASS_LETTER_SMALL);
          break;
        case 'S':
          oled_write_S(point_start.x + 4, point_start.y - 5, COMPASS_LETTER_SMALL);
          break;
        case 'W':
          oled_write_W(point_start.x + 4, point_start.y - 5, COMPASS_LETTER_SMALL);
          break;
        case 'E':
          oled_write_E(point_start.x + 4, point_start.y - 5, COMPASS_LETTER_SMALL);
          break;
        }
      }
    }
  }
}

void position_set_pitch(float tilt_new)
{
  head_track.pitch = tilt_new * -1.0;
}
void position_set_roll(float roll_new)
{
  head_track.roll = roll_new;
}
void position_set_azimuth(float pan_new)
{
  if (pan_new < 0) {
    head_track.azimuth = pan_new + 360.0;
  } else {
    head_track.azimuth = pan_new;
  }
}

void position_add_point(struct Position_Data_T point_data)
{
  uint32_t i = 0;
  for (i = 0; i < POINTS_MAX; i++) {
    if (positions_memory[i].name[0] == 0)
    {
      positions_memory[i] = point_data;
      break;
    }
  }
}

static void set_point_name(struct Position_Data_T *point, const char* name)
{
  uint8_t length = strlen(name);
  if (length >= 15) {
    length = 15;
  }

  point->name[length] = 0;

  memcpy(point->name, name, length);
}

void position_Thread()
{
/* waiting until log functions initialize */
  // rt_sleep_ms(3000);
  LOGI("Position thread started");

  oled_init(100);

/* generate initial points to be displayed */
  struct Position_Data_T point_data;
  point_data.azimuth = 359.0;
  point_data.pitch = 10.0;
  point_data.distance = 23;
  set_point_name(&point_data, "Roman");
  point_data.point_type = DIAMOND;
  position_add_point(point_data);

  point_data.azimuth = 5.0;
  point_data.pitch = 5.0;
  point_data.distance = 12;
  set_point_name(&point_data, "Seria");
  point_data.point_type = DIAMOND;
  position_add_point(point_data);

  point_data.azimuth = 30.0;
  point_data.pitch = 6.0;
  point_data.distance = 83;
  set_point_name(&point_data, "Oleq");
  point_data.point_type = DIAMOND;

  position_add_point(point_data);
  point_data.azimuth = 33.0;
  point_data.pitch = 10.0;
  point_data.distance = 102;
  set_point_name(&point_data, "Marko");
  point_data.point_type = DIAMOND;
  position_add_point(point_data);

  position_add_point(point_data);
  point_data.azimuth = 350.0;
  point_data.pitch = 6.0;
  point_data.distance = 47;
  set_point_name(&point_data, "Eugein");
  point_data.point_type = DIAMOND;
  position_add_point(point_data);

  position_add_point(point_data);
  point_data.azimuth = 345.0;
  point_data.pitch = 5.0;
  point_data.distance = 60;
  set_point_name(&point_data, "RANGER");
  point_data.point_type = SQUARE;
  position_add_point(point_data);

  position_add_point(point_data);
  point_data.azimuth = 310.0;
  point_data.pitch = 5.0;
  point_data.distance = 2460;
  set_point_name(&point_data, "HQ");
  point_data.point_type = TRIANGLE;
  position_add_point(point_data);

  // point_data.azimuth = 105.0;
  // point_data.pitch = 2.0;
  // point_data.name[4] = '4';
  // point_data.point_type = X_SHAPE;
  // position_add_point(point_data);

  // point_data.azimuth = 170;
  // point_data.pitch = 8;
  // point_data.name[4] = '5';
  // point_data.point_type = DIAMOND;
  // position_add_point(point_data);

  // point_data.azimuth = 230.0;
  // point_data.pitch = -12.0;
  // point_data.name[4] = '6';
  // point_data.point_type = SQUARE;
  // position_add_point(point_data);

  // point_data.azimuth = 230.0;
  // point_data.pitch = 6.0;
  // point_data.name[4] = '7';
  // point_data.point_type = TRIANGLE;
  // position_add_point(point_data);

  // point_data.azimuth = 300.0;
  // point_data.pitch = -4.0;
  // point_data.name[4] = '8';
  // point_data.point_type = SQUARE;
  // position_add_point(point_data);

  // point_data.azimuth = 345.0;
  // point_data.pitch = 8.0;
  // point_data.name[4] = '9';
  // point_data.point_type = TRIANGLE;
  // position_add_point(point_data);

  // point_data.azimuth = 350.0;
  // point_data.pitch = 2.0;
  // point_data.name[4] = 'A';
  // point_data.point_type = CIRCLE;
  // position_add_point(point_data);

  // point_data.azimuth = 358.0;
  // point_data.pitch = -3.0;
  // point_data.name[4] = 'B';
  // point_data.point_type = DIAMOND;
  // position_add_point(point_data);

  int16_t y = 0;

  while (1) {
    oled_clean();

    process_all_points(head_track, positions_memory, 12, ROLL_ADJUST);
    process_compass(head_track, compass_array, COMPASS_ELEMENTS, ROLL_IGNORE);

    oled_write_pixel(63, 31); // middle point dot - helpfull for development

    oled_update();
    rt_sleep_ms(25);

    // oled_clean();
    // oled_write_line(100, 0,  127, 0);
    // oled_write_line(100, 8,  127, 8);
    // oled_write_line(100, 16, 127, 16);
    // oled_write_line(100, 24, 127, 24);
    // oled_write_line(100, 32, 127, 32);
    // oled_write_line(100, 40, 127, 40);
    // oled_write_line(100, 48, 127, 48);
    // oled_write_line(100, 56, 127, 56);

    // oled_write_text(64, y, "T", 14, true);

    // oled_write_line(0, y, 20, y);
    // y++;
    // if (y == 64) {
    //   y = 0;
    // }

    // oled_update();
    // rt_sleep_ms(250);
  }
}