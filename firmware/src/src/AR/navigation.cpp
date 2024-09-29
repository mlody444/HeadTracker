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
#include <string.h>
#include <math.h>

#include "log.h"
#include "position.h"
#include "navigation.h"
#include "common_ar.h"

#define POINTS_MAX 32
#define UPDATE_MAX 10
#define EARTH_R    6371000.0
#define NAME_MAX   16
#define DEL_POS    36000000

#define DEL_LAT 18000000
#define DEL_LON 9000000

struct NAV_POINT {
    char name[NAME_MAX];
    struct NAV_CORDS cords;
    bool update;
    enum Point_Type_T point_type;
};

struct NAV_POINT_V2 {
    char name[NAME_MAX];
    struct NAV_CORDS cords;
    bool update;
    enum Point_Type_T point_type;
};

navi_data_v3_s nav_points_v2[POINTS_MAX];
navi_data_v3_s empty_point = {"", 18000000, 9000000, 0, {ID_EMPTY, 0, 0}, 0xFF, DIAMOND};
struct NAV_CORDS self_pos;

static bool self_position_available();
static void update_all();
static void update_point(uint32_t point);
static void del_point(uint32_t point);
static void del_all_points();

static float calculate_azimuth(float lat_self, float lon_self, float lat_point, float lon_point);
static float calculate_distance(float lat_self, float lon_self, float lat_point, float lon_point);
static float calculate_pitch(uint32_t distance, int32_t alt_diff);
static void init_nav_points_v2();

static bool self_position_available()
{
    if (self_pos.lat < -90.0 || self_pos.lat > 90.0 ||self_pos.lon < -180.0 || self_pos.lon > 180.0)
        return false;

    return true;
}

static void update_all()   // clear flag for all
{
    uint8_t i = 0;

    for (i = 0; i < POINTS_MAX; i++) {
        if (nav_points_v2[i].name[0] != '\0') {
            nav_points_v2[i].update = true;
        }
    }
}

static float calculate_azimuth(float lat_self, float lon_self, float lat_point, float lon_point)
{
    if (lat_self == lat_point && lon_self == lon_point) return 0;
    if (lon_self == lon_point){
        if (lat_self > lat_point)
            return PI;
        else
            return 0;
    }

    float y = cos(lat_point) * sin(lon_point - lon_self);
    float x = cos(lat_self) * sin(lat_point) - sin(lat_self) * cos(lat_point) * cos(lon_point - lon_self);
    float azimuthRadians = atan2(y, x); //potential bug after conversion could be assymptot value

    if (azimuthRadians < 0) {
        azimuthRadians += 2 * PI;
    }

    return azimuthRadians;
}

static float calculate_distance(float lat_self, float lon_self, float lat_point, float lon_point)
{
    float a = sin((lat_point - lat_self) / 2.0);
    float b = sin((lon_point - lon_self) / 2.0);
    float c = a * a + cos(lat_self) * cos(lat_point) * b * b;
    float distanceRadians = 2.0 * asin(sqrt(c));
    return distanceRadians;
}

static float calculate_pitch(uint32_t distance, int32_t alt_diff)
{
    if (distance < 3) {
        return 0.0;
    }

    return ((tan((float)alt_diff / (float)distance)) * RAD_TO_DEG);
}

static void update_point(uint32_t point)
{
    int32_t alt_diff = nav_points_v2[point].cords.alt - self_pos.alt;
    float lat2 = nav_points_v2[point].cords.lat;
    float lon2 = nav_points_v2[point].cords.lon;
    float lat1 = self_pos.lat;
    float lon1 = self_pos.lon;
    uint32_t distance = 0;
    float azimuth = 0.0;
    float pitch = 0.0;

    if (point >= POINTS_MAX) {
        LOGI("ERR - nav update point - point out of range: %d", point);
        return;
    }

    if (nav_points_v2[point].name[0] == '\0') {
        LOGI("ERR - nav update point - missing point: %d", point);
        return;
    }

    if (self_position_available() == false) {
        LOGI("ERR - nav update point - missing self position");
        return;
    }

    if (nav_points_v2[point].ttl == 0) {
        return;
    }

    distance = (uint32_t)(calculate_distance(lat1, lon1, lat2, lon2) * EARTH_R);
    azimuth = calculate_azimuth(lat1, lon1, lat2, lon2) * RAD_TO_DEG;
    pitch = calculate_pitch(distance, alt_diff);

    position_add_point(nav_points_v2[point].name,
                       strlen(nav_points_v2[point].name),
                       azimuth, pitch,
                       distance,
                       nav_points_v2[point].point_type,
                       nav_points_v2[point].nav);

    nav_points_v2[point].update = false;
}

static void del_point(uint32_t point)
{
    if (point >= POINTS_MAX) {
        LOGI("ERR - nav del_point - point out of range: %d", point);
    }

    position_del_point((uint16_t)nav_points_v2[point].nav.id);

    memset(&nav_points_v2[point], 0, sizeof(navi_data_v3_s));
}

static void del_all_points()
{
    uint32_t i = 0;

    for (i = 0; i < POINTS_MAX; i++) {
        if (nav_points_v2[i].name[0] != '\0'){
            del_point(i);
        }
    }
}

static uint32_t search_for_id(uint16_t id)
{
    uint32_t i = 0;
    for (i = 0; i < POINTS_MAX; i++) {
        if (nav_points_v2[i].nav.id == id) {
            return i;
        }
    }

    return POINTS_MAX;
}

static void init_nav_points_v2()
{
    uint32_t i = 0;
    for (i = 0; i < POINTS_MAX; i++) {
        nav_points_v2[i].nav.id = ID_EMPTY;
    }
}

void navigation_add_point_v2(navi_data_v3_s *point)
{
    uint16_t i = 0;

    if (point == nullptr) {
        LOGI("Error - navigation_add_point_v2 point is nullptr");
        return;
    }

    if (point->name[0] == '\0') {
        LOGI("Error - navigation_add_point_v2 point name is empty");
        return;
    }

    if (point->cords.lat > DEL_LAT || point->cords.lat < -DEL_LAT) {
        LOGI("Error - navigation_add_point_v2 lat is incorrect");
        return;
    }

    if (point->cords.lon > DEL_LON || point->cords.lon < -DEL_LON) {
        LOGI("Error - navigation_add_point_v2 lon is incorrect");
        return;
    }

    if (point->cords.alt > 8000 || point->cords.alt < -1000) {
        LOGI("Error - navigation_add_point_v2 alt is incorrect");
        return;
    }

    if (point->nav.id == 0 || point->nav.id == 0xFFF) {
        LOGI("Error - navigation_add_point_v2 ID is incorrect");
        return;
    }

    if (point->point_type >= POINT_TYPE_MAX) {
        LOGI("Error - navigation_add_point_v2 ID is incorrect");
        return;
    }

    LOGI("Adding point = %c%c%c%c, lat = %d, lon = %d, alt = %d", point->name[0], point->name[1], point->name[2], point->name[3],
                                                                  (int32_t)(point->cords.lat * RAD_TO_DEG * 100.0),
                                                                  (int32_t)(point->cords.lon * RAD_TO_DEG * 100.0),
                                                                  point->cords.alt);

    i = search_for_id(point->nav.id); // check if point already exist
    if (i == POINTS_MAX) {
        i = search_for_id(ID_EMPTY); // check if there is memory for new point
    }

    if (i < POINTS_MAX) {    // check if there is memory to store new point
        nav_points_v2[i] = *point;
        return;
    }

    LOGI("Error - navigation_add_point_v2 no space in memory");
}

void navigation_update_myself(NAV_CORDS_RAW myself_raw)
{
    if (myself_raw.lat > DEL_LAT || myself_raw.lat < -DEL_LAT) {
        LOGI("Error - navigation_update_myself lat is incorrect");
        return;
    }

    if (myself_raw.lon > DEL_LON || myself_raw.lon < -DEL_LON) {
        LOGI("Error - navigation_update_myself lon is incorrect");
        return;
    }

    if (myself_raw.alt > 8000 || myself_raw.alt < -1000) {
        LOGI("Error - navigation_update_myself alt is incorrect");
        return;
    }

    self_pos.lat = (DEG_TO_RAD * myself_raw.lat) / DIGITS;
    self_pos.lon = (DEG_TO_RAD * myself_raw.lon) / DIGITS;
    self_pos.alt = myself_raw.alt;
}

void navigation_del_pos(uint16_t id)
{
    uint32_t i = 0;
    if (id == ID_EMPTY) {
        for (i = 0; i < POINTS_MAX; i++) {
           nav_points_v2[i] = empty_point;
        }
        return;
    }

    i = search_for_id(id);

    if (i == POINTS_MAX) {
        LOGI("WARNING - navigation_del_pos there is not point with ID = %d", id);
        return;
    }

    LOGI("navigation_del_pos id = %d, i = %d", id, i);
    position_del_point(id);
    nav_points_v2[i] = empty_point;
}

void navigation_del_all()
{
    navigation_del_pos(ID_EMPTY);
}

void navigation_Thread()
{
    uint32_t update_counter = 0;
    uint32_t pos = 0;
    uint32_t i = 0;

    init_nav_points_v2();

    while (1) {
        update_counter = UPDATE_MAX;
        i = 0;

        // process navigation points
        while (update_counter > 0 && i < POINTS_MAX && self_position_available()) {
            if (nav_points_v2[pos].update == true) {
                update_counter--;
                update_point(pos);
            }

            pos++;
            i++;
            if (pos >= POINTS_MAX) {
                pos = 0;
            }
        }

        rt_sleep_ms(100);
    }
}