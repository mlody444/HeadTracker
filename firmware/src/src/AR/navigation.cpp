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
#define DIGITS     100000.0
#define COMB_DEG_TO_RAD 0.0000001745329251994329576f
#define EARTH_R         6371000.0
#define NAME_MAX   16
#define NAV_DELAY  100

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

struct NAV_POINT nav_points[POINTS_MAX];
navi_data_v3_s nav_points_v2[POINTS_MAX];
navi_data_v3_s empty_point;
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
        if (nav_points[i].name[0] != '\0') {
            nav_points[i].update = true;
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
    int32_t alt_diff = nav_points[point].cords.alt - self_pos.alt;
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

    if (nav_points[point].name[0] == '\0') {
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

    position_add_point(nav_points[point].name, strlen(nav_points[point].name), azimuth, pitch, distance, nav_points[point].point_type);
    nav_points[point].update = false;
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
        if (nav_points[i].name[0] != '\0'){
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
        nav_points_v2[i].nav.id = 0xFFF;
    }
}

static void ttl_decrement()
{
    uint32_t i = 0;
    for (i = 0; i < POINTS_MAX; i++) {
        if (nav_points_v2[i].ttl == 0xFF || nav_points_v2[i].ttl == 0 ||
            nav_points_v2[i].nav.id == NAV_ID_EMPTY) {
            continue;
        }
        nav_points_v2[i].ttl--;
        if (nav_points_v2[i].ttl == 0) {

        }
    }
}

void navigation_add_point_v2(navi_data_v3_s *point)
{
    uint8_t i = 0;

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

    i = search_for_id(point->nav.id); // check if point already exist
    if (i == NAV_ID_EMPTY) {
        i = search_for_id(NAV_ID_EMPTY); // check if there is memory for new point
    }

    if (i == NAV_ID_EMPTY) {
        nav_points_v2[i] = *point;
    }

    LOGI("Error - navigation_add_point_v2 no space in memory");
}

void navigation_update_self_pos(int32_t lat, int32_t lon, int32_t alt)
{
    if (lat > DEL_LAT || lat < -DEL_LAT) {
        LOGI("Error - navigation_update_self_pos lat is incorrect");
        return;
    }

    if (lon > DEL_LON || lon < -DEL_LON) {
        LOGI("Error - navigation_update_self_pos lon is incorrect");
        return;
    }

    if (alt > 8000 || alt < -1000) {
        LOGI("Error - navigation_update_self_pos alt is incorrect");
        return;
    }

    self_pos.lat = (DEG_TO_RAD * lat) / DIGITS;
    self_pos.lon = (DEG_TO_RAD * lon) / DIGITS;
    self_pos.alt = alt;
}

void navigation_del_pos(uint16_t id)
{
    uint32_t i = 0;
    if (id == NAV_ID_EMPTY) {
        for (i = 0; i < POINTS_MAX; i++) {
           nav_points_v2[i] = empty_point;
        }
        return;
    }

    i = search_for_id(id);
    nav_points_v2[i] = empty_point;
}

typedef struct __attribute__((__packed__))  {
    char name[16];
    int32_t lat;
    int32_t lon;
    int32_t alt;
    enum Point_Type_T point_type;
} navi_data_v2_s;

// Stadion, Cinema, castle, post

navi_data_v2_s Stadion = {"Stadion", 4981934, 2404813, 289, DIAMOND_C};
navi_data_v2_s Cinema  = {"Cinema",  4982104, 2402509, 280, CIRCLE};
navi_data_v2_s Castle  = {"Castle",  4983289, 2402436, 300, TRIANGLE};
navi_data_v2_s Post    = {"Post",    4983775, 2402384, 260, X_SHAPE};

navi_data_v2_s MyPhone = {"My",      4983220, 2404215, 260, DIAMOND};

void navigation_Thread()
{
    uint32_t update_counter = 0;
    uint32_t sec_timer = 0;
    uint32_t pos = 0;
    uint32_t i = 0;

    init_nav_points_v2();

    // navigation_add_point(Stadion.name, strlen(Stadion.name), Stadion.lat, Stadion.lon, Stadion.alt, Stadion.point_type);
    // navigation_add_point(Cinema.name,  strlen(Cinema.name),  Cinema.lat,  Cinema.lon,  Cinema.alt,  Cinema.point_type);
    // navigation_add_point(Castle.name,  strlen(Castle.name),  Castle.lat,  Castle.lon,  Castle.alt,  Castle.point_type);
    // navigation_add_point(Post.name,    strlen(Post.name),    Post.lat,    Post.lon,    Post.alt,    Post.point_type);
    // navigation_add_point(MyPhone.name, strlen(MyPhone.name), MyPhone.lat, MyPhone.lon, MyPhone.alt, MyPhone.point_type);

    rt_sleep_ms(3500);

    while (1) {
        update_counter = UPDATE_MAX;
        sec_timer += NAV_DELAY;
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

        // process TTL timer
        if (sec_timer > 1000) {
            sec_timer -= 1000;
            ttl_decrement();
        }

        rt_sleep_ms(NAV_DELAY);
    }
}