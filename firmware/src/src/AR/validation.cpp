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
#include <string.h>

#include "defines.h"
#include "common_ar.h"
#include "navigation.h"

#define NAME_STATION_1 "Stadion"
#define NAME_STATION_2 "Stadion_v2"
#define NAME_STATION_3 "Stadion_v3"

navi_data_v3_raw_s Stadion = {"Stadion", 4981934, 2404813, 289, {1, 0, 0},   0xFF, DIAMOND_C};
navi_data_v3_raw_s Cinema  = {"Cinema",  4982104, 2402509, 280, {8, 0, 0},   100,  CIRCLE};
navi_data_v3_raw_s Castle  = {"Castle",  4983289, 2402436, 300, {213, 0, 0}, 50,   TRIANGLE};
navi_data_v3_raw_s Post    = {"Post",    4983775, 2402384, 260, {3, 0, 0},   200,  X_SHAPE};
// navi_data_v3_raw_s MyPhone = {"My",      4983220, 2404215, 260, 5,   170,  DIAMOND};

static void test_process(navi_data_v3_raw_s incoming_friendly)
{
    navi_data_v3_s processed_friendly;

    memcpy(processed_friendly.name, incoming_friendly.name, 16);
    processed_friendly.cords.lat = (DEG_TO_RAD * incoming_friendly.lat) / DIGITS;
    processed_friendly.cords.lon = (DEG_TO_RAD * incoming_friendly.lon) / DIGITS;
    processed_friendly.cords.alt = incoming_friendly.alt;
    processed_friendly.nav = incoming_friendly.nav;
    processed_friendly.ttl = incoming_friendly.ttl;
    processed_friendly.point_type = incoming_friendly.point_type;
    processed_friendly.update = true;

    navigation_add_point_v2(&processed_friendly);
}

NAV_CORDS_RAW myself = {4983220, 2404215, 260};

void validation_Thread()
{
    test_process(Stadion);
    test_process(Cinema);
    test_process(Castle);
    test_process(Post);

    while (1) {
        navigation_update_myself(myself);

        Stadion.lat = 4981934;
        Stadion.lon = 2404813;
        Stadion.alt = 289;
        memcpy(Stadion.name, NAME_STATION_1, sizeof(NAME_STATION_1));
        test_process(Stadion);
        rt_sleep_ms(2000);
        memcpy(Stadion.name, NAME_STATION_2, sizeof(NAME_STATION_2));
        test_process(Stadion);
        rt_sleep_ms(2000);
        Stadion.lat = 4981700;
        Stadion.lon = 2404913;
        Stadion.alt = 250;
        memcpy(Stadion.name, NAME_STATION_3, sizeof(NAME_STATION_3));
        test_process(Stadion);
        rt_sleep_ms(2000);
        navigation_del_pos(Stadion.nav.id);
        rt_sleep_ms(2000);

    }
}
