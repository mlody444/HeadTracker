#ifndef NAVIGATION_HH
#define NAVIGATION_HH

#include "common_ar.h"

void navigation_add_point(char name[], uint8_t size, int32_t lat, int32_t lon, int32_t alt, enum Point_Type_T point_type);

void navigation_update_self_pos(int32_t lat, int32_t lon, int32_t alt);
void navigation_del_pos(uint16_t id);
void navigation_del_all();
void navigation_add_point_v2(navi_data_v3_s *point);

void navigation_Thread();

#endif /* NAVIGATION_HH */