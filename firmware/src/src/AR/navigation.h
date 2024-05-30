#ifndef NAVIGATION_HH
#define NAVIGATION_HH

#include "common_ar.h"

void navigation_add_point(char name[], uint8_t size, int32_t lat, int32_t lon, int32_t alt, enum Point_Type_T point_type);

void navigation_Thread();

#endif /* NAVIGATION_HH */