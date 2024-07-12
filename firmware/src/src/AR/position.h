#ifndef POSITION_HH
#define POSITION_HH

#include "common_ar.h"

#define MAX_NAME_LENGTH 16

struct Position_Data_T {
  float azimuth;
  float pitch;
  uint32_t distance;
  uint32_t time_stamp;
  enum Point_Type_T point_type;
  char name[MAX_NAME_LENGTH];
};

void position_set_roll(float tilt_new);
void position_set_pitch(float roll_new);
void position_set_azimuth(float pan_new);
void position_add_point(struct Position_Data_T point_data);
void position_add_point(char name[], uint8_t length, float azimuth, float pitch, uint32_t distance, enum Point_Type_T point_type);
void position_del_point(char *name, uint8_t size);
void position_Thread();

#endif /* POSITION_HH */