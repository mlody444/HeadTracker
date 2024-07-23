#ifndef COMMON_AR_HH
#define COMMON_AR_HH

#define PI 3.1415926535897932384626433f
#define NAV_ID_EMPTY 0xFFF
#define NAME_MAX 16

enum Point_Type_T {
    DIAMOND    = 0,
    DIAMOND_C  = 1, // cropped
    SQUARE     = 2,
    SQUARE_C   = 3, // cropped
    CIRCLE     = 4,
    TRIANGLE   = 5,
    TRIANGLE_C = 6, // cropped
    X_SHAPE    = 7,
    X_SHAPE_C  = 8, // cropped
    POINT_TYPE_MAX
};

typedef struct __attribute__((__packed__)) {
    uint16_t id : 12;
    uint16_t always_show : 1;
    uint16_t reserved : 3;
} point_data;

typedef struct __attribute__((__packed__))  {
    char name[NAME_MAX];
    int32_t lat;
    int32_t lon;
    int16_t alt;
    point_data nav;  // 12bits ID, 4 bits for flag, 0 - always show,
    uint8_t ttl;  // time to life (0xFF - unlimited)
    enum Point_Type_T point_type;
} navi_data_v3_raw_s;

struct NAV_CORDS {
    float lat;
    float lon;
    int32_t alt;
};

typedef struct __attribute__((__packed__))  {
    char name[NAME_MAX];
    struct NAV_CORDS cords;
    point_data nav;  // 12bits ID, 4 bits for flag, 0 - always show,
    uint8_t ttl;  // time to life (0xFF - unlimited)
    enum Point_Type_T point_type;
    bool update;
} navi_data_v3_s;

#endif /* COMMON_AR_HH */