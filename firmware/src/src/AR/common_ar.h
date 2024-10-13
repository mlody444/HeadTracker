#ifndef COMMON_AR_HH
#define COMMON_AR_HH

#define PI 3.1415926535897932384626433f
#define ID_EMPTY 0xFFF
#define NAME_MAX 16
#define DIGITS   100000.0

// #define DEB
#define ERR
#define WAR

#ifdef DEB
#define debug(text, ...) LOGI("DEBUG %s %s() line:%d:\r\n" text, __FILE__, __func__, __LINE__ __VA_OPT__(,)__VA_ARGS__)
#else
#define debug(...)
#endif // DEB

#ifdef ERR
#define error(text, ...) LOGI("ERROR %s %s() line:%d:\r\n" text, __FILE__, __func__, __LINE__ __VA_OPT__(,)__VA_ARGS__)
#else
#define error(...)
#endif // ERR

#ifdef WAR
#define warning(text, ...) LOGI("WARNING %s %s() line:%d:\r\n" text, __FILE__, __func__, __LINE__ __VA_OPT__(,)__VA_ARGS__)
#else
#define warning(...)
#endif // WAR

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

struct NAV_CORDS_RAW {
    int32_t lat;
    int32_t lon;
    int16_t alt;
};

struct NAV_CORDS {
    float lat;
    float lon;
    int16_t alt;
};

typedef struct __attribute__((__packed__))  {
    char name[NAME_MAX];
    struct NAV_CORDS cords;
    point_data nav;  // 12bits ID, 4 bits for flag, 0 - always show,
    uint8_t ttl;  // time to life (0xFF - unlimited)
    enum Point_Type_T point_type;
    bool update;
} navi_data_v3_s;

void raw_to_processed(navi_data_v3_raw_s *incoming_friendly, navi_data_v3_s *processed_friendly);

#endif /* COMMON_AR_HH */