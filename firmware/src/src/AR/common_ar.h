#ifndef COMMON_AR_HH
#define COMMON_AR_HH

#define PI 3.1415926535897932384626433f

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

#endif /* COMMON_AR_HH */