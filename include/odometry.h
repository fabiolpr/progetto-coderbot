#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <pthread.h>

#define MILLIMETERS_PER_TICK_LEFT 0.1114790994
#define MILLIMETERS_PER_TICK_RIGHT 0.1117455841
#define MINIMUM_DELTA_MILLIMETERS 1
#define B 120
#define THRESHOLD 0.005

typedef struct {
    float x;
    float y;
    float theta;
} position_t;

extern position_t position;

void findNewPose(float mm_sx, float mm_dx, float average_mm);

#endif // ODOMETRY_H
