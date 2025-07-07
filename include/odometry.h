#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <pthread.h>

#define MILLIMETERS_PER_TICK_LEFT 0.1114790994
#define MILLIMETERS_PER_TICK_RIGHT 0.1117455841
#define MINIMUM_DELTA_MILLIMETERS 5
#define B 120
#define THRESHOLD 0.005
#define SQUARE(X) X * X

extern float mm_sx;
extern float mm_dx;

extern double pose[3][3];
extern double newPose[3][3];
extern double rt[3][3];
extern double r[3][3];
extern double t1[3][3];
extern double t2[3][3];
extern double temp[3][3];

typedef struct {
    float x;
    float y;
    float theta;
} position_t;

extern position_t position;


void moltiplica_matrici_3x3(double matA[3][3], double matB[3][3], double result[3][3]);
void findNewPose(float mm_sx, float mm_dx, float average_mm);

#endif // ODOMETRY_H
