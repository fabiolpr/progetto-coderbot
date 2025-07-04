#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <pthread.h>

#define MILLIMETERS_PER_TICK_LEFT 0.1114790994
#define MILLIMETERS_PER_TICK_RIGHT 0.1117455841
#define B 120
#define THRESHOLD 0.005

extern float mm_sx;
extern float mm_dx;

extern float pose[3][3];
extern float newPose[3][3];
extern float rt[3][3];
extern float r[3][3];
extern float t1[3][3];
extern float t2[3][3];
extern float temp[3][3];

typedef struct {
    float x;
    float y;
    float theta;
} position_t;

extern position_t position;


void moltiplica_matrici_3x3(float matA[3][3], float matB[3][3], float result[3][3]);
void findNewPose(int l_ticks_odo, int r_ticks_odo);

#endif // ODOMETRY_H
