#ifndef CARTESIAN_CONTROL_H
#define CARTESIAN_CONTROL_H

#include <stdbool.h>

#define N_POINTS 50 // numero di punti


// Definizione del tipo point_t
typedef struct {
    float x;
    float y;
} point_t;

// Variabili globali
extern point_t waypoints[N_POINTS];      // array del percorso
extern int current_point;  // indice corrente nel array del percorso
extern float speed;           // velocità generale
extern float speed_l;         // velocità ruota sinistra
extern float speed_r;         // velocità ruota destra

// Funzioni
void generate_arc_points(point_t points[], int num_points, float cx, float cy, float radius, float start_angle, float end_angle);

bool cartesian_control();

#endif // ARC_CONTROL_H
