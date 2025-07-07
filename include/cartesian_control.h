#ifndef CARTESIAN_CONTROL_H
#define CARTESIAN_CONTROL_H

#include <stdbool.h>

#define ANGLE_TOLLERANCE 0.025 //angolo minimo considerato come errore
#define N_POINTS 50 // numero di punti

// ATTENZIONE! modificare la seguente macro comporta una modifica dei tempi di esecuzione del task di controllo cartesiano
#define POINTS_TO_CHECK 30 //numero di punti che vengono controllati durante la ricerca del punto più vicino

// Definizione del tipo Point
typedef struct {
    float x;
    float y;
} Point;

// Variabili globali
extern Point waypoints[N_POINTS];      // array di waypoints
extern int current_position;  // posizione corrente nel percorso
extern float speed;           // velocità generale
extern float speed_l;         // velocità ruota sinistra
extern float speed_r;         // velocità ruota destra
extern int current_point;

// Funzioni
void generate_arc_points(Point* points, int num_points, float cx, float cy, float radius, float start_angle, float end_angle);

int nearest_point_position(Point* waypoints, int num_points, float* pose_dof);

bool cartesian_control();

#endif // ARC_CONTROL_H
