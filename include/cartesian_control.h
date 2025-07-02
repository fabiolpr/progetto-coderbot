#ifndef CARTESIAN_CONTROL_H
#define CARTESIAN_CONTROL_H

#define ANGLE_TOLLERANCE 0.025
#define SPEED 50

// Definizione del tipo Point
typedef struct {
    float x;
    float y;
} Point;

// Variabili globali
extern Point* waypoints;      // array di waypoints
extern int N_POINTS;          // numero di waypoints
extern int current_position;  // posizione corrente nel percorso
extern float speed_l;         // velocità ruota sinistra
extern float speed_r;         // velocità ruota destra

// Funzioni
void generate_arc_points(Point* points, int num_points, float cx, float cy, float radius, float start_angle, float end_angle);

int nearest_point_position(Point* waypoints, int num_points, float* pose_dof);

void cartesian_control(void);

#endif // ARC_CONTROL_H
