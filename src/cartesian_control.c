#include <math.h>
#include "cartesian_control.h"
#include "odometry.h"

#define ANGLE_TOLLERANCE 0.0175 //angolo minimo considerato come errore
#define TARGET_POINT_INCREMENT 3
#define POINTS_TO_CHECK 30 //numero di punti che vengono controllati durante la ricerca del punto più vicino
#define MINIMUM_SPEED_CORRECTION_FACTOR 0.5

point_t waypoints[N_POINTS];      // array di waypoints
float speed;
float speed_l = 0;                  // velocità ruota sinistra
float speed_r = 0;                  // velocità ruota destra
int current_point = 0;
/*  la correzione dell'errore della velocità viene calcolata con una funzione lineare (multiplier * x + MINIMUM_SPEED_CORRECTION_FACTOR)
    vogliamo che con errore 0 sia a MINIMUM_SPEED_CORRECTION_FACTOR, e con errore massimo, cioè M_PI, sia a 1.
    multiplier * M_PI + MINIMUM_SPEED_CORRECTION_FACTOR = 1, quindi:*/
const float multiplier = (1 - MINIMUM_SPEED_CORRECTION_FACTOR) / M_PI;

float get_distance_squared_between_points(float x1, float y1, float x2, float y2) {
    float dx = x1 - x2;
    float dy = y1 - y2;
    return dx * dx + dy * dy;
}

void generate_arc_points(point_t points[], int num_points, float cx, float cy, float radius, float start_angle, float end_angle) {
    for (int i = 0; i < num_points; ++i) {
        float t = (float)i / (num_points - 1);
        float angle = start_angle + t * (end_angle - start_angle);
        points[i].x = cx + radius * cos(angle);
        points[i].y = cy + radius * sin(angle);
    }
}

int find_nearest_point(const point_t waypoints[], int num_points, position_t position, int start_index){
    float min_distance_squared =
    get_distance_squared_between_points(waypoints[start_index].x, waypoints[start_index].y, position.x, position.y);

    int end_index = start_index + POINTS_TO_CHECK;
    if(end_index > num_points)
        end_index = num_points;
    for (int i = start_index; i < end_index; ++i) {
        float distance_squared =
        get_distance_squared_between_points(waypoints[i].x, waypoints[i].y, position.x, position.y);

        if (distance_squared < min_distance_squared) {
            min_distance_squared = distance_squared;
            start_index = i;
        }
    }

    return start_index;
}

bool cartesian_control() {
    /* CARTESIAN CONTROLLER */
    // POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO
    
    //SISTEMA!
        position.x = position.x;
        position.y = position.y;
        position.theta = position.theta;

    current_point = find_nearest_point(waypoints, N_POINTS, position, current_point);
    if(current_point >= N_POINTS - TARGET_POINT_INCREMENT){
        // FINE
        return true;
    }

    // waypoints[current_point + TARGET_POINT_INCREMENT] POSIZIONE che si PUNTA (per evitare errori, non troppo vicina)
    float delta_x = waypoints[current_point + TARGET_POINT_INCREMENT].x - position.x;
    float delta_y = waypoints[current_point + TARGET_POINT_INCREMENT].y - position.y;
    float desired_theta = atan2(delta_y, delta_x); 
    float delta_theta_c = desired_theta - position.theta; // errore dell'angolo

    // NORMALIZZAZIONE delta_theta_c in [-π, π]
    while (delta_theta_c > M_PI) delta_theta_c -= 2 * M_PI;
    while (delta_theta_c < -M_PI) delta_theta_c += 2 * M_PI;

    // CONFRONTO con TOLLERANZA dell'(errore dell')ANGOLO
    if(fabs(delta_theta_c) < ANGLE_TOLLERANCE){
        // ANGOLO (piu' o meno) CORRETTO, si PROCEDE in LINEA RETTA per PUNTARE alla POSIZIONE
        speed_l = speed;
        speed_r = speed;
    } else {
        // CORREZIONE VELOCITA' per STERZARE

        //tra MINIMUM_SPEED_CORRECTION_FACTOR e 1
        float correction_factor = multiplier * fabs(delta_theta_c) + MINIMUM_SPEED_CORRECTION_FACTOR;
        // <= alla velocità
        correction_factor *= speed;
        if(delta_theta_c < 0){
            // sterzare a DESTRA
            speed_l = speed;
            speed_r = speed - correction_factor;
        } else {
            // sterzare a SINISTRA
            speed_l = speed - correction_factor;
            speed_r = speed;
        }
    }

    return false;
}
