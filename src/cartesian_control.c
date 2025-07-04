#include "cartesian_control.h"
#include "odometry.h" // per pose_dof
#include <math.h>

Point waypoints[N_POINTS];      // array di waypoints
float speed;
float speed_l;                  // velocità ruota sinistra
float speed_r;                  // velocità ruota destra

void generate_arc_points(Point* points, int num_points, float cx, float cy, float radius, float start_angle, float end_angle) {
    for (int i = 0; i < num_points; ++i) {
        float t = (float)i / (num_points - 1);
        float angle = start_angle + t * (end_angle - start_angle);
        points[i].x = cx + radius * cos(angle);
        points[i].y = cy + radius * sin(angle);
    }
}

int nearest_point_position(Point* waypoints, int num_points, float* pose_dof){
    int nearest_index = 0;
    float min_distance_squared = (waypoints[0].x - pose_dof[0]) * (waypoints[0].x - pose_dof[0]) +
                                 (waypoints[0].y - pose_dof[1]) * (waypoints[0].y - pose_dof[1]);

    for (int i = 1; i < num_points; ++i) {
        float dx = waypoints[i].x - pose_dof[0];
        float dy = waypoints[i].y - pose_dof[1];
        float distance_squared = dx * dx + dy * dy;

        if (distance_squared < min_distance_squared) {
            min_distance_squared = distance_squared;
            nearest_index = i;
        }
    }

    return nearest_index;
}

bool cartesian_control() {
    /* CARTESIAN CONTROLLER */
    // POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO
    float pose_dof[3];
    if (pthread_mutex_trylock(&position.lock) == 0) {	
        pose_dof[0] = position.x;
        pose_dof[1] = position.y;
        pose_dof[2] = position.theta;
    	pthread_mutex_unlock(&position.lock);
	} else
        return false;

    int current_position = nearest_point_position(waypoints, N_POINTS, pose_dof);
    printf("siamo al punto: %d\n", current_position); 
    if(current_position >= N_POINTS - 3){
        // FINE
        speed_l = 0;
        speed_r = 0;
        return true;
    }

    // waypoints[current_position + 3] POSIZIONE che si PUNTA (per evitare errori, non troppo vicina, quindi +3 posizioni)
    // waypoints[current_position].XY POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO
    float delta_x = waypoints[current_position + 3].x - waypoints[current_position].x;
    float delta_y = waypoints[current_position + 3].y - waypoints[current_position].y;
    float desired_theta = atan2(delta_y, delta_x); 
    float delta_theta_c = desired_theta - pose_dof[2]; // errore dell'angolo

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
        if(delta_theta_c < 0){
            // sterzare a DESTRA
            speed_l = speed;
            speed_r = speed * (1 / fabs((delta_theta_c * 180) / M_PI)); // correzione VELOCITA' PROPORZIONALE all'ERRORE
        } else {
            // sterzare a SINISTRA
            speed_l = speed * (1 / fabs((delta_theta_c * 180) / M_PI)); // correzione VELOCITA' PROPORZIONALE all'ERRORE
            speed_r = speed;
        }
    }

    return false;
}
