#include <math.h>
#include "cartesian_control.h"
#include "odometry.h" // per pose_dof

Point waypoints[N_POINTS];      // array di waypoints
float speed;
float speed_l = 0;                  // velocità ruota sinistra
float speed_r = 0;                  // velocità ruota destra
int current_point = 0;

void generate_arc_points(Point* points, int num_points, float cx, float cy, float radius, float start_angle, float end_angle) {
    for (int i = 0; i < num_points; ++i) {
        float t = (float)i / (num_points - 1);
        float angle = start_angle + t * (end_angle - start_angle);
        points[i].x = cx + radius * cos(angle);
        points[i].y = cy + radius * sin(angle);
    }
}

int find_nearest_point(Point* waypoints, int num_points, float* pose_dof, int start_index){
    float min_distance_squared = (waypoints[start_index].x - pose_dof[0]) * (waypoints[start_index].x - pose_dof[0]) +
                                 (waypoints[start_index].y - pose_dof[1]) * (waypoints[start_index].y - pose_dof[1]);

    int end_index = start_index + POINTS_TO_CHECK;
    if(end_index > num_points)
        end_index = num_points;
    for (int i = start_index; i < end_index; ++i) {
        float dx = waypoints[i].x - pose_dof[0];
        float dy = waypoints[i].y - pose_dof[1];
        float distance_squared = dx * dx + dy * dy;

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
    float pose_dof[3];
    
    //SISTEMA!
        pose_dof[0] = position.x;
        pose_dof[1] = position.y;
        pose_dof[2] = position.theta;

    current_point = find_nearest_point(waypoints, N_POINTS, pose_dof, current_point);
    printf("siamo al punto: %d\n", current_point); 
    if(current_point >= N_POINTS - 3){
        // FINE
        return true;
    }

    // waypoints[current_point + 3] POSIZIONE che si PUNTA (per evitare errori, non troppo vicina, quindi +3 posizioni)
    // waypoints[current_point].XY POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO
    float delta_x = waypoints[current_point + 3].x - waypoints[current_point].x;
    float delta_y = waypoints[current_point + 3].y - waypoints[current_point].y;
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
