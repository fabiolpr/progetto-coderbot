#include <math.h>
#include "odometry.h"

double pose[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};              // MATRICE per POSE (totale)
double newPose[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};           // MATRICE per POSE (corrente)
double rt[3][3], r[3][3], t1[3][3], t2[3][3], temp[3][3];           // MATRICI di SUPPORTO

// struct della posizione
position_t position = {.x = 0, .y = 0, .theta = 0};

// Moltiplica due matrici 3x3: C = A * B
void moltiplica_matrici_3x3(const double matA[3][3], const double matB[3][3], double result[3][3]) {
    for (int i = 0; i < 3; i++) {
        double a0 = matA[i][0];
        double a1 = matA[i][1];
        double a2 = matA[i][2];
        result[i][0] = a0 * matB[0][0] + a1 * matB[1][0] + a2 * matB[2][0];
        result[i][1] = a0 * matB[0][1] + a1 * matB[1][1] + a2 * matB[2][1];
        result[i][2] = a0 * matB[0][2] + a1 * matB[1][2] + a2 * matB[2][2];
    }
}

/* ODOMETRIA */
void findNewPose(float mm_sx, float mm_dx, float average_mm) {
    // calcolo ANGOLO
    double delta_theta = -(mm_sx - mm_dx) / B;

    // SOGLIA (ticks sinistra e destra mai esattamente uguali, anche se dritto)
    if (fabs(delta_theta) < THRESHOLD) {
        // movimento: DRITTO
        // calcolo matrice di ROTOTRASLAZIONE corrente
        rt[0][0] = 1; rt[0][1] = 0; rt[0][2] = average_mm;
        rt[1][0] = 0; rt[1][1] = 1; rt[1][2] = 0;
        rt[2][0] = 0; rt[2][1] = 0; rt[2][2] = 1;
    } else {
        // movimento: CURVANDO
        double d = (mm_dx / delta_theta) - (B / 2);
        // TRASLAZIONE al CIR
        t1[0][0] = 1; t1[0][1] = 0; t1[0][2] = 0;
        t1[1][0] = 0; t1[1][1] = 1; t1[1][2] = -d;
        t1[2][0] = 0; t1[2][1] = 0; t1[2][2] = 1;
        // ROTAZIONE
        r[0][0] = cos(delta_theta); r[0][1] = -sin(delta_theta); r[0][2] = 0;
        r[1][0] = sin(delta_theta); r[1][1] = cos(delta_theta); r[1][2] = 0;
        r[2][0] = 0; r[2][1] = 0; r[2][2] = 1;
        // TRASLAZIONE dal CIR
        t2[0][0] = 1; t2[0][1] = 0; t2[0][2] = 0;
        t2[1][0] = 0; t2[1][1] = 1; t2[1][2] = d;
        t2[2][0] = 0; t2[2][1] = 0; t2[2][2] = 1;
        // calcolo matrice di ROTOTRASLAZIONE corrente
        moltiplica_matrici_3x3(t2, r, temp);      
        moltiplica_matrici_3x3(temp, t1, rt);     
    }

    // calcolo (nuova) POSE
    moltiplica_matrici_3x3(pose, rt, newPose);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            pose[r][c] = newPose[r][c];

    // salvataggio della posizione	
    	position.x = pose[0][2];
        position.y = pose[1][2];
        position.theta = atan2(pose[1][0], pose[0][0]);
}
