/* DEFINIZIONE COSTANTI */
#define STEP_LEFT 0.1263372089                              // DISTANZA (mm) per TICK (sinistra)
#define STEP_RIGHT 0.1288004121                             // DISTANZA (mm) per TICK (destra)

#define B 120                                               // DISTANZA tra le RUOTE

#define THRESHOLD 0.005                                     // SOGLIA per DISTINGUERE tra DRITTO o CURVANDO
                                                            // (ticks sinistra e destra mai esattamente uguali, anche se dritto)

  float mm_sx;                                                        // MILLIMETRI PERCORSI dalla RUOTA
    float mm_dx;

    double pose[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};              // MATRICE per POSE (totale)
    double pose_dof[3] = {0, 0, 0,};                                    // VETTORE per POSE (totale)
    double newPose[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};           // MATRICE per POSE (corrente)
    double rt[3][3], r[3][3], t1[3][3], t2[3][3], temp[3][3];           // MATRICI di SUPPORTO
                                                                        // (rototraslazione, rotazione, traslazione al CIR, traslazione dal CIR, temporanea)

            /* ODOMETRIA */
            // calcolo MILLIMETRI percorsi
            mm_sx = l_ticks_odo * STEP_LEFT; // ruota sinistra
            mm_dx = r_ticks_odo * STEP_RIGHT; // ruota destra
            
            // calcolo ANGOLO
            double delta_theta = -(mm_sx - mm_dx) / B;

            // SOGLIA (ticks sinistra e destra mai esattamente uguali, anche se dritto)
            if (fabs(delta_theta) < THRESHOLD) {
                // movimento: DRITTO
                // calcolo matrice di ROTOTRASLAZIONE corrente
                rt[0][0] = 1; rt[0][1] = 0; rt[0][2] = (mm_sx + mm_dx) / 2;
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
            
            // creazione VETTORE per POSE
            pose_dof[0] = pose[0][2]; // posizione x
            pose_dof[1] = pose[1][2]; // posizione y
            pose_dof[2] = atan2(pose[1][0], pose[0][0]); // angolo theta
        }