#include "libcoderbot/include/motor.h"
#include "libcoderbot/include/encoder.h"
#include "libcoderbot/include/cbdef.h"
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#define TEMPO_NS 10e9
#define PERIODO_CICLO_NS 2e7

typedef struct timespec timespec_t;

timespec_t diff_update(const timespec_t* const prev, timespec_t* ts) {
    timespec_t ret;
    if(clock_gettime(CLOCK_MONOTONIC, ts) < 0)
        exit(EXIT_FAILURE);
    ret.tv_sec = ts->tv_sec - prev->tv_sec;
    ret.tv_nsec = ts->tv_nsec - prev->tv_nsec;
    return ret;
}

int main(int argc, char* argv[]) {
    //inizializzazione di GPIO
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Errore inizializzazione GPIO\n");
        return EXIT_FAILURE;
    }

    //lettura del duty cycle passato come input
    if (argc < 2) {
        printf("Uso: %s <numero corrispondente al duty cycle>\n", argv[0]);
        return 1;
    }
    float dutyCycle = atof(argv[1]);

    //istanziazione degli struct
    timespec_t inizio, fine, delta, dormi;

    
    cbMotor_t left_motor = {
        .pin_fw = PIN_LEFT_FORWARD,
        .pin_bw = PIN_LEFT_BACKWARD
    };

    cbMotor_t right_motor = {
        .pin_fw = PIN_RIGHT_FORWARD,
        .pin_bw = PIN_RIGHT_BACKWARD
    };

    cbEncoder_t left_encoder = {
        .pin_a = PIN_ENCODER_LEFT_A,
        .pin_b = PIN_ENCODER_LEFT_B,
        .last_gpio = 0,
        .level_a = 0,
        .level_b = 0,
        .ticks = 0,
        .bad_ticks = 0
    };

    cbEncoder_t right_encoder = {
        .pin_a = PIN_ENCODER_RIGHT_A,
        .pin_b = PIN_ENCODER_RIGHT_B,
        .last_gpio = 0,
        .level_a = 0,
        .level_b = 0,
        .ticks = 0,
        .bad_ticks = 0
    };

    //chiamate alla libreria GPIO
    cbMotorGPIOinit(&left_motor);
    cbMotorGPIOinit(&right_motor);
    cbEncoderGPIOinit(&left_encoder);
    cbEncoderGPIOinit(&right_encoder);
    cbEncoderRegisterISRs(&left_encoder, 1000); // 1000 = 1ms, accetta un nuovo impulso tick dopo 1ms
    cbEncoderRegisterISRs(&right_encoder, 1000); // 1000 = 1ms, accetta un nuovo impulso tick dopo 1ms

    //istanziazione delle variabili richieste dal ciclo
    int numeroCicli = TEMPO_NS / PERIODO_CICLO_NS;
    double sommaTickSx = 0;
    double sommaTickDx = 0;
    double mediaTickSx, mediaTickDx;
    //avvia motori
    cbMotorMove(&left_motor, forward, dutyCycle);
    cbMotorMove(&right_motor, forward, dutyCycle);
    //ciclo con tempo di esecuzione fisso
    for(int i = 1; i <= numeroCicli; i++) {
        clock_gettime(CLOCK_MONOTONIC, &inizio);

        printf("tick di questo ciclo- SX: %d, DX: %d\n", left_encoder.ticks, right_encoder.ticks);
        sommaTickSx += left_encoder.ticks;
        sommaTickDx += right_encoder.ticks;

        mediaTickSx = sommaTickSx / i;
        mediaTickDx = sommaTickDx / i;

        printf("tick medi- SX: %.17f, DX: %.17f\n", mediaTickSx, mediaTickDx);

        left_encoder.ticks = 0;
        right_encoder.ticks = 0;

    
        delta = diff_update(&inizio, &fine);
        dormi.tv_sec = 0;
        dormi.tv_nsec = PERIODO_CICLO_NS - delta.tv_nsec;
        nanosleep(&dormi, NULL);
    }

    cbMotorReset(&left_motor);
    cbMotorReset(&right_motor);
    gpioTerminate();
    return EXIT_SUCCESS;
}
