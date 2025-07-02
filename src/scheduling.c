#include "motor.h"
#include "encoder.h"
#include "cbdef.h"
#include "odometria.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pigpio.h>
#include <pthread.h>
#include <linux/sched.h>
#include <sched.h>
#include <sys/syscall.h>
#include <stdint.h>
#include <signal.h>

// macro
#define MAX_DUTY_CYCLE 0.6
#define CYCLE_ERROR_TO_DUTY_CYCLE 0.1
#define OVERALL_ERROR_TO_DUTY_CYCLE 0.1
#define CYCLE_PERIOD_NS 2e7
#define TIMESOURCE CLOCK_MONOTONIC

// variabili globali
int speed;

// definizione degli struct e i loro rispettivi tipi
struct sched_attr {
	__u32 size;              /* Size of this structure */
	__u32 sched_policy;      /* Policy (SCHED_*) */
	__u64 sched_flags;       /* Flags */
	__s32 sched_nice;        /* Nice value (SCHED_OTHER,
	                          SCHED_BATCH) */
	__u32 sched_priority;    /* Static priority (SCHED_FIFO,
	                          SCHED_RR) */
	/* For SCHED_DEADLINE */
	__u64 sched_runtime;
	__u64 sched_deadline;
	__u64 sched_period;

	/* Utilization hints */
	__u32 sched_util_min;
	__u32 sched_util_max;
};

typedef struct timespec timespec_t;

typedef struct task {
	pthread_t tid;
	void* (*entry_point)(void*);
} task_t;

typedef struct odometry_data {
	pthread_mutex_t lock;
	bool are_ticks_reset;
	int ticks;
} odometry_data_t;

typedef struct control_args {
	cbMotor_t* motor;
	cbEncoder_t* encoder;
	float millimeters_per_tick;
	odometry_data_t* odometry;
} control_args_t;

typedef struct odometry_args {
	odometry_data_t* left_data;
	odometry_data_t* right_data;
} odometry_args_t;

// funzioni
void dl_miss_handler() {
	puts("DEADLINE MISS!");
	fflush(stdout);
	(void) signal(SIGXCPU, SIG_DFL);
}

static inline int set_sched_deadline(uint64_t runtime_ns,
									 uint64_t deadline_ns,
									 uint64_t period_ns) {
	struct sched_attr attr = {
		.size = sizeof(attr),
		.sched_flags = 0 | SCHED_FLAG_DL_OVERRUN,
		.sched_policy = SCHED_DEADLINE,
		.sched_runtime = runtime_ns,
		.sched_deadline = deadline_ns,
		.sched_period = period_ns
	};
	if(syscall(__NR_sched_setattr, syscall(__NR_gettid), &attr, 0) != 0) {
		perror("set_sched_deadline: sched_setattr");
		return(1);
	}
	(void) signal(SIGXCPU, dl_miss_handler); // register signal handler
	return(0);
}

static void ts_update(timespec_t* const upd) {
	if(clock_gettime(TIMESOURCE, upd) < 0) {
		perror("gettime: ");
		exit(EXIT_FAILURE);
	}
}

static timespec_t ts_delta(timespec_t* const before, timespec_t* const now) {
	timespec_t diff;
	//assegna l'ultimo tempo calcolato a "before"
	*before = *now;
	//sovrascrivi l'ultimo tempo con il nuovo tempo attuale
	ts_update(now);
	//trova il delta e restituiscilo
	diff.tv_sec = now->tv_sec - before->tv_sec;
	diff.tv_nsec = now->tv_nsec - before->tv_nsec;
    if (diff.tv_nsec < 0) {
        --diff.tv_sec;
        diff.tv_nsec += 1e9; //' +1sec
    }
	return diff;
}

void* motor_control_entry(void* arg) {
	// impostazione del thread per lo scheduler EDF
	set_sched_deadline(1e6, 1e7, CYCLE_PERIOD_NS);

	// creazioni delle variabili necessarie
	control_args_t args = *(control_args_t*)arg;
	odometry_data_t* odometry = args.odometry;
	int last_ticks = 0;
	int overall_error_ticks = 0;
	int ticks_since_odometry_update = 0;
	int target;
	float duty_cycle;
	cbDir_t direction;
	timespec_t before, now, difference;
	ts_update(&now);
	// inizio ciclo
	for(;;) {
		//salvo i tick percorsi e il delta tempo tra questo ciclo e il precedente
		difference = ts_delta(&before, &now);
		int total_ticks = args.encoder->ticks;
		target = speed * (difference.tv_nsec / 1e9) / args.millimeters_per_tick;
		int ticks = total_ticks - last_ticks;
		last_ticks = total_ticks;

		//aggiorno i dati per la posizione e l'angolazione
		if (pthread_mutex_trylock(&odometry->lock) == 0) {
			if(odometry->are_ticks_reset) {
				ticks_since_odometry_update = ticks;
				odometry->are_ticks_reset = false;
			} else {
				ticks_since_odometry_update += ticks;
			}
			odometry->ticks = ticks_since_odometry_update;
    		pthread_mutex_unlock(&odometry->lock);
		}

		// calcolo gli errori relativi a quanti tick ho percorso
		int cycle_error_ticks = target - ticks;
		overall_error_ticks += cycle_error_ticks;

		// imposto il duty cycle per rimediare al errore
		duty_cycle = cycle_error_ticks * CYCLE_ERROR_TO_DUTY_CYCLE;
		duty_cycle += overall_error_ticks * OVERALL_ERROR_TO_DUTY_CYCLE;

		if(cycle_error_ticks < 0) {
			duty_cycle = -duty_cycle;
			direction = backward;
		}
		else direction = forward;

		if(duty_cycle > MAX_DUTY_CYCLE)
			duty_cycle = MAX_DUTY_CYCLE;

		// imposto la velocità e direzione del motore
		cbMotorMove(args.motor, direction, duty_cycle);

		printf("Duty cycle %f\n", duty_cycle);
		printf("errore %d\n", overall_error_ticks);

		// aspetto il prossimo periodo oppure termino il thread se richiesto
		pthread_testcancel();
		sched_yield();
	}

	pthread_exit(EXIT_SUCCESS);
}

void* odometry_entry(void* arg) {
	set_sched_deadline(1e6, 1e7, CYCLE_PERIOD_NS);

	odometry_args_t args = *(odometry_args_t*)arg;
	odometry_data_t* odometry_data_left = args.left_data;
	odometry_data_t* odometry_data_right = args.right_data;
	
	for(;;) {
	
		if (pthread_mutex_trylock(&odometry_data_left->lock) == 0 &&
			pthread_mutex_trylock(&odometry_data_right->lock) == 0) {

			findNewPose(odometry_data_left->ticks, odometry_data_right->ticks);
			odometry_data_left->are_ticks_reset = true;
			odometry_data_right->are_ticks_reset = true;
			
    		pthread_mutex_unlock(&odometry_data_left->lock);
    		pthread_mutex_unlock(&odometry_data_right->lock);
		}

		printf("x: %f, y:%f, angolo: %f\n", pose_dof[0], pose_dof[1], pose_dof[2]);

		pthread_testcancel();
		sched_yield();
	}

	pthread_exit(EXIT_SUCCESS);
}

int main(int argc, char* argv[]) {
	// inizializzazione di GPIO
	if (gpioInitialise() < 0) {
        fprintf(stderr, "Errore inizializzazione GPIO\n");
        exit(EXIT_FAILURE);
    }
	// lettura del valore passato al programma
    if (argc < 2) {
        printf("Uso: %s <numero intero corrispondente ai millimetri al secondo da percorrere>\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    speed = atoi(argv[1]);

	// creazione degli struct relativi a motori e encoder
	cbMotor_t left_motor = {PIN_LEFT_FORWARD, PIN_LEFT_BACKWARD};
	cbMotor_t right_motor = {PIN_RIGHT_FORWARD, PIN_RIGHT_BACKWARD};
	cbEncoder_t left_encoder = {PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B, GPIO_PIN_NC, 0, 0, 0};
    cbEncoder_t right_encoder = {PIN_ENCODER_RIGHT_A, PIN_ENCODER_RIGHT_B, GPIO_PIN_NC, 0, 0, 0};
	
	// chiamate a GPIO
	cbMotorGPIOinit(&left_motor);
    cbMotorGPIOinit(&right_motor);
    cbEncoderGPIOinit(&left_encoder);
    cbEncoderGPIOinit(&right_encoder);
    cbEncoderRegisterISRs(&left_encoder, 100);
    cbEncoderRegisterISRs(&right_encoder, 100);

	//creazione degli struct per l'odometria
	odometry_data_t odometry_data_left = {PTHREAD_MUTEX_INITIALIZER, false};
	odometry_data_t odometry_data_right = {PTHREAD_MUTEX_INITIALIZER, false};

	// creazione degli struct rper gli argument dei thread
	control_args_t left_motor_control_args = {&left_motor, &left_encoder, MILLIMETERS_PER_TICK_LEFT, &odometry_data_left};
	control_args_t right_motor_control_args = {&right_motor, &right_encoder, MILLIMETERS_PER_TICK_RIGHT, &odometry_data_right};
	odometry_args_t odometry_args = {&odometry_data_left, &odometry_data_right};
	// creazione degli struct relativi ai thread
	task_t left_motor_control = { .entry_point = motor_control_entry };
	task_t right_motor_control = { .entry_point = motor_control_entry };
	task_t odometry_task = { .entry_point = odometry_entry };

	// creazione dei thread
	if(pthread_create(&(left_motor_control.tid), NULL, left_motor_control.entry_point, &left_motor_control_args) != 0) {
		perror("main: pthread_create: left_motor_control");
		exit(EXIT_FAILURE);
	}
	if(pthread_create(&(right_motor_control.tid), NULL, right_motor_control.entry_point, &right_motor_control_args) != 0) {
		perror("main: pthread_create: right_motor_control");
		exit(EXIT_FAILURE);
	}
	if(pthread_create(&(odometry_task.tid), NULL, odometry_task.entry_point, &odometry_args) != 0) {
		perror("main: pthread_create: odometry_task");
		exit(EXIT_FAILURE);
	}

	// pausa
	sleep(6);

	// terminazione del programma
	pthread_cancel(left_motor_control.tid);
	pthread_cancel(right_motor_control.tid);
	pthread_cancel(odometry_task.tid);
    cbMotorReset(&left_motor);
    cbMotorReset(&right_motor);
    gpioTerminate();
	printf("fine\n");
	exit(EXIT_SUCCESS);
}