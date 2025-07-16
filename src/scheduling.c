// librerie esterne
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

//header del progetto
#include "cbdef.h"
#include "motor.h"
#include "encoder.h"
#include "odometry.h"
#include "cartesian_control.h"

// macro
#define MAX_DUTY_CYCLE 0.6
#define MINIMUM_SPEED 5
#define MAXIMUM_SPEED 50
#define MAXIMUM_WHEEL_ERROR_MILLIMETERS 10
#define CYCLE_ERROR_TO_DUTY_CYCLE 15e5
#define OVERALL_ERROR_TO_DUTY_CYCLE 5e5
#define MOTOR_CONTROL_PERIOD_NS 2e6
#define CARTESIAN_CONTROL_PERIOD_NS 1e7
#define PATH_INDEX 0

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

typedef struct stopwatch {
	clockid_t time_source;
	long delta;
	timespec_t start;
	timespec_t end;
} stopwatch_t;

typedef struct task {
	void* (*entry_point)(void*);
	long worst_execution_time;
	pthread_t tid;
} task_t;

typedef struct odometry_data {
	pthread_mutex_t lock;
	bool are_ticks_reset;
	int ticks;
} odometry_data_t;

typedef struct motor_control_args {
	char* motor_name;
	cbMotor_t* motor;
	cbEncoder_t* encoder;
	float millimeters_per_tick;
	float* target_speed;
	odometry_data_t* odometry;
	long* worst_execution_time;
	pthread_t* control_thread_id;
} motor_control_args_t;

typedef struct cartesian_control_args {
	odometry_data_t* left_data;
	odometry_data_t* right_data;
	long* worst_execution_time;
} cartesian_control_args_t;

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

static void find_current_time(stopwatch_t* stopwatch) {
	if(clock_gettime(stopwatch->time_source, &stopwatch->start) < 0) {
		perror("gettime");
		exit(EXIT_FAILURE);
	}
}

static void update_stopwatch(stopwatch_t* stopwatch) {
	//assegna l'ultimo tempo calcolato ad "end"
	stopwatch->end = stopwatch->start;
	//sovrascrivi l'ultimo tempo con il nuovo tempo attuale
	find_current_time(stopwatch);
	//trova il delta
	int32_t delta_seconds = stopwatch->start.tv_sec - stopwatch->end.tv_sec;
	stopwatch->delta = stopwatch->start.tv_nsec + delta_seconds * 1e9 - stopwatch->end.tv_nsec;
}

static void update_worst_time(stopwatch_t* thread_stopwatch, long time_spent_locked, long* worst_time) {
		update_stopwatch(thread_stopwatch);
		long time = thread_stopwatch->delta + time_spent_locked;
		if(*worst_time < time)
			*worst_time = time;
}


void* motor_control_entry(void* arg) {
	// impostazione del thread per lo scheduler EDF
	set_sched_deadline(1e6, MOTOR_CONTROL_PERIOD_NS, MOTOR_CONTROL_PERIOD_NS);
	//creazione dei cronometri per la task
	stopwatch_t thread_execution_stopwatch = {.time_source = CLOCK_THREAD_CPUTIME_ID, .delta = 0};
	find_current_time(&thread_execution_stopwatch);
	stopwatch_t mutex_locked_time_stopwatch = {.time_source = CLOCK_MONOTONIC, .delta = 0};

	// creazioni delle variabili necessarie
	motor_control_args_t args = *(motor_control_args_t*)arg;
	odometry_data_t* odometry = args.odometry;
	float overall_error_ticks = 0;
	int ticks_since_odometry_update = 0;
	float target;
	float duty_cycle;
	float maximum_error_ticks = MAXIMUM_WHEEL_ERROR_MILLIMETERS / args.millimeters_per_tick;
	cbDir_t direction;
	stopwatch_t tick_stopwatch = {.time_source = CLOCK_MONOTONIC, .delta = 0};
	find_current_time(&tick_stopwatch);
	// inizio ciclo
	for(;;) {
		//salvo i tick percorsi e il delta tempo tra questo ciclo e il precedente
		find_current_time(&mutex_locked_time_stopwatch);
		pthread_mutex_lock(&args.encoder->tick_lock);
		update_stopwatch(&mutex_locked_time_stopwatch);

		update_stopwatch(&tick_stopwatch);
		int ticks = args.encoder->ticks;
		args.encoder->ticks = 0;

        pthread_mutex_unlock(&args.encoder->tick_lock);

		target = *(args.target_speed) * (tick_stopwatch.delta / 1e9) / args.millimeters_per_tick;

		//aggiorno i dati per la posizione e l'angolazione
		ticks_since_odometry_update += ticks;
		if (pthread_mutex_trylock(&odometry->lock) == 0) {
			if(odometry->are_ticks_reset) {
				ticks_since_odometry_update -= odometry->ticks;
				odometry->are_ticks_reset = false;
			}
			odometry->ticks = ticks_since_odometry_update;
    		pthread_mutex_unlock(&odometry->lock);
		}

		// calcolo gli errori relativi a quanti tick ho percorso
		float cycle_error_ticks = target - ticks;
		overall_error_ticks += cycle_error_ticks;

		// imposto il duty cycle per rimediare al errore
		duty_cycle = cycle_error_ticks * CYCLE_ERROR_TO_DUTY_CYCLE / MOTOR_CONTROL_PERIOD_NS;
		duty_cycle += overall_error_ticks * OVERALL_ERROR_TO_DUTY_CYCLE / MOTOR_CONTROL_PERIOD_NS;

		if(duty_cycle < 0) {
			duty_cycle = -duty_cycle;
			direction = backward;
		}
		else direction = forward;

		if(duty_cycle > MAX_DUTY_CYCLE)
			duty_cycle = MAX_DUTY_CYCLE;

		// imposto la velocità e direzione del motore
		cbMotorMove(args.motor, direction, duty_cycle);

		printf("Motore: \"%s\", duty cycle %f, errore %f\n", args.motor_name, duty_cycle, overall_error_ticks);

		//termina il programma se il numero di tick accumulati è troppo elevato
		if(overall_error_ticks > maximum_error_ticks)
			pthread_cancel(*args.control_thread_id);

		//aggiorno il peggiore tempo di esecuzione
		update_worst_time(&thread_execution_stopwatch, mutex_locked_time_stopwatch.delta, args.worst_execution_time);
		// aspetto il prossimo periodo oppure termino il thread se richiesto
		pthread_testcancel();
		sched_yield();
	}

	pthread_exit(EXIT_SUCCESS);
}


void* cartesian_control_entry(void* arg) {
	// impostazione del thread per lo scheduler EDF
	set_sched_deadline(2e6, CARTESIAN_CONTROL_PERIOD_NS, CARTESIAN_CONTROL_PERIOD_NS);
	//creazione dei cronometri per la task
	stopwatch_t thread_execution_stopwatch = {.time_source = CLOCK_THREAD_CPUTIME_ID, .delta = 0};
	find_current_time(&thread_execution_stopwatch);
	stopwatch_t mutex_locked_time_stopwatch = {.time_source = CLOCK_MONOTONIC, .delta = 0};

	cartesian_control_args_t args = *(cartesian_control_args_t*)arg;
	odometry_data_t* odometry_data_left = args.left_data;
	odometry_data_t* odometry_data_right = args.right_data;
	
	cartesian_control();
	for(;;) {
		
		//prendere i lock insieme per evitare discrepanze di tick tra le due ruote
		find_current_time(&mutex_locked_time_stopwatch);
		pthread_mutex_lock(&odometry_data_left->lock);
		pthread_mutex_lock(&odometry_data_right->lock);
		update_stopwatch(&mutex_locked_time_stopwatch);

		int ticks_left = odometry_data_left->ticks;
		int ticks_right = odometry_data_right->ticks;
		float left_mm = ticks_left * MILLIMETERS_PER_TICK_LEFT;
		float right_mm = ticks_right * MILLIMETERS_PER_TICK_RIGHT;
		float average_mm = (left_mm + right_mm) / 2;
		bool is_delta_large_enough = average_mm > MINIMUM_DELTA_MILLIMETERS;
		
		if(is_delta_large_enough) {
			odometry_data_right->are_ticks_reset = true;
			odometry_data_left->are_ticks_reset = true;
		}

		pthread_mutex_unlock(&odometry_data_left->lock);
		pthread_mutex_unlock(&odometry_data_right->lock);

		if(is_delta_large_enough) {
			find_new_pose(left_mm, right_mm, average_mm);

			printf("x: %f, y:%f, angolo: %f\n", position.x, position.y, position.theta);
			printf("Siamo al punto: %d\n", current_point);
			//printf("new Position(%f, %f, %f), ", position.x, position.y, position.theta);

			if(cartesian_control()) {
				//se è stato completato il percorso
				update_worst_time(&thread_execution_stopwatch, mutex_locked_time_stopwatch.delta, args.worst_execution_time);
				pthread_exit(EXIT_SUCCESS);
			}
		}

		update_worst_time(&thread_execution_stopwatch, mutex_locked_time_stopwatch.delta, args.worst_execution_time);
		pthread_testcancel();
		sched_yield();
	}
}


int main(int argc, char* argv[]) {
	// lettura del valore passato al programma
    if (argc < 2) {
        printf("Uso: %s <numero corrispondente ai millimetri al secondo da percorrere>\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    speeds.general_speed = atof(argv[1]);
	if(speeds.general_speed < MINIMUM_SPEED || speeds.general_speed > MAXIMUM_SPEED) {
		printf("la velocità deve essere tra %d e %d, estremi inclusi\n", MINIMUM_SPEED, MAXIMUM_SPEED);
		exit(EXIT_FAILURE);
	}

	// inizializzazione di GPIO
	if (gpioInitialise() < 0) {
        fprintf(stderr, "Errore inizializzazione GPIO\n");
        exit(EXIT_FAILURE);
    }

	// creazione degli struct relativi a motori e encoder
	cbMotor_t left_motor = {PIN_LEFT_FORWARD, PIN_LEFT_BACKWARD};
	cbMotor_t right_motor = {PIN_RIGHT_FORWARD, PIN_RIGHT_BACKWARD};
	cbEncoder_t left_encoder = {PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B, GPIO_PIN_NC, PTHREAD_MUTEX_INITIALIZER};
    cbEncoder_t right_encoder = {PIN_ENCODER_RIGHT_A, PIN_ENCODER_RIGHT_B, GPIO_PIN_NC, PTHREAD_MUTEX_INITIALIZER};
	
	// chiamate a GPIO
	cbMotorGPIOinit(&left_motor);
    cbMotorGPIOinit(&right_motor);
    cbEncoderGPIOinit(&left_encoder);
    cbEncoderGPIOinit(&right_encoder);
    cbEncoderRegisterISRs(&left_encoder, 0);
    cbEncoderRegisterISRs(&right_encoder, 0);

	//creazione degli struct per l'odometria
	odometry_data_t odometry_data_left = {.lock = PTHREAD_MUTEX_INITIALIZER, .are_ticks_reset = false, .ticks = 0};
	odometry_data_t odometry_data_right = {.lock = PTHREAD_MUTEX_INITIALIZER, .are_ticks_reset = false, .ticks = 0};

	// creazione degli struct relativi ai thread
	task_t left_motor_control_task = {.entry_point = motor_control_entry, .worst_execution_time = 0};
	task_t right_motor_control_task = {.entry_point = motor_control_entry, .worst_execution_time = 0};
	task_t cartesian_control_task = {.entry_point = cartesian_control_entry, .worst_execution_time = 0};

	// creazione degli struct per gli argument dei thread
	char left_name[] = "sinistra";
	motor_control_args_t left_motor_control_args ={
		left_name,
		&left_motor,
		&left_encoder,
		MILLIMETERS_PER_TICK_LEFT,
		&speeds.left_wheel_speed,
		&odometry_data_left,
		&left_motor_control_task.worst_execution_time,
		&cartesian_control_task.tid
	};
	char right_name[] = "destra";
	motor_control_args_t right_motor_control_args = {
		right_name,
		&right_motor,
		&right_encoder,
		MILLIMETERS_PER_TICK_RIGHT,
		&speeds.right_wheel_speed,
		&odometry_data_right,
		&right_motor_control_task.worst_execution_time,
		&cartesian_control_task.tid
	};
	cartesian_control_args_t cartesian_control_args = {
		&odometry_data_left,
		&odometry_data_right,
		&cartesian_control_task.worst_execution_time
	};

	//creazione dei punti del arco
	switch(PATH_INDEX) {
		case 0:
			/* per ruotare intorno ad un righello di 30 cm:
			   15 cm di raggio, + un cm per i bordi del righello + 7 cm tra il lato estreno della ruota e il centro del robot*/
			generate_arc_points(waypoints, N_POINTS, 0, 0, 230, 230, -1.57, 1.57);
			break;
		case 1:
			// per percorrere un righello di 30 cm
			generate_straight_line_points(waypoints, N_POINTS, 0, 0, 0, 300, 0);
			break;

		case 2:
			// percorso a forma di 8
			{
				int quarter = N_POINTS / 4;
				float radius = 200;
				generate_arc_points(waypoints, quarter, 0, 0, -radius, radius, 1.57, -1.57);
				generate_straight_line_points(waypoints, quarter, quarter, 0, -radius * 2, -radius * 2, 0);
				generate_arc_points(waypoints, quarter, quarter * 2, -radius * 2, -radius, radius, 1.57, 4.71);
				generate_straight_line_points(waypoints, quarter, N_POINTS - quarter, -radius * 2, -radius * 2, 0, 0);
			}
			break;

		default:
		perror("main: selezione del percorso non valida");
		exit(EXIT_FAILURE);
	}
	for(int i = 0; i < N_POINTS; ++i) 
		//printf("new Point(%f, %f), ", waypoints[i].x, waypoints[i].y);
		printf("punto %d: (%f, %f)\n", i, waypoints[i].x, waypoints[i].y);
	puts("\n");

	// creazione dei thread
	if(pthread_create(&(left_motor_control_task.tid), NULL, left_motor_control_task.entry_point, &left_motor_control_args) != 0) {
		perror("main: pthread_create: left_motor_control_task");
		exit(EXIT_FAILURE);
	}
	if(pthread_create(&(right_motor_control_task.tid), NULL, right_motor_control_task.entry_point, &right_motor_control_args) != 0) {
		perror("main: pthread_create: right_motor_control_task");
		exit(EXIT_FAILURE);
	}
	if(pthread_create(&(cartesian_control_task.tid), NULL, cartesian_control_task.entry_point, &cartesian_control_args) != 0) {
		perror("main: pthread_create: cartesian_control_task");
		exit(EXIT_FAILURE);
	}

	// attendi la terminazione
	if(pthread_join(cartesian_control_task.tid, NULL) != 0) {
		perror("main: pthread_join: cartesian_control_task");
		exit(EXIT_FAILURE);
	} else {
		puts("Percorso completato");
	}

	// terminazione del programma
	puts("Terminazione...");
	pthread_cancel(left_motor_control_task.tid);
	pthread_cancel(right_motor_control_task.tid);
	pthread_cancel(cartesian_control_task.tid);
    cbMotorReset(&left_motor);
    cbMotorReset(&right_motor);
    gpioTerminate();
	puts("Fine");
	printf("tempo peggiore della task di controllo del motore sinistro: %ld ns\n", left_motor_control_task.worst_execution_time);
	printf("tempo peggiore della task di controllo del motore destro: %ld  ns\n", right_motor_control_task.worst_execution_time);
	printf("tempo peggiore della task di controllo cartesiano: %ld ns\n", cartesian_control_task.worst_execution_time);
	exit(EXIT_SUCCESS);
}