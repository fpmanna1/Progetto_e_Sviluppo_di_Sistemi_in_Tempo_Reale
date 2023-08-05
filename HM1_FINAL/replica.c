//------------------- REPLICA.C ---------------------- 
#define _GNU_SOURCE
#include <sched.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mqueue.h>
#include <fcntl.h>
#include <string.h>
#include "rt-lib.h"
#include "parameters.h"

//emulates the controller

static int keep_on_running = 1;


struct shared_int {
	int value;
	pthread_mutex_t lock;
};
struct shared_int shared_avg_sensor;
struct shared_int shared_control; 

int buffer[BUF_SIZE];
int head = 0; 


void * acquire_filter_loop(void * par) {
	
	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);

	// Messaggio da prelevare dal driver
	char message [MAX_MSG_SIZE];

	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda sensor del plant in lettura 
	mqd_t sensor_qd;
	if ((sensor_qd = mq_open (SENSOR_QUEUE_NAME, O_RDONLY | O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("acquire filter loop: mq_open (sensor)");
		exit (1);
	}
	unsigned int sum = 0;
	int cnt = BUF_SIZE;
	
	
	while (keep_on_running)
	{
		wait_next_activation(th);
		
		// PRELIEVO DATI dalla coda del PLANT
		if (mq_receive(sensor_qd, message,MAX_MSG_SIZE,NULL) == -1){
			perror ("acquire filter loop: mq_receive (actuator)");	
			break;						//DEBUG
		}
		else{ 
			buffer[head] = atoi(message);
			sum += buffer[head];
			head = (head+1)%BUF_SIZE;
			cnt--;

			//printf("\t\t\t\tbuffer[%d]=%d, sum=%d\n",head,buffer[head],sum); //DEBUG

			// calcolo media sulle ultime BUF_SIZE letture
			if (cnt == 0) {
				cnt = BUF_SIZE;
				pthread_mutex_lock(&shared_avg_sensor.lock);
				shared_avg_sensor.value = sum/BUF_SIZE;
				//printf("\t\t\t\tavg_sensor.value=%d\n",shared_avg_sensor.value); //DEBUG
				pthread_mutex_unlock(&shared_avg_sensor.lock);
				sum = 0;
			}	
		}
	}
	
		

	/* Clear */
    if (mq_close (sensor_qd) == -1) {
        perror ("acquire filter loop: mq_close sehsor_qd");
        exit (1);
    }

	return 0;
}


void * control_loop(void * par) {

	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);
	
	// Messaggio da prelevare dal reference
	char message [MAX_MSG_SIZE];
	
	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda per il reference, in lettura e non bloccante
	mqd_t reference_qd;
	if ((reference_qd = mq_open (REFERENCE_QUEUE_NAME, O_RDONLY | O_CREAT | O_NONBLOCK, QUEUE_PERMISSIONS,&attr)) == -1) {
		// receive non bloccante perchè altrimenti il controllore rimarrebbe bloccato indefinitamente in attesa del task aperiodico
		perror ("control loop (replica): mq_open (reference)");
		exit (1);
	}

	unsigned int reference = 110;

	unsigned int plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	

	while (keep_on_running)
	{
		
		wait_next_activation(th);
	
		// legge il plant state 
		pthread_mutex_lock(&shared_avg_sensor.lock);
		plant_state = shared_avg_sensor.value;
		pthread_mutex_unlock(&shared_avg_sensor.lock);

		// riceve la reference (in modo non bloccante)
		if (mq_receive(reference_qd, message,MAX_MSG_SIZE,NULL) == -1){
			//printf ("No reference ...\n");							//DEBUG
		}
		else{
			//printf ("Reference received: %s.\n",message);			//DEBUG
			reference = atoi(message);
		}

		// calcolo della legge di controllo
		error = reference - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
		else control_action = 3;

		// aggiorna la control action
		pthread_mutex_lock(&shared_control.lock);
		shared_control.value = control_action;
		pthread_mutex_unlock(&shared_control.lock);
	}
	
	
	/* Clear */
    if (mq_close (reference_qd) == -1) {
        perror ("control loop: mq_close reference_qd");
        exit (1);
    }
	return 0;
}

void * actuator_loop(void * par) {

	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);

	// Messaggio da prelevare dal driver
	char message [MAX_MSG_SIZE];

	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda actuator del plant in scrittura 
	mqd_t actuator_qd;
	if ((actuator_qd = mq_open (ACTUATOR_QUEUE_NAME, O_WRONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("actuator  loop: mq_open (actuator)");
		exit (1);
	}	

	unsigned int control_action = 0;
	unsigned int control = 0;
	while (keep_on_running)
	{
		
		wait_next_activation(th);
		
		// prelievo della control action
		pthread_mutex_lock(&shared_control.lock);
		control_action = shared_control.value;
		pthread_mutex_unlock(&shared_control.lock);
		
		switch (control_action) {
			case 1: control = 1; break;
			case 2:	control = -1; break;
			case 3:	control = 0; break;
			default: control = 0;
		}
		//printf("Control: %d\n",control); //DEBUG
		sprintf (message, "%d", control);
		//invio del controllo al driver del plant
		if (mq_send (actuator_qd, message, strlen (message) + 1, 0) == -1) {
		    perror ("Sensor driver: Not able to send message to controller");
		    continue;
		}
	}

	/* Clear */
    if (mq_close (actuator_qd) == -1) {
        perror ("Actuator loop: mq_close actuator_qd");
        exit (1);
    }
	return 0;
}

void* watchdog(void* par)
{
	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);

	// Messaggio da prelevare dal driver
	char message [MAX_MSG_SIZE];

	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda per gli heart beat del controllore  
	mqd_t wd_qd;
	if ((wd_qd = mq_open (WDOG_QUEUE_NAME, O_RDONLY | O_CREAT , QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("actuator  loop: mq_open (actuator)");
		exit (1);
	}	

	int flag=1;

	while(keep_on_running)
	{
		wait_next_activation(th);

		struct timespec now;
		struct timespec t;
		
		now.tv_nsec = 0;
		now.tv_sec = 0;

		clock_gettime(CLOCK_REALTIME, &now);
		t.tv_nsec = now.tv_nsec;
		t.tv_sec = now.tv_sec;

		timespec_add_us(&t, TICK_TIME*BUF_SIZE*2);

		pthread_t acquire_filter_thread;
   		pthread_t control_thread;
    	pthread_t actuator_thread;

		pthread_mutex_init(&shared_avg_sensor.lock, NULL);
		pthread_mutex_init(&shared_control.lock, NULL);

		pthread_attr_t myattr;
		struct sched_param myparam;

		pthread_attr_init(&myattr);
		pthread_attr_setschedpolicy(&myattr, SCHED_FIFO);
		pthread_attr_setinheritsched(&myattr, PTHREAD_EXPLICIT_SCHED); 

		// ACQUIRE FILTER THREAD
		periodic_thread acquire_filter_th;
		acquire_filter_th.period = TICK_TIME;
		acquire_filter_th.priority = 50;

		myparam.sched_priority = acquire_filter_th.priority;
		pthread_attr_setschedparam(&myattr, &myparam);

		cpu_set_t cpuset;
   		CPU_ZERO(&cpuset);
   		CPU_SET(0, &cpuset);
    	pthread_setaffinity_np(acquire_filter_thread, sizeof(cpu_set_t), &cpuset);

		// CONTROL THREAD
		periodic_thread control_th;
		control_th.period = TICK_TIME*BUF_SIZE;
		control_th.priority = 45;
		myparam.sched_priority = control_th.priority;
		
		pthread_attr_setschedparam(&myattr, &myparam); 
		pthread_setaffinity_np(control_thread, sizeof(cpu_set_t), &cpuset);

		// ACTUATOR THREAD
		periodic_thread actuator_th;
		actuator_th.period = TICK_TIME*BUF_SIZE;
		actuator_th.priority = 45;

		pthread_attr_setschedparam(&myattr, &myparam);
		pthread_setaffinity_np(actuator_thread, sizeof(cpu_set_t), &cpuset);

		// se entro due periodi del task la receive fallisce, allora avvio le repliche
		if(mq_timedreceive(wd_qd, message, MAX_MSG_SIZE, NULL, &t) == -1){
			if(flag==1){
			// avvio thread repliche
			
				int ref = atoi(message);
				//printf ("Reference Attuale : %d\n", ref); // DEBUG
				printf("----------Replica Attiva----------------\n");
 
				pthread_create(&acquire_filter_thread,&myattr,acquire_filter_loop,(void*)&acquire_filter_th); 
				pthread_create(&control_thread,&myattr,control_loop,(void*)&control_th);
				pthread_create(&actuator_thread,&myattr,actuator_loop,(void*)&actuator_th);
				
				flag=0;
			}
		}else{
			if(flag==0){
				printf("----------Replica Disattiva-------------\n");
			
				// aggiungere logica per la disattivazione delle repliche
				pthread_attr_destroy(&myattr);
				pthread_kill(acquire_filter_thread, 0);
				pthread_kill(control_thread, 0);
				pthread_kill(actuator_thread, 0);
				flag=1;
			}
		}
	}

	if (mq_close (wd_qd) == -1) {
        perror ("wdog loop: mq_close wd_qd");
        exit (1);
	}

}


int main(void)
{
	
	printf("The replica is STARTED! [press 'q' to stop]\n");
 	
	pthread_t watchdog_thread;

	pthread_attr_t myattr;
	struct sched_param myparam;

	pthread_attr_init(&myattr);
	pthread_attr_setschedpolicy(&myattr, SCHED_FIFO);
	pthread_attr_setinheritsched(&myattr, PTHREAD_EXPLICIT_SCHED);  	

	// WATCHDOG THREAD
	periodic_thread wdog_th;
	wdog_th.period = TICK_TIME*BUF_SIZE*2;
	wdog_th.priority = 40; // più bassa degli altri task

	myparam.sched_priority = wdog_th.priority;
	pthread_attr_setschedparam(&myattr, &myparam); 
	cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
 
	pthread_create(&watchdog_thread, &myattr, watchdog, (void*)&wdog_th);
	pthread_setaffinity_np(watchdog_thread, sizeof(cpu_set_t), &cpuset);

	pthread_attr_destroy(&myattr);
	
	
	/* Wait user exit commands*/
	while (1) {
   		if (getchar() == 'q') break;
  	}
	keep_on_running = 0;

	if (mq_unlink (REFERENCE_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink reference queue");
        exit (1);
    }   

	if (mq_unlink (WDOG_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink wdog queue");
        exit (1);
    }

 	printf("The Replica is STOPPED\n");

	return 0;
}



