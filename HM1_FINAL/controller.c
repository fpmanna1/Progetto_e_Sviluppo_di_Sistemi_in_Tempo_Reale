//------------------- CONTROLLER.C ---------------------- 
#define _GNU_SOURCE
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
#include <sched.h>
#include "parameters.h"

//emulates the controller

static int keep_on_running = 1;

void init(pthread_attr_t* attr, struct sched_param *myparam, periodic_thread* af, periodic_thread* c, periodic_thread* a, periodic_thread* ds);
// inizializzazione delle strutture condivise

// code del server
static mqd_t req_ds; 
static mqd_t res_ds;

struct shared_int {
	int value;
	pthread_mutex_t lock;
};
struct shared_int shared_avg_sensor;
struct shared_int shared_control; 

unsigned int heart_beat;

int buffer[BUF_SIZE];
int head = 0; 

void * acquire_filter_loop(void * par) {
	// preleva gli ultimi 5 valori letti dal sensore (acquire), ne fa una media (filter) e scrive 
	// questo valore in una variabile condivisa.
	
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
	
	// Apriamo la coda sensor del plant in lettura, in modo non bloccante
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

		// Prelievo dati dalla coda del PLANT
		if (mq_receive(sensor_qd, message,MAX_MSG_SIZE,NULL) == -1){
			perror ("acquire filter loop: mq_receive (actuator)");	
			break;						//DEBUG
		}
		else{ 
			//pthread_mutex_lock(&buffer.lock);
			buffer[head] = atoi(message); // conversione da stringa a intero
			sum += buffer[head];
			head = (head+1)%BUF_SIZE;
			//pthread_mutex_unlock(&buffer.lock);
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
	int count=0;
	
	// Messaggio da prelevare dal reference
	char message [MAX_MSG_SIZE];
	char message2 [MAX_MSG_SIZE];
	
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
		perror ("control loop: mq_open (reference)");
		exit (1);
	}

	// Apriamo la coda del watchdog, in sola scrittura
	mqd_t wd_qd;
	if((wd_qd = mq_open(WDOG_QUEUE_NAME, O_WRONLY | O_CREAT , QUEUE_PERMISSIONS, &attr)) == -1){
		perror ("control loop: mq_open (watchdog)");
		exit(2);
	}

	
	unsigned int reference = 110;

	unsigned int period_count = 0;
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

		heart_beat = reference;
		sprintf(message2, "%d", heart_beat);
		// invio heart beat ogni 2 periodi sulla coda del watchdog
		if(count%2 == 0){
			if(mq_send(wd_qd, message2, strlen(message2) + 1, 0) == -1)
			{
				perror ("Control Loop: Not able to send heartbeat to watchdog");
				continue;
			}
		}
		
		// calcolo della legge di controllo
		error = reference - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
		else control_action = 3;

		// aggiorna la control action
		pthread_mutex_lock(&shared_control.lock);
		shared_control.value = control_action; // scrittura nella var. condivisa "control"
		pthread_mutex_unlock(&shared_control.lock);

		count++;
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


void* ds(void* param)
{
    periodic_thread *th = (periodic_thread *) param;
	start_periodic_timer(th,0);
	printf("\nDeferrable Server Alive!\n");
    char message[MAX_MSG_SIZE];
    char request[MAX_MSG_SIZE];

    struct mq_attr attr;
	
	/*struct message_t{
		int avg_sensor;
		int control;
		//int buf[BUF_SIZE];
		int reference;
	};  */

	//struct message_t message;
	// nel tentare di scrivere nella coda i valori di diagnostica, in fase di esecuzione compariva sempre l'errore
	// "message too long" nella send.
	

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// apertura della coda delle richieste in sola lettura, in modo bloccante (differenza col PS)
    if ((req_ds = mq_open (REQ_DS_QUEUE_NAME, O_RDONLY | O_CREAT , QUEUE_PERMISSIONS, &attr)) == -1) {
		perror ("DS: mq_open (req_ds)");
		exit (1);
	}
	
	// apertura coda delle risposte in sola scrittura
	if ((res_ds = mq_open (RES_DS_QUEUE_NAME, O_WRONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1) {
		perror ("DS: mq_open (res_ds)");
		exit (1);
	}

    while(keep_on_running){
        wait_next_activation(th);
        
		// receive bloccante sulla coda delle richieste
        if(mq_receive(req_ds, request, MAX_MSG_SIZE, NULL) == -1){
            printf("NO requests\n");
        }

        char s_avg_sensor[4];
        char s_control[2];
        char s_buffer0[4];
		char s_buffer1[4];
		char s_buffer2[4];
		char s_buffer3[4];
		char s_buffer4[4];

        char s_reference[4];
        //shared_control.value = 40; */
        pthread_mutex_lock(&shared_avg_sensor.lock);
		sprintf(s_avg_sensor, "%d", shared_avg_sensor.value);
        pthread_mutex_unlock(&shared_avg_sensor.lock);

		pthread_mutex_lock(&shared_control.lock);
		sprintf(s_control, "%d",shared_control.value);
		pthread_mutex_unlock(&shared_control.lock);

		sprintf(s_buffer0, "%d", buffer[0]);
		sprintf(s_buffer1, "%d", buffer[1]);
		sprintf(s_buffer2, "%d", buffer[2]);
		sprintf(s_buffer3, "%d", buffer[3]);
		sprintf(s_buffer4, "%d", buffer[4]);

		
		strcpy(message, s_avg_sensor);
		strcat(message, " ");
		strcat(message, s_control);
		strcat(message, " " ); 
		strcat(message, s_buffer0);
		strcat(message, " ");
		strcat(message, s_buffer1); 
		/*strcat(message, " "); 
		strcat(message, s_buffer2); */


		//message.avg_sensor = shared_avg_sensor.value;
		//message.control = shared_control.value; 
	

        if(mq_send(res_ds, (const char*)&message, sizeof(message), 0) == -1){
            perror("Send di diagnostica fallito\n");
        }
        
    }
    if (mq_close (req_ds) == -1) {
        perror ("ds : mq_close req_ds");
        exit (1);
	}

    if (mq_close (res_ds) == -1) {
        perror ("ds: mq_close req_ds");
        exit (1);
	}

	if (mq_unlink (REQ_DS_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink req_ds queue");
        exit (1);
    }

	if (mq_unlink (RES_DS_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink res_ds queue");
        exit (1);
    }
}



int main(void)
{
	printf("The controller is STARTED! [press 'q' to stop]\n");
 	
	pthread_t acquire_filter_thread;
    pthread_t control_thread;
    pthread_t actuator_thread;
	pthread_t ds_thread;

	periodic_thread control_th;
	periodic_thread actuator_th;
	periodic_thread acquire_filter_th;
	periodic_thread ds_th;	

	pthread_attr_t myattr;
	struct sched_param myparam;

	init(&myattr, &myparam, &acquire_filter_th, &control_th, &actuator_th, &ds_th);
	cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
   
	pthread_create(&acquire_filter_thread,&myattr,acquire_filter_loop,(void*)&acquire_filter_th);
	pthread_setaffinity_np(acquire_filter_thread, sizeof(cpu_set_t), &cpuset);
    
	pthread_create(&control_thread,&myattr,control_loop,(void*)&control_th);
	pthread_setaffinity_np(control_thread, sizeof(cpu_set_t), &cpuset);
	
	pthread_create(&actuator_thread,&myattr,actuator_loop,(void*)&actuator_th);
	pthread_setaffinity_np(actuator_thread, sizeof(cpu_set_t), &cpuset);
	
	pthread_create(&ds_thread, &myattr, ds,(void*)&ds_th);
	pthread_setaffinity_np(ds_thread, sizeof(cpu_set_t), &cpuset);

	pthread_attr_destroy(&myattr);
	
	
	/* Wait user exit commands*/
	while (1) {
   		if (getchar() == 'q') break;
  	}
	keep_on_running = 0;

 	printf("The controller is STOPPED\n");

	return 0;
}

void init(pthread_attr_t* attr, struct sched_param* myparam, periodic_thread* af, periodic_thread* c, periodic_thread* a, periodic_thread* ds){
	
	pthread_mutex_init(&shared_avg_sensor.lock, NULL);
	pthread_mutex_init(&shared_control.lock, NULL);

	pthread_attr_init(attr);
	pthread_attr_setschedpolicy(attr, SCHED_FIFO);
	pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED); 

	af->period = TICK_TIME;
	af->priority = 50;

	myparam->sched_priority = af->priority;
	pthread_attr_setschedparam(attr, myparam); 

	c->period = TICK_TIME*BUF_SIZE;
	c->priority = 45;

	myparam->sched_priority = c->priority;
	pthread_attr_setschedparam(attr, myparam);

	a->period = TICK_TIME*BUF_SIZE;
	a->priority = 45;

	pthread_attr_setschedparam(attr, myparam);

	ds->period = TICK_TIME / 2;
	ds->priority = 60;

	pthread_attr_setschedparam(attr, myparam);
}




