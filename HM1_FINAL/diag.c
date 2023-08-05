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
#include <sched.h>
#define _GNU_SOURCE

static mqd_t req_ds; 
static mqd_t res_ds; 

void* diag()
{
    /*struct message_t{
		int avg_sensor;
		int control;
		//int buf[BUF_SIZE];
		int reference;
	};  */

    //struct message_t pippo;

    char request[MAX_MSG_SIZE];
    char response[MAX_MSG_SIZE];

    struct mq_attr attr;

    attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;

    // apertura coda delle richieste di diagnostica in sola scrittura
    if((req_ds = mq_open(REQ_DS_QUEUE_NAME, O_WRONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1){
        perror("diag: mq_open(req_ds)");
        exit(1);
    }

    if(mq_send(req_ds, "r", strlen(request)+1, 0) == -1){ // il secondo parametro dovrebbe essere request
        perror("Diag: not to able to send request to Deferrable Server");
    }

    // apertura della coda delle richieste di diagnostica in sola lettura
    if((res_ds = mq_open(RES_DS_QUEUE_NAME, O_RDONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1){
        perror("diag: mq_open(res_ds)");
        exit(1);
    }
    

    if(mq_receive(res_ds, response,  sizeof(response) , NULL) == -1){
        perror("Diag: not to able to receive messages from Deferrable Server");
    }

    printf("----Messaggio di diagnostica----\n");
    printf("avg_sensor , control , buffer, reference\n");
    printf("%s\n", response);
    /*printf("%d\n", pippo.avg_sensor);
    printf("%d\n", pippo.control); */

    if (mq_close (req_ds) == -1) {
        perror ("diag : mq_close req_ds");
        exit (1);
	} 

    if (mq_close (res_ds) == -1) {
        perror ("diag : mq_close req_ds");
        exit (1);
	}
}

int main(){

    pthread_t thread_diag;
    pthread_attr_t attr;
    
    pthread_attr_init(&attr);
    pthread_create(&thread_diag, &attr, diag, NULL);
    pthread_attr_destroy(&attr);
    
    pthread_exit(NULL);
}