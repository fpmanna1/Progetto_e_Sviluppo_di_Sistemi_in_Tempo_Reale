
/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include <time.h>


#define ENCODER_TICK_PERIOD 5000 //pdMS_TO_TICKS(5UL)
#define NSEC_PER_SEC 1000000000ULL

#define ENC_TASK_PRIO ( tskIDLE_PRIORITY + 3 )
#define RT_TASK_PRIO ( tskIDLE_PRIORITY + 2 )
#define BUDDY_TASK_PRIO ( tskIDLE_PRIORITY + 1 )
#define	DIAGNOSTIC_TASK_PRIO	( tskIDLE_PRIORITY + 4 )

#define TICK_PER_MS 1


static void task1(void* pvParameter);
static void task2(void* pvParameter);
static void enc(void* pvParameter);
static void scope(void* pvParameter);
static void diagnostic(void* pvParameter);

/* encoder */
struct enc_str
{
    unsigned int slit;		//valori oscillanti tra 0 e 1
    unsigned int home_slit;	//1 se in home, 0 altrimenti
    SemaphoreHandle_t xLock;
};

static struct enc_str enc_data;

struct _rising_edge {
    unsigned int count;
    SemaphoreHandle_t xLock;
};

static struct _rising_edge_ rising_edge;

struct _round_time {
    TickType_t time_diff;
    SemaphoreHandle_t xLock;
};

static struct _round_time round_time;

struct _slack_rt1 {
    TickType_t slack_time;
    SemaphoreHandle_t xLock;
};
static struct _slack_rt1 slack_rt1;

struct _slack_rt2 {
    TickType_t slack_time;
    SemaphoreHandle_t xLock;
};
static struct _slack_rt2 slack_rt2;

/***************************** THREADS ************************************/

static void rt_task1(void* pvParameter)
{

    (void)pvParameter;

    unsigned int last_value = 0;
    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = pdMS_TO_TICKS(ENCODER_TICK_PERIOD / 2);


    xSemaphoreTake(rising_edge.xLock, portMAX_DELAY);

    rising_edge.count = 0;

    xSemaphoreGive(rising_edge.xLock);

    /* initialise xNextWakeTime - this only needs to be done once */
    xNextWakeTime = xTaskGetTickCount();

    for(;;) {
    	TickType_t r= xTaskGetTickCount();
    	
        vTaskDelayUntil(&xNextWakeTime, xBlockTime); /* wait next activation */

        xSemaphoreTake(rising_edge.xLock, portMAX_DELAY);

        if (last_value == 0 && enc_data.slit == 1) { // fronte di salita
            last_value = 1;

            xSemaphoreTake(rising_edge.xLock, portMAX_DELAY);

            rising_edge.count++;

            xSemaphoreGive(rising_edge.xLock);
        }
        else
            if (last_value == 1 && enc_data.slit == 0) { // fronte di discesa
                last_value = 0; // così quando sarà alto la prossima volta si accorgerà del rising edge
            }

        xSemaphoreGive(rising_edge.xLock);
        
        //Slack time
        TickType_t FinishTime = xTaskGetTickCount();
        
        if (r > FinishTime) {
            xSemaphoreTake(slack_rt1.xLock, portMAX_DELAY);

            slack_rt1.slack_time = r - FinishTime;

            xSemaphoreGive(slack_rt1.xLock);

        }
    }
}

static void rt_task2(void* pvParameter)
{

    (void)pvParameter;

    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = pdMS_TO_TICKS(ENCODER_TICK_PERIOD / 2);
  
    TickType_t time_home;
    TickType_t last_time_home;

    int first_measure = 1;
    int last_home_slit = 0;

    //portTICK_RATE_MS(time_home);
//#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
    //configTICK_RATE_HZ;
    for (;;) {
    	
    	TickType_t r = xTaskGetTickCount();
    	
        vTaskDelayUntil(&xNextWakeTime, xBlockTime); /* wait next activation */

        xSemaphoreTake(enc_data.xLock, portMAX_DELAY);

        if (enc_data.home_slit == 1 && last_home_slit == 0) {
            last_home_slit = 1;
            if (first_measure) {
             
                last_time_home = xTaskGetTickCount();
                last_time_home *= 1000; // conversione in nanosecondi

                first_measure = 0;
            }
            else {
                time_home = xTaskGetTickCount();
                time_home *= 1000; // conversione in nanosecondi

                xSemaphoreTake(round_time.xLock, portMAX_DELAY);

                round_time.time_diff = time_home - last_time_home;
                xSemaphoreGive(round_time.xLock);

                last_time_home = time_home;
            }
        }
        else if (enc_data.home_slit == 0) {
            last_home_slit = 0;
        }

        xSemaphoreGive(enc_data.xLock);
        
        /* Slack Time*/

        TickType_t FinishTime = xTaskGetTickCount();
        if (r > FinishTime) {

            xSemaphoreTake(slack_rt2.xLock, portMAX_DELAY);

            slack_rt2.slack_time = r - FinishTime;

            xSemaphoreGive(slack_rt2.xLock);
        }


    }
}

static void scope(void* pvParameter) /*buddy thread*/
{

    (void)pvParameter;

    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = pdMS_TO_TICKS(ENCODER_TICK_PERIOD * 2);
    
    unsigned int count = 0;
    float diff_us = 0;
    unsigned int rpm = 0;

    for (;;) {
        vTaskDelayUntil(&xNextWakeTime, xBlockTime); /* wait next activation */

        xSemaphoreTake(rising_edge.xLock, portMAX_DELAY);

        count = rising_edge.count;

        xSemaphoreGive(rising_edge.xLock);

        printf("Rising Edge Counter : %d\t", count);

        xSemaphoreTake(round_time.xLock, portMAX_DELAY);

        diff_us = round_time.time_diff;	

        xSemaphoreGive(round_time.xLock);
		if (diff_us !=0){
			rpm = (unsigned int)(60 * 1000 / diff_us);  // in microseconds 60*1000000/1000
		}
        //printf("diff : %f\t",diff_us);				//DEBUG
        printf("RPM : %u\n", rpm);
    }
}

static void enc(void* pvParameter)
{
    printf("Encoder Start\n");
    
	(void)pvParameter;
	
    xSemaphoreTake(enc_data.xLock, portMAX_DELAY);

    enc_data.slit = 0;
    enc_data.home_slit = 0;

    xSemaphoreGive(enc_data.xLock);

    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = pdMS_TO_TICKS(ENCODER_TICK_PERIOD);
    unsigned int count = 0;
    unsigned int slit_count = 0;
    unsigned int prev_slit = 0;

    /* Randomized period (75-750 RPM) */
    srand(time(NULL));
    unsigned int semi_per = (rand() % 10) + 1;
    //semi_per = 5;								//DEBUG


    for (;;) {
        vTaskDelayUntil(&xNextWakeTime, xBlockTime); /* wait next activation */

        xSemaphoreTake(enc_data.xLock, portMAX_DELAY);

        prev_slit = enc_data.slit;
        if (count % semi_per == 0) {
            enc_data.slit++;
            enc_data.slit %= 2;
        }

        if (prev_slit == 0 && enc_data.slit == 1) 					//fronte di salita
            slit_count = (++slit_count) % 8;

        if (slit_count == 0) enc_data.home_slit = enc_data.slit;
        else enc_data.home_slit = 0;

        //printf("%d:\t\t %d %d\n",count,enc_data.slit,enc_data.home_slit);	//DEBUG encoder
        count++;

        xSemaphoreGive(enc_data.xLock);
    }
}


static void diagnostic(void* pvParameter) {
	
	printf("Diagnostic Start\n");
	
	(void)pvParameter;
    TickType_t avg_slack = 0;
    int i = 0;
    int rounds = 100;

    TickType_t xNextWakeTime;
    const TickType_t xBlockTime = pdMS_TO_TICKS(ENCODER_TICK_PERIOD);

    

    xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xNextWakeTime, xBlockTime);

        xSemaphoreTake(slack_rt1.xLock, portMAX_DELAY);
        xSemaphoreTake(slack_rt2.xLock, portMAX_DELAY);

        avg_slack += (slack_rt1.slack_time + slack_rt2.slack_time) / 2;

        xSemaphoreGive(slack_rt1.xLock);
        xSemaphoreGive(slack_rt2.xLock);

        i++;

        if (i == rounds) {
            avg_slack = avg_slack / rounds;
            printf("*******SLACK TIME: %ld us*******\n", avg_slack);
            i = 0;
        }
    }
}

/*** SEE THE COMMENTS AT THE TOP OF THIS FILE ***/
void main_blinky(void)
{
    enc_data.xLock = xSemaphoreCreateMutex();
    rising_edge.xLock = xSemaphoreCreateMutex();
    round_time.xLock = xSemaphoreCreateMutex();
    slack_rt1.xLock = xSemaphoreCreateMutex();
    slack_rt2.xLock = xSemaphoreCreateMutex();

	
    xTaskCreate(enc, "encoder", configMINIMAL_STACK_SIZE, (void*)0, ENC_TASK_PRIO, NULL);
    xTaskCreate(rt_task1, "rt_task1", configMINIMAL_STACK_SIZE, (void*)0, RT1_TASK_PRIO, NULL);
    xTaskCreate(rt_task1, "rt_task2", configMINIMAL_STACK_SIZE, (void*)0, RT2_TASK_PRIO, NULL);
    xTaskCreate(scope, "scope", configMINIMAL_STACK_SIZE, (void*)0, BUDDY_TASK_PRIO, NULL);
    xTaskCreate(diagnostic, "diagnostic", configMINIMAL_STACK_SIZE, (void*)0, DIAGNOSTIC_TASK_PRIO, NULL);
   

    vTaskStartScheduler();

    vSemaphoreDelete(enc_data.xLock);
    vSemaphoreDelete(rising_edge.xLock);
    vSemaphoreDelete(round_time.xLock);
    vSemaphoreDelete(slack_rt1.xLock);
    vSemaphoreDelete(slack_rt2.xLock);
}
