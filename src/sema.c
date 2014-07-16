#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
typedef xQueueHandle xSemaphoreHandle;
#include <freertos/semphr.h>
#include "sema.h"

xSemaphoreHandle flag;
xSemaphoreHandle flagx;
xQueueHandle xQueue;
void Fgive(void * pvParameters) {
	vSemaphoreCreateBinary(flag);
	vSemaphoreCreateBinary(flagx);
	xQueue = xQueueCreate( 2, sizeof( int ) );
	vTaskDelay(1000);
   int item =1;
   printf("turn on sema3\n");
	xQueueSend( xQueue, &item ,500 );


int a=0;

	while(1){
	//	printf("Give Flag to3\n");
		xSemaphoreGive(flagx);
		vTaskDelay(1000);


	//	 printf("Give Flag to2\n");
//	    xSemaphoreGive(flag);
//	    vTaskDelay(1000);
	    a++;

	   if(a==7){
		   printf("turn off sema3\n");
		item =0;
	    xQueueSend( xQueue, &item ,500 );
	   }
	   if(a==14){
		   printf("turn on sema3\n");
		item =1;
	    xQueueSend( xQueue, &item ,500 );
	    a=1;
	   }


	}}


