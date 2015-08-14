#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "sema.h"
typedef xQueueHandle xSemaphoreHandle;

void Ftakex(void * pvParameters) {

	vTaskDelay(1000);
	int ff = 0;

	while (1) {
		xQueueReceive( xQueue, &ff, 100 );
		while (ff == 1) {
			if ((xSemaphoreTake(flagx, 99999999)) == pdTRUE) {
				printf("sema 3  Get Flag!!!\n");
			}
			if ( xQueueReceive( xQueue, &ff, 100 ) )
			{
				printf("ff = %d \n", ff);
			}
			if (ff < 1)
				printf("sema3  are closed!!!\n");
		}
		vTaskDelay(1000);
	}
}
