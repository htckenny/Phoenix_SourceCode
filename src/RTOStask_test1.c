#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sema.h"
#include <freertos/queue.h>
typedef xQueueHandle xSemaphoreHandle;
#include <freertos/semphr.h>

void RTOS1(void * pvParameters) {

	while(1){

		vTaskDelay(2000);
	//	printf("Suspend RTOS2 \n");
		vTaskSuspend(test2);

		vTaskDelay(2000);
	//	printf("Resume RTOS2 \n");
		vTaskResume(test2);
		vTaskDelete(test2);
	                   }
}

