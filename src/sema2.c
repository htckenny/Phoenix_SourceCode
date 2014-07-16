#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
typedef xQueueHandle xSemaphoreHandle;
#include <freertos/semphr.h>
#include "sema.h"


void Ftake(void * pvParameters) {
	vTaskDelay(1000);
	while(1){

		if((xSemaphoreTake(flag,999999))== pdTRUE){
		printf("sema2  Get Flag!!!\n");
	                   }

}
}
