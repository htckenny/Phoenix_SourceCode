#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sema.h"
#include <freertos/queue.h>
typedef xQueueHandle xSemaphoreHandle;
#include <freertos/semphr.h>

int gg=0;
void RTOS2(void * pvParameters) {

	vTaskDelay(1000);
	printf("RTOS!!!!!! 1111\n");
	vTaskDelay(1000);
	printf("RTOS!!!!!! 2222\n");
	vTaskDelay(1000);
	printf("RTOS!!!!!! 3333\n");
	vTaskDelay(1000);
	printf("RTOS!!!!!! 4444\n");
	vTaskDelay(1000);
	printf("RTOS!!!!!! 5555\n");
	vTaskDelay(1000);
	printf("RTOS!!!!!! 6666\n");
	vTaskDelay(1000);
	printf("RTOS!!!!!! 7777\n");


	while(1){



		taskENTER_CRITICAL();
		{
			vTaskDelay(300);
		printf("RTOS1 %d\n",gg);
	      gg++;

	      vTaskDelay(300);
	      		printf("RTOS2 %d\n",gg);
	      	      gg++;

	     	    vTaskDelay(300);
	      	    		printf("RTOS3 %d\n",gg);
	      	    	      gg++;

	     	    	    vTaskDelay(300);
	      	    	    		printf("RTOS4 %d\n",gg);
	      	    	    	      gg++;

		}
		taskEXIT_CRITICAL();

}
}
