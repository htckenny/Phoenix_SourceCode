#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
typedef xQueueHandle xSemaphoreHandle;
#include "mode_control.h"

/**
 * xQueueHandle wod_flag;
 * xQueueHandle hk_flag;
 * xQueueHandle inms_flag;
 */
void task_name(void * pvParameters) {    /// choose the name you want
	int flag=0;

	while(1){
	/* waiting for mode_control task to turn it on */
	xQueueReceive(wod_flag,&flag,999999999 );
	/////change wod_flag if are not developing wod task
	while(flag==1){

		/// write your task here



	}}}
