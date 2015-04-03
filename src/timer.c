#include <util/timestamp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <dev/cpu.h>
#include <util/delay.h>
#include "parameter.h"


void timesyn(){
	static timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 1000);
	virtual_clock = t.tv_sec;
	printf("OBC time syn: %d \r\n",virtual_clock);
}

void Clock_Task(void * pvParameters) {
	printf("Clock Task activated \r\n");
	int time_counter=0;
	int gps_counter=0;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1000;

	timesyn();

	while(1){
		xLastWakeTime = xTaskGetTickCount();
		virtual_clock++;
		if(time_counter==0)
			timesyn();
	/*	if(gps_counter==0)
	      gpstimesyn();              */

		time_counter++;
		gps_counter++;
		if(time_counter==60)
			time_counter=0;

		if(gps_counter==7200)
			gps_counter=0;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}
