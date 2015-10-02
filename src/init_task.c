/*
 * init_task.c
 *
 *  Created on: 	2015/08/13	By rusei
 *  Last Update: 	2015/09/23	By Kenny
 *      Author: rusei, Kenny
 */

#include "fs.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <dev/cpu.h>
#include <dev/i2c.h>
#include <util/hexdump.h>
#include "parameter.h"
#include <csp/csp.h>
#include "subsystem.h"
#include <util/timestamp.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "Tele_function.h"

#define E_NO_ERR -1

void Init_Task(void * pvParameters) {

	if (parameter_init()  == Error)
		printf("Can't read parameter from fs\n");
	else
		printf("Loaded parameter from fs\n");
	/* Activate Battery check task */
	extern void BatteryCheckTask(void * pvParameters);
	xTaskCreate(BatteryCheckTask, (const signed char *) "BatCk", 1024 * 4, NULL, 2, NULL);
	
	/*   Idle 30M in first flight  */
	if (parameters.ant_deploy_flag == 0) {
		printf("Idle 30 Minutes before deploy  \r\n");
		printf("You can Key-in 'idleunlock' to cancel this idle  \r\n");
		printf("Or Key-in 'testmode' to enter ground test mode \r\n");
	}

	while (parameters.ant_deploy_flag == 0) {
		static int count = 0;
		vTaskDelay(5000);
		count += 5;
		printf("----------------------------------------------------- \r\n");
		printf("Left %d second to deploy devices \r\n", (1800 - count));
		if (count >= 1800) {
			parameters.ant_deploy_flag = 1;
			para_w();           //update to parameter system

		}
	}
	/*   Deploy Device  */
	if (idleunlocks != 1) {
		deploy_antenna();
		// printf("Antenna  deployed \n");
	}

	printf("-----------------------------------------\n");
	printf("Active Telecom Task, User can start to upload Ground Telecommand\n");
	extern void Telecom_Task(void * pvParameters);
	xTaskCreate(Telecom_Task, (const signed char * ) "COM", 1024 * 4, NULL, 2, NULL);
	/* Activate WOD collecting task, and start to transmit the beacon */
	extern void vTaskwod(void * pvParameters);
	xTaskCreate(vTaskwod, (const signed char * ) "WOD", 1024 * 4, NULL, 2, &wod_task);

	/* change to the ADCS mode */ 
	HK_frame.mode_status_flag = 2;


	/** End of init */
	vTaskDelete(NULL);
}

/*--------------------------------------------------------------------*/

