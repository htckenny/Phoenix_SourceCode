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
#include "tele_function.h"
#include <nanomind.h>

extern void BatteryCheck_Task(void * pvParameters);
extern void Telecom_Task(void * pvParameters);
extern void WOD_Task(void * pvParameters);
extern void Anomaly_Monitor_Task(void * pvParameters);
extern uint16_t battery_read();

void Init_Task(void * pvParameters) {
	timestamp_t t;
#if antenna_deploy
	uint8_t txdata;
#endif
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	lastCommandTime = t.tv_sec;
	/* Initialize the system parameters */
	if (parameter_init()  == Error)
		printf("Can't read parameter from fs\n");
	else
		printf("Loaded parameter from fs\n");

	/* Idle 30 minutes in first flight  */
	if (parameters.ant_deploy_flag == 0) {
		printf("Idle 30 Minutes before deploy  \r\n");
		printf("You can Key-in 'idleunlock' to cancel this idle time \r\n");
		printf("Or Key-in 'testmode' to enter ground test mode \r\n");
	}

	while (parameters.ant_deploy_flag == 0) {
		static int count = 0;
		vTaskDelay(5 * delay_time_based);
		count += 5;
		printf("----------------------------------------------------- \r\n");
		printf("Left %d second to deploy devices \r\n", (1800 - count));
		if (count >= 1800) {
			parameters.ant_deploy_flag = 1;
			para_w_flash();
		}
		if (idleunlocks == 1)
			break;
		battery_read(); /* To avoid EPS watchdog timer */
	}

#if !ground_Test_Mode
	/*   Deploy Device  */
	if (idleunlocks != 1) {
		deploy_antenna(ant_deploy_timeout);
		printf("Antenna Deployed!!\n");
	}
#endif
	printf("-----------------------------------------\n");

	/* Activate Battery check task */
	if (bat_check_task == NULL)
		xTaskCreate(BatteryCheck_Task, (const signed char *) "BatCk", 1024 * 4, NULL, 2, &bat_check_task);
	/* Activate telecom task, enable receiver to receive command from GS */
	if (com_task == NULL)
		xTaskCreate(Telecom_Task, (const signed char * ) "COM", 1024 * 4, NULL, 3, &com_task);
	/* Activate WOD collecting task, and start to transmit the beacon */
	if (wod_task == NULL)
		xTaskCreate(WOD_Task, (const signed char * ) "WOD", 1024 * 4, NULL, 1, &wod_task);
	/* Activate Anomaly Monitor task, monitoring all the anomaies */
	if (Anom_mon_task == NULL)
		xTaskCreate(Anomaly_Monitor_Task, (const signed char *) "Anom", 1024 * 4, NULL, 2, &Anom_mon_task);

	vTaskDelay(5 * delay_time_based);
	if (parameters.first_flight == 1) {
		while (antenna_status_check() == Error) {
#if !antenna_deploy
			break;
#endif
			printf("Deploy again(in the loop)\n");
			deploy_antenna(0);
			vTaskDelay(30 * delay_time_based);
		}

#if antenna_deploy
		/* Disarm ant board */
		txdata = 172;
		if (i2c_master_transaction_2(0, ant_node, &txdata, 1, 0, 0, com_delay) == E_NO_ERR) {
			printf("Disarm antenna\n");
		}
#endif

	}
	/* change to the ADCS mode */
	HK_frame.mode_status_flag = 2;

	/* End of init */
	init_task = NULL;
	vTaskDelete(NULL);
}