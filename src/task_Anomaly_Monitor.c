/*
 * task_Anomaly_Monitor.c
 *
 *  Created on: 	2016/03/14
 *  Last updated: 	2016/03/14
 *  Author: 		Kenny Huang
 */

#include <util/timestamp.h>
#include <util/delay.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nanomind.h>
#include <csp/csp_endian.h>
#include <dev/i2c.h>
#include <string.h>
#include <time.h>
#include <fat_sd/ff.h>

#include "fs.h"
#include "tele_function.h"
#include "subsystem.h"
#include "parameter.h"

#define overCurrentThreshold_ADCS_5		100
#define overCurrentThreshold_ADCS_33	100
#define overCurrentThreshold_GPS		100

void Anomaly_Monitor_Task(void * pvParameters)
{
	int outRangeCounter_ADCS_5 = 0;
	int outRangeCounter_ADCS_33 = 0;
	int outRangeCounter_GPS = 0;
	uint8_t txbuf[2];
	uint8_t rxbuf[66];
	uint16_t subCurrent[6];

	while (1) {

		txbuf[0] = 0x08;
		txbuf[1] = 0x02;
		if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 64 + 2, eps_delay) == E_NO_ERR) {
			memcpy(&subCurrent[0], &rxbuf[2], 2);	// i5	ADCS 	H1-47
			// memcpy(&subCurrent[1], &rxbuf[4], 2);	// i5	 		H1-49
			// memcpy(&subCurrent[2], &rxbuf[6], 2);	// i5	SEUV 	H1-51
			memcpy(&subCurrent[3], &rxbuf[8], 2);	// i3.3	ADCS 	H1-48
			memcpy(&subCurrent[4], &rxbuf[10], 2);	// i3.3	GPS 	H1-50
			// memcpy(&subCurrent[5], &rxbuf[12], 2);	// i3.3	SEUV 	H1-52
		}

		if (subCurrent[0] > overCurrentThreshold_ADCS_5 ) {
			printf("Out of range %d\n", subCurrent[0]);
			outRangeCounter_ADCS_5 ++;
			printf("outCounter = %d\n", outRangeCounter_ADCS_5);
			if (outRangeCounter_ADCS_5 >= 6) {
				generate_Error_Report(3, &subCurrent[0]);
			}
		}
		else {
			outRangeCounter_ADCS_5 = 0;
		}

		if (subCurrent[3] > overCurrentThreshold_ADCS_33 ) {
			printf("Out of range %d\n", subCurrent[0]);
			outRangeCounter_ADCS_33 ++;
			printf("outCounter = %d\n", outRangeCounter_ADCS_33);
			if (outRangeCounter_ADCS_33 >= 6) {
				generate_Error_Report(2, &subCurrent[3]);
			}
		}
		else {
			outRangeCounter_ADCS_33 = 0;
		}

		if (subCurrent[4] > overCurrentThreshold_GPS ) {
			printf("Out of range %d\n", subCurrent[0]);
			outRangeCounter_GPS ++;
			printf("outCounter = %d\n", outRangeCounter_GPS);
			if (outRangeCounter_GPS >= 6) {
				generate_Error_Report(4, &subCurrent[4]);
			}
		}
		else {
			outRangeCounter_GPS = 0;
		}



	

		vTaskDelay(5 * delay_time_based);
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}
