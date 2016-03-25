/*
 * task_Anomaly_Monitor.c
 *
 *  Created on: 	2016/03/14
 *  Last updated: 	2016/03/24
 *  Author: 		Kenny Huang
 */

#include <util/timestamp.h>
#include <util/delay.h>
#include <util/hexdump.h>
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
#define overCurrentThreshold_ADCS_33	200
#define overCurrentThreshold_GPS		500

#define temperature_test			0
#define current_Test				1
#define full_test					0
extern int TS1_4();
extern int TS5();
extern int TS6();
extern int TS7();
extern int TS8();
extern int TS9();

void Anomaly_Monitor_Task(void * pvParameters)
{
	int outRangeCounter_ADCS_5 = 0;
	int outRangeCounter_ADCS_33 = 0;
	int outRangeCounter_GPS = 0;
	int outRangeCounter_temp[11] = {0};
	uint8_t txbuf[2];
	uint8_t rxbuf[66];
	uint16_t subCurrent[6];
	uint16_t subTemperature[11];
	int16_t subTemperature_int[11];
	while (1) {

#if temperature_test
		/* OBC Temperature */
		TS9();
		subTemperature[0] = ThermalFrame.T9;
		printf("\t\t\tTemperature %.1f degree\r\n", ((((subTemperature[0] * 2493.0) / 1023) - 424) / 6.25));
		printf("\E[1A\r");

		/* Operational Temperature Range -40 to +60 */
		if (((((subTemperature[0] * 2493.0) / 1023) - 424) / 6.25) > 60 || ((((subTemperature[0] * 2493.0) / 1023) - 424) / 6.25) < - 40) {
			outRangeCounter_temp[0] ++;
			if (outRangeCounter_temp[0] >= 6) {
				generate_Error_Report(9, subTemperature[0]);
				outRangeCounter_temp[0] = 0;
			}
		}
		else {
			outRangeCounter_temp[0] = 0;
		}
		/* ----------------------------------------------------- */
		vTaskDelay(5 * delay_time_based);
		/* COM Temperature */
		TS5();
		subTemperature[1] = ThermalFrame.T5;
		printf("\t\t\tTemperature %.1f degree\r\n", (subTemperature[1] * (-0.0546) + 189.5522));
		printf("\E[1A\r");

		/* Operational Temperature Range -40 to +85 */
		if ((subTemperature[1] * (-0.0546) + 189.5522) > 85 || (subTemperature[1] * (-0.0546) + 189.5522) < - 40) {
			outRangeCounter_temp[1] ++;
			if (outRangeCounter_temp[1] >= 6) {
				generate_Error_Report(10, subTemperature[1]);
				outRangeCounter_temp[1] = 0;
			}
		}
		else {
			outRangeCounter_temp[1] = 0;
		}
		/* ----------------------------------------------------- */
		vTaskDelay(5 * delay_time_based);
		/* Antenna Temperature */
		TS6();
		subTemperature[2] = ThermalFrame.T6;
		printf("\t\t\t(raw)Vout %.1f mV\r\n", (subTemperature[2] * 3.3 * 1000 / 1023));
		printf("\E[1A\r");

		/* Operational Temperature Range -20 to +60 */
		if ((subTemperature[2] * 3.3 * 1000 / 1023) > 2313 || (subTemperature[2] * 3.3 * 1000 / 1023) < 1448) {
			outRangeCounter_temp[2] ++;
			if (outRangeCounter_temp[2] >= 6) {
				generate_Error_Report(11, subTemperature[2]);
				outRangeCounter_temp[2] = 0;
			}
		}
		else {
			outRangeCounter_temp[2] = 0;
		}
		vTaskDelay(5 * delay_time_based);
		/* ----------------------------------------------------- */
		/* EPS Temperature */
		TS1_4();
		subTemperature_int[2] = (ThermalFrame.T1 + ThermalFrame.T2 + ThermalFrame.T3) / 3;
		subTemperature_int[3] = ThermalFrame.T4;

		printf("\t\t\ttTemperature %d degree\r\n", subTemperature_int[2]);
		printf("\t\t\ttTemperature %d degree\r\n", subTemperature_int[3]);
		printf("\E[1A\r");

		/* Operational Temperature Range -10 to +60 */
		if (subTemperature_int[2] > 125 || subTemperature_int[2] < -40) {
			outRangeCounter_temp[5] ++;
			if (outRangeCounter_temp[5] >= 6) {
				generate_Error_Report(16, subTemperature_int[2]);
				outRangeCounter_temp[5] = 0;
			}
		}
		else {
			outRangeCounter_temp[5] = 0;
		}
		if (subTemperature_int[3] > 125 || subTemperature_int[3] < -40) {
			outRangeCounter_temp[6] ++;
			if (outRangeCounter_temp[6] >= 6) {
				generate_Error_Report(17, subTemperature_int[3]);
				outRangeCounter_temp[6] = 0;
			}
		}
		else {
			outRangeCounter_temp[6] = 0;
		}

		/* ----------------------------------------------------- */
		vTaskDelay(5 * delay_time_based);

#endif
#if full_test
		if (adcs_task != NULL) {
#endif

#if current_Test
			txbuf[0] = 0x08;
			txbuf[1] = 0x02;
			if (i2c_master_transaction_2(0, stm_eps_node, &txbuf, 2, &rxbuf, 10 + 2, eps_delay) == E_NO_ERR) {
				memcpy(&subCurrent[0], &rxbuf[2], 2);	// i5	ADCS 	H1-47
				// memcpy(&subCurrent[1], &rxbuf[4], 2);	// i5	 		H1-49
				// memcpy(&subCurrent[2], &rxbuf[6], 2);	// i5	SEUV 	H1-51
				memcpy(&subCurrent[3], &rxbuf[8], 2);	// i3.3	ADCS 	H1-48
				memcpy(&subCurrent[4], &rxbuf[10], 2);	// i3.3	GPS 	H1-50
				// memcpy(&subCurrent[5], &rxbuf[12], 2);	// i3.3	SEUV 	H1-52
				hex_dump(&subCurrent[0], 10);
			}
			printf("ADCS 5: %d\n", subCurrent[0]);
			if (subCurrent[0] > overCurrentThreshold_ADCS_5 ) {
				outRangeCounter_ADCS_5 ++;
				printf("Out of range detected, counter = %d", outRangeCounter_ADCS_5);
				if (outRangeCounter_ADCS_5 >= 6) {
					generate_Error_Report(3, subCurrent[0]);
					outRangeCounter_ADCS_5 = 0;
				}
			}
			else {
				outRangeCounter_ADCS_5 = 0;
			}
			printf("ADCS 33: %d\n", subCurrent[3]);
			if (subCurrent[3] > overCurrentThreshold_ADCS_33 ) {
				outRangeCounter_ADCS_33 ++;
				printf("Out of range detected, counter = %d", outRangeCounter_ADCS_33);
				if (outRangeCounter_ADCS_33 >= 6) {
					generate_Error_Report(2, subCurrent[3]);
					outRangeCounter_ADCS_33 = 0;
				}
			}
			else {
				outRangeCounter_ADCS_33 = 0;
			}
			printf("GPS: %d\n", subCurrent[4]);
			if (subCurrent[4] > overCurrentThreshold_GPS ) {
				outRangeCounter_GPS ++;
				printf("Out of range detected, counter = %d", outRangeCounter_GPS);
				if (outRangeCounter_GPS >= 6) {
					power_control(2, OFF);	//power off GPS
					use_GPS_header = 0;
					generate_Error_Report(4, subCurrent[4]);
					outRangeCounter_GPS = 0;
				}
			}
			else {
				outRangeCounter_GPS = 0;
			}
			/* ----------------------------------------------------- */
			vTaskDelay(5 * delay_time_based);
#endif
#if temperature_test
			/* ADCS ARM Temperature */
			TS7();
			subTemperature_int[0] = ThermalFrame.T7;
			printf("\t\t\ttTemperature %d degree\r\n", subTemperature_int[0]);
			printf("\E[1A\r");

			/* Operational Temperature Range -10 to +60 */
			if (subTemperature_int[0] > 60 || subTemperature_int[0] < -10) {
				outRangeCounter_temp[3] ++;
				if (outRangeCounter_temp[3] >= 6) {
					generate_Error_Report(13, subTemperature_int[0]);
					outRangeCounter_temp[3] = 0;
				}
			}
			else {
				outRangeCounter_temp[3] = 0;
			}
			/* ----------------------------------------------------- */
			vTaskDelay(5 * delay_time_based);
			/* ADCS Rate Sensor & Magnetometer Temperature */
			TS8();
			subTemperature_int[1] = ThermalFrame.T8;
			printf("\t\t\ttTemperature %d degree\r\n", subTemperature_int[1]);
			printf("\E[1A\r");

			/* Operational Temperature Range -10 to +60 */
			if (subTemperature_int[1] > 60 || subTemperature_int[1] < -10) {
				outRangeCounter_temp[4] ++;
				if (outRangeCounter_temp[4] >= 6) {
					generate_Error_Report(14, subTemperature_int[1]);
					outRangeCounter_temp[4] = 0;
				}
			}
			else {
				outRangeCounter_temp[4] = 0;
			}
			/* ----------------------------------------------------- */
			vTaskDelay(5 * delay_time_based);
#endif
#if full_test
		}
		vTaskDelay(5 * delay_time_based);
#endif

	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}
