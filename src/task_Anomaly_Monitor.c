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

#define overCurrentThreshold_ADCS_5		40
#define overCurrentThreshold_ADCS_33	120
#define overCurrentThreshold_GPS		420

#define temperature_test			1
#define current_Test				1
#define full_test					1

extern int TS1_4();
extern int TS5();
extern int TS6();
extern int TS7();
extern int TS8();
extern int TS9();

void Anomaly_Monitor_Task(void * pvParameters)
{
	uint8_t txbuf[2];
	uint8_t rxbuf[66];
#if current_Test
	int outRangeCounter_ADCS_5 = 0;
	int outRangeCounter_ADCS_33 = 0;
	int outRangeCounter_GPS = 0;
	uint16_t subCurrent[6] = {0};
#endif
#if temperature_test
	int outRangeCounter_temp[11] = {0};
	uint16_t subTemperature[11];
	int16_t subTemperature_int[11];
	int8_t adcsTemp[2];
#endif
	while (1) {

#if temperature_test
		/* OBC Temperature */
		TS9();
		subTemperature[0] = ThermalFrame.T9;
		printf("OBC Temperature %.1f degree\t", ((((subTemperature[0] * 2493.0) / 1023) - 424) / 6.25));

		/* Operational Temperature Range -40 to +60 */
		if (((((subTemperature[0] * 2493.0) / 1023) - 424) / 6.25) > 60 || ((((subTemperature[0] * 2493.0) / 1023) - 424) / 6.25) < - 40) {
			outRangeCounter_temp[0] ++;
			printf("counter = %d\n", outRangeCounter_temp[0]);
			if (outRangeCounter_temp[0] >= 6) {
				generate_Error_Report(9, subTemperature[0]);
				outRangeCounter_temp[0] = 0;
			}
		}
		else {
			printf("\n");
			outRangeCounter_temp[0] = 0;
		}
		/* ----------------------------------------------------- */
		vTaskDelay(5 * delay_time_based);
		/* COM Temperature */
		TS5();
		subTemperature[1] = ThermalFrame.T5;
		printf("COM Temperature %.1f degree\t", (subTemperature[1] * (-0.0546) + 189.5522));

		/* Operational Temperature Range -40 to +85 */
		if ((subTemperature[1] * (-0.0546) + 189.5522) > 85 || (subTemperature[1] * (-0.0546) + 189.5522) < - 40) {
			outRangeCounter_temp[1] ++;
			printf("counter = %d\n", outRangeCounter_temp[1]);
			if (outRangeCounter_temp[1] >= 6) {
				generate_Error_Report(10, subTemperature[1]);
				outRangeCounter_temp[1] = 0;
			}
		}
		else {
			printf("\n");
			outRangeCounter_temp[1] = 0;
		}
		/* ----------------------------------------------------- */
		vTaskDelay(5 * delay_time_based);
		/* Antenna Temperature */
		if (TS6() == No_Error) {
			subTemperature[2] = ThermalFrame.T6;
		}
		else
			subTemperature[2] = 0;
		printf("Antenna (raw)Vout %.1f mV\t", (subTemperature[2] * 3.3 * 1000 / 1023));

		/* Operational Temperature Range -20 to +60 */
		if ((subTemperature[2] * 3.3 * 1000 / 1023) > 2313 || (subTemperature[2] * 3.3 * 1000 / 1023) < 1448) {
			outRangeCounter_temp[2] ++;
			printf("counter = %d\n", outRangeCounter_temp[2]);
			if (outRangeCounter_temp[2] >= 6) {
				generate_Error_Report(11, subTemperature[2]);
				outRangeCounter_temp[2] = 0;
			}
		}
		else {
			printf("\n");
			outRangeCounter_temp[2] = 0;
		}
		vTaskDelay(5 * delay_time_based);
		/* ----------------------------------------------------- */
		/* EPS Temperature */
		/* Operational Temperature Range -40 to +125 */
		TS1_4();
		subTemperature_int[2] = (ThermalFrame.T1 + ThermalFrame.T2 + ThermalFrame.T3) / 3;
		subTemperature_int[3] = ThermalFrame.T4;

		printf("EPS Temperature %d degree\t", subTemperature_int[2]);
		if (subTemperature_int[2] > 125 || subTemperature_int[2] < -40) {
			outRangeCounter_temp[5] ++;
			printf("counter = %d\n", outRangeCounter_temp[5]);
			if (outRangeCounter_temp[5] >= 6) {
				generate_Error_Report(17, subTemperature_int[2]);
				outRangeCounter_temp[5] = 0;
			}
		}
		else {
			printf("\n");
			outRangeCounter_temp[5] = 0;
		}

		printf("BAT Temperature %d degree\t", subTemperature_int[3]);
		if (subTemperature_int[3] > 125 || subTemperature_int[3] < -40) {
			outRangeCounter_temp[6] ++;
			printf("counter = %d\n", outRangeCounter_temp[6]);
			if (outRangeCounter_temp[6] >= 6) {
				generate_Error_Report(18, subTemperature_int[3]);
				outRangeCounter_temp[6] = 0;
			}
		}
		else {
			printf("\n");
			outRangeCounter_temp[6] = 0;
		}

		/* ----------------------------------------------------- */
		vTaskDelay(5 * delay_time_based);

#endif
#if full_test
		if (adcs_task != NULL) {


#if current_Test
			txbuf[0] = 0x08;
			txbuf[1] = 0x02;
			if (i2c_master_transaction_2(0, stm_eps_node, &txbuf, 2, &rxbuf, 10 + 2, eps_delay) == E_NO_ERR) {
				subCurrent[0] = (rxbuf[2] << 8) + rxbuf[3];
				subCurrent[3] = (rxbuf[8] << 8) + rxbuf[9];
				subCurrent[4] = (rxbuf[10] << 8) + rxbuf[11];
				// memcpy(&subCurrent[0], &rxbuf[2], 2);	// i5	ADCS 	H1-47
				// memcpy(&subCurrent[1], &rxbuf[4], 2);	// i5	 		H1-49
				// memcpy(&subCurrent[2], &rxbuf[6], 2);	// i5	SEUV 	H1-51
				// memcpy(&subCurrent[3], &rxbuf[8], 2);	// i3.3	ADCS 	H1-48
				// memcpy(&subCurrent[4], &rxbuf[10], 2);	// i3.3	GPS 	H1-50
				// memcpy(&subCurrent[5], &rxbuf[12], 2);	// i3.3	SEUV 	H1-52
			}
			printf("ADCS 5: \t%d mA\t", subCurrent[0]);
			if (subCurrent[0] > overCurrentThreshold_ADCS_5 ) {
				outRangeCounter_ADCS_5 ++;
				printf("counter = %d\n", outRangeCounter_ADCS_5);
				if (outRangeCounter_ADCS_5 >= 6) {
					generate_Error_Report(3, subCurrent[0]);
					outRangeCounter_ADCS_5 = 0;
				}
			}
			else {
				printf("\n");
				outRangeCounter_ADCS_5 = 0;
			}
			printf("ADCS 33: \t%d mA\t", subCurrent[3]);
			if (subCurrent[3] > overCurrentThreshold_ADCS_33 ) {
				outRangeCounter_ADCS_33 ++;
				printf("counter = %d\n", outRangeCounter_ADCS_33);
				if (outRangeCounter_ADCS_33 >= 6) {
					generate_Error_Report(2, subCurrent[3]);
					outRangeCounter_ADCS_33 = 0;
				}
			}
			else {
				printf("\n");
				outRangeCounter_ADCS_33 = 0;
			}
			printf("GPS: \t\t%d mA\t", subCurrent[4]);
			if (subCurrent[4] > overCurrentThreshold_GPS ) {
				outRangeCounter_GPS ++;
				printf("counter = %d\n", outRangeCounter_GPS);
				if (outRangeCounter_GPS >= 6) {
					power_control(2, OFF);	//power off GPS
					use_GPS_header = 0;
					generate_Error_Report(4, subCurrent[4]);
					outRangeCounter_GPS = 0;
				}
			}
			else {
				printf("\n");
				outRangeCounter_GPS = 0;
			}
			/* ----------------------------------------------------- */
			vTaskDelay(5 * delay_time_based);
#endif
#if temperature_test
			/* ADCS ARM Temperature */
			TS7();
			subTemperature_int[0] = ThermalFrame.T7;
			printf("ADCS ARM Temperature %d degree\t", subTemperature_int[0]);

			/* Operational Temperature Range -10 to +60 */
			if (subTemperature_int[0] > 60 || subTemperature_int[0] < -10) {
				outRangeCounter_temp[3] ++;
				printf("counter = %d\n", outRangeCounter_temp[3]);
				if (outRangeCounter_temp[3] >= 6) {
					generate_Error_Report(13, subTemperature_int[0]);
					outRangeCounter_temp[3] = 0;
				}
			}
			else {
				printf("\n");
				outRangeCounter_temp[3] = 0;
			}
			/* ----------------------------------------------------- */
			vTaskDelay(5 * delay_time_based);
			/* ADCS Rate Sensor & Magnetometer Temperature */
			TS8();
			subTemperature_int[1] = ThermalFrame.T8;
			adcsTemp[0] = subTemperature_int[1] % 256;
			printf("ADCS Rate Temperature %d degree\t", adcsTemp[0]);
			/* Operational Temperature Range -10 to +60 */
			if (adcsTemp[0] > 60 || adcsTemp[0] < -10) {
				outRangeCounter_temp[4] ++;
				printf("counter = %d\n", outRangeCounter_temp[4]);
				if (outRangeCounter_temp[4] >= 6) {
					generate_Error_Report(14, adcsTemp[0]);
					outRangeCounter_temp[4] = 0;
				}
			}
			else {
				printf("\n");
				outRangeCounter_temp[4] = 0;
			}
			adcsTemp[1] = (subTemperature_int[1] >> 8) ;
			printf("ADCS Magnetometer Temperature %d degree\t", adcsTemp[1]);
			/* Operational Temperature Range -10 to +60 */
			if (adcsTemp[1] > 60 || adcsTemp[1] < -10) {
				outRangeCounter_temp[7] ++;
				printf("counter = %d\n", outRangeCounter_temp[7]);
				if (outRangeCounter_temp[7] >= 6) {
					generate_Error_Report(15, adcsTemp[1]);
					outRangeCounter_temp[7] = 0;
				}
			}
			else {
				printf("\n");
				outRangeCounter_temp[7] = 0;
			}
			/* ----------------------------------------------------- */
			vTaskDelay(5 * delay_time_based);
#endif
		}
		vTaskDelay(5 * delay_time_based);
#endif
	}
	/** End of Task, Should Never Reach This */
	vTaskDelete(NULL);
}
