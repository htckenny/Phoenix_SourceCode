#include <dev/i2c.h>
#include <util/timestamp.h>
#include <util/hexdump.h>
#include <freertos/FreeRTOS.h>
#include <string.h>
#include <nanomind.h>
#include <csp/csp_endian.h>
#include <time.h>
#include <fat_sd/ff.h>
#include <inttypes.h>
/* Self defined header file*/
#include "subsystem.h"
#include "parameter.h"
#include "fs.h"

extern struct tm wtime_to_date(wtime wt);

void GPS_task(void* pvParameters)
{
	uint8_t txBuffer[2];
	uint8_t rxBuffer[6];
	timestamp_t t;
	time_t tt;
	wtime wt;
	struct tm tmbuf;
	time_t t_of_day;
	uint16_t gps_week;
	uint32_t gps_second;
	portTickType xLastWakeTime;
	portTickType xFrequency;

	vTaskDelay(5 * delay_time_based);
	/* Power ON GPS */
	power_control(2, ON);
	uint16_t time_counter = 1800;	/* 30 minutes */
	uint8_t GPS_information[30] = {0};

	xLastWakeTime = xTaskGetTickCount();
	xFrequency = 30 * delay_time_based;
	GPS_delete();
	while (1) {
		txBuffer[0] = 168;
		if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer, 6, adcs_delay) == E_NO_ERR) {
			memcpy(&GPS_information[0], &rxBuffer[0], 6);
			/* Solution computed */
			if (rxBuffer[0] == 0) {
				printf("Solution Computed !\n");
				vTaskDelay(2 * delay_time_based);
				txBuffer[0] = 169; /* Raw GPS Time */
				if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer[0], 6, adcs_delay) == E_NO_ERR) {

					/* Get week number and elapsed time */
					memcpy(&gps_week, &rxBuffer[0], 2);
					memcpy(&gps_second, &rxBuffer[2], 4);

					wt.week = gps_week;
					wt.sec = gps_second / 1000;

					if (wt.week != 0 || wt.sec != 0) {
						/* Construct struct time to epoch seconds */
						tmbuf = wtime_to_date(wt);
						t_of_day = mktime(&tmbuf);
						ctime(&t_of_day);
						
						t.tv_nsec = 0;
						t.tv_sec = t_of_day ;
						obc_timesync(&t, 1000);
						tt = t.tv_sec;
						lastCommandTime = t.tv_sec;
						printf("OBC time Sync by GPS to : %s\r\n", ctime(&tt));
					}
					memcpy(&GPS_information[6], &rxBuffer[0], 6);
				}
				vTaskDelay(1 * delay_time_based);
				txBuffer[0] = 170; /* Raw GPS X */
				if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer[0], 6, adcs_delay) == E_NO_ERR) {
					memcpy(&GPS_information[12], &rxBuffer[0], 6);
				}
				vTaskDelay(1 * delay_time_based);
				txBuffer[0] = 171; /* Raw GPS Y */
				if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer[0], 6, adcs_delay) == E_NO_ERR) {
					memcpy(&GPS_information[18], &rxBuffer[0], 6);
				}
				vTaskDelay(1 * delay_time_based);
				txBuffer[0] = 172; /* Raw GPS Z */
				if (i2c_master_transaction_2(0, adcs_node, &txBuffer, 1, &rxBuffer[0], 6, adcs_delay) == E_NO_ERR) {
					memcpy(&GPS_information[24], &rxBuffer[0], 6);
				}
			}
			else
				printf("Insufficient observations\n");
			GPS_write(GPS_information);
			hex_dump(&GPS_information[0], 30);
		}
		time_counter -= 30;

		printf("time counter = %d\n", time_counter);
		if (time_counter == 0) {
			break;
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency );
	}
	power_control(2, OFF);
	gps_task = NULL;
	vTaskDelete(NULL);
}
