/*
 * EOP_Task.c
 *
 *  Created on: 	2015/08/21
 *  Last update:	2015/11/09
 *      Author: Kenny Huang
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <dev/cpu.h>
#include <dev/i2c.h>
#include <util/hexdump.h>
#include <util/timestamp.h>
#include <io/nanomind.h>
#include <csp/csp.h>

#include "fs.h"
#include "parameter.h"
#include "subsystem.h"

void EOP_Task(void * pvParameters) {
	uint8_t uchar_eop[28];
	uint8_t rxbuf[48];
	uint8_t txbuf = 0x88; 			//ID = 136 Current ADCS state
	timestamp_t t;
	vTaskDelay(1000);
	while (1) {

		printf("collecting EOP packet\n");
		/* get Time LSB */
		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		memcpy(&uchar_eop[0], &t.tv_sec, 4);

		/* Get Time, Attitude, Position, Velocity information from ADCS */
		if (i2c_master_transaction(0, adcs_node, &txbuf, 1, &rxbuf, 48, adcs_delay) == E_NO_ERR) {
			// printf("Get Time, Attitude, Position from ADCS");
			memcpy(&uchar_eop[4], &rxbuf[18], 24);
			hex_dump(uchar_eop, 28);	/* TODO: Delete this when flight */
			eop_write(uchar_eop);
		}
		else {
			printf("Error, cannot communicate with ADCS\n");
		}
		vTaskDelay(10000);
	}
	/** End of init */
	vTaskDelete(NULL);
}