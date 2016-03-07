/*
 * EOP_Task.c
 *
 *  Created on: 	2015/08/21
 *  Last update:	2016/01/26
 *      Author: Kenny Huang
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <time.h>
#include <dev/cpu.h>
#include <dev/i2c.h>
#include <util/hexdump.h>
#include <util/timestamp.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>
#include <nanomind.h>

#include "fs.h"
#include "parameter.h"
#include "subsystem.h"

void EOP_Task(void * pvParameters) {
	uint8_t uchar_eop[eop_length];
	uint8_t rxbuf[48];
	uint8_t txbuf[2];
	uint16_t vbatt;
	uint16_t cursun;
	uint16_t cursys;
	unsigned int p_sun;
	unsigned int p_sys;
	int power_counter = 0;
	for (int i = 0 ; i < eop_length ; i++) {
		uchar_eop[i] = 0;
	}
	vTaskDelay(20 * delay_time_based);
	while (1) {
		printf("collecting EOP packet\n");
		txbuf[0] = 0x88;	//ID = 136 Current ADCS state
		/* Get Time, Attitude, Position, Velocity information from ADCS */
		if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &rxbuf, 48, adcs_delay) == E_NO_ERR) {
			printf("Get Time, Attitude, Position from ADCS\n");
			memcpy(&uchar_eop[4], &rxbuf[18], 24);
		}
		else {
			printf("(EOP)Error, cannot communicate with ADCS\n");
		}
		power_counter ++;
		if (power_counter == 6) {
			/* eps_hk_vi_t */
			txbuf[0] = 8;
			txbuf[1] = 1;
			if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 22, adcs_delay) == E_NO_ERR) {
				memcpy(&vbatt, &rxbuf[10], 2);
				memcpy(&cursun, &rxbuf[18], 2);
				memcpy(&cursys, &rxbuf[20], 2);
				p_sun = (float) cursun * (float) vbatt / 1000.0;
				p_sys = (float) cursys * (float) vbatt / 1000.0;
				memcpy(&uchar_eop[28], &p_sun, 2);
				memcpy(&uchar_eop[30], &p_sys, 2);
			}
		}
		eop_write_dup(uchar_eop);
		hex_dump(&uchar_eop, eop_length);
		vTaskDelay(10 * delay_time_based);
	}
	/** End of init */
	vTaskDelete(NULL);
}
