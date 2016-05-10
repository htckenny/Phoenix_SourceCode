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
	uint8_t counter = 0;
	uint8_t uchar_eop[eop_length];
	uint8_t rxbuf[48];
	uint8_t txbuf[2];
	uint16_t vbatt;
	uint16_t cursun;
	uint16_t cursys;
	uint16_t p_sun;
	uint16_t p_sys;
	timestamp_t t;

	for (int i = 0 ; i < eop_length ; i++) {
		uchar_eop[i] = 0;
	}
	vTaskDelay(20 * delay_time_based);
	while (1) {
		printf("collecting EOP packet\n");
		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		t.tv_sec -= 946684800;
		memcpy(&uchar_eop[0], &t.tv_sec, 4);

		txbuf[0] = 0x88;	//ID = 136 Current ADCS state
		/* Get Time, Attitude, Position, Velocity information from ADCS */
		if (i2c_master_transaction_2(0, adcs_node, &txbuf, 1, &rxbuf, 48, adcs_delay) == E_NO_ERR) {
			printf("Get Time, Attitude, Position from ADCS\n");
			memcpy(&uchar_eop[4], &rxbuf[18], 24);
		}
		else {
			printf("(EOP)Error, cannot communicate with ADCS\n");
		}
		counter ++;

		if ((counter % 2) == 1) {
			/* eps_hk_vi_t */
			txbuf[0] = 8;
			txbuf[1] = 1;
			if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 22, eps_delay) == E_NO_ERR) {
				memcpy(&vbatt, &rxbuf[8], 2);
				memcpy(&cursun, &rxbuf[16], 2);
				memcpy(&cursys, &rxbuf[18], 2);
				vbatt = csp_ntoh16(vbatt);
				cursun = csp_ntoh16(cursun);
				cursys = csp_ntoh16(cursys);
				p_sun = (float) cursun * (float) vbatt / 1000.0;
				p_sys = (float) cursys * (float) vbatt / 1000.0;
				printf("p_sun = %d\tp_sys = %d\n", p_sun, p_sys);
				memcpy(&uchar_eop[28], &p_sun, 2);
				memcpy(&uchar_eop[30], &p_sys, 2);
			}
			counter = 1;
		}
		if (parameters.crippled_Mode == 0) {
			eop_write(uchar_eop, 0);
			eop_write(uchar_eop, 1);
		}
		else
			eop_write_crippled(uchar_eop);

		hex_dump(&uchar_eop[0], eop_length);
		vTaskDelay(30 * delay_time_based);
	}
	/** End of init */
	vTaskDelete(NULL);
}
