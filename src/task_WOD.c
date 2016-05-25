/*
 * task_WOD.c
 *
 *  Created on: 	2014/05/05
 *  Last updated: 	2016/01/28
 *  Author: 		Kenny Huang
 */

#include <time.h>
#include <math.h>

#include <fat_sd/ff.h>
#include <dev/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <util/hexdump.h>
#include <util/csp_buffer.h>
#include <util/log.h>
#include <util/delay.h>
#include <io/nanopower2.h>
#include <nanomind.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>

#include "fs.h"
#include "parameter.h"
#include "subsystem.h"
#include "tele_function.h"

#define __max(a,b) \
   ({__typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define __min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

static timestamp_t t;
uint8_t wod[232];
uint8_t beaconWod [8];

/* calculate the position of the memory and then store in. */
void sa(int byte_no, int bit_no) {
	int num = 0;
	switch (bit_no)
	{
	case 0:
		num = 128;		break;
	case 1:
		num = 64;		break;
	case 2:
		num = 32;		break;
	case 3:
		num = 16;		break;
	case 4:
		num = 8;		break;
	case 5:
		num = 4;		break;
	case 6:
		num = 2;		break;
	case 7:
		num = 1;		break;
	default:
		num = 0;
		printf("error, do not have this bit number");
		break;
	}
	wod[byte_no - 1] = wod[byte_no - 1] + num;
}

/* normally for mode(first bit of 57bits) */
void calbit(int dataSet) {
	int curbit;
	//32 data set
	if (dataSet > 32)
		printf("error, data set no more than 32");

	curbit = 57 * (dataSet - 1);
	sa(curbit / 8 + 5, curbit % 8); //+1(next)+4(32bit time)

}
/* for the wod choose which dataset and data and the value */
void calmulbit(int dataSet, int data, int value) {
	//data=1:batvol ,data=2:batcurr.............data=7:tempbat
	int curbit;
	int i , j;
	int rec[8];
	//32 data set
	if (dataSet > 32)
		printf("error, data set no more than 32");
	if (value > 255)
		printf("value out of range!!\n");
	for (i = 0; i < 8; i++) {
		rec[i] = 0;
	}
	for (i = 0; i < 8; i++) {
		if (value == 0)
			break;
		if (value >= (1 << (7 - i))) {
			rec[i] = 1;
			value = value - (1 << (7 - i));
		}
	}
	for (j = 0; j < 8; j++) {
		if (rec[j] == 1) {
			curbit = 57 * (dataSet - 1) + 8 * (data - 1) + j + 1;
			sa(curbit / 8 + 5, curbit % 8); //+1(next)+4(32bit time)
		}
	}
}
int getWodFrame(int fnum) {

	if (fnum < 1)
		return Error;

	if (fnum == 1) {
		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		t.tv_sec -= 946684800;
		memcpy(&wod[0], &t.tv_sec, 4);
	}
	unsigned int mode = 1;
	if (HK_frame.mode_status_flag == 0)
		mode = 0;

	uint8_t batVoltage = 0;
	uint8_t batCurrent = 0;
	uint8_t bus3v3Current = 0;
	uint8_t bus5v0Current = 0;
	uint8_t tempComm = 0 ;
	uint8_t tempEps = 0 ;
	uint8_t tempBat = 0 ;
	uint8_t txdata[19] = {};

	uint16_t val[4];

	uint16_t EPS_HK[12];
	uint8_t txbuf[2];
	uint8_t rxbuf[64 + 2];


	txdata[0] = 0x25;
	if (i2c_master_transaction_2(0, com_tx_node, &txdata, 1, &val, 8, com_delay) == E_NO_ERR) {
		tempComm  = __max(__min(floor(4 * (float)(189.5522 - 0.0546 * val[1] ) + 60), 255), 0);
	} else
		return Error;

	txbuf[0] = 0x08;

	if (i2c_master_transaction_2(0, eps_node, &txbuf, 1, &rxbuf, 43 + 2, eps_delay) == E_NO_ERR) {
		memcpy(&EPS_HK[0], &rxbuf[10], 2);	//Vbatt
		memcpy(&EPS_HK[1], &rxbuf[12], 2);	//ibat
		memcpy(&EPS_HK[8], &rxbuf[14], 2);	//EPS temp
		memcpy(&EPS_HK[9], &rxbuf[16], 2);	//EPS temp
		memcpy(&EPS_HK[10], &rxbuf[18], 2);	//EPS temp
		memcpy(&EPS_HK[11], &rxbuf[20], 2);	//batt temp
	}
	txbuf[0] = 0x08;
	txbuf[1] = 0x02;
	if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 64 + 2, eps_delay) == E_NO_ERR) {
		memcpy(&EPS_HK[2], &rxbuf[8], 2);	//i3.3
		memcpy(&EPS_HK[3], &rxbuf[10], 2);	//i3.3
		memcpy(&EPS_HK[4], &rxbuf[12], 2);	//i3.3
		memcpy(&EPS_HK[5], &rxbuf[2], 2);	//i5
		memcpy(&EPS_HK[6], &rxbuf[4], 2);	//i5
		memcpy(&EPS_HK[7], &rxbuf[6], 2);	//i5
	}
	for (int i = 0 ; i < 12 ; i++) {
		EPS_HK[i] = csp_hton16(EPS_HK[i]);
	}

	/*SEUV Sun Light Flag Check */
	// if ((chkparam->curin[0] + chkparam->curin[1] + chkparam->curin[2]) > 400)
	// 	HK_frame.sun_light_flag = 1;
	// else
	// 	HK_frame.sun_light_flag = 0;

	printf("WOD raw data\n");
	printf("vbat = %u\n", EPS_HK[0]);
	printf("ibat = %u\n", EPS_HK[1]);
	printf("i3.3 = %u\n", (EPS_HK[2] + EPS_HK[3] + EPS_HK[4]));
	printf("i5.0 = %u\n", (EPS_HK[5] + EPS_HK[6] + EPS_HK[7]));
	printf("EPS temp = %u\n", (EPS_HK[8] + EPS_HK[9] + EPS_HK[10]) / 3);
	printf("BAT temp = %u\n", EPS_HK[11]);
	printf("COM temp = %.2f\n", 189.5522 - 0.0546 * val[1]);

	batVoltage = __max(__min(floor(20 * ((float)EPS_HK[0] / 1000) - 60), 255), 0);
	batCurrent = __max(__min(floor(127 * ((float)EPS_HK[1] / 1000) + 127), 255), 0);
	bus3v3Current = __max(__min(floor((((float)EPS_HK[2] + (float)EPS_HK[3] + (float)EPS_HK[4]) / 25)), 255), 0);
	bus5v0Current = __max(__min(floor((((float)EPS_HK[5] + (float)EPS_HK[6] + (float)EPS_HK[7]) / 25)), 255), 0);
	tempEps = __max(__min(floor(4 * ((((float)EPS_HK[8] + (float)EPS_HK[9] + (float)EPS_HK[10]) / 3)) + 60), 255), 0);
	tempBat = __max(__min(floor(4 * (((float)EPS_HK[11])) + 60), 255), 0);

	if (mode)	calbit(fnum);
	calmulbit(fnum, 1, batVoltage);
	calmulbit(fnum, 2, batCurrent);
	calmulbit(fnum, 3, bus3v3Current);
	calmulbit(fnum, 4, bus5v0Current);
	calmulbit(fnum, 5, tempComm);
	calmulbit(fnum, 6, tempEps);
	calmulbit(fnum, 7, tempBat);

	/* Beacon Message UPDATE */
	beacon_frame.batVoltage = batVoltage;
	beacon_frame.batCurrent = batCurrent;
	beacon_frame.bus3v3Current = bus3v3Current;
	beacon_frame.bus5v0Current = bus5v0Current;
	beacon_frame.tempComm = tempComm;
	beacon_frame.tempEps = tempEps;
	beacon_frame.tempBat = tempBat;

	return No_Error;
}

/**
 * This task is used for generate beacon signal, and tranmit to the GS
 * @param pvParameters [description]
 */
void beacon_Task(void * pvParameters) {

	uint8_t beacon_withSID[9];
	vTaskDelay(30 * delay_time_based);
	portTickType xLastWakeTime;
	portTickType xFrequency;

	/* Set the delay time during one sampling operation*/
	xLastWakeTime = xTaskGetTickCount();
	while (1) {

		if (parameters.first_flight == 1) {
			xFrequency = 10 * delay_time_based;		// when early orbit, beacon period = 10 sec
		}
		else if (parameters.beacon_period > 0) {
			xFrequency = parameters.beacon_period * delay_time_based;
		}
		else if (HK_frame.mode_status_flag == 0) {	// Safe mode
			xFrequency = 20 * delay_time_based;
		}
		else {
			xFrequency = 30 * delay_time_based;
		}
		/* Beacon Message UPDATE */
		if (HK_frame.mode_status_flag == 2 && parameters.first_flight == 1)
			HK_frame.mode_status_flag = 4;
		beacon_frame.mode = HK_frame.mode_status_flag;

		printf("-- Send Beacon with %d--\n", (int)xFrequency / 100);
		beacon_withSID[0] = 254;
		memcpy(&beacon_withSID[1], &beacon_frame.mode, 8);
		SendPacketWithCCSDS_AX25(&beacon_withSID[0], 9, obc_apid, 3, 25);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void WOD_Task(void * pvParameters) {
	printf("Active WOD Task\n");
	xTaskCreate(beacon_Task, (const signed char *) "beacon", 1024 * 4, NULL, 2, &beacon_task);

	while (1) {

		for (int i = 1; i <= 32; i++)
		{
			if (getWodFrame(i) == No_Error)
				printf("--------WOD Frame %d get--------\n", i);
			else {
				printf("--------WOD Frame %d get FAIL!--------\n", i);
				// i = 0;
			}
			vTaskDelay(60 * delay_time_based);
		}
		if (HK_frame.mode_status_flag != 5) {
			if (parameters.crippled_Mode == 0)
				wod_write_dup(&wod[0]);
			else
				wod_write_crippled(&wod[0]);
		}

		for (int i = 0; i < 232; i++) {
			wod[i] = 0;
		}
	}
}