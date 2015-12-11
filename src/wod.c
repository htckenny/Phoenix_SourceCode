#include <time.h>
#include <math.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <dev/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <util/hexdump.h>
#include <util/csp_buffer.h>
#include <util/log.h>
#include <util/delay.h>
#include <io/nanopower2.h>
#include <io/nanomind.h>
#include <csp/csp.h>
#include <csp/csp_endian.h>
#include "parameter.h"
#include "subsystem.h"
#include "Tele_function.h"

#define __max(a,b) \
   ({__typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define __min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#define E_NO_ERR -1

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
			//printf("%d\n",curbit/8+5);
			//printf("%d\n",curbit%8);
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
		wod[0] = t.tv_sec;
		wod[1] = t.tv_sec >> 8;
		wod[2] = t.tv_sec >> 16;
		wod[3] = t.tv_sec >> 24;
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


	uint16_t val[7];
	txdata[0] = com_rx_hk;
	if (i2c_master_transaction(0, com_rx_node, &txdata, 1, &val, com_rx_hk_len, com_delay) == E_NO_ERR) {
		tempComm  = __max(__min(floor(4 * (float)(189.5522-0.0546*val[5] ) + 60), 255), 0);
	} else
		return Error;
	// printf("temp com = %" PRIu16 "\n", val[5]);
	eps_hk_t * chkparam;
	i2c_frame_t * frame = csp_buffer_get(I2C_MTU);
	if (frame == NULL)
		return Error;


	frame->dest = NODE_EPS;
	frame->data[0] = EPS_PORT_HK; // get hk
	frame->data[1] = 0;
	frame->len = 2;
	frame->len_rx = 2 + (uint8_t) sizeof(eps_hk_t);
	frame->retries = 0;

	if ( i2c_send(0, frame, 0) != E_NO_ERR) {
		csp_buffer_free(frame);
		return Error;
	}

	if (i2c_receive(0, &frame, 200) != E_NO_ERR) {

		return Error;
	}

	chkparam = (eps_hk_t *)&frame->data[2];
	eps_hk_unpack(chkparam);
	csp_buffer_free(frame);


	/*SEUV Sun Light Flag Check */
	if ((chkparam->curin[0] + chkparam->curin[1] + chkparam->curin[2]) > 400)
		HK_frame.sun_light_flag = 1;
	else
		HK_frame.sun_light_flag = 0;

	printf("WOD raw data\n");
	printf("vbat = %u\n", chkparam->vbatt);
	printf("ibat = %u\n", chkparam->cursys);
	printf("i3.3 = %u\n", (chkparam->curout[0] + chkparam->curout[1] + chkparam->curout[2]));
	printf("i5.0 = %u\n", (chkparam->curout[3] + chkparam->curout[4] + chkparam->curout[5]));
	printf("EPS temp = %u\n", (chkparam->temp[0] + chkparam->temp[1] + chkparam->temp[2]) / 3);
	printf("BAT temp = %u\n", chkparam->temp[3]);
	printf("COM temp = %f\n", 189.5522-0.0546*val[5]);

	batVoltage = __max(__min(floor(20 * ((float)chkparam->vbatt / 1000) - 60), 255), 0);
	batCurrent = __max(__min(floor(127 * ((float)chkparam->cursys / 1000) + 127), 255), 0);
	bus3v3Current = __max(__min(floor((((float)chkparam->curout[0] + (float)chkparam->curout[1] + (float)chkparam->curout[2]) / 25)), 255), 0);
	bus5v0Current = __max(__min(floor((((float)chkparam->curout[3] + (float)chkparam->curout[4] + (float)chkparam->curout[5]) / 25)), 255), 0);
	tempEps = __max(__min(floor(4 * ((((float)chkparam->temp[0] + (float)chkparam->temp[1] + (float)chkparam->temp[2]) / 3)) + 60), 255), 0);
	tempBat = __max(__min(floor(4 * (((float)chkparam->temp[3])) + 60), 255), 0);

	//printf("tmpCOM=%u\n",tempComm);
	//printf("vbat=%u\n",batVoltage);
	//printf("ibat=%u\n",batCurrent);
	//printf("i3.3=%u\n",bus3v3Current);
	// printf("i5.0=%u\n",bus5v0Current);
	// printf("tmpEPS=%u\n",tempEps);
	//  printf("temBAT=%u\n",tempBat);


	if (mode)	calbit(fnum);		// wodd.dataSet[fnum].mode = 0 or 1;
	calmulbit(fnum, 1, batVoltage);	// wodd.dataSet[fnum].bat_voltage = batVoltage;
	calmulbit(fnum, 2, batCurrent);	// wodd.dataSet[fnum].bat_current=0;
	calmulbit(fnum, 3, bus3v3Current); // wodd.dataSet[fnum].bus3v3_current=0;
	calmulbit(fnum, 4, bus5v0Current); // wodd.dataSet[fnum].bus5v0_current=0;
	calmulbit(fnum, 5, tempComm);		// wodd.dataSet[fnum].comm_temp= tempComm;
	calmulbit(fnum, 6, tempEps);		// wodd.dataSet[fnum].eps_temp = tempEps;
	calmulbit(fnum, 7, tempBat);		// wodd.dataSet[fnum].battery_temp = tempBat;
	//for beacon


	/*beacon message UPDATE */
	beacon_frame.mode = mode;
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
void beacon_task(void * pvParameters) {

	int period = 30000;		//Normally the beacon period is 30 sec
	vTaskDelay(60000);

	while (1) {

		if (parameters.first_flight == 1){
			period = 10000;		// when early orbit, beacon period = 10 sec
		}
		else if (parameters.beacon_period > 0){
			period = parameters.beacon_period * 1000;
		}
		printf("-- Send Beacon with %d--\n", parameters.beacon_period);
		SendPacketWithCCSDS_AX25(&beacon_frame.mode, 8, obc_apid, 0, 0);
		vTaskDelay(period);
	}
}

void vTaskwod(void * pvParameters) {
	printf("Active WOD Task\n");
	vTaskDelay(60000);
	xTaskCreate(beacon_task, (const signed char *) "beacon", 1024 * 4, NULL, 2, NULL);

	while (1) {


		//get the 32 dataSet . dtime must be modify to 1 minute, i ->32
		for (int i = 1; i <= 32; i++)
		{
			if (getWodFrame(i) == No_Error)
				printf("--------WOD Frame %d get--------\n", i);
			else {
				printf("--------WOD Frame %d get FAIL!--------\n", i);
				i = 0;
			}
			vTaskDelay(60000);
		}

		if (wod_write(&wod[0]) != No_Error) {
			printf("write WOD into FS error");
		} else
			printf("write a WOD into FS ");

		for (int i = 0; i < 232; i++) {
			wod[i] = 0;
		}
	}
}