/*
* Battery CCV.c
*
*  Created on: 	2016/02/02
*  Last update:	2016/02/02
*      Author: Kenny Huang, Eddie Yeh
*/
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <util/hexdump.h>
#include <util/timestamp.h>
#include <csp/csp_endian.h>
#include <dev/i2c.h>
#include <nanomind.h>
#include <fat_sd/ff.h>
#include <vfs/vfs.h>
#include "parameter.h"
#include "tele_function.h"
#include "subsystem.h"
#include "fs.h"


FATFS fs[2];
FRESULT res;
FIL file;
UINT br, bw;
FILINFO *fno;
int cleanfile = 0;
char fileName1[30];
extern int eps_write(uint8_t *frameCont, int choose);
void EPS_Task(void * pvParameters)
{
	uint8_t uchar_eps[12]; // neccessary
	uint8_t rxbuf[131]; //total housekeeping data
	uint8_t txbuf[2];
	txbuf[0] = 0x08; 			//ID = 8 EPS HK port
	txbuf[1] = 0x00;
	int i = 0, period = 0, rate[180], j = 0, ccv = 0;
	double delay = 0 , time = 0;
	portTickType start_time = 0, stop_time = 0;
	char choose;
	uint16_t vboost3 = 0, vbatt = 0, current_in = 0, current_out = 0, btemp, cursun = 0;
	char time_unit[4];
	unsigned int ms = 0;
	// clear all memery space
	for (i = 0 ; i < 12 ; i++)
		uchar_eps[i] = 0;

	printf("a. Charge cycle \n"); //ASCII a=97
	printf("b. Record the closed circuit voltage \n");
	printf("c. Discharge cycle \n");
	printf("d. Overcharge cycle \n");
	printf("e. Discharge after overcharge cycle \n");
	printf("f. Overdischarge cycle \n");
	printf("g. Charge after overdischarge cycle or external battery short \n");
	printf("h. External battery short \n");
	printf("Point which test ");
	scanf("%c", &choose);

	switch (choose)
	{
	case 'a':
		delay = 60 * delay_time_based;
		strcpy(time_unit, "min");
		break;

	case 'b' :
		delay = 1 * delay_time_based;
		ccv = 30;
		strcpy(time_unit, "sec");
		break;

	case 'g' :
		delay = 1 * delay_time_based;
		strcpy(time_unit, "sec");
		break;

	case 'h' :
		strcpy(time_unit, "sec");
		break;

	default:
		delay = 1 * delay_time_based;
		strcpy(time_unit, "sec");
		if (choose == 99)
		{
			delay = 60 * delay_time_based;
			strcpy(time_unit, "min");
		}

	}

	for (i = 0; i < 180; i++)
		rate[i] = 0;

	i = 0;

	while (1)
	{
		ms = (stop_time - start_time) * (1000 / configTICK_RATE_HZ);
		printf("\n %5.3f %s \n", (double)ms , time_unit);
		start_time = xTaskGetTickCount();
		if (choose == 104 && time > 10) // change the external battery short sample time
			delay = 10;
		if (choose == 104 && time > 1800)
			break;
		i++;
		if (i2c_master_transaction_2(0, eps_node, &txbuf, 2, &rxbuf, 133, eps_delay) == E_NO_ERR)
		{
			memcpy(&uchar_eps[0], &rxbuf[6], 2); // vboost
			memcpy(&uchar_eps[2], &rxbuf[8], 2); //vbatt
			memcpy(&uchar_eps[4], &rxbuf[14], 2); //current in
			memcpy(&uchar_eps[6], &rxbuf[16], 2); //current from boost converters
			memcpy(&uchar_eps[8], &rxbuf[18], 2); //current out
			memcpy(&uchar_eps[10], &rxbuf[122], 2); // battery temperature

			//hex_dump(uchar_eps, 12);		//printf data

			// arrange the right format
			vboost3 = uchar_eps[0];
			vboost3 = (vboost3 << 8) + uchar_eps[1];

			vbatt = uchar_eps[2];
			vbatt = (vbatt << 8) + uchar_eps[3];

			current_in = uchar_eps[4];
			current_in = (current_in << 8) + uchar_eps[5];

			cursun = uchar_eps[6];
			cursun = (cursun << 8) + uchar_eps[7];

			current_out = uchar_eps[8];
			current_out = (current_out << 8) + uchar_eps[9];

			btemp = uchar_eps[10];
			btemp = (btemp << 8) + uchar_eps[11];

			printf("Input voltage: %4" PRIu16 "\n", vboost3);
			printf("Battery voltage: %4" PRIu16 "\n", vbatt);
			printf("Input current: %4" PRIu16 "\n", current_in);
			printf("Input battery current: %4" PRIu16 "\n", cursun);
			printf("Output current: %4" PRIu16 "\n", current_out);
			printf("Battery temperature: %4" PRIu16 "\n", btemp);
		}
		else
			printf("Error, cannot communicate with EPS\n");

		eps_write(uchar_eps, choose);			//write into SD card

		if (i == ccv && choose == 98) // Record ccv(closed circuit voltage)
			break;

		if ((choose == 97 || choose == 103) && cursun < 100) // wherether does the circumstance of the lower current maintent over 3 mins
			period++;

		if ((choose == 97 || choose == 103) && cursun >= 100)
			period = 0;

		if (choose > 98 && choose < 103) //overcharge or discharge ¡Boverdischarge judge mechamism
		{
			if (abs((vbatt - rate[j])) >= 0.005 * rate[j] && (vbatt - rate[j]) != 0) //wherether does the circumstance change
				period = 0;
			else
				period++;

			printf("vbatt:%d rate[%d]:%d period:%d\n", vbatt, j, rate[j], period);
			rate[j] = vbatt;
			j++;

			if (j == 180 && choose > 99)
				j = 0;
			if (j == 3 && choose == 99) //wherether dose it reach the discharge cycle threshold
				j = 0;
		}

		if (period > 3 && choose == 97) // trigger the warning signal for charge
		{
			delay = 10 * delay_time_based;
			break;
		}

		if (period > 180 && choose > 99) // trigger the warning signal for overcharge
		{
			delay = 10 * delay_time_based;
			break;
		}
		vTaskDelay(delay);
		stop_time = xTaskGetTickCount();
	}

	for (i = 0; i < 60; i++) //warning signal
	{
		printf(" break time : %4d sec \n", i * 10);
		vTaskDelay(delay);
	}
	vTaskDelete(NULL);
}

int eps_write(uint8_t *frameCont, int choose)
{
	if (cleanfile == 0)
	{
		if (choose == 97)
		{
			f_unlink("Battery/battery_a.bin");
			strcpy(fileName1, "Battery/battery_a.bin");
		}
		if (choose == 98)
		{
			f_unlink("Battery/battery_b.bin");
			strcpy(fileName1, "Battery/battery_b.bin");
		}
		if (choose == 99)
		{
			f_unlink("Battery/battery_c.bin");
			strcpy(fileName1, "Battery/battery_c.bin");
		}
		if (choose == 100)
		{
			f_unlink("Battery/battery_d.bin");
			strcpy(fileName1, "Battery/battery_d.bin");
		}
		if (choose == 101)
		{
			f_unlink("Battery/battery_e.bin");
			strcpy(fileName1, "Battery/battery_e.bin");
		}
		if (choose == 102)
		{
			f_unlink("Battery/battery_f.bin");
			strcpy(fileName1, "Battery/battery_f.bin");
		}
		if (choose == 103)
		{
			f_unlink("Battery/battery_g.bin");
			strcpy(fileName1, "Battery/battery_g.bin");
		}
		if (choose == 104)
		{
			f_unlink("Battery/battery_h.bin");
			strcpy(fileName1, "Battery/battery_h.bin");
		}
	}
	cleanfile = 1;

	//write into file
	res = f_open(&file, fileName1, FA_OPEN_ALWAYS | FA_WRITE );
	if (res != FR_OK)
	{
		printf("open fail .. \n");
		f_close(&file);
		return Error;
	}
	f_lseek(&file, file.fsize);
	res = f_write(&file, &frameCont[0], 2, &bw);
	if (res != FR_OK)
	{
		printf("write fail .. \n");
		f_close(&file);
		return Error;
	}

	res = f_write(&file, &frameCont[2], 2, &bw);
	res = f_write(&file, &frameCont[4], 2, &bw);
	res = f_write(&file, &frameCont[6], 2, &bw);
	res = f_write(&file, &frameCont[8], 2, &bw);
	res = f_write(&file, &frameCont[10], 2, &bw);

	f_close(&file);
	return No_Error;
}