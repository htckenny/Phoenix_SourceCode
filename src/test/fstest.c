#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include <util/hexdump.h>
#include <util/timestamp.h>
#include <nanomind.h>
#include <dev/i2c.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>

#include "fs.h"
#include "parameter.h"
#include "subsystem.h"

#define INMSWRITEDATA	0
#define SDCARDTEST		0
#define TEST_FILENAME	0
#define TEST_i2c_tx_len	0
#define Test_Delay		0
#define format_SD		0
#define Test_downlink		0
#define Test_downlink_2		1
void vTaskfstest(void * pvParameters) {

#if format_SD
	FATFS fs;
	BYTE label;
	f_mount(parameters.SD_partition_flag, NULL);
	f_mount(parameters.SD_partition_flag, &fs);

	label = parameters.SD_partition_flag;
	if (parameters.SD_partition_flag == 0) {
		f_mkfs(label, 0, 0);
		f_mkdir("0:/INMS_DATA");
		f_mkdir("0:/EOP_DATA");
		f_mkdir("0:/WOD_DATA");
		f_mkdir("0:/HK_DATA");
		f_mkdir("0:/SEUV_DATA");
	}
	else if (parameters.SD_partition_flag == 1) {
		f_mkfs(label, 0, 0);
		f_mkdir("1:/INMS_DATA");
		f_mkdir("1:/EOP_DATA");
		f_mkdir("1:/WOD_DATA");
		f_mkdir("1:/HK_DATA");
		f_mkdir("1:/SEUV_DATA");
	}

	while (1) {
		vTaskDelay(1000);
	}
#endif

#if TEST_i2c_tx_len
	int test_number = 256;
	uint8_t txbuf[test_number];
	uint8_t rxbuf[10];
	int result;
	for (int i = 0 ; i < test_number ; i++) {
		txbuf[i] = i;
	}
	vTaskDelay(3000);
	while (1) {
		if ((result = i2c_master_transaction(0, adcs_node, &txbuf, test_number, 0, 0, adcs_delay)) == E_NO_ERR) {
			if (i2c_master_transaction(0, adcs_node, 0, 0, &rxbuf, 10, adcs_delay) == E_NO_ERR) {
				printf("write %d bytes\n", test_number);
			}
			hex_dump(rxbuf, 10);
		}
		else {
			printf("error type = %d\n", result);
		}
		vTaskDelay(3000);
	}
#endif

#if TEST_FILENAME
	while (1) {
		for (int i = 0 ; i < 2; i++) {
			printf("%s\n", fileName_WOD[i]);
			printf("%s\n", fileName_EOP[i]);
			printf("%s\n", fileName_INMS[i]);
			printf("%s\n", fileName_SEUV[i]);
			printf("%s\n", fileName_HK[i]);
		}
		vTaskDelay(5000);

	}

#endif

#if INMSWRITEDATA
	// FATFS fs;
	vTaskDelay(3 * delay_time_based);
	printf("start FS test task\n");

	// f_mount(0, &fs);
	// struct tm  ts;
	// char buf[80];
	// timestamp_t t;
	// t.tv_sec = 0;
	// t.tv_nsec = 0;
	// obc_timesync(&t, 6000);
	// time_t tt = t.tv_sec;
	uint8_t errPacketTotal [196];
	for (int i = 0 ; i < 196 ; i++) {
		errPacketTotal[i] = i ;
	}
	while (1) {
		// t.tv_sec = 0;
		// t.tv_nsec = 0;
		// obc_timesync(&t, 6000);
		// time_t tt = t.tv_sec;
		// time(&tt);
		// ts = *localtime(&tt);
		// strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);
		// vTaskDelay(0.5 * delay_time_based);
		// memcpy(&errPacketTotal[0], &t.tv_sec, 4);
		hex_dump(&errPacketTotal, 196);
		inms_data_write_dup(errPacketTotal);
		vTaskDelay(5 * delay_time_based);
	}

#endif

#if SDCARDTEST
	FATFS fs[2];
	FRESULT res;
	FIL file, file2;
	UINT br, bw;
	vTaskDelay(5000);
	char testData[] = {0x9D, 0x00, 0x1C, 0x96, 0x8D, 0xCE, 0x7E, 0x99, 0x8D, 0xDF};


	f_mount(0, &fs[0]);
	if (res != FR_OK)
		printf("\r\n f_mount() 0 fail .. \r\n");
	else
		printf("\r\n f_mount() 0 success .. \r\n");

	f_mount(1, &fs[1]);
	if (res != FR_OK)
		printf("\r\n f_mount() 1 fail .. \r\n");
	else
		printf("\r\n f_mount() 1 success .. \r\n");



	while (1) {

		res = f_open(&file, "0:/test_0.bin", FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		if (res != FR_OK)
			printf("\r\n f_open() 0 fail .. \r\n");
		else
			printf("\r\n f_open() 0 success .. \r\n");


		res = f_open(&file2, "1:/test_1.bin", FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		if (res != FR_OK)
			printf("\r\n f_open() 1 fail .. \r\n");
		else
			printf("\r\n f_open() 1 success .. \r\n");



		//將pointer指向文件最後面
		f_lseek(&file, file.fsize);

		res = f_write(&file, testData, 10, &bw);

		// hex_dump(frameCont, inms_data_length);
		if (res != FR_OK) {
			printf("\r\n sd_write() 0 fail .. \r\n");
		}
		else {
			printf("\r\n sd_write() 0 success .. \r\n");

		}
		f_close(&file);


		vTaskDelay(5000);
		res = f_write(&file2, testData, 10, &bw);
		if (res != FR_OK) {
			printf("\r\n sd_write() 1 fail .. \r\n");
		}
		else {
			printf("\r\n sd_write() 1 success .. \r\n");
		}
		f_close(&file2);

		vTaskDelay(5000);
	}

	f_mount(0, NULL);
	f_mount(1, NULL);

#endif

#if Test_Delay
	portTickType xLastWakeTime;
	portTickType xFrequency = 1000;
	xFrequency = 1 * delay_time_based;
	int delay = 1;
	while (1) {
		xLastWakeTime = xTaskGetTickCount();

		printf("Test delay with %d s\n", delay);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		// vTaskDelay(delay * 1000);
	}
#endif

#if Test_downlink
	vTaskDelay(3 * delay_time_based);
	printf("start FS test task\n");

	uint8_t errPacketTotal [196];
	uint16_t serial_number = 0;
	while (1) {
		memcpy(&errPacketTotal[0], &serial_number, 2);
		hex_dump(&errPacketTotal, 196);
		inms_data_write(errPacketTotal, 0);
		serial_number ++;
		vTaskDelay(1 * delay_time_based);
	}
#endif

#if Test_downlink_2
	vTaskDelay(3 * delay_time_based);
	char fileName[45];
	char numberc[5];
	int number = 0;

	uint8_t errPacketTotal [196];
	uint16_t serial_number = 0;
	FRESULT res;
	FIL file;
	UINT  bw;

while (1) {
	memcpy(&errPacketTotal[0], &serial_number, 2);
	hex_dump(&errPacketTotal, 196);

	sprintf(numberc, "%d", number);
	strcpy(fileName, "0:/EOP_DATA/");
	strcat(fileName, numberc);
	strcat(fileName, "TESTDATA_TWO1");
	strcat(fileName, ".dat");
	printf("%s\n", fileName);


	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\nf_open() fail .. \r\n");
	}
	//將pointer指向文件最後面
	f_lseek(&file, file.fsize);

	res = f_write(&file, errPacketTotal, inms_data_length, &bw);

	if (res != FR_OK) {
		printf("\rinms_write() fail .. \r\n");
		f_close(&file);
	}
	else {
		printf("\rinms_write() success .. \r\n");
		f_close(&file);
	}
	serial_number ++;
	number ++;
	vTaskDelay(1 * delay_time_based);
}
#endif

}

