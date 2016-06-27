#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include <util/hexdump.h>
#include <util/timestamp.h>
#include <nanomind.h>
#include <dev/i2c.h>
#include <dev/adc.h>
#include <vfs/vfs.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>

#include "fs.h"
#include "parameter.h"
#include "subsystem.h"

#define INMSWRITEDATA		0
#define SDCARDTEST			0
#define TEST_FILENAME		0
#define TEST_i2c_tx_len		0
#define Test_Delay			0
#define format_SD			0
#define Test_downlink		0
#define Test_downlink_2		0
#define Test_xHandle		0
#define Test_Err_Packet		0
#define Test_ECEF_ECI		0
#define Test_infinite_loop	0
#define Test_OBC_Temp		0
#define Test_Crippled_mode	0
#define Test_Malloc			0
#define Test_boot_conf		0
#define Test_Queue			0

extern void encode_time (char buf[], char * fileName);

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
	portTickType xFrequency;
	xLastWakeTime = xTaskGetTickCount();
	xFrequency = 1 * delay_time_based;
	while (1) {

		printf("Test delay with %d s\n", xFrequency / delay_time_based);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
	portTickType start_time, stop_time;
	char fileName[45];
	char fileName_decode[9];
	int number = 0;

	uint8_t errPacketTotal [eop_length] = {0};
	uint16_t serial_number = 0;
	FRESULT res;
	FIL file, file_record;
	UINT  bw;
	struct tm  ts;
	char buf[20];
	unsigned int ms;
	uint16_t msss;
	extern void encode_time(char buf[], char * fileName );
	vTaskDelay(5 * delay_time_based);

	while (1) {

		start_time = xTaskGetTickCount();


		memcpy(&errPacketTotal[0], &serial_number, 2);
		// hex_dump(&errPacketTotal, eop_length);

		strcpy(fileName, "0:/TT_DATA/");
		/* Get current time */
		timestamp_t t;
		t.tv_sec = 0;
		t.tv_nsec = 0;
		obc_timesync(&t, 6000);
		time_t tt = t.tv_sec;
		time(&tt);
		/* Format time, "ddd yyyy-mm-dd hh:mm:ss zzz" */
		ts = *localtime(&tt);
		strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);
#if _USE_LFN
		strcat(fileName, buf);
#else
		encode_time(buf, fileName_decode);
		strcat(fileName, fileName_decode);
#endif

		strcat(fileName, ".dat");
		printf("%s\n", fileName);


		res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_WRITE );
		if (res != FR_OK)
		{
			// printf("\r\nf_open() fail .. \r\n");
		}
		else {
			// printf("f_open() success .. \r\n");
			f_lseek(&file, file.fsize);
		}
		res = f_write(&file, errPacketTotal, eop_length, &bw);

		if (res != FR_OK) {
			// printf("\rtest_write() fail .. \r\n");
			f_close(&file);
		}
		else {
			// printf("\rtest_write() success .. \r\n");
			f_close(&file);
		}
		serial_number ++;
		number ++;

		stop_time = xTaskGetTickCount();
		ms = (stop_time - start_time) * (1000 / configTICK_RATE_HZ);
		printf("Wrote 32 bytes in %u ms \r\n", ms);

		memcpy(&msss, &ms, 2);
		f_open(&file_record, "0:test.bin", FA_OPEN_ALWAYS | FA_WRITE );
		f_lseek(&file_record, file_record.fsize);
		f_write(&file_record, &msss, 2, &bw);

		f_close(&file_record);
		vTaskDelay(1 * delay_time_based);
	}
#endif

#if Test_xHandle
	extern void vTaskInmsTemperatureMonitor (void * pvParameters);
	while (1) {

		if (inms_temp_moniter != NULL) {
			printf("delete\n");
			vTaskDelete(inms_temp_moniter);
			inms_temp_moniter = NULL;
		}
		vTaskDelay(0.5 * delay_time_based);
		if (inms_temp_moniter == NULL) {
			printf("create\n");
			xTaskCreate(vTaskInmsTemperatureMonitor, (const signed char * ) "InmsTM", 1024 * 4, NULL, 1, &inms_temp_moniter);
		}
		vTaskDelay(0.5 * delay_time_based);

	}
#endif

#if Test_Err_Packet
	vTaskDelay(5 * delay_time_based);
	uint16_t vbatt = 7500;
	int16_t temp = -20;
// generate_Error_Report(1);
	while (1) {
		generate_Error_Report(1, vbatt);
		vTaskDelay(5 * delay_time_based);
		generate_Error_Report(2, temp);
		vTaskDelay(5 * delay_time_based);
	}
#endif



#if Test_ECEF_ECI
// vTaskDelay(5 * delay_time_based);
	int16_t	r_ECI[3] = {0};
	int32_t	r_ECEF[3] = {0};
	float ECEF_time[6] = {0};
// char ECEF_buf[6][5] = {{0}};
// timestamp_t t;
// struct tm  ts;
// char buf[20];
// while (1) {
// t.tv_sec = 0;
// t.tv_nsec = 0;
// obc_timesync(&t, 6000);
// time_t tt = t.tv_sec;
// time(&tt);

// /* Format time, "ddd yyyy-mm-dd hh:mm:ss zzz" */
// ts = *localtime(&tt);
// strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);
// printf("%s\n", buf);
// strncpy(ECEF_buf[0], &buf[0], 4);
// ECEF_time[0] = atoi(ECEF_buf[0]);	// Year
// strncpy(ECEF_buf[1], &buf[4], 2);
// ECEF_time[1] = atoi(ECEF_buf[1]);	// Month
// strncpy(ECEF_buf[2], &buf[6], 2);
// ECEF_time[2] = atoi(ECEF_buf[2]);	// Day
// strncpy(ECEF_buf[3], &buf[9], 2);
// ECEF_time[3] = atoi(ECEF_buf[3]);	// Hour
// strncpy(ECEF_buf[4], &buf[11], 2);
// ECEF_time[4] = atoi(ECEF_buf[4]);	// Minute
// strncpy(ECEF_buf[5], &buf[13], 2);
// ECEF_time[5] = atoi(ECEF_buf[5]);	// Second

	ECEF_time[0] =  2000    ;
	ECEF_time[1] =  1      ;
	ECEF_time[2] =  1      ;
	ECEF_time[3] =  0      ;
	ECEF_time[4] =  0      ;
	ECEF_time[5] =  0      ;

	r_ECEF[0] = 4258291;
	r_ECEF[1] = 6989010;
	r_ECEF[2] = 1023000;
	printf("\n1: ");
	ECEFtoECI(ECEF_time,  r_ECEF,  r_ECI);

	ECEF_time[0] =  2003    ;
	ECEF_time[1] =  7      ;
	ECEF_time[2] =  31      ;
	ECEF_time[3] =  12      ;
	ECEF_time[4] =  45      ;
	ECEF_time[5] =  0      ;

	r_ECEF[0] = -2398122;
	r_ECEF[1] = -193021;
	r_ECEF[2] = 5911324;
	printf("\n2: ");
	ECEFtoECI(ECEF_time,  r_ECEF,  r_ECI);

	ECEF_time[0] =  2007    ;
	ECEF_time[1] =  12      ;
	ECEF_time[2] =  10      ;
	ECEF_time[3] =  18      ;
	ECEF_time[4] =  32      ;
	ECEF_time[5] =  55      ;

	r_ECEF[0] = 4500000;
	r_ECEF[1] = 3500000;
	r_ECEF[2] = 4000423;
	printf("\n3: ");
	ECEFtoECI(ECEF_time,  r_ECEF,  r_ECI);

	ECEF_time[0] =  2013    ;
	ECEF_time[1] =  4      ;
	ECEF_time[2] =  30      ;
	ECEF_time[3] =  16      ;
	ECEF_time[4] =  30      ;
	ECEF_time[5] =  30      ;

	r_ECEF[0] = 5489213;
	r_ECEF[1] = 1318013;
	r_ECEF[2] = -6999999;
	printf("\n4: ");
	ECEFtoECI(ECEF_time,  r_ECEF,  r_ECI);

	ECEF_time[0] =  2016    ;
	ECEF_time[1] =  9      ;
	ECEF_time[2] =  15      ;
	ECEF_time[3] =  8      ;
	ECEF_time[4] =  0      ;
	ECEF_time[5] =  51      ;

	r_ECEF[0] = -6900090;
	r_ECEF[1] = -800283;
	r_ECEF[2] = -1230120;
	printf("\n5: ");
	ECEFtoECI(ECEF_time,  r_ECEF,  r_ECI);

	ECEF_time[0] =  2017    ;
	ECEF_time[1] =  10      ;
	ECEF_time[2] =  10      ;
	ECEF_time[3] =  22      ;
	ECEF_time[4] =  10      ;
	ECEF_time[5] =  10      ;

	r_ECEF[0] = 4010101;
	r_ECEF[1] = -1010101;
	r_ECEF[2] = 5101010;
	printf("\n6: ");
	ECEFtoECI(ECEF_time,  r_ECEF,  r_ECI);

// memcpy(&ucharAdcs[16], &r_ECI[0], 2);
// memcpy(&ucharAdcs[18], &r_ECI[1], 2);
// memcpy(&ucharAdcs[20], &r_ECI[2], 2);
	vTaskDelete(NULL);
// }

#endif

#if Test_infinite_loop
	vTaskDelay(5 * delay_time_based);
	int times = 0;
	while (1) {
		vTaskDelay(5*delay_time_based);
		times ++;
		printf("hey %d\n",times);
		if (times == 2)
			break;
	}

#endif



#if Test_OBC_Temp

	vTaskDelay(5 * delay_time_based);

	uint16_t * adc_buffer;
// uint16_t
	while (1) {
		adc_buffer = adc_start_blocking(1);


		printf("%d\n", adc_buffer[0]);
		printf("%f\n", ((((adc_buffer[0] * 2493.0) / 1023) - 424) / 6.25));
		vTaskDelay(5 * delay_time_based);
	}


	vTaskDelete(NULL);
#endif

#if Test_Crippled_mode

	vTaskDelay(1 * delay_time_based);

	uint8_t test[wod_length];
	test[0] = 0;
	test[1] = 0;
	test[2] = 0;
	test[3] = 0x1c;
	for (int i = 4; i < wod_length - 4; i++)
	{
		test[i] = i;
	}
	while (1) {
		if (parameters.crippled_Mode == 0)
			inms_data_write_dup(test);
		else {
		// 	hk_write_crippled(test);
		// 	vTaskDelay(5 * delay_time_based);
		// 	inms_data_write_crippled(test);
		// 	vTaskDelay(5 * delay_time_based);
		// 	seuv_write_crippled(test);
		// 	vTaskDelay(5 * delay_time_based);
		// 	eop_write_crippled(test);
		// 	
		// 	
			wod_write_crippled(test);


		}
		vTaskDelay(5 * delay_time_based);
	}
	// int fd, bytes;
	// int size = 4;
	// // struct tm  ts;
	// // char buf[20];

	// char * data_w = pvPortMalloc(size);
	// // char * data_r = pvPortMalloc(size);
	// if (data_w == NULL) {
	// 	printf("Failed to allocate memory buffer\r\n");
	// }
	// data_w = "123";
	// char fileName[45];

	// // strcpy(fileName, "0:/HK_DATA/");
	// // strcpy(fileName, "/sd1/HK_DATA/");
	// strcpy(fileName, "/boot/HK_DATA.bin");


	// fd = open(fileName, O_CREAT | O_APPEND | O_RDWR);
	// if (fd < 0) {
	// 	printf("Failed to open %s\r\n", fileName);
	// }
	// lseek(fd, 0, SEEK_END);
	// bytes = write(fd, data_w, size);
	// if (bytes != size) {
	// 	printf("Failed to write test data to %s (wrote %d bytes)\r\n", fileName, bytes);
	// }

	// close(fd);

// 	fd = open(fileName, O_RDONLY);
// 	if (fd < 0) {
// 		printf("Failed to open %s\r\n", fileName);
// 	}
// //
// 	bytes = read(fd, data_r, size);
// 	if (bytes != size) {
// 		printf("Failed to read test data from %s (read %u bytes)\r\n", fileName, bytes);
// 	}

// 	hex_dump(data_r, 4);
// 	close(fd);
	vTaskDelete(NULL);
#endif

#if Test_Malloc
	uint8_t * script = NULL;
	int size ;
	for (int i = 0; i < 7; i++)
	{
		printf("i = %d\n", i);
		size = 2000 + 110 * i;
		if (!script) {
			printf("in\n");
			script = malloc(size);
		}
		else {
			free (script);
			script = malloc(size);
		}
		for (int i = 0; i < size; i++)
		{
			script[i] = i % 256;
		}

	}

	printf("success\n");
	vTaskDelete(NULL);
#endif

#if Test_boot_conf
	uint8_t test[] = {0x42, 0x41, 0x31, 0x33, 0x42, 0x42, 0x31, 0x44, 0x35};
	printf("test1\n");
	image_boot_write(test);
	vTaskDelete(NULL);
#endif	
#if Test_Queue
	long lValueToSend;
	portBASE_TYPE xStatus;
	xQueueHandle xQueue;
	xQueue = xQueueCreate (5, sizeof(long));
	while(1) {

		xStatus = xQueueSendToBack (xQueue, &lValueToSend, 0);
		taskYIELD();

	}
#endif	
}
