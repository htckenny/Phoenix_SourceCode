#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include <dev/i2c.h>

#include <io/nanomind.h>
#include <util/hexdump.h>
#include <util/timestamp.h>
#include <csp/csp_endian.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>

#include "parameter.h"
#include "Tele_function.h"
#include "subsystem.h"
// #include <fat/ff.h>
#include "fs.h"

#define maxlength 	20
#define maxNum		50
FATFS fs;
FRESULT res;
FIL file;
UINT br, bw;
uint8_t buffer[300];
FILINFO *fno;
extern int findMaxBuf(uint8_t sortbuf[]);

/* Start of Schedule related FS function */

/**
 * Dump all the scheduled command stored in the FS
 * @return error code
 */
int schedule_dump()
{
	uint8_t txlen;
	uint8_t sche_buf[maxNum * maxlength] = {0};
	
	if (schedule_read(sche_buf) == Error)
		return Error;
	// for (int i = 0 ; i < 60 ; i++){
	// 	printf("%d ", sche_buf[i]);
	// }
	// printf("%d\n", );
	int lastNum = findMaxBuf(sche_buf);
	printf("%d\n", lastNum);
	for (int i = 0 ; i < lastNum ; i++) {
		txlen = 20;
		SendPacketWithCCSDS_AX25(&sche_buf[0 + 20 * i], txlen, obc_apid, 11, 17);
	}
	if (lastNum != 0) {
		hex_dump(sche_buf, maxlength * lastNum);
	}
	return No_Error;
}
/**
 * Shift the current list of command for some time
 * @param  frameCont input contents
 * @return           error code
 */
int schedule_shift(uint8_t *frameCont)
{
	// printf("in shift -test2\n");
	int32_t shift_time = 0 ;
	uint8_t sche_buf[maxNum * maxlength] = {0};
	uint32_t sche_time[maxNum] = { 0 };
	int lastNum = 0;

	if (schedule_read(sche_buf) == Error)
		return Error;
	else {
		// printf("sche_buf = %d\n", sche_buf[1][0]);
		lastNum = findMaxBuf(sche_buf);
		// vTaskDelay(5000);
	}
	printf("lastNum = %d\n", lastNum);
	memcpy(&shift_time, frameCont, 4);
	shift_time = csp_ntoh32(shift_time);
	printf("in shift time = %ld\n", shift_time);
	// shift_time = csp_ntoh32(shift_time);
	for (int i = 0 ; i < lastNum ; i++) {
		memcpy(&sche_time[i], &sche_buf[i * 20 + 2], 4);
		sche_time[i] = csp_ntoh32(sche_time[i]);
		// sche_time[i] = (sche_buf[i][1] << 24)
		// 	+ (sche_buf[i][2] << 16)
		// 	+ (sche_buf[i][3] << 8)
		// 	+ (sche_buf[i][4])
		// 	+ (sche_buf[i][5] >> 8);
		// for (int i = 0 ; i < 4 ; i++){
		// 	printf("frameCont[%d] = %d\n", i, frameCont[i]);
		// }
		
		// printf("s before = %d\n", shift_time);

		// shift_time = (frameCont[0] << 24)
		// 	+ (frameCont[1] << 16)
		// 	+ (frameCont[2] << 8)
		// 	+ (frameCont[3]);

		sche_time[i] += shift_time ;
		printf("%d %" PRIu32 "\n", i, sche_time[i]);
		sche_time[i] = csp_ntoh32(sche_time[i]);
		memcpy(&sche_buf[i * 20 + 2], &sche_time[i], 4);
		// for (int i = 0; i < 4; i++){
		// 	printf("sche_buf[%d] = %d\n",i , sche_buf[i]);
		// }
		// sche_buf[i][4] = sche_time[i] >> 24 ;
		// sche_buf[i][3] = ((sche_time[i] - (sche_buf[i][4] << 24)) >> 16);
		// sche_buf[i][2] = ((sche_time[i] - (sche_buf[i][4] << 24) - (sche_buf[i][3] << 16))>> 8);
		// sche_buf[i][1] = (sche_time[i] - (sche_buf[i][4] << 24) - (sche_buf[i][3] << 16) - (sche_buf[i][2] << 8));
	}



	// if(schedule_read(&sche_buf) == Error)
	// 	return Error;
	// else{
	// 	lastNum = findMaxBuf(sche_buf);
	// }
	// printf("lastNum = %d\n", lastNum);
//	int buffer_length[lastNum];
	// for (int j = 0 ; j < lastNum; j++){
	// 	for (int i = 0 ; i < 200 ; i++){
	// 		printf("buf = %d\n", sche_buf[j][i]);
	// 		if (sche_buf[j][i] == 0x0a){
	// 			buffer_length[j] = i ;
	// 			printf("fuck\n");
	// 			break;
	// 		}
	// 	}
	// }
	// for(int i = 0 ; i < lastNum ; i++){
	// 	printf("buffer lenght = %d\n",buffer_length[i]);
	// }

	if (schedule_reset() == 1) {
		return Error;
	}
	else {
		printf("reset success !! \n");
	}
	for (int i = 0 ; i < lastNum ; i++) {
		// printf("%d\n", sizeof(sche_buf[i]));
		if (schedule_write(&sche_buf[1 + 20 * i]) == Error)	//??
			return Error;
		else {
			schedule_new_command_flag = 1 ;
		}
	}
	return No_Error;
}
int schedule_delete(int range, uint8_t * frameCont)
{
	//f_mount(0, &fs);
	//char fileName[]="0:/OnB_Sch.txt";
	uint8_t sche_buf[maxNum * maxlength] = {0};
	uint8_t sche_update[maxNum * maxlength] = {0};
	int sche_delete_record[maxNum] = {0};
	schedule_read(sche_buf);
	int lastNum = findMaxBuf(sche_buf);
	int update_series = 0;
	uint32_t sche_time[maxNum] = { 0 };
	uint32_t time_t1 = { 0 };
	uint32_t time_t2 = { 0 };

	memcpy(&time_t1, &frameCont[10], 4);
	// time_t1 = (frameCont[10] << 24)
	// 	+ (frameCont[11] << 16)
	// 	+ (frameCont[12] << 8)
	// 	+ (frameCont[13])
	// 	+ (frameCont[14] >> 8);
	if (range == 1) {
		memcpy(&time_t2, &frameCont[15], 4);
		// time_t2 = (frameCont[15] << 24)
		// 	+ (frameCont[16] << 16)
		// 	+ (frameCont[17] << 8)
		// 	+ (frameCont[18])
		// 	+ (frameCont[19] >> 8);
	}
	time_t1 = csp_ntoh32(time_t1);
	time_t2 = csp_ntoh32(time_t2);
	printf("t1 : %" PRIu32 "\n", time_t1);
	printf("t2 : %" PRIu32 "\n", time_t2);

	printf("last num = %d \n", lastNum);
	for (int i = 0 ; i < lastNum ; i++) {
		memcpy(&sche_time[i], &sche_buf[i * 20 + 2], 4);
		// sche_time[i] = (sche_buf[i][1] << 24)
		// 	+ (sche_buf[i][2] << 16)
		// 	+ (sche_buf[i][3] << 8)
		// 	+ (sche_buf[i][4])
		// 	+ (sche_buf[i][5] >> 8);
		sche_time[i] = csp_ntoh32(sche_time[i]);
		printf("sch time = %" PRIu32 "\n", sche_time[i] );
		switch (range) {
		case 1 :
			// delete command between T1 & T2
			if (sche_time[i] >= time_t1 && sche_time[i] <= time_t2) {
				sche_delete_record[i] = -1;
			}
			break;

		case 2 :
			// delete command less than or equals to T1
			if (sche_time[i] <= time_t1) {
				sche_delete_record[i] = -1;
			}
			break;
		case 3 :
			// delete command greater than or equals to T1
			if (sche_time[i] >= time_t1) {
				sche_delete_record[i] = -1;
			}

			break;
		}
		if (sche_delete_record[i] != -1) {
			// copy the content from the old schedule list to new one
			memcpy(&sche_update[update_series * 19], &sche_buf[i * 20 + 1], 19);
			update_series ++ ;
		}
	}

	for (int i = 0 ; i < lastNum ; i++ ){
		printf("delete record: %d\n", sche_delete_record[i]);
	}
	for (int i = 0 ; i < update_series ; i++ ){
		hex_dump(&sche_update[19 * i], 19);
	}
	
	if (schedule_reset() != 1)
		printf("schedule reset success\n");
	for (int i = 0 ; i < update_series ; i++) {
		if (schedule_write(&sche_update[19 * i]) == Error)
			return Error;
	}

	return No_Error;

}
int schedule_reset() {

	f_mount(0, &fs);
	char fileName[] = "0:/OnB_Sch.bin";
	res = f_unlink(fileName);	  //先刪除
	f_mount(0, NULL);

	parameters.schedule_series_number = 0 ;
	para_w();
	schedule_unlink_flag = 1;
	if (res != FR_OK) {
		printf("\r\n schedule file f_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n schedule file f_unlink() success .. \r\n");
		return No_Error;
	}

}
int schedule_write(uint8_t frameCont[])
{
	f_mount(0, &fs);
	char fileName[] = "0:/OnB_Sch.bin";
	// char nextLine[] = "\n";
	char snumber[] = "0";

	printf("schedule_series_number = %d\n", parameters.schedule_series_number);
	sprintf(snumber, "%d", parameters.schedule_series_number);

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\nopen() fail .. \r\n");		
	}
	else {
		printf("\r\nopen() success .. \r\n");		
	}
	f_lseek(&file, file.fsize);
	// printf("size of snumber : %d\n", sizeof(snumber));
	// schedule_series_number = 0;
	// para_r();
	
	res = f_write(&file, snumber, 1, &bw);
	if (res != FR_OK) {
		printf("\r\nwrite() fail .. \r\n");		
	}
	else {
		printf("\r\nwrite() success .. \r\n");		
	}

	for (int i = 0 ; i < 19 ; i++) {
		printf("%u ", frameCont[i]);
	}
	// printf("\nsize of the frame pass into the write function: %d\n", length);
	// f_lseek(&file, file.fsize);
	res = f_write(&file, frameCont, 19, &bw);
	if (res != FR_OK) {
		printf("\r\nOn Board Schedule write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\nOn Board Schedule write() success .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		parameters.schedule_series_number ++;
		para_w();
		return No_Error;
	}
}
int schedule_read(uint8_t * txbuf)
{
	// printf("%d", parameters.sche_store_count);
	// char *
	// printf("inside \n");
	// uint8_t buf[50][50];
	f_mount(0, &fs);
	char fileName[] = "0:/OnB_Sch.bin";
	// int total_line = 0;
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_open() success .. \r\n");
	}
	
	int c = 0 ;
	while (1) {
		res = f_read(&file, &buffer, 1, &br); 

		// if (res == FR_OK) {
		// 	printf("f_read() success .. \r\n");
		// }
		// else {
		// 	printf("f_read() fail .. \r\n");
		// }
		
		memcpy(&txbuf[c++], &buffer, 1);

		// c = c + 16;
		if (f_eof(&file)) {break;}
	}
	printf("c = %d\n", c);
	if (res != FR_OK) {
		printf("\r\n schedule_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\n schedule_read() success .. \r\n");
		// for (int i = 0; i < total_line; i++)
		// {
		// 	memcpy(&txbuf[i], &buf[i], maxlength);
		// }
		// for (int i = 0; i < 10; i++)
		// {
		// 	for (int j = 0; j < 5; j++)
		// 	{
		// 		printf("buf[%d][%d] = %d\n", j, i, txbuf[j][i]);
		// 	}
		// }
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

/** End of Schedule related FS function*/
/*  ---------------------------------------------------  */	
/** Start of old downlink/delete related FS function*/

int downlink_data_before_t(uint8_t datatype, uint32_t time1) {
	/*      Now T1 is not convert yet, please check T1 endian to h      */

	char * fileName;
	uint8_t size;
	uint32_t datatime;

	if (datatype == 1) {
		fileName = "0:/hk.bin";
		size = hk_length;
	}
	else if (datatype == 2) {
		fileName = "0:/inms.bin";
		size = inms_data_length;
	}
	else if (datatype == 3) {
		fileName = "0:/seuv.bin";
		size = seuv_length;
	}
	else if (datatype == 4) {
		fileName = "0:/eop.bin";
		size = eop_length;
	}
	else if (datatype == 5) {
		fileName = "0:/wod.bin";
		size = wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}
	
	while (1) {
		res = f_read(&file, &buffer, size, &br);
		if (res == FR_OK) {
			memcpy(&datatime, &buffer, 4);
			// printf("data time 1 = %"PRIu32"\n", datatime);
			datatime = csp_ntoh32(datatime);
			// printf("data time 1 = %"PRIu32"\n", datatime);

			if (datatime <= time1)
				SendDataWithCCSDS_AX25(datatype, &buffer[0]);
		}
		else {
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}

		if (f_eof(&file))
			break;
	}


	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}

int downlink_data_after_t(uint8_t datatype, uint16_t time1) {
	/*      Now T1 is not convert yet, please check T1 endian to h      */
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	if (datatype == 1) {
		fileName = "0:/hk.bin";
		size = hk_length;
	}
	else if (datatype == 2) {
		fileName = "0:/inms.bin";
		size = inms_data_length;
	}
	else if (datatype == 3) {
		fileName = "0:/seuv.bin";
		size = seuv_length;
	}
	else if (datatype == 4) {
		fileName = "0:/eop.bin";
		size = eop_length;
	}
	else if (datatype == 5) {
		fileName = "0:/wod.bin";
		size = wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while (1) {


		res = f_read(&file, &buffer, size, &br);
		if (res == FR_OK) {
			memcpy(&datatime, &buffer, 4);
			datatime = csp_ntoh32(datatime);
			if ((datatime > time1) | (datatime == time1))
				SendDataWithCCSDS_AX25(datatype, &buffer[0]);
		}
		else {
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}

		if (f_eof(&file))
			break;
	}

	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
int downlink_data_between_t(uint8_t datatype, uint16_t time1, uint16_t time2) {
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	if (datatype == 1) {
		fileName = "0:/hk.bin";
		size = hk_length;
	}
	else if (datatype == 2) {
		fileName = "0:/inms.bin";
		size = inms_data_length;
	}
	else if (datatype == 3) {
		fileName = "0:/seuv.bin";
		size = seuv_length;
	}
	else if (datatype == 5) {
		fileName = "0:/wod.bin";
		size = wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}
	while (1) {
		res = f_read(&file, &buffer, size, &br);
		if (res == FR_OK) {
			memcpy(&datatime, &buffer, 4);
			datatime = csp_ntoh32(datatime);
			if ((datatime < time1) | (datatime == time1)) {
				if ((datatime > time2) | (datatime == time2)) {
					SendDataWithCCSDS_AX25(datatype, &buffer[0]);
				}
			}

			if ((datatime < time2) | (datatime == time2)) {
				if ((datatime > time1) | (datatime == time1)) {
					SendDataWithCCSDS_AX25(datatype, &buffer[0]);
				}
			}
		}
		else {
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}
		if (f_eof(&file))
			break;
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}


int delete_data_before_t(uint8_t datatype, uint32_t time1) {
	/*      Now T1 is not convert yet, please check T1 endian to h      */
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	int count = 0;
	int a;
	if (datatype == 1) {
		fileName = "0:/hk.bin";
		size = hk_length;
	}
	else if (datatype == 2) {
		fileName = "0:/inms.bin";
		size = inms_data_length;
	}
	else if (datatype == 3) {
		fileName = "0:/seuv.bin";
		size = seuv_length;
	}
	else if (datatype == 4) {
		fileName = "0:/eop.bin";
		size = eop_length;
	}
	else if (datatype == 5) {
		fileName = "0:/wod.bin";
		size = wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while (1) {
		res = f_read(&file, &buffer, size, &br);
		if (res == FR_OK) {
			memcpy(&datatime, &buffer, 4);
			datatime = csp_ntoh32(datatime);

			if ((datatime < time1) | (datatime == time1)) {
				f_lseek(&file, count * size);
				for (a = 0; a < size; a++)
					buffer[a] = 0;
				res = f_write(&file, &buffer, size, &bw);
			}

		}
		else {
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}
		if (res != FR_OK)
			return Error;
		if (f_eof(&file))
			break;
		count++;
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}

int delete_data_after_t(uint8_t datatype, uint16_t time1) {
	/*      Now T1 is not convert yet, please check T1 endian to h      */
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	int count = 0;
	int a;
	if (datatype == 1) {
		fileName = "0:/hk.bin";
		size = hk_length;
	}
	else if (datatype == 2) {
		fileName = "0:/inms.bin";
		size = inms_data_length;
	}
	else if (datatype == 3) {
		fileName = "0:/seuv.bin";
		size = seuv_length;
	}
	else if (datatype == 4) {
		fileName = "0:/eop.bin";
		size = eop_length;
	}
	else if (datatype == 5) {
		fileName = "0:/wod.bin";
		size = wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while (1) {


		res = f_read(&file, &buffer, size, &br);
		if (res == FR_OK) {
			memcpy(&datatime, &buffer, 4);
			datatime = csp_ntoh32(datatime);
			if ((datatime > time1) | (datatime == time1)) {
				f_lseek(&file, count * size);
				for (a = 0; a < size; a++) {
					buffer[a] = 0;
				}
				res = f_write(&file, &buffer, size, &bw);
			}
		}
		else {
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}
		if (res != FR_OK)
			return Error;
		if (f_eof(&file))
			break;
		count++;
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
int delete_data_between_t(uint8_t datatype, uint16_t time1, uint16_t time2) {
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	int count = 0;
	int a;
	if (datatype == 1) {
		fileName = "0:/hk.bin";
		size = hk_length;
	}
	else if (datatype == 2) {
		fileName = "0:/inms.bin";
		size = inms_data_length;
	}
	else if (datatype == 3) {
		fileName = "0:/seuv.bin";
		size = seuv_length;
	}
	else if (datatype == 5) {
		fileName = "0:/wod.bin";
		size = wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while (1) {
		res = f_read(&file, &buffer, size, &br);
		if (res == FR_OK) {
			memcpy(&datatime, &buffer, 4);
			datatime = csp_ntoh32(datatime);
			if ((datatime < time1) | (datatime == time1)) {
				if ((datatime > time2) | (datatime == time2)) {
					f_lseek(&file, count * size);
					for (a = 0; a < size; a++)
						buffer[a] = 0;
					res = f_write(&file, &buffer, size, &bw);
				}
			}
			if ((datatime < time2) | (datatime == time2)) {
				if ((datatime > time1) | (datatime == time1)) {
					f_lseek(&file, count * size);
					for (a = 0; a < size; a++)
						buffer[a] = 0;
					res = f_write(&file, &buffer, size, &bw);
				}
			}
		}
		else {
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}
		if (res != FR_OK)
			return Error;
		if (f_eof(&file))
			break;
		count++;
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
/** End of old downlink/delete related FS function*/
/*  ---------------------------------------------------  */	
/** Start of INMS data related FS function*/
int inms_data_write(uint8_t frameCont[])
{
	f_mount(0, &fs);
    struct tm  ts;
    char buf[80];
    char s[] = "0:/INMS_DATA/";
    char fileName[40];

	timestamp_t t;

    strcpy(fileName, s);

	/* Get current time */
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	time_t tt = t.tv_sec;
	time(&tt);
	// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
	ts = *localtime(&tt);
	strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);

	strcat(fileName, buf);
	strcat(fileName, "_INMS_TW01");
	strcat(fileName, ".dat");
	printf("%s\n", fileName);
	
	// printf("%d\n", frameCont[0]);
	// printf("%d\n", frameCont[1]);
	// printf("%d\n", frameCont[2]);

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}
	//將pointer指向文件最後面
	f_lseek(&file, file.fsize);

	res = f_write(&file, frameCont, inms_data_length, &bw);

	// hex_dump(frameCont, inms_data_length);
	if (res != FR_OK) {
		printf("\r\n inms_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\n inms_write() success .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int inms_data_read(char fileName[], void * txbuf) { 

	// f_mount(0, &fs);
	// printf("%s\n", fileName);
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	} else {
		printf("\r\n f_open() success .. \r\n");
	}
	int c = 0 ;
	while (1) {
		res = f_read(&file, &buffer, 1, &br); 
		memcpy(txbuf + (c++), &buffer, 1);
		// c++;
		if (f_eof(&file)) {break;}
	}

	// res = f_read(&file, &buffer, inms_data_length, &br);

	if (res != FR_OK) {
		printf("\r\n inmsdata_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\n inmsdata_read() success .. \r\n");
		// memcpy(txbuf, &buffer, inms_data_length);
		f_close(&file);
		// f_mount(0, NULL);
		return No_Error;
	}
}

int inms_data_delete(char fileName[]) {
	// f_mount(0, &fs);
	
	res = f_unlink(fileName);	  //先刪除

	if (res != FR_OK)
	{
		printf("%s\r\n f_unlink() fail .. \r\n", fileName);
		return Error;
	}
	else {
		printf("%s\r\n f_unlink() success .. \r\n", fileName);
		return No_Error;
	}
}

int inms_script_write(int buffNum, uint8_t scriptCont[], int delete_flag, int length) {

	f_mount(0, &fs);
	char fileName[100];

	if (buffNum == 8)
		strcpy(fileName, "0:/INMS/Running.bin");
	else if (buffNum == 0)
		strcpy(fileName, "0:/INMS/IDLE0.bin");
	else if (buffNum == 1)
		strcpy(fileName, "0:/INMS/IDLE1.bin");
	else if (buffNum == 2)
		strcpy(fileName, "0:/INMS/IDLE2.bin");
	else if (buffNum == 3)
		strcpy(fileName, "0:/INMS/IDLE3.bin");
	else if (buffNum == 4)
		strcpy(fileName, "0:/INMS/IDLE4.bin");
	else if (buffNum == 5)
		strcpy(fileName, "0:/INMS/IDLE5.bin");
	else if (buffNum == 6)
		strcpy(fileName, "0:/INMS/IDLE6.bin");
	else if (buffNum == 7)
		strcpy(fileName, "0:/INMS/IDLE7.bin");	//for testing

		printf("%s\n", fileName);
	if (delete_flag == 1) {
		res = f_unlink(fileName);	  //先刪除

		if (res != FR_OK) {
			printf("\r\n f_unlink() fail .. \r\n");
		}
		else {
			printf("\r\n f_unlink() success .. \r\n");
		}
	}
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_open() success .. \r\n");
	}

	//將pointer指向文件最後面
	
	res = f_lseek(&file, file.fsize);
	// if (res != FR_OK) {
	// 	printf("point to the last fail ..\n");
	// }
	// else {
	// 	printf("point to the last success ..\n");
	// }
	printf(" 4 = %d\n", scriptCont[4]);
	printf("write content with %d bytes: \n", length) ;	
	res = f_write(&file, scriptCont, length, &bw);
	if (res != FR_OK) {
		printf("\r\n f_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\n f_write() success .. \r\n");

	}
	hex_dump(scriptCont, length);

	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
void inms_script_read(int buffNum, int packlength, void * txbuf) {

	f_mount(0, &fs);

	char fileName[100];
	if (buffNum == 8) {
		strcpy(fileName, "0:/INMS/Running.bin");
	}
	else if (buffNum == 0) {
		strcpy(fileName, "0:/INMS/IDLE0.bin");
	}
	else if (buffNum == 1) {
		strcpy(fileName, "0:/INMS/IDLE1.bin");
	}
	else if (buffNum == 2) {
		strcpy(fileName, "0:/INMS/IDLE2.bin");
	}
	else if (buffNum == 3) {
		strcpy(fileName, "0:/INMS/IDLE3.bin");
	}
	else if (buffNum == 4) {
		strcpy(fileName, "0:/INMS/IDLE4.bin");
	}
	else if (buffNum == 5) {
		strcpy(fileName, "0:/INMS/IDLE5.bin");
	}
	else if (buffNum == 6) {
		strcpy(fileName, "0:/INMS/IDLE6.bin");
	}
	else if (buffNum == 7) {
		strcpy(fileName, "0:/INMS/IDLE7.bin");
	}

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

	if (res != FR_OK) {
		printf("f_open() fail .. \r\n");
	}
	else {
		printf("f_open() success .. \r\n");
	}
	// int c = 0;
	// while (1) {
	// res = f_read(&file, &buffer, packlength, &br); 

	// if (res == FR_OK) {
	// 	printf("f_read() success .. \r\n");
	// }
	// else {
	// 	printf("f_read() fail .. \r\n");
	// }
	
	// memcpy(&txbuf, &buffer, packlength);
		// c = c + 16;
		// if (f_eof(&file)) {break;}
	
	int c = 0;
	while (1) {
		res = f_read(&file, buffer, 1, &br); 

		// if (res == FR_OK) {
		// 	printf("f_read() success .. \r\n");
		// }
		// else {
		// 	printf("f_read() fail .. \r\n");
		// }
		
		// memcpy(&txbuf[c++], &buffer, 1);
		memcpy(txbuf + c, &buffer, 1);
		c = c + 1;
		if (f_eof(&file)) {break;}
	}
	printf("%d\n", c);

	f_close(&file);
	f_mount(0, NULL);

}

int inms_script_length(int buffNum) {

	f_mount(0, &fs);
	int packlength = 0;
	char fileName[100];
	// printf("i  = %d \n", buffNum);
	if (buffNum == 8) {
		strcpy(fileName, "0:/INMS/Running.bin");
	}
	else if (buffNum == 0) {
		strcpy(fileName, "0:/INMS/Idle0.bin");
	}
	else if (buffNum == 1) {
		strcpy(fileName, "0:/INMS/Idle1.bin");
	}
	else if (buffNum == 2) {
		strcpy(fileName, "0:/INMS/Idle2.bin");
	}
	else if (buffNum == 3) {
		strcpy(fileName, "0:/INMS/Idle3.bin");
	}
	else if (buffNum == 4) {
		strcpy(fileName, "0:/INMS/Idle4.bin");
	}
	else if (buffNum == 5) {
		strcpy(fileName, "0:/INMS/Idle5.bin");
	}
	else if (buffNum == 6) {
		strcpy(fileName, "0:/INMS/Idle6.bin");
	}
	else if (buffNum == 7) {
		strcpy(fileName, "0:/INMS/IDLE7.bin");
	}

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_open() success .. \r\n");
	}

	for (int i = 0; i < 300; i++) {
		buffer[i] = 0;
	}

	res = f_read(&file, buffer, 2, &br);
	printf("buffer0 = %02X\n", buffer[0]);
	printf("buffer1 = %02X\n", buffer[1]);
	if (res == FR_OK) {
		if (buffer[0] != 0)
			packlength = buffer[0] + (buffer[1] << 8);
		else
			packlength = 0;
	}
	else {
		printf("\r\n f_read() fail .. \r\n");
	}

	printf("packlength : %d\n", packlength);
	f_close(&file);
	f_mount(0, NULL);
	return packlength;
}
/** End of INMS data/script related FS function*/
/*  ---------------------------------------------------  */	
/** Start of WOD data related FS function*/

int wod_write(uint8_t * frameCont )
{
	// f_mount(0, &fs);
	// char fileName[] = "0:/wod.bin";
	f_mount(0, &fs);
 	struct tm  ts;
 	char buf[80];
	char s[] = "0:/WOD_DATA/";
	char fileName[40];

	timestamp_t t;

  	strcpy(fileName, s);

	// Get current time
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	time_t tt = t.tv_sec;
	time(&tt);
	// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
	ts = *localtime(&tt);
	strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);

	strcat(fileName, buf);
	strcat(fileName, "_WOD_TW01");
	strcat(fileName, ".dat");
	printf("%s\n", fileName);

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}

	f_lseek(&file, file.fsize);
	res = f_write(&file, frameCont, wod_length, &bw);

	if (res != FR_OK) {
		printf("\r\n wod_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\n wod_write() success .. \r\n");

		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int wod_read(char fileName[], void * txbuf) // serial =1~N
{ 

	// if (serial == 0) serial = 1;
	// f_mount(0, &fs);
	// char fileName[20];
	// char s[] = "0:/wod/wod_";
	// strcpy(fileName, s);
	// int head = (serial - 1) / 100;
	// char num[5];
	// sprintf(num, "%d", head);
	// strcat(fileName, num);
	// strcat(fileName, ".bin");


	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_open() success .. \r\n");
	}
	// int tmp = (serial - 1) % 100;
	// f_lseek(&file, tmp * wod_length);
	f_lseek(&file, file.fsize);
	res = f_read(&file, &buffer, wod_length, &br);

	if (res != FR_OK) {
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		memcpy(txbuf, &buffer, wod_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int wod_delete()
{
	f_mount(0, &fs);
	char fileName[] = "0:/wod.bin";
	res = f_unlink(fileName);	  //先刪除

	if (res != FR_OK) {
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}

/** End of WOD data related FS function*/
/*  ---------------------------------------------------  */	
/** Start of SEUV data related FS function*/

int seuv_write()
{
	f_mount(0, &fs);
 	struct tm  ts;
 	char buf[80];
	char s[] = "0:/SEUV_DATA/";
	char fileName[40];

	timestamp_t t;

  	strcpy(fileName, s);

	// Get current time
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	time_t tt = t.tv_sec;
	time(&tt);
	// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
	ts = *localtime(&tt);
	strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);

	strcat(fileName, buf);
	strcat(fileName, "_SEUV_TW01");
	strcat(fileName, ".dat");
	printf("%s\n", fileName);

	// f_mount(0, &fs);
	// char fileName[] = "0:/seuv.bin";

	seuvFrame.packettime = csp_hton32(t.tv_sec);
	printf("sample = %d\n", seuvFrame.samples);

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
		printf("SEUV  f_open fail!!\n");
	else
		printf("SEUV f_open success \n");
	f_lseek(&file, file.fsize);
	res = f_write(&file, &seuvFrame.packettime, (int)sizeof(seuv_frame_t), &bw);

	if (res != FR_OK) {
		printf("SEUV  f_write fail!!\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("SEUV f_write success\n");
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int seuv_read(char fileName[], void * txbuf) 
{ 

	// if (serial == 0) serial = 1;
	// f_mount(0, &fs);
	// char fileName[20];
	// char s[] = "0:/SEUV_DATA/seuv_";
	// strcpy(fileName, s);
	// int head = (serial - 1) / 100;
	// char num[5];
	// sprintf(num, "%d", head);
	// strcat(fileName, num);
	// strcat(fileName, ".bin");


	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_open() success .. \r\n");
	}
	// int tmp = (serial - 1) % 100;
	// f_lseek(&file, tmp * seuv_length);
	res = f_read(&file, &buffer, seuv_length, &br);

	if (res != FR_OK) {
		printf("\r\n seuv_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\n seuv_read() success .. \r\n");
		memcpy(txbuf, &buffer, seuv_length);
		f_close(&file);
		// f_mount(0, NULL);
		return No_Error;
	}
}
int seuv_delete(char fileName[]) 
{
	// f_mount(0, &fs);
	// char fileName[] = "0:/seuv.bin";
	res = f_unlink(fileName);	  //先刪除

	if (res != FR_OK) {
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}
/** End of SEUV data related FS function*/
/*  ---------------------------------------------------  */	
/** Start of HK data related FS function*/

int hk_write(uint8_t * frameCont )
{
	// f_mount(0, &fs);
	// char fileName[] = "0:/hk.bin";

	f_mount(0, &fs);
    struct tm  ts;
    char buf[80];
    char s[] = "0:/HK_DATA/";
    char fileName[40];

	timestamp_t t;

    strcpy(fileName, s);

	// Get current time
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	time_t tt = t.tv_sec;
	time(&tt);
	// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
	ts = *localtime(&tt);
	strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);

	strcat(fileName, buf);
	strcat(fileName, "_HK_TW01");
	strcat(fileName, ".dat");
	printf("%s\n", fileName);
	
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	f_lseek(&file, file.fsize);
	res = f_write(&file, frameCont, hk_length, &bw);

	if (res != FR_OK) {
		printf("\r\n hk_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\n hk_write() success .. \r\n");
		f_close(&file);

		f_mount(0, NULL);
		return No_Error;
	}
}

int hk_read(char fileName[], void * txbuf) // serial =1~N
{ 

	// if (serial == 0) serial = 1;
	// f_mount(0, &fs);
	// char fileName[20];
	// char s[] = "0:/hk/hk_";
	// strcpy(fileName, s);
	// int head = (serial - 1) / 100;
	// char num[5];
	// sprintf(num, "%d", head);
	// strcat(fileName, num);
	// strcat(fileName, ".bin");


	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_open() success .. \r\n");
	}
	// int tmp = (serial - 1) % 100;
	// f_lseek(&file, tmp * hk_length);
	res = f_read(&file, &buffer, hk_length, &br);

	if (res != FR_OK) {
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		memcpy(txbuf, &buffer, hk_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int hk_delete() 
{
	f_mount(0, &fs);
	char fileName[] = "0:/hk.bin";
	res = f_unlink(fileName);	  //先刪除

	if (res != FR_OK) {
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}

/** End of HK data related FS function*/
/*  ---------------------------------------------------  */	
/** Start of EOP data related FS function*/

int eop_write(uint8_t * frameCont )
{
	f_mount(0, &fs);
    struct tm  ts;
    char buf[80];
    char s[] = "0:/EOP_DATA/";
    char fileName[40];

	timestamp_t t;

    strcpy(fileName, s);

	// Get current time
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	time_t tt = t.tv_sec;
	time(&tt);
	// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
	ts = *localtime(&tt);
	strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);

	strcat(fileName, buf);
	strcat(fileName, "_EOP_TW01");
	strcat(fileName, ".dat");
	printf("%s\n", fileName);
	
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	f_lseek(&file, file.fsize);
	res = f_write(&file, frameCont, eop_length, &bw);

	if (res != FR_OK) {
		printf("\r\n eop_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		printf("\r\n eop_write() success .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int eop_read(char fileName[], void * txbuf) 
{ 
	
	// f_mount(0, &fs);	
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_open() success .. \r\n");
	}	
	res = f_read(&file, &buffer, eop_length, &br);

	if (res != FR_OK) {
		printf("\r\n eop_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		memcpy(txbuf, &buffer, eop_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int eop_delete(char fileName[]) 
{

	f_mount(0, &fs);	
	res = f_unlink(fileName);	  //先刪除

	if (res != FR_OK) {
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}
/*  ---------------------------------------------------  */	
/** Start of parameter related FS function*/

int para_r()  // serial =1~N
{ 

	f_mount(0, &fs);
	char fileName[] = "0:/para.bin";
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n f_open() success .. \r\n");
		// printf("%d\n", (int)sizeof(parameter_t));
		res = f_read(&file, &buffer, (int)sizeof(parameter_t), &br);
	}

	if (res != FR_OK) {
		printf("\r\n para_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		
		memcpy(&parameters.first_flight, &buffer, (int)sizeof(parameter_t));
		hex_dump(&buffer, (int)sizeof(parameter_t));
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int para_w() // serial =1~N
{  


	f_mount(0, &fs);
	char fileName[] = "0:/para.bin";
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_write open() success .. \r\n");
	}

	res = f_write(&file, &parameters.first_flight, (int)sizeof(parameter_t), &br);

	if (res != FR_OK) {
		printf("\r\n para write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int para_d()  // serial =1~N
{ 

	f_mount(0, &fs);
	char fileName[] = "0:/para.bin";
	res = f_unlink(fileName);	  //先刪除

	if (res != FR_OK) {
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}

}

/** End of parameter related FS function*/
/*  ---------------------------------------------------  */	
/** Start of ADCS parameter related FS function*/

int adcs_para_r() 
{  // serial =1~N

	f_mount(0, &fs);
	char fileName[] = "0:/adcs_para.bin";
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n f_open() success .. \r\n");
		res = f_read(&file, &buffer, (int)sizeof(adcs_para_t), &br);
	}

	if (res != FR_OK) {
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		f_close(&file);
		f_mount(0, NULL);
		memcpy(&adcs_para.strategy, &buffer, (int)sizeof(adcs_para_t)); //import
		hex_dump(&buffer, (int)sizeof(adcs_para_t));
		return No_Error;
	}
}

int adcs_para_w() 
{  // serial =1~N


	f_mount(0, &fs);
	char fileName[] = "0:/adcs_para.bin";
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		printf("\r\n f_write open() success .. \r\n");
	}

	res = f_write(&file, &adcs_para.strategy, (int)sizeof(adcs_para_t), &br);

	if (res != FR_OK) {
		printf("\r\n para write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int adcs_para_d() 
{  // serial =1~N

	f_mount(0, &fs);
	char fileName[] = "0:/adcs_para.bin";
	res = f_unlink(fileName);	  //先刪除

	if (res != FR_OK) {
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}

/** End of ADCS parameter related FS function*/
/*  ---------------------------------------------------  */	
/** Start of data dump related FS function*/

int inms_data_dump() 
{
	f_mount(0, &fs);
	char fileName[] = "0:/inms.bin";
	int count = 0;
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");

	}
	else {
		while (1) {
			res = f_read(&file, &buffer, inms_data_length, &br);
			if (res == FR_OK) {
				printf("INMS Data Dump packet '%d' \r\n", count);
				hex_dump(&buffer[0], inms_data_length);
				count++;
			}
			else {
				printf("f_read() fail .. \r\n");
				break;
			}
			if (f_eof(&file)) {break;}
		}
	}


	f_close(&file);
	f_mount(0, NULL);

	return No_Error;
}


int seuv_data_dump() 
{
	f_mount(0, &fs);
	char fileName[] = "0:/seuv.bin";
	int count = 0;
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\n f_open() fail .. \r\n");

	}
	else {
		while (1) {


			res = f_read(&file, &buffer, seuv_length, &br);

			if (res == FR_OK) {
				printf("SEUV Data Dump packet '%d' \r\n", count);
				hex_dump(&buffer[0], seuv_length);
				count++;
			}
			else {
				printf("f_read() fail .. \r\n");
				break;
			}
			if (f_eof(&file)) {break;}
		}
	}
	memcpy(&seuvFrame.packettime, &buffer[0], seuv_length);
	seuvFrame.packettime = csp_ntoh32(seuvFrame.packettime);

	printf("SEUV seuvFrame.packettime = %u \r\n", (unsigned int)seuvFrame.packettime);
	printf("SEUV seuvFrame.samples = %u \r\n", (unsigned int)seuvFrame.samples);
	printf("CH1 AVG = %f \r\n", seuvFrame.ch1AVG);
	printf("CH1 STD = %f \r\n", seuvFrame.ch1STD);
	printf("CH2 AVG = %f \r\n", seuvFrame.ch2AVG);
	printf("CH2 STD = %f \r\n", seuvFrame.ch2STD);
	printf("CH3 AVG = %f \r\n", seuvFrame.ch3AVG);
	printf("CH3 STD = %f \r\n", seuvFrame.ch3STD);
	printf("CH4 AVG = %f \r\n", seuvFrame.ch4AVG);
	printf("CH4 STD = %f \r\n", seuvFrame.ch4STD);

	f_close(&file);
	f_mount(0, NULL);



	return No_Error;
}

int wod_data_dump() 
{
	f_mount(0, &fs);
	char fileName[] = "0:/wod.bin";
	int count = 0;
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");

	}
	else {
		while (1) {


			res = f_read(&file, &buffer, wod_length, &br);

			if (res == FR_OK) {
				printf("WOD Data Dump packet '%d' \r\n", count);
				hex_dump(&buffer[0], wod_length);
				count++;
			}
			else {
				printf("f_read() fail .. \r\n");
				break;
			}
			if (f_eof(&file)) {break;}
		}
	}

	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}

int hk_data_dump() 
{
	f_mount(0, &fs);
	char fileName[] = "0:/hk.bin";
	int count = 0;
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");

	}
	else {
		while (1) {


			res = f_read(&file, &buffer, hk_length, &br);

			if (res == FR_OK) {
				printf("HK Data Dump packet '%d' \r\n", count);
				hex_dump(&buffer[0], hk_length);
				count++;
			}
			else {
				printf("f_read() fail .. \r\n");
				break;
			}
			if (f_eof(&file)) {break;}
		}
	}


	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
int thermal_data_dump() 
{
	f_mount(0, &fs);
	char fileName[] = "0:/t_obc.bin";
	int count = 0;
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");

	}
	else {
		while (1) {


			res = f_read(&file, &ThermalFrame.packet_number, (int)sizeof(thermal_frame_t), &br);

			if (res == FR_OK) {
				printf("Thermal Data Dump packet '%d' \r\n", count);
				printf("NUM = %d ,T1= %04X ,T2 %04X ,T3= %04X ,T4= %04X ,T5= %04X \n", (int)ThermalFrame.packet_number, ThermalFrame.T1, ThermalFrame.T2, ThermalFrame.T3, ThermalFrame.T4, ThermalFrame.T5);
				printf("T6= %04X ,T7 %04X ,T8= %04X ,T9= %04X ,T10= %04X \n", ThermalFrame.T6, ThermalFrame.T7, ThermalFrame.T8, ThermalFrame.T9, ThermalFrame.T10);
				printf("==================================== \r\n");
				count++;
			}
			else {
				printf("f_read() fail .. \r\n");
				break;
			}
			if (f_eof(&file)) {break;}
		}
	}


	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
/** End of data dump related FS function*/
/*  ---------------------------------------------------  */	
/** Start of thermal related FS function*/

int thurmal_1_w() 
{  // serial =1~N
	f_mount(0, &fs);
	char fileName[] = "0:/t_obc.bin";

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

	if (res != FR_OK) {
		printf("\r\n @@ %d @@ \r\n", res);
		printf("\r\n f_open() fail .. \r\n");
	}
	else {
		//printf("\r\n f_open() success .. \r\n");
	}

	f_lseek(&file, file.fsize);
	res = f_write(&file, &ThermalFrame.packet_number, (int)sizeof(thermal_frame_t), &br);

	if (res != FR_OK) {
		printf("\r\n thurmal_1_w write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}

	return No_Error;
}

int thurmal_2_w() 
{  // serial =1~N
	f_mount(0, &fs);
	char fileName[] = "0:/t_inms.bin";

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n @@ %d @@ \r\n", res);
		printf("\r\n f_open() fail .. \r\n");
	}
	else {

		//printf("\r\n f_open() success .. \r\n");
	}
	f_lseek(&file, file.fsize);
	res = f_write(&file, &Tdata[0], 178, &br);

	if (res != FR_OK) {
		printf("\r\n thurmal_2_w write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else {
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}

	return No_Error;
}

int T_data_d() 
{  // serial =1~N

	f_mount(0, &fs);
	char fileName[] = "0:/t_obc.bin";
	res = f_unlink(fileName);	  //先刪除

	if (res != FR_OK)
	{
		printf("\r\n t_obc.bin f_unlink() fail .. \r\n");

	}
	else {
		printf("\r\n t_obc.bin f_unlink() success .. \r\n");

	}


	f_mount(0, &fs);
	char fileName2[] = "0:/t_inms.bin";
	res = f_unlink(fileName2);	  //先刪除

	if (res != FR_OK)
	{
		printf("\r\n t_inms.bin f_unlink() fail .. \r\n");

	}
	else {
		printf("\r\n t_inms.bin f_unlink() success .. \r\n");

	}

	return No_Error;
}

/** End of thermal related FS function*/
/*  ---------------------------------------------------  */	
/** Start of scan file related FS function*/

int scan_files_Downlink (
    char* path,        		/* Start node to be scanned (also used as work area) */
    int mode,
    uint32_t timeRec_T1, 
    uint32_t timeRec_T2
)
{
    FRESULT res;
    FILINFO fno;
    FATDIR dir;
    char *fn;   			/* This function assumes non-Unicode configuration */
  	char timeref[16];    
 	char full_path[45];
 	char t_year[5]="0";
    char t_mon[2]={0};
    char t_mday[2]={0};
    char t_hour[2]={0};
    char t_min[2]={0};
    char t_sec[2]={0};
    uint8_t hk_data[hk_length] = {0};
    uint8_t inms_data[inms_data_length * 2] = {0};
  	uint8_t seuv_data[seuv_length] = {0};
  	uint8_t eop_data[eop_length] = {0};
	uint8_t wod_data[wod_length] = {0};
	uint32_t inms_2nd = 0;
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];  	 /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        // i = strlen(path);
        for (;;) {
        	// printf("hi\n");
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            // if (fno.fattrib & AM_DIR) {                    /* It is a directory */
            //     sprintf(&path[i], "/%s", fn);
            //     res = scan_files(path, testTime);
            //     path[i] = 0;
            //     if (res != FR_OK) break;
            // } 
            // else {                                       
            // 
            // printf("%s/%s\n", path, fn);
            sprintf(full_path, "%s/%s", path, fn);         
            printf("%s\n", full_path);
            
            strncpy(timeref, fn, 15);						/* cut the time part of the file name */
            timeref[15] = 0;
            // printf("%s\n", timeref);

		    strncpy(t_year , &timeref[0], 4);
		    t_year[4] = 0;
		    strncpy(t_mon  , &timeref[4], 2);
		    strncpy(t_mday , &timeref[6], 2);
		    strncpy(t_hour , &timeref[9], 2);
		    strncpy(t_min , &timeref[11], 2);
		    strncpy(t_sec , &timeref[13], 2);

	        struct tm t;
		    time_t t_of_day;
		  
		    t.tm_year = atoi(t_year) - 1900;
		    t.tm_mon = atoi(t_mon) - 1;   			    // Month, 0 - jan
		    t.tm_mday = atoi(t_mday);          			// Day of the month
		    t.tm_hour = atoi(t_hour);     
		    t.tm_min = atoi(t_min);
		    t.tm_sec = atoi(t_sec);
		    t.tm_isdst = -1;       						// Is DST on? 1 = yes, 0 = no, -1 = unknown

		    t_of_day = mktime(&t);						/* Construct struct time to epoch seconds */
		    t_of_day -= 946684800;
			
			ctime(&t_of_day);
		    printf("epoch: %"PRIu32"\n", t_of_day );
		    // printf("OBC time is: %s\r\n", ctime(&t_of_day));
	    	
		    switch(mode){
		    	case 1:
		    		if (timeRec_T1 < (unsigned)t_of_day && timeRec_T2 > (unsigned)t_of_day){
		    			printf("mode = 1 down link \n");
		    			if (strcmp(path, "0:/HK_DATA") == 0){
		    				hk_read(full_path, hk_data);
			    			hex_dump(&hk_data, hk_length);
			    			SendDataWithCCSDS_AX25(1, &hk_data[0]);
		    			}
		    			else if (strcmp(path, "0:/INMS_DATA") == 0){
		    				inms_data_read(full_path, inms_data);
			    			hex_dump(&inms_data, inms_data_length);
			    			SendDataWithCCSDS_AX25(2, &inms_data[0]);
			    			memcpy(&inms_2nd ,&inms_data[196], 4);
			    			if (inms_2nd > 0){
			    				hex_dump(&inms_data[196], inms_data_length);
			    				SendDataWithCCSDS_AX25(2, &inms_data[196]);
			    			}
			    		}
			    		else if (strcmp(path, "0:/SEUV_DATA") == 0){
		    				seuv_read(full_path, seuv_data);
			    			hex_dump(&seuv_data, seuv_length);
			    			SendDataWithCCSDS_AX25(3, &seuv_data[0]);
			    		}
			    		else if (strcmp(path, "0:/EOP_DATA") == 0){
		    				eop_read(full_path, eop_data);
			    			hex_dump(&eop_data, eop_length);
			    			SendDataWithCCSDS_AX25(4, &eop_data[0]);
			    		}
			    		else if (strcmp(path, "0:/WOD_DATA") == 0){
		    				wod_read(full_path, wod_data);
			    			hex_dump(&wod_data, wod_length);
			    			SendDataWithCCSDS_AX25(5, &wod_data[0]);
			    		}
		    			// SendPacketWithCCSDS_AX25(&beacon_frame.mode, 8, obc_apid, 0, 0);
		    			vTaskDelay(500);
		    		}
		    		break;
	    		case 2:
    			   	if (timeRec_T1 > (unsigned)t_of_day){
    			   		printf("mode = 2 down link \n");
    			   		if (strcmp(path, "0:/HK_DATA") == 0){
		    				hk_read(full_path, hk_data);
			    			hex_dump(&hk_data, hk_length);
			    			SendDataWithCCSDS_AX25(1, &hk_data[0]);
		    			}
		    			else if (strcmp(path, "0:/INMS_DATA") == 0){
		    				inms_data_read(full_path, inms_data);
			    			hex_dump(&inms_data, inms_data_length);			    			
			    			SendDataWithCCSDS_AX25(2, &inms_data[0]);
			    			memcpy(&inms_2nd ,&inms_data[196], 4);
			    			if (inms_2nd > 0)
			    				SendDataWithCCSDS_AX25(2, &inms_data[196]);
			    		}
			    		else if (strcmp(path, "0:/SEUV_DATA") == 0){
		    				seuv_read(full_path, seuv_data);
			    			hex_dump(&seuv_data, seuv_length);
			    			SendDataWithCCSDS_AX25(3, &seuv_data[0]);
			    		}
			    		else if (strcmp(path, "0:/EOP_DATA") == 0){
		    				eop_read(full_path, eop_data);
			    			hex_dump(&eop_data, eop_length);
			    			SendDataWithCCSDS_AX25(4, &eop_data[0]);
			    		}
			    		else if (strcmp(path, "0:/WOD_DATA") == 0){
		    				inms_data_read(full_path, wod_data);
			    			hex_dump(&wod_data, wod_length);
			    			SendDataWithCCSDS_AX25(5, &wod_data[0]);
			    		}
			    		vTaskDelay(500);
    			   	}
	    			break;
	    		case 3:
	    			if (timeRec_T1 < (unsigned)t_of_day){
    			   		printf("mode = 3 down link \n");
    			   		if (strcmp(path, "0:/HK_DATA") == 0){
		    				hk_read(full_path, hk_data);
			    			hex_dump(&hk_data, hk_length);
			    			SendDataWithCCSDS_AX25(1, &hk_data[0]);
		    			}
		    			else if (strcmp(path, "0:/INMS_DATA") == 0){
		    				inms_data_read(full_path, inms_data);
			    			hex_dump(&inms_data, inms_data_length);
			    			SendDataWithCCSDS_AX25(2, &inms_data[0]);
			    			memcpy(&inms_2nd ,&inms_data[196], 4);
			    			if (inms_2nd > 0)
			    				SendDataWithCCSDS_AX25(2, &inms_data[196]);
			    		}
			    		else if (strcmp(path, "0:/SEUV_DATA") == 0){
		    				seuv_read(full_path, seuv_data);
			    			hex_dump(&seuv_data, seuv_length);
			    			SendDataWithCCSDS_AX25(3, &seuv_data[0]);
			    		}
			    		else if (strcmp(path, "0:/EOP_DATA") == 0){
		    				eop_read(full_path, eop_data);
			    			hex_dump(&eop_data, eop_length);
			    			SendDataWithCCSDS_AX25(4, &eop_data[0]);
			    		}
			    		else if (strcmp(path, "0:/WOD_DATA") == 0){
		    				inms_data_read(full_path, wod_data);
			    			hex_dump(&wod_data, wod_length);
			    			SendDataWithCCSDS_AX25(5, &wod_data[0]);
			    		}
			    		vTaskDelay(500);
    			   	}
	    			break;
	    		default:
	    			printf("range error\n");
	    			break;
		    }
        }
    }
    return res;
}
int scan_files_Delete (
    char* path,        		/* Start node to be scanned (also used as work area) */
    int mode,
    uint32_t timeRec_T1, 
    uint32_t timeRec_T2
)
{
    FRESULT res;
    FILINFO fno;
    FATDIR dir;
    char *fn;   			/* This function assumes non-Unicode configuration */
  	char timeref[16];    
 	char full_path[45];
 	char t_year[5]="0";
    char t_mon[2]={0};
    char t_mday[2]={0};
    char t_hour[2]={0};
    char t_min[2]={0};
    char t_sec[2]={0};
	// uint8_t ErrCode;
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];  	 /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        // i = strlen(path);
        for (;;) {
        	// printf("hi\n");
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            // if (fno.fattrib & AM_DIR) {                    /* It is a directory */
            //     sprintf(&path[i], "/%s", fn);
            //     res = scan_files(path, testTime);
            //     path[i] = 0;
            //     if (res != FR_OK) break;
            // } 
            // else {                                       
            // 
            // printf("%s/%s\n", path, fn);
            sprintf(full_path, "%s/%s", path, fn);         
            printf("%s\n", full_path);
            
            strncpy(timeref, fn, 15);						/* cut the time part of the file name */
            timeref[15] = 0;
            // printf("%s\n", timeref);

		    strncpy(t_year , &timeref[0], 4);
		    t_year[4] = 0;
		    strncpy(t_mon  , &timeref[4], 2);
		    strncpy(t_mday , &timeref[6], 2);
		    strncpy(t_hour , &timeref[9], 2);
		    strncpy(t_min , &timeref[11], 2);
		    strncpy(t_sec , &timeref[13], 2);

	        struct tm t;
		    time_t t_of_day;
		  
		    t.tm_year = atoi(t_year) - 1900;
		    t.tm_mon = atoi(t_mon) - 1;   			    // Month, 0 - jan
		    t.tm_mday = atoi(t_mday);          			// Day of the month
		    t.tm_hour = atoi(t_hour);     
		    t.tm_min = atoi(t_min);
		    t.tm_sec = atoi(t_sec);
		    t.tm_isdst = -1;       						// Is DST on? 1 = yes, 0 = no, -1 = unknown

		    t_of_day = mktime(&t);						/* Construct struct time to epoch seconds */
		    t_of_day -= 946684800;
			
			ctime(&t_of_day);
		    printf("epoch: %"PRIu32"\n", t_of_day );
		    // printf("OBC time is: %s\r\n", ctime(&t_of_day));
	    	
		    switch(mode){
		    	case 1:
		    		if (timeRec_T1 < (unsigned)t_of_day && timeRec_T2 > (unsigned)t_of_day){
		    			printf("mode = 1 delete \n");
		    			if (strcmp(path, "0:/HK_DATA") == 0){
							hk_delete(full_path);
		    			}
		    			else if (strcmp(path, "0:/INMS_DATA") == 0){
		    				inms_data_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/SEUV_DATA") == 0){
		    				seuv_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/EOP_DATA") == 0){
		    				eop_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/WOD_DATA") == 0){
		    				wod_delete(full_path);	
		    			}
		    		}
		    		
		    		break;
	    		case 2:
    			   	if (timeRec_T1 > (unsigned)t_of_day){
    			   		printf("mode = 2 delete \n");
    			   		if (strcmp(path, "0:/HK_DATA") == 0){
							hk_delete(full_path);
		    			}
		    			else if (strcmp(path, "0:/INMS_DATA") == 0){
		    				inms_data_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/SEUV_DATA") == 0){
		    				seuv_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/EOP_DATA") == 0){
		    				eop_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/WOD_DATA") == 0){
		    				wod_delete(full_path);	
		    			}		    			
    			   	}
    			   
	    			break;
	    		case 3:
	    			if (timeRec_T1 < (unsigned)t_of_day){
    			   		printf("mode = 3 delete \n");
    			   		if (strcmp(path, "0:/HK_DATA") == 0){
							hk_delete(full_path);
		    			}
		    			else if (strcmp(path, "0:/INMS_DATA") == 0){
		    				inms_data_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/SEUV_DATA") == 0){
		    				seuv_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/EOP_DATA") == 0){
		    				eop_delete(full_path);	
		    			}
		    			else if (strcmp(path, "0:/WOD_DATA") == 0){
		    				wod_delete(full_path);	
		    			}
    			   	}
	    			break;
	    		default:
	    			printf("Range error Type 1 ~ 3\n");
	    			break;
		    }
        }
    }
    return res;
}