#include <stdint.h>
#include <string.h>
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

#define maxlength 	20
#define maxNum		50
FATFS fs[2];
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

	if (schedule_read_flash(sche_buf) == Error)
		return Error;

	int lastNum = findMaxBuf(sche_buf);
	printf("%d\n", lastNum);
	txlen = 20;
	for (int i = 0 ; i < lastNum ; i++) {
		SendPacketWithCCSDS_AX25(&sche_buf[0 + 20 * i], txlen, obc_apid, 11, 17);
	}
	// if (lastNum != 0) {
	// 	hex_dump(sche_buf, maxlength * lastNum);
	// }
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

	if (schedule_read_flash(sche_buf) == Error)
		return Error;
	else {
		lastNum = findMaxBuf(sche_buf);
	}
	printf("lastNum = %d\n", lastNum);
	memcpy(&shift_time, frameCont, 4);
	shift_time = csp_ntoh32(shift_time);
	printf("in shift time = %ld\n", shift_time);

	for (int i = 0 ; i < lastNum ; i++) {
		memcpy(&sche_time[i], &sche_buf[i * 20 + 2], 4);
		sche_time[i] = csp_ntoh32(sche_time[i]);

		sche_time[i] += shift_time ;
		printf("%d %" PRIu32 "\n", i, sche_time[i]);
		sche_time[i] = csp_ntoh32(sche_time[i]);
		memcpy(&sche_buf[i * 20 + 2], &sche_time[i], 4);
	}

	if (schedule_reset_flash() == 1) {
		return Error;
	}
	else {
		printf("reset success !! \n");
	}
	for (int i = 0 ; i < lastNum ; i++) {
		// printf("%d\n", sizeof(sche_buf[i]));
		if (schedule_write_flash(&sche_buf[1 + 20 * i]) == Error)	//??
			return Error;
		else {
			schedule_new_command_flag = 1 ;
		}
	}
	return No_Error;
}
int schedule_delete(int range, uint8_t * frameCont)
{
	uint8_t sche_buf[maxNum * maxlength] = {0};
	uint8_t sche_update[maxNum * maxlength] = {0};
	int sche_delete_record[maxNum] = {0};
	schedule_read_flash(sche_buf);
	int lastNum = findMaxBuf(sche_buf);
	int update_series = 0;
	uint32_t sche_time[maxNum] = { 0 };
	uint32_t time_t1 = { 0 };
	uint32_t time_t2 = { 0 };

	memcpy(&time_t1, &frameCont[10], 4);

	if (range == 1) {
		memcpy(&time_t2, &frameCont[15], 4);
	}
	time_t1 = csp_ntoh32(time_t1);
	time_t2 = csp_ntoh32(time_t2);
	printf("t1 : %" PRIu32 "\n", time_t1);
	printf("t2 : %" PRIu32 "\n", time_t2);

	printf("last num = %d \n", lastNum);
	for (int i = 0 ; i < lastNum ; i++) {
		memcpy(&sche_time[i], &sche_buf[i * 20 + 2], 4);

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

	for (int i = 0 ; i < lastNum ; i++ ) {
		printf("delete record: %d\n", sche_delete_record[i]);
	}
	for (int i = 0 ; i < update_series ; i++ ) {
		hex_dump(&sche_update[19 * i], 19);
	}

	if (schedule_reset_flash() != 1)
		printf("schedule reset success\n");
	for (int i = 0 ; i < update_series ; i++) {
		if (schedule_write_flash(&sche_update[19 * i]) == Error)
			return Error;
	}

	return No_Error;

}
int schedule_reset() {

	char fileName[] = "0:/OnB_Sch.bin";
	res = f_unlink(fileName);

	parameters.schedule_series_number = 0 ;
	para_w_flash();
	schedule_unlink_flag = 1;
	if (res != FR_OK) {
		printf("\r\nschedule file f_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\nschedule file f_unlink() success .. \r\n");
		return No_Error;
	}
}
int schedule_write(uint8_t frameCont[])
{
	char fileName[] = "0:/OnB_Sch.bin";
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
	res = f_write(&file, frameCont, 19, &bw);
	if (res != FR_OK) {
		printf("\r\nOn Board Schedule write() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		printf("\r\nOn Board Schedule write() success .. \r\n");
		f_close(&file);
		parameters.schedule_series_number ++;
		para_w_flash();
		return No_Error;
	}
}

int schedule_read(uint8_t * txbuf)
{

	char fileName[] = "0:/OnB_Sch.bin";
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\nf_open() fail .. \r\n");
	}
	else {
		printf("\r\nf_open() success .. \r\n");
	}

	int c = 0 ;
	while (1) {
		res = f_read(&file, &buffer, 1, &br);
		memcpy(&txbuf[c++], &buffer, 1);
		if (f_eof(&file)) {break;}
	}
	printf("c = %d\n", c);
	if (res != FR_OK) {
		printf("\r\nschedule_read() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		printf("\r\nschedule_read() success .. \r\n");
		f_close(&file);
		return No_Error;
	}
}
int schedule_read_flash(uint8_t * txbuf) {

	char * buf;
	struct stat stat_buf;
	size_t size;

	char path[] = "/boot/OnB_Sch.bin";

	/* Open file */
	FILE * fp = fopen(path, "r");
	if (!fp) {
		printf("read: cannot open %s\r\n", path);
	}

	/* Read file size */
	if (stat(path, &stat_buf) != 0) {
		printf("read: cannot stat %s\r\n", path);
		goto err_stat;
	}

	size = stat_buf.st_size;

	/* Allocate buffer */
	buf = pvPortMalloc(size);
	if (!buf) {
		printf("read: cannot allocate memory for %s\r\n", path);
		goto err_stat;
	}

	/* Read file */
	if (fread(buf, 1, size, fp) != size) {
		printf("read: failed to read %zu bytes from %s\r\n", size, path);
		goto err;
	}

	/* Finally, hexdump */
	printf("Success read from OnB_Sch.bin\n");
	hex_dump(buf, size);
	memcpy(txbuf, buf, size);
	fclose(fp);
	return No_Error;
err:
	vPortFree(buf);
	return Error;
err_stat:
	fclose(fp);
	return Error;
}
int schedule_write_flash(uint8_t frameCont[]) {
	int fd, bytes;
	/* Test */
	char snumber[] = "0";
	sprintf(snumber, "%d", parameters.schedule_series_number);

	int size = 19;

	char path[] = "/boot/OnB_Sch.bin";

	/* Open file */
	fd = open(path, O_CREAT | O_RDWR | O_APPEND);
	if (fd < 0) {
		printf("Failed to open %s\r\n", path);
		goto err;
	}
	/* write Data into flash */

	bytes = write(fd, snumber, 1);

	bytes = write(fd, frameCont, 19);

	if (bytes != size) {
		printf("Failed to write test data to %s (wrote %d bytes)\r\n", path, bytes);
		goto err;
	}
	else {
		printf("Success write into OnB_Sch.bin\n");
		parameters.schedule_series_number ++;
		para_w_flash();
	}
	close(fd);
	return No_Error;

err:
	return Error;
}
int schedule_reset_flash() {

	struct stat st;
	int ret;

	/* Get args */
	char path[] = "/boot/OnB_Sch.bin";

	if (stat(path, &st) < 0) {
		printf("rm: cannot stat %s\r\n", path);
	}

	if (st.st_mode & S_IFDIR)
		ret = rmdir(path);
	else
		ret = remove(path);

	if (ret != 0) {
		printf("rm: cannot remove %s\r\n", path);
		return Error;
	}
	else {
		parameters.schedule_series_number = 0 ;
		para_w_flash();
		schedule_unlink_flag = 1;
		return No_Error;
	}
}

/** End of Schedule related FS function*/
/*  ---------------------------------------------------  */

/*  ---------------------------------------------------  */
/** Start of INMS data related FS function*/
void inms_data_write_dup(uint8_t frameCont[])
{
	uint8_t frame[inms_data_length];
	memcpy(frame, frameCont, inms_data_length);
	inms_data_write(frame, 0);
	// vTaskDelay(0.1 * delay_time_based);
	inms_data_write(frame, 1);
}
int inms_data_write(uint8_t frameCont[], int SD_partition)
{
	struct tm  ts;
	char buf[20];

	char fileName[45];
	if (SD_partition == 0)
		strcpy(fileName, "0:INMS_DATA/");
	else
		strcpy(fileName, "1:INMS_DATA/");

	/* Get current time */
	timestamp_t t;
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


	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\nf_open() fail .. \r\n");
	}
	//將pointer指向文件最後面
	f_lseek(&file, file.fsize);

	res = f_write(&file, frameCont, inms_data_length, &bw);

	// hex_dump(frameCont, inms_data_length);
	if (res != FR_OK) {
		printf("\rinms_write() %d fail .. \r\n", SD_partition);
		f_close(&file);
		return Error;
	}
	else {
		printf("\rinms_write() %d success .. \r\n", SD_partition);
		f_close(&file);
		return No_Error;
	}
}

int inms_data_read(char fileName[], void * txbuf) {

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\nf_open() fail .. \r\n");
	} else {
		printf("\r\nf_open() success .. \r\n");
	}
	int c = 0 ;
	while (1) {
		res = f_read(&file, &buffer, 1, &br);
		memcpy(txbuf + (c++), &buffer, 1);
		// c++;
		if (f_eof(&file)) {break;}
	}
	// printf("%d\n", c);
	if (res != FR_OK) {
		printf("\r\ninmsdata_read() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		printf("\r\ninmsdata_read() success .. \r\n");
		f_close(&file);
		return No_Error;
	}
}

int inms_data_delete(char fileName[]) {

	res = f_unlink(fileName);

	if (res != FR_OK)
	{
		printf("%s\r\nf_unlink() fail .. \r\n", fileName);
		return Error;
	}
	else {
		printf("%s\r\nf_unlink() success .. \r\n", fileName);
		return No_Error;
	}
}

int inms_script_write_flash(int buffNum, uint8_t scriptCont[], int delete_flag, int size) {

	int fd, bytes, ret;
	struct stat st;
	char path[22];

	if (buffNum == 0)
		strcpy(path, "/boot/INMS/idle0.bin");
	else if (buffNum == 1)
		strcpy(path, "/boot/INMS/idle1.bin");
	else if (buffNum == 2)
		strcpy(path, "/boot/INMS/idle2.bin");
	else if (buffNum == 3)
		strcpy(path, "/boot/INMS/idle3.bin");
	else if (buffNum == 4)
		strcpy(path, "/boot/INMS/idle4.bin");
	else if (buffNum == 5)
		strcpy(path, "/boot/INMS/idle5.bin");
	else if (buffNum == 6)
		strcpy(path, "/boot/INMS/idle6.bin");
	else if (buffNum == 7)
		strcpy(path, "/boot/INMS/idle7.bin");
	else if (buffNum == 8)
		strcpy(path, "/boot/INMS/idle8.bin");

	if (delete_flag == 1) {
		if (stat(path, &st) < 0) {
			printf("rm: cannot stat %s\r\n", path);
		}

		if (st.st_mode & S_IFDIR)
			ret = rmdir(path);
		else
			ret = remove(path);

		if (ret != 0) {
			printf("rm: cannot remove %s\r\n", path);
		}		
	}

	/* Open file */
	fd = open(path, O_CREAT | O_APPEND | O_RDWR);
	if (fd < 0) {
		printf("Failed to open %s\r\n", path);
	}
	/* write Data into flash */
	bytes = write(fd, scriptCont, size);

	if (bytes != size) {
		printf("Failed to write test data to %s (wrote %d bytes)\r\n", path, bytes);
		goto err;
	}
	else {
		printf("Success write into %s\n", path);
	}
	close(fd);
	return No_Error;

err:
	close(fd);
	return Error;
}
int inms_script_length_flash(int buffNum) {

	char * buf;
	struct stat stat_buf;
	size_t size;
	char path[22];
	int packlength = 0;

	if (buffNum == 0)
		strcpy(path, "/boot/INMS/idle0.bin");
	else if (buffNum == 1)
		strcpy(path, "/boot/INMS/idle1.bin");
	else if (buffNum == 2)
		strcpy(path, "/boot/INMS/idle2.bin");
	else if (buffNum == 3)
		strcpy(path, "/boot/INMS/idle3.bin");
	else if (buffNum == 4)
		strcpy(path, "/boot/INMS/idle4.bin");
	else if (buffNum == 5)
		strcpy(path, "/boot/INMS/idle5.bin");
	else if (buffNum == 6)
		strcpy(path, "/boot/INMS/idle6.bin");
	else if (buffNum == 7)
		strcpy(path, "/boot/INMS/idle7.bin");
	else if (buffNum == 8)
		strcpy(path, "/boot/INMS/idle8.bin");

	/* Open file */
	FILE * fp = fopen(path, "r");
	if (!fp) {
		printf("read: cannot open %s\r\n", path);
		goto err_stat;
	}

	/* Read file size */
	if (stat(path, &stat_buf) != 0) {
		printf("read: cannot stat %s\r\n", path);
		goto err_stat;
	}

	size = 2;

	/* Allocate buffer */
	buf = pvPortMalloc(size);
	if (!buf) {
		printf("read: cannot allocate memory for %s\r\n", path);
	}

	/* Read file */
	if (fread(buf, 1, size, fp) != size) {
		printf("read: failed to read %zu bytes from %s\r\n", size, path);
	}

	/* Finally, hexdump */
	if (buf[0] != 0)
		packlength = buf[0] + (buf[1] << 8);
	else
		packlength = 0;

	fclose(fp);
	return packlength;


err_stat:
	fclose(fp);
	return 0;

}
int inms_script_read_flash(int buffNum, int packlength, void * txbuf) {

	char * buf;
	struct stat stat_buf;
	size_t size;
	char path[22];

	if (buffNum == 0)
		strcpy(path, "/boot/INMS/idle0.bin");
	else if (buffNum == 1)
		strcpy(path, "/boot/INMS/idle1.bin");
	else if (buffNum == 2)
		strcpy(path, "/boot/INMS/idle2.bin");
	else if (buffNum == 3)
		strcpy(path, "/boot/INMS/idle3.bin");
	else if (buffNum == 4)
		strcpy(path, "/boot/INMS/idle4.bin");
	else if (buffNum == 5)
		strcpy(path, "/boot/INMS/idle5.bin");
	else if (buffNum == 6)
		strcpy(path, "/boot/INMS/idle6.bin");
	else if (buffNum == 7)
		strcpy(path, "/boot/INMS/idle7.bin");
	else if (buffNum == 8)
		strcpy(path, "/boot/INMS/idle8.bin");
	
	/* Open file */
	FILE * fp = fopen(path, "r");
	if (!fp) {
		printf("read: cannot open %s\r\n", path);
		goto err_stat;
	}

	/* Read file size */
	if (stat(path, &stat_buf) != 0) {
		printf("read: cannot stat %s\r\n", path);
		goto err_stat;
	}

	size = stat_buf.st_size;

	/* Allocate buffer */
	buf = pvPortMalloc(size);
	if (!buf) {
		printf("read: cannot allocate memory for %s\r\n", path);
		goto err_stat;
	}

	/* Read file */
	if (fread(buf, 1, size, fp) != size) {
		printf("read: failed to read %zu bytes from %s\r\n", size, path);
		goto err;
	}

	/* Finally, hexdump */
	printf("Success read from Script %d\n", buffNum);
	// hex_dump(buf, size);
	memcpy(txbuf, buf, size);

	fclose(fp);
	return No_Error;
err:
	vPortFree(buf);
	return Error;
err_stat:
	fclose(fp);
	return Error;
}
int inms_script_delete_flash(int buffNum) {

	struct stat st;
	int ret;
	char path[22];
	/* Get args */
	if (buffNum == 0)
		strcpy(path, "/boot/INMS/idle0.bin");
	else if (buffNum == 1)
		strcpy(path, "/boot/INMS/idle1.bin");
	else if (buffNum == 2)
		strcpy(path, "/boot/INMS/idle2.bin");
	else if (buffNum == 3)
		strcpy(path, "/boot/INMS/idle3.bin");
	else if (buffNum == 4)
		strcpy(path, "/boot/INMS/idle4.bin");
	else if (buffNum == 5)
		strcpy(path, "/boot/INMS/idle5.bin");
	else if (buffNum == 6)
		strcpy(path, "/boot/INMS/idle6.bin");
	else if (buffNum == 7)
		strcpy(path, "/boot/INMS/idle7.bin");
	else if (buffNum == 8)
		strcpy(path, "/boot/INMS/idle8.bin");

	if (stat(path, &st) < 0) {
		printf("rm: cannot stat %s\r\n", path);
	}

	if (st.st_mode & S_IFDIR)
		ret = rmdir(path);
	else
		ret = remove(path);

	if (ret != 0) {
		printf("rm: cannot remove %s\r\n", path);
	}
	else {
		printf("rm: success remove %s\r\n", path);
	}
	return No_Error;
}
/** End of INMS data/script related FS function*/
/*  ---------------------------------------------------  */
/** Start of WOD data related FS function*/
void wod_write_dup(uint8_t frameCont[])
{
	uint8_t frame[wod_length];
	memcpy(frame, frameCont, wod_length);
	wod_write(frame, 0);
	wod_write(frame, 1);
}
int wod_write(uint8_t * frameCont, int SD_partition )
{
	struct tm  ts;
	char buf[20];
	char fileName[45];
	if (SD_partition == 0)
		strcpy(fileName, "0:/WOD_DATA/");
	else
		strcpy(fileName, "1:/WOD_DATA/");

	timestamp_t t;
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
		printf("\r\nf_open() fail .. \r\n");
	}

	f_lseek(&file, file.fsize);
	res = f_write(&file, frameCont, wod_length, &bw);

	if (res != FR_OK) {
		printf("\r\nwod_write() %d fail .. \r\n", SD_partition);
		f_close(&file);
		return Error;
	}
	else {
		printf("\r\nwod_write() %d success .. \r\n", SD_partition);

		f_close(&file);
		return No_Error;
	}
}

int wod_read(char fileName[], void * txbuf) // serial =1~N
{
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\nf_open() fail .. \r\n");
	}
	else {
		printf("\r\nf_open() success .. \r\n");
	}
	f_lseek(&file, file.fsize);
	res = f_read(&file, &buffer, wod_length, &br);

	if (res != FR_OK) {
		printf("\r\nwod_read() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		memcpy(txbuf, &buffer, wod_length);
		f_close(&file);
		return No_Error;
	}
}

int wod_delete(char filename[])
{
	res = f_unlink(filename);	  

	if (res != FR_OK) {
		printf("\r\nf_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\nf_unlink() success .. \r\n");
		return No_Error;
	}
}

/** End of WOD data related FS function*/
/*  ---------------------------------------------------  */
/** Start of SEUV data related FS function*/
void seuv_write_dup()
{	
	seuv_write(0);
	seuv_write(1);
}
int seuv_write(int SD_partition)
{
	struct tm  ts;
	char buf[20];
	
	char fileName[45];
	if (SD_partition == 0)
		strcpy(fileName, "0:/SEUV_DATA/");
	else
		strcpy(fileName, "1:/SEUV_DATA/");

	timestamp_t t;
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

	seuvFrame.packettime = t.tv_sec;
	// printf("sample = %d\n", seuvFrame.samples);

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
		printf("SEUV  f_open fail!!\n");
	else
		printf("SEUV f_open success \n");

	// printf("%d\n", (int)sizeof(seuv_frame_t) );
	hex_dump(&seuvFrame, seuv_length);
	f_lseek(&file, file.fsize);
	res = f_write(&file, &seuvFrame, (int)sizeof(seuv_frame_t), &bw);

	if (res != FR_OK) {
		printf("SEUV  f_write %d fail!!\n", SD_partition);
		f_close(&file);
		return Error;
	}
	else {
		printf("SEUV f_write %d success\n", SD_partition);
		f_close(&file);
		return No_Error;
	}
}

int seuv_read(char fileName[], void * txbuf)
{
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\nf_open() fail .. \r\n");
	}
	else {
		printf("\r\nf_open() success .. \r\n");
	}
	res = f_read(&file, &buffer, seuv_length, &br);

	if (res != FR_OK) {
		printf("\r\nseuv_read() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		printf("\r\nseuv_read() success .. \r\n");
		memcpy(txbuf, &buffer, seuv_length);
		f_close(&file);
		return No_Error;
	}
}
int seuv_delete(char fileName[])
{
	res = f_unlink(fileName);

	if (res != FR_OK) {
		printf("\r\nf_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\nf_unlink() success .. \r\n");
		return No_Error;
	}
}
/** End of SEUV data related FS function*/
/*  ---------------------------------------------------  */
/** Start of HK data related FS function*/
void hk_write_dup(uint8_t * frameCont)
{
	uint8_t frame[hk_length];
	memcpy(frame, frameCont, hk_length);
	hk_write(frame, 0);
	hk_write(frame, 1);
}
int hk_write(uint8_t * frameCont, int SD_partition)
{

	struct tm  ts;
	char buf[20];

	char fileName[45];
	if (SD_partition == 0)
		strcpy(fileName, "0:/HK_DATA/");
	else
		strcpy(fileName, "1:/HK_DATA/");

	// Get current time
	timestamp_t t;
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
		printf("\r\nhk_write() %d fail .. \r\n", SD_partition);
		f_close(&file);
		return Error;
	}
	else {
		printf("\r\nhk_write() %d success .. \r\n", SD_partition);
		f_close(&file);
		return No_Error;
	}
}

int hk_read(char fileName[], void * txbuf) // serial =1~N
{
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\nf_open() fail .. \r\n");
	}
	else {
		printf("\r\nf_open() success .. \r\n");
	}
	res = f_read(&file, &buffer, hk_length, &br);

	if (res != FR_OK) {
		printf("\r\nhk_read() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		memcpy(txbuf, &buffer, hk_length);
		f_close(&file);
		return No_Error;
	}
}

int hk_delete(char fileName[])
{
	res = f_unlink(fileName);	  

	if (res != FR_OK) {
		printf("\r\nf_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\nf_unlink() success .. \r\n");
		return No_Error;
	}
}

/** End of HK data related FS function*/
/*  ---------------------------------------------------  */
/** Start of EOP data related FS function*/
void eop_write_dup(uint8_t frameCont []) 
{
	uint8_t frame[eop_length];
	memcpy(frame, frameCont, eop_length);
	eop_write(frame, 0);
	eop_write(frame, 1);
}

int eop_write(uint8_t *frameCont, int SD_partition)		//SD_partition available : 0 & 1
{
	struct tm  ts;
	char buf[20];
	char fileName[45];
	char s[] = "";

	if (SD_partition == 0)
		strcpy(s, "0:/EOP_DATA/");
	else
		strcpy(s, "1:/EOP_DATA/");

	strcpy(fileName, s);
	/* Get current time */
	timestamp_t t;
	t.tv_sec = 0;
	t.tv_nsec = 0;
	obc_timesync(&t, 6000);
	memcpy(&frameCont[0], &t.tv_sec, 4);
	time_t tt = t.tv_sec;
	time(&tt);
	/* Format time, "ddd yyyy-mm-dd hh:mm:ss zzz" */
	ts = *localtime(&tt);
	strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &ts);

	strcat(fileName, buf);
	strcat(fileName, "_EOP_TW01");
	strcat(fileName, ".dat");
	printf("%s\n", fileName);

	

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_WRITE );
	f_lseek(&file, file.fsize);
	res = f_write(&file, frameCont, eop_length, &bw);
	if (res != FR_OK) {
		printf("\rEOP_write() %d fail .. \r\n", SD_partition);
		f_close(&file);
		return Error;
	}
	else {
		printf("\rEOP_write() %d success .. \r\n", SD_partition);
		f_close(&file);
		// if (SD_partition == 1)
		// 	hex_dump(frameCont, eop_length);
		return No_Error;
	}

}

int eop_read(char fileName[], void * txbuf)
{

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ );
	if (res != FR_OK) {
		printf("\r\nf_open() fail .. \r\n");
	}
	else {
		printf("\r\nf_open() success .. \r\n");
	}
	res = f_read(&file, &buffer, eop_length, &br);

	if (res != FR_OK) {
		printf("\r\neop_read() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		memcpy(txbuf, &buffer, eop_length);
		f_close(&file);
		return No_Error;
	}
}

int eop_delete(char fileName[])
{
	res = f_unlink(fileName);

	if (res != FR_OK) {
		printf("\r\nf_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\nf_unlink() success .. \r\n");
		return No_Error;
	}
}
/*  ---------------------------------------------------  */
/** Start of parameter related FS function*/
void para_d_flash() {
	struct stat st;
	int ret;

	/* Get args */
	char path[] = "/boot/para.bin";

	if (stat(path, &st) < 0) {
		printf("rm: cannot stat %s\r\n", path);
	}

	if (st.st_mode & S_IFDIR)
		ret = rmdir(path);
	else
		ret = remove(path);

	if (ret != 0) {
		printf("rm: cannot remove %s\r\n", path);
	}
}
void para_w_flash() {
	int fd, bytes;
	int size = (int)sizeof(parameter_t);

	char path[] = "/boot/para.bin";

	/* Open file */
	fd = open(path, O_CREAT | O_TRUNC | O_RDWR);
	if (fd < 0) {
		printf("Failed to open %s\r\n", path);
	}
	/* write Data into flash */
	bytes = write(fd, &parameters.first_flight, size);

	if (bytes != size) {
		printf("Failed to write test data to %s (wrote %d bytes)\r\n", path, bytes);
	}
	else {
		printf("Success write into para.bin\n");
	}
	close(fd);
}
int para_r_flash() {

	char * buf;
	struct stat stat_buf;
	size_t size;

	char path[] = "/boot/para.bin";

	/* Open file */
	FILE * fp = fopen(path, "r");
	if (!fp) {
		printf("read: cannot open %s\r\n", path);
	}

	/* Read file size */
	if (stat(path, &stat_buf) != 0) {
		printf("read: cannot stat %s\r\n", path);
		goto err_stat;
	}

	size = stat_buf.st_size;

	/* Allocate buffer */
	buf = pvPortMalloc(size);
	if (!buf) {
		printf("read: cannot allocate memory for %s\r\n", path);
		goto err_stat;
	}

	/* Read file */
	if (fread(buf, 1, size, fp) != size) {
		printf("read: failed to read %zu bytes from %s\r\n", size, path);
		goto err;
	}

	/* Finally, hexdump */
	printf("Success read from para.bin\n");
	// hex_dump(buf, size);
	memcpy(&parameters.first_flight, buf, size);
	fclose(fp);
	return No_Error;
err:
	vPortFree(buf);
	return Error;
err_stat:
	fclose(fp);
	return Error;
}
/** End of parameter related FS function*/
/*  ---------------------------------------------------  */
/** Start of ADCS parameter related FS function*/

int adcs_para_r() {	

	char fileName[] = "0:/adcs_para.bin";
	res = f_open(&file, fileName, FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\nf_open() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\nf_open() success .. \r\n");
		res = f_read(&file, &buffer, (int)sizeof(adcs_para_t), &br);
	}

	if (res != FR_OK) {
		printf("\r\nwod_read() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		f_close(&file);
		memcpy(&adcs_para.strategy, &buffer, (int)sizeof(adcs_para_t)); //import
		hex_dump(&buffer, (int)sizeof(adcs_para_t));
		return No_Error;
	}
}

int adcs_para_w()
{	
	char fileName[] = "0:/adcs_para.bin";
	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK) {
		printf("\r\nf_open() fail .. \r\n");
	}
	else {
		printf("\r\nf_write open() success .. \r\n");
	}

	res = f_write(&file, &adcs_para.strategy, (int)sizeof(adcs_para_t), &br);

	if (res != FR_OK) {
		printf("\r\npara write() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		f_close(&file);
		return No_Error;
	}
}

int adcs_para_d()
{	
	char fileName[] = "0:/adcs_para.bin";
	res = f_unlink(fileName);	 

	if (res != FR_OK) {
		printf("\r\nf_unlink() fail .. \r\n");
		return Error;
	}
	else {
		printf("\r\nf_unlink() success .. \r\n");
		return No_Error;
	}
}

/** End of ADCS parameter related FS function*/
/*  ---------------------------------------------------  */
/** Start of thermal related FS function*/

int thurmal_1_w()
{	// serial =1~N
	char fileName[] = "0:/t_obc.bin";

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

	if (res != FR_OK) {
		printf("\r\n@@ %d @@ \r\n", res);
		printf("\r\nf_open() fail .. \r\n");
	}
	else {
		//printf("\r\nf_open() success .. \r\n");
	}

	f_lseek(&file, file.fsize);
	res = f_write(&file, &ThermalFrame.packet_number, (int)sizeof(thermal_frame_t), &br);

	if (res != FR_OK) {
		printf("\r\nthurmal_1_w write() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		f_close(&file);
		return No_Error;
	}

	return No_Error;
}

int thurmal_2_w()
{	// serial =1~N
	char fileName[] = "0:/t_inms.bin";

	res = f_open(&file, fileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
	if (res != FR_OK)
	{
		printf("\r\n@@ %d @@ \r\n", res);
		printf("\r\nf_open() fail .. \r\n");
	}
	else {

		//printf("\r\nf_open() success .. \r\n");
	}
	f_lseek(&file, file.fsize);
	res = f_write(&file, &Tdata[0], 178, &br);

	if (res != FR_OK) {
		printf("\r\nthurmal_2_w write() fail .. \r\n");
		f_close(&file);
		return Error;
	}
	else {
		f_close(&file);
		return No_Error;
	}

	return No_Error;
}

int T_data_d()
{	// serial =1~N

	char fileName[] = "0:/t_obc.bin";
	res = f_unlink(fileName);

	if (res != FR_OK)
	{
		printf("\r\nt_obc.bin f_unlink() fail .. \r\n");

	}
	else {
		printf("\r\nt_obc.bin f_unlink() success .. \r\n");

	}


	char fileName2[] = "0:/t_inms.bin";
	res = f_unlink(fileName2);

	if (res != FR_OK)
	{
		printf("\r\nt_inms.bin f_unlink() fail .. \r\n");

	}
	else {
		printf("\r\nt_inms.bin f_unlink() success .. \r\n");

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
	char t_year[5] = "0";
	char t_mon[2] = {0};
	char t_mday[2] = {0};
	char t_hour[2] = {0};
	char t_min[2] = {0};
	char t_sec[2] = {0};
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
			// printf("epoch: %"PRIu32"\n", t_of_day );
			// printf("OBC time is: %s\r\n", ctime(&t_of_day));

			switch (mode) {
			case 1:
				if (timeRec_T1 < (unsigned)t_of_day && timeRec_T2 > (unsigned)t_of_day) {
					printf("mode = 1 down link \n");
					if (strcmp(path, fileName_HK[parameters.SD_partition_flag]) == 0) {
						hk_read(full_path, hk_data);
						hex_dump(&hk_data, hk_length);
						SendDataWithCCSDS_AX25(1, &hk_data[0]);
					}
					else if (strcmp(path, fileName_INMS[parameters.SD_partition_flag]) == 0) {
						inms_data_read(full_path, inms_data);
						SendDataWithCCSDS_AX25(2, &inms_data[0]);
						memcpy(&inms_2nd , &inms_data[196], 4);
						if (inms_2nd > 0) {
							SendDataWithCCSDS_AX25(2, &inms_data[196]);
							for (int i = 0; i < 196; ++i) {
								inms_data[i+196] = 0;
							}
						}
					}
					else if (strcmp(path, fileName_SEUV[parameters.SD_partition_flag]) == 0) {
						seuv_read(full_path, seuv_data);
						hex_dump(&seuv_data, seuv_length);
						SendDataWithCCSDS_AX25(3, &seuv_data[0]);
					}
					else if (strcmp(path, fileName_EOP[parameters.SD_partition_flag]) == 0) {
						eop_read(full_path, eop_data);
						hex_dump(&eop_data, eop_length);
						SendDataWithCCSDS_AX25(4, &eop_data[0]);
					}
					else if (strcmp(path, fileName_WOD[parameters.SD_partition_flag]) == 0) {
						wod_read(full_path, wod_data);
						hex_dump(&wod_data, wod_length);
						SendDataWithCCSDS_AX25(5, &wod_data[0]);
					}
					// SendPacketWithCCSDS_AX25(&beacon_frame.mode, 8, obc_apid, 0, 0);
					vTaskDelay(0.5 * delay_time_based);
				}
				break;
			case 2:
				if (timeRec_T1 > (unsigned)t_of_day) {
					printf("mode = 2 down link \n");
					if (strcmp(path, fileName_HK[parameters.SD_partition_flag]) == 0) {
						hk_read(full_path, hk_data);
						hex_dump(&hk_data, hk_length);
						SendDataWithCCSDS_AX25(1, &hk_data[0]);
					}
					else if (strcmp(path, fileName_INMS[parameters.SD_partition_flag]) == 0) {
						inms_data_read(full_path, inms_data);
						SendDataWithCCSDS_AX25(2, &inms_data[0]);
						memcpy(&inms_2nd , &inms_data[196], 4);
						if (inms_2nd > 0) {
							SendDataWithCCSDS_AX25(2, &inms_data[196]);
							for (int i = 0; i < 196; ++i) {
								inms_data[i+196] = 0;
							}
						}
					}
					else if (strcmp(path, fileName_SEUV[parameters.SD_partition_flag]) == 0) {
						seuv_read(full_path, seuv_data);
						hex_dump(&seuv_data, seuv_length);
						SendDataWithCCSDS_AX25(3, &seuv_data[0]);
					}
					else if (strcmp(path, fileName_EOP[parameters.SD_partition_flag]) == 0) {
						eop_read(full_path, eop_data);
						hex_dump(&eop_data, eop_length);
						SendDataWithCCSDS_AX25(4, &eop_data[0]);
					}
					else if (strcmp(path, fileName_WOD[parameters.SD_partition_flag]) == 0) {
						inms_data_read(full_path, wod_data);
						hex_dump(&wod_data, wod_length);
						SendDataWithCCSDS_AX25(5, &wod_data[0]);
					}
					vTaskDelay(0.5 * delay_time_based);
				}
				break;
			case 3:
				if (timeRec_T1 < (unsigned)t_of_day) {
					printf("mode = 3 down link \n");
					if (strcmp(path, fileName_HK[parameters.SD_partition_flag]) == 0) {
						hk_read(full_path, hk_data);
						hex_dump(&hk_data, hk_length);
						SendDataWithCCSDS_AX25(1, &hk_data[0]);
					}
					else if (strcmp(path, fileName_INMS[parameters.SD_partition_flag]) == 0) {
						inms_data_read(full_path, inms_data);
						SendDataWithCCSDS_AX25(2, &inms_data[0]);
						memcpy(&inms_2nd , &inms_data[196], 4);
						if (inms_2nd > 0) {
							SendDataWithCCSDS_AX25(2, &inms_data[196]);
							for (int i = 0; i < 196; ++i) {
								inms_data[i+196] = 0;
							}
						}
					}
					else if (strcmp(path, fileName_SEUV[parameters.SD_partition_flag]) == 0) {
						seuv_read(full_path, seuv_data);
						hex_dump(&seuv_data, seuv_length);
						SendDataWithCCSDS_AX25(3, &seuv_data[0]);
					}
					else if (strcmp(path, fileName_EOP[parameters.SD_partition_flag]) == 0) {
						eop_read(full_path, eop_data);
						hex_dump(&eop_data, eop_length);
						SendDataWithCCSDS_AX25(4, &eop_data[0]);
					}
					else if (strcmp(path, fileName_WOD[parameters.SD_partition_flag]) == 0) {
						inms_data_read(full_path, wod_data);
						hex_dump(&wod_data, wod_length);
						SendDataWithCCSDS_AX25(5, &wod_data[0]);
					}
					vTaskDelay(0.5 * delay_time_based);
				}
				break;
			default:
				printf("range error\n");
				break;
			}
			if (abort_transfer_flag == 1) {
				abort_transfer_flag = 0;
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
	char t_year[5] = "0";
	char t_mon[2] = {0};
	char t_mday[2] = {0};
	char t_hour[2] = {0};
	char t_min[2] = {0};
	char t_sec[2] = {0};
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
			// printf("%s\n", full_path);

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
			// printf("epoch: %"PRIu32"\n", t_of_day );
			// printf("OBC time is: %s\r\n", ctime(&t_of_day));

			switch (mode) {
			case 1:
				if (timeRec_T1 < (unsigned)t_of_day && timeRec_T2 > (unsigned)t_of_day) {
					// printf("mode = 1 delete \n");
					if (strcmp(path, fileName_HK[parameters.SD_partition_flag]) == 0) {
						hk_delete(full_path);
					}
					else if (strcmp(path, fileName_INMS[parameters.SD_partition_flag]) == 0) {
						inms_data_delete(full_path);
					}
					else if (strcmp(path, fileName_SEUV[parameters.SD_partition_flag]) == 0) {
						seuv_delete(full_path);
					}
					else if (strcmp(path, fileName_EOP[parameters.SD_partition_flag]) == 0) {
						eop_delete(full_path);
					}
					else if (strcmp(path, fileName_WOD[parameters.SD_partition_flag]) == 0) {
						wod_delete(full_path);
					}
				}

				break;
			case 2:
				if (timeRec_T1 > (unsigned)t_of_day) {
					// printf("mode = 2 delete \n");
					if (strcmp(path, fileName_HK[parameters.SD_partition_flag]) == 0) {
						hk_delete(full_path);
					}
					else if (strcmp(path, fileName_INMS[parameters.SD_partition_flag]) == 0) {
						inms_data_delete(full_path);
					}
					else if (strcmp(path, fileName_SEUV[parameters.SD_partition_flag]) == 0) {
						seuv_delete(full_path);
					}
					else if (strcmp(path, fileName_EOP[parameters.SD_partition_flag]) == 0) {
						eop_delete(full_path);
					}
					else if (strcmp(path, fileName_WOD[parameters.SD_partition_flag]) == 0) {
						wod_delete(full_path);
					}
				}

				break;
			case 3:
				if (timeRec_T1 < (unsigned)t_of_day) {
					// printf("mode = 3 delete \n");
					if (strcmp(path, fileName_HK[parameters.SD_partition_flag]) == 0) {
						hk_delete(full_path);
					}
					else if (strcmp(path, fileName_INMS[parameters.SD_partition_flag]) == 0) {
						inms_data_delete(full_path);
					}
					else if (strcmp(path, fileName_SEUV[parameters.SD_partition_flag]) == 0) {
						seuv_delete(full_path);
					}
					else if (strcmp(path, fileName_EOP[parameters.SD_partition_flag]) == 0) {
						eop_delete(full_path);
					}
					else if (strcmp(path, fileName_WOD[parameters.SD_partition_flag]) == 0) {
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
/* ---------------------------------------------------  */
/* ---------------- Self defined String ----------------*/
char* fileName_HK[2]		= { "0:/HK_DATA",	"1:/HK_DATA"};
char* fileName_INMS[2]		= { "0:/INMS_DATA",	"1:/INMS_DATA"};
char* fileName_SEUV[2]		= { "0:/SEUV_DATA",	"1:/SEUV_DATA"	};
char* fileName_EOP[2]		= { "0:/EOP_DATA",	"1:/EOP_DATA"};
char* fileName_WOD[2]		= { "0:/WOD_DATA", 	"1:/WOD_DATA"	};
/* TODO: Test the function scan is okay or not with these predefined filename*/
