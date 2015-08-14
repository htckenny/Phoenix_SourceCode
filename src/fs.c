#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <util/hexdump.h>
#include "parameter.h"
#include <dev/i2c.h>
#include "Tele_function.h"
#include "subsystem.h"
#include <csp/csp_endian.h>

#define maxlength 	200
#define maxNum		50
FATFS fs;
FRESULT res;
FIL file;
UINT br,bw;
uint8_t buffer[300];
FILINFO *fno;
extern int findMaxBuf(uint8_t sortbuf[][maxlength]);

int schedule_dump()
{
	uint8_t txlen;
	uint8_t sche_buf[maxNum][maxlength];
	int lastNum = findMaxBuf(sche_buf);
	if(schedule_read(&sche_buf[0][0]) == Error)
		return Error;
	for (int i = 0 ; i < lastNum ; i++){
		txlen= 200;
		SendPacketWithCCSDS_AX25(&sche_buf[i][0],txlen,obc_apid,11,17);
	}
	return No_Error;
}
int schedule_shift(uint8_t *frameCont)
{
	// printf("in shift -test2\n");
	uint32_t shift_time = 0 ;
	uint8_t sche_buf[maxNum][maxlength];
	uint32_t sche_time[maxNum] = { 0 };
	int lastNum = 0;

	if(schedule_read(&sche_buf) == Error)
		return Error;
	else{
		// printf("sche_buf = %d\n", sche_buf[1][0]);
		lastNum = findMaxBuf(sche_buf);
		// vTaskDelay(5000);
	}
	printf("lastNum = %d\n", lastNum);
	for (int i = 0 ; i <lastNum ; i++){
		memcpy(&sche_time[i], &sche_buf[i][2], 4);
		// sche_time[i] = (sche_buf[i][1] << 24)
		// 	+ (sche_buf[i][2] << 16)
		// 	+ (sche_buf[i][3] << 8)
		// 	+ (sche_buf[i][4])
		// 	+ (sche_buf[i][5] >> 8);
		// for (int i = 0 ; i < 4 ; i++){
		// 	printf("frameCont[%d] = %d\n", i, frameCont[i]);
		// }
		memcpy(&shift_time, &frameCont[0], 4);
		// printf("s before = %d\n", shift_time);

		// shift_time = (frameCont[0] << 24)
		// 	+ (frameCont[1] << 16)
		// 	+ (frameCont[2] << 8)
		// 	+ (frameCont[3]);

		sche_time[i] += shift_time ;
		
		memcpy(&sche_buf[i][2], &sche_time[i], 4);
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

	if (schedule_reset() == 1){
		return Error;
	}
	else{
		printf("reset success !! \n");
	}
	for(int i = 0 ; i < lastNum ; i++){
		// printf("%d\n", sizeof(sche_buf[i]));
		if (schedule_write(sche_buf[i]+1, sche_buf[i][1]-1) == Error)
			return Error;
		else{
			schedule_new_command_flag = 1 ;			
		}
	}
	return No_Error;
}
int schedule_delete(int range, uint8_t * frameCont)
{
	//f_mount(0, &fs);
	//char fileName[]="0:/OnB_Sch.txt";
	uint8_t sche_buf[maxNum][maxlength];
	uint8_t sche_update[maxNum][maxlength];
	int sche_delete_record[maxNum] = {0};
	schedule_read(&sche_buf);
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
	if (range == 1){
		memcpy(&time_t2, &frameCont[15], 4);
		// time_t2 = (frameCont[15] << 24)
		// 	+ (frameCont[16] << 16)
		// 	+ (frameCont[17] << 8)
		// 	+ (frameCont[18])
		// 	+ (frameCont[19] >> 8);
	}

	for (int i = 0 ; i <lastNum ; i++){
		memcpy(&sche_time[i], &sche_buf[i][1], 4);
		// sche_time[i] = (sche_buf[i][1] << 24)
		// 	+ (sche_buf[i][2] << 16)
		// 	+ (sche_buf[i][3] << 8)
		// 	+ (sche_buf[i][4])
		// 	+ (sche_buf[i][5] >> 8);

		switch (range){
			case 1 :
				// delete command between T1 & T2
				if (sche_time[i] >= time_t1 && sche_time[i] <= time_t2){
					sche_delete_record[i] = -1;
				}
				break;

			case 2 :
				// delete command less than or equals to T1
				if (sche_time[i] <= time_t1){
					sche_delete_record[i] = -1;
				}
				break;
			case 3 :
				// delete command greater than or equals to T1
				if (sche_time[i] >= time_t1){
					sche_delete_record[i] = -1;
				}

				break;
		}
		if (sche_delete_record[i] != -1){
			// copy the content from the old schedule list to new one
			*sche_update[update_series] = *sche_buf[i];
			sche_update[update_series][0] = update_series;
			update_series ++ ;
		}
	}
	if (schedule_reset() != 1)
		printf("schedule reset success\n");
	for(int i = 0 ; i < update_series ; i++){
		if (schedule_write(sche_update[i], 255) == Error)
			return Error;

	}

	return No_Error;

}
int schedule_reset(){

	f_mount(0, &fs);
	char fileName[]="0:/OnB_Sch.txt";
	res = f_unlink(fileName);	  //先刪除
	f_mount(0, NULL);
	schedule_series_number = 0 ;
	schedule_unlink_flag = 1;
	if(res!=FR_OK){
		printf("\r\n schedule file f_unlink() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n schedule file f_unlink() success .. \r\n");
		return No_Error;
	}

}
int schedule_write(uint8_t * frameCont, int length)
{
	f_mount(0, &fs);
	char fileName[] = "0:/OnB_Sch.txt";
	// char nextLine[] = "\n";
	char snumber[]="0";
	
	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	f_lseek(&file,file.fsize);
	// printf("size of snumber : %d\n", sizeof(snumber));
	sprintf(snumber, "%d", schedule_series_number);
	res = f_write(&file, snumber, 1, &bw);
	schedule_series_number ++; 
	for (int i = 0 ; i < length ; i++) {
		printf("%u ", frameCont[i]);
	}
	printf("\nsize of the frame pass into the write function: %d\n", length);
	res = f_write(&file, frameCont, length,&bw);
	// printf("size of nextLine: %d\n", sizeof(nextLine));
	if(res!=FR_OK){
		printf("\r\nOn Board Schedule write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		printf("\r\nOn Board Schedule write() success .. \r\n");
		res = f_write(&file, "\n", 1, &bw);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}
int schedule_read(uint8_t * txbuf[maxlength][maxlength])
{
	// printf("%d", parameters.sche_store_count);
	// char *
	// printf("inside \n");
	uint8_t buf[50][50];
	f_mount(0, &fs);
	char fileName[] = "0:/OnB_Sch.txt";
	int total_line = 0;
	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}
	else{
		printf("\r\n f_open() success .. \r\n");
	}
	// printf("before eof while loop\n");
	int buffer_length = 0;
	int num = 0;
	while(f_eof(&file) == 0 ){
		//max command length need to confirm.
		if (f_gets((TCHAR)&buffer, 50, &file) != NULL){
			total_line ++;
			printf("line %d\n", total_line);
			for (int i = 0 ; i < 300 ; i++){
				if (buffer[i] == 0x0a){
					buffer_length = i ;
					printf("buffer length = %d\n", buffer_length);
					break;
				}
			}
			for(int i = 0 ; i < buffer_length ; i++){
				printf("[%d] : %x\n", i, buffer[i]);
			}
			memcpy(&buf[num], &buffer, buffer_length);
			num++;
		}
		// printf("%x\n", buffer[total_line-1]);
	}

	if(res != FR_OK){
		printf("\r\n schedule_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		// printf("test!!\n");
		for (int i = 0; i < total_line; i++)
		{
			memcpy(&txbuf[i], &buf[i], maxlength);
		}
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
int downlink_data_before_t(uint8_t datatype,uint32_t time1){
	/*      Now T1 is not convert yet, please check T1 endian to h      */

	char * fileName;
	uint8_t size;
	uint32_t datatime;

	if(datatype==1){
		fileName="0:/hk.bin";
		size=hk_length;
	}
	else if(datatype==2){
		fileName="0:/inms.bin";
		size=inms_data_length;
	}
	else if(datatype==3){
		fileName="0:/seuv.bin";
		size=seuv_length;
	}
	else if(datatype==4){
		fileName="0:/eop.bin";
		size=eop_length;
	}
	else if(datatype==5){
		fileName="0:/wod.bin";
		size=wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while(1){
		res = f_read(&file,&buffer,size,&br);
		if(res == FR_OK){
			memcpy(&datatime,&buffer,4);
			datatime=csp_ntoh32(datatime);

			if((datatime<time1) | (datatime==time1))
				SendDataWithCCSDS_AX25(datatype,&buffer[0]);
		}
		else{
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}

		if(f_eof(&file))
			break;
	}


	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}

int downlink_data_after_t(uint8_t datatype,uint16_t time1){
	/*      Now T1 is not convert yet, please check T1 endian to h      */
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	if(datatype==1){
		fileName="0:/hk.bin";
		size=hk_length;
	}
	else if(datatype==2){
		fileName="0:/inms.bin";
		size=inms_data_length;
	}
	else if(datatype==3){
		fileName="0:/seuv.bin";
		size=seuv_length;
	}
	else if(datatype==4){
		fileName="0:/eop.bin";
		size=eop_length;
	}
	else if(datatype==5){
		fileName="0:/wod.bin";
		size=wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while(1){


		res = f_read(&file,&buffer,size,&br);
		if(res == FR_OK){
			memcpy(&datatime,&buffer,4);
			datatime=csp_ntoh32(datatime);
			if((datatime>time1)|(datatime==time1))
				SendDataWithCCSDS_AX25(datatype,&buffer[0]);
		}
		else{
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}

		if(f_eof(&file))
			break;
	}

	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
int downlink_data_between_t(uint8_t datatype,uint16_t time1,uint16_t time2){
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	if(datatype==1){
		fileName="0:/hk.bin";
		size=hk_length;
	}
	else if(datatype==2){
		fileName="0:/inms.bin";
		size=inms_data_length;
	}
	else if(datatype==3){
		fileName="0:/seuv.bin";
		size=seuv_length;
	}
	else if(datatype==5){
		fileName="0:/wod.bin";
		size=wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}
	while(1){
		res = f_read(&file,&buffer,size,&br);
		if(res == FR_OK){
			memcpy(&datatime,&buffer,4);
			datatime=csp_ntoh32(datatime);
			if((datatime<time1)|(datatime==time1)){
				if((datatime>time2)|(datatime==time2)){
					SendDataWithCCSDS_AX25(datatype,&buffer[0]);
				}
			}

			if((datatime<time2)|(datatime==time2)){
				if((datatime>time1)|(datatime==time1)){
					SendDataWithCCSDS_AX25(datatype,&buffer[0]);
				}
			}
		}
		else{
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}
		if(f_eof(&file))
			break;
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}


int delete_data_before_t(uint8_t datatype,uint32_t time1){
	/*      Now T1 is not convert yet, please check T1 endian to h      */
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	int count=0;
	int a;
	if(datatype==1){
		fileName="0:/hk.bin";
		size=hk_length;
	}
	else if(datatype==2){
		fileName="0:/inms.bin";
		size=inms_data_length;
	}
	else if(datatype==3){
		fileName="0:/seuv.bin";
		size=seuv_length;
	}
	else if(datatype==4){
		fileName="0:/eop.bin";
		size=eop_length;
	}
	else if(datatype==5){
		fileName="0:/wod.bin";
		size=wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while(1){
		res = f_read(&file,&buffer,size,&br);
		if(res == FR_OK){
			memcpy(&datatime,&buffer,4);
			datatime=csp_ntoh32(datatime);

			if((datatime<time1)|(datatime==time1)){
				f_lseek(&file,count*size);
				for(a=0;a<size;a++)
					buffer[a]=0;
				res = f_write(&file,&buffer,size,&bw);
			}

		}
		else{
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}
		if(res != FR_OK)
			return Error;
		if(f_eof(&file))
			break;
		count++;
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}

int delete_data_after_t(uint8_t datatype,uint16_t time1){
	/*      Now T1 is not convert yet, please check T1 endian to h      */
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	int count=0;
	int a;
	if(datatype==1){
		fileName="0:/hk.bin";
		size=hk_length;
	}
	else if(datatype==2){
		fileName="0:/inms.bin";
		size=inms_data_length;
	}
	else if(datatype==3){
		fileName="0:/seuv.bin";
		size=seuv_length;
	}
	else if(datatype==4){
		fileName="0:/eop.bin";
		size=eop_length;
	}
	else if(datatype==5){
		fileName="0:/wod.bin";
		size=wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while(1){


		res = f_read(&file,&buffer,size,&br);
		if(res == FR_OK){
			memcpy(&datatime,&buffer,4);
			datatime=csp_ntoh32(datatime);
			if((datatime>time1)|(datatime==time1)){
				f_lseek(&file,count*size);
				for(a=0;a<size;a++){
					buffer[a]=0;
				}
				res = f_write(&file,&buffer,size,&bw);
			}
		}
		else{
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}
		if(res != FR_OK)
			return Error;
		if(f_eof(&file))
			break;
		count++;
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
int delete_data_between_t(uint8_t datatype,uint16_t time1,uint16_t time2){
	char * fileName;
	uint8_t size;
	uint32_t datatime;
	int count=0;
	int a;
	if(datatype==1){
		fileName="0:/hk.bin";
		size=hk_length;
	}
	else if(datatype==2){
		fileName="0:/inms.bin";
		size=inms_data_length;
	}
	else if(datatype==3){
		fileName="0:/seuv.bin";
		size=seuv_length;
	}
	else if(datatype==5){
		fileName="0:/wod.bin";
		size=wod_length;
	}
	else
		return Error;

	f_mount(0, &fs);
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}

	while(1){
		res = f_read(&file,&buffer,size,&br);
		if(res == FR_OK){
			memcpy(&datatime,&buffer,4);
			datatime=csp_ntoh32(datatime);
			if((datatime<time1)|(datatime==time1)){
				if((datatime>time2)|(datatime==time2)){
					f_lseek(&file,count*size);
					for(a=0;a<size;a++)
						buffer[a]=0;
					res = f_write(&file,&buffer,size,&bw);
				}
			}
			if((datatime<time2)|(datatime==time2)){
				if((datatime>time1)|(datatime==time1)){
					f_lseek(&file,count*size);
					for(a=0;a<size;a++)
						buffer[a]=0;
					res = f_write(&file,&buffer,size,&bw);
				}
			}
		}
		else{
			f_close(&file);
			f_mount(0, NULL);
			return Error;
		}
		if(res != FR_OK)
			return Error;
		if(f_eof(&file))
			break;
		count++;
	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}


int inms_data_write(uint8_t frameCont[] )
{
	f_mount(0, &fs);
	char fileName[]="0:/inms.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}
	//將pointer指向文件最後面
	f_lseek(&file,file.fsize);

	res = f_write(&file,&frameCont,inms_data_length,&bw);

	if(res!=FR_OK){
		printf("\r\n inms_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		printf("\r\n inms_write() success .. \r\n");
		f_close(&file);

		f_mount(0, NULL);
		return No_Error;
	}
}

int inms_data_read(int serial,void * txbuf){   // serial =1~N

	if(serial==0) serial=1;
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/inms/inms_";
	strcpy(fileName,s);
	int head = (serial-1)/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
	int tmp = (serial-1) % 100;
	f_lseek(&file,tmp*inms_data_length);
	res = f_read(&file,&buffer,inms_data_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		memcpy(txbuf,&buffer,inms_data_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int inms_data_delete(){
	f_mount(0, &fs);
	char fileName[]="0:/inms.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
	{
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}

int inms_script_write(int buffNum,uint8_t scriptCont[]){

	f_mount(0, &fs);
	char fileName[100];

	if(buffNum==8){
		strcpy(fileName,"0:/INMS/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/INMS/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/INMS/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/INMS/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/INMS/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/INMS/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/INMS/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/INMS/Idle6.bin");}

	printf("%s\n",fileName);

	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK){
		printf("\r\n f_unlink() fail .. \r\n");
	}
	else{
		printf("\r\n f_unlink() success .. \r\n");
	}

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );


	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}
	else{
		printf("\r\n f_open() success .. \r\n");
	}

	//將pointer指向文件最後面
	res = f_lseek(&file,file.fsize);

	hex_dump(scriptCont,157);
	res = f_write(&file,&scriptCont,(int)scriptCont[0],&bw);
	if(res!=FR_OK){
		printf("\r\n f_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		printf("\r\n f_write() success .. \r\n");

	}
	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
void inms_script_read(int buffNum,int packlength,void * txbuf){

	f_mount(0, &fs);

	char fileName[100];
	if(buffNum==8){
		strcpy(fileName,"0:/INMS/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/INMS/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/INMS/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/INMS/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/INMS/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/INMS/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/INMS/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/INMS/Idle6.bin");}

	res = f_open(&file, fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );

	if(res!=FR_OK){
		printf("f_open() fail .. \r\n");
	}
	else{
		printf("f_open() success .. \r\n");
	}

	while(1){
		res = f_read(&file,&buffer,packlength,&br); //20->packlength

		if(res == FR_OK){
			printf("f_read() success .. \r\n");
		}
		else{
			printf("f_read() fail .. \r\n");
		}
		if(f_eof(&file)){break;}
	}

	memcpy(txbuf,&buffer,packlength);


	f_close(&file);
	f_mount(0, NULL);

}

int inms_script_length(int buffNum){

	f_mount(0, &fs);
	int packlength =0;
	char fileName[100];
	if(buffNum==8){
		strcpy(fileName,"0:/INMS/Running.bin");}
	else if(buffNum==0){
		strcpy(fileName,"0:/INMS/Idle0.bin");}
	else if(buffNum==1){
		strcpy(fileName,"0:/INMS/Idle1.bin");}
	else if(buffNum==2){
		strcpy(fileName,"0:/INMS/Idle2.bin");}
	else if(buffNum==3){
		strcpy(fileName,"0:/INMS/Idle3.bin");}
	else if(buffNum==4){
		strcpy(fileName,"0:/INMS/Idle4.bin");}
	else if(buffNum==5){
		strcpy(fileName,"0:/INMS/Idle5.bin");}
	else if(buffNum==6){
		strcpy(fileName,"0:/INMS/Idle6.bin");}

	res = f_open(&file, fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );

	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}
	else{
		printf("\r\n f_open() success .. \r\n");
	}

	for(int i =0;i<300;i++){
		buffer[i]=0;
	}

	res = f_read(&file,buffer,1,&br);
	printf("buffer = %02X\n",buffer[0]);
	if(res == FR_OK){
		if(buffer[0] != 0)
			packlength = buffer[0];
		else
			packlength = 0;
	}
	else{
		printf("\r\n f_read() fail .. \r\n");
	}

	printf("packlength : %d\n",packlength);
	f_close(&file);
	f_mount(0, NULL);
	return packlength;
}


int wod_write(uint8_t * frameCont )
{
	f_mount(0, &fs);
	char fileName[]="0:/wod.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}

	f_lseek(&file,file.fsize);
	res = f_write(&file,frameCont,wod_length,&bw);

	if(res!=FR_OK){
		printf("\r\n wod_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		printf("\r\n wod_write() success .. \r\n");

		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int wod_read(int serial,void * txbuf){   // serial =1~N

	if(serial==0) serial=1;
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/wod/wod_";
	strcpy(fileName,s);
	int head = (serial-1)/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}
	else{
		printf("\r\n f_open() success .. \r\n");
	}
	int tmp = (serial-1) % 100;
	f_lseek(&file,tmp*wod_length);
	res = f_read(&file,&buffer,wod_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		memcpy(txbuf,&buffer,wod_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int wod_delete(){
	f_mount(0, &fs);
	char fileName[]="0:/wod.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK){
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}



int seuv_write()
{
	f_mount(0, &fs);
	char fileName[]="0:/seuv.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
		printf("SEUV  f_open fail!!\n");
	f_lseek(&file,file.fsize);
	res = f_write(&file,&seuvFrame.packettime,(int)sizeof(seuv_frame_t),&bw);
	if(res!=FR_OK)
		printf("SEUV  f_open fail!!\n");
	if(res!=FR_OK){
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int seuv_read(int serial,void * txbuf){   // serial =1~N

	if(serial==0) serial=1;
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/seuv/seuv_";
	strcpy(fileName,s);
	int head = (serial-1)/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}
	else{
		printf("\r\n f_open() success .. \r\n");
	}
	int tmp = (serial-1) % 100;
	f_lseek(&file,tmp*seuv_length);
	res = f_read(&file,&buffer,seuv_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		memcpy(txbuf,&buffer,seuv_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}
int seuv_delete(){
	f_mount(0, &fs);
	char fileName[]="0:/seuv.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK){
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}


int hk_write(uint8_t * frameCont )
{
	f_mount(0, &fs);
	char fileName[]="0:/hk.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	f_lseek(&file,file.fsize);
	res = f_write(&file,frameCont,hk_length,&bw);

	if(res!=FR_OK){
		printf("\r\n hk_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		printf("\r\n hk_write() success .. \r\n");
		f_close(&file);

		f_mount(0, NULL);
		return No_Error;
	}
}

int hk_read(int serial,void * txbuf){   // serial =1~N

	if(serial==0) serial=1;
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/hk/hk_";
	strcpy(fileName,s);
	int head = (serial-1)/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}
	else{
		printf("\r\n f_open() success .. \r\n");
	}
	int tmp = (serial-1) % 100;
	f_lseek(&file,tmp*hk_length);
	res = f_read(&file,&buffer,hk_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		memcpy(txbuf,&buffer,hk_length);
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int hk_delete(){
	f_mount(0, &fs);
	char fileName[]="0:/hk.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK){
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}


int para_r(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/para.bin";
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n f_open() success .. \r\n");
		res = f_read(&file,&buffer,(int)sizeof(parameter_t),&br);
	}

	if(res != FR_OK){
		printf("\r\n para_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		f_close(&file);
		f_mount(0, NULL);
		memcpy(&parameters.first_flight,&buffer,(int)sizeof(parameter_t));
		hex_dump(&buffer,(int)sizeof(parameter_t));
		return No_Error;
	}
}

int para_w(){   // serial =1~N


	f_mount(0, &fs);
	char fileName[]="0:/para.bin";
	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}
	else{
		printf("\r\n f_write open() success .. \r\n");
	}

	res = f_write(&file,&parameters.first_flight,(int)sizeof(parameter_t),&br);

	if(res != FR_OK){
		printf("\r\n para write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int para_d(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/para.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK){
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}

}


int adcs_para_r(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/adcs_para.bin";
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n f_open() success .. \r\n");
		res = f_read(&file,&buffer,(int)sizeof(adcs_para_t),&br);
	}

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		f_close(&file);
		f_mount(0, NULL);
		memcpy(&adcs_para.strategy,&buffer,(int)sizeof(adcs_para_t));  //import
		hex_dump(&buffer,(int)sizeof(adcs_para_t));
		return No_Error;
	}
}

int adcs_para_w(){   // serial =1~N


	f_mount(0, &fs);
	char fileName[]="0:/adcs_para.bin";
	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");
	}
	else{
		printf("\r\n f_write open() success .. \r\n");
	}

	res = f_write(&file,&adcs_para.strategy,(int)sizeof(adcs_para_t),&br);

	if(res != FR_OK){
		printf("\r\n para write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;
	}
	else{
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}
}

int adcs_para_d(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/adcs_para.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK){
		printf("\r\n f_unlink() fail .. \r\n");
		return Error;
	}
	else{
		printf("\r\n f_unlink() success .. \r\n");
		return No_Error;
	}
}
int inms_data_dump(){
	f_mount(0, &fs);
	char fileName[]="0:/inms.bin";
	int count=0;
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");

	}
	else{
		while(1){
			res = f_read(&file,&buffer,inms_data_length,&br);
			if(res == FR_OK){
				printf("INMS Data Dump packet '%d' \r\n",count);
				hex_dump(&buffer[0],inms_data_length);
				count++;
			}
			else{
				printf("f_read() fail .. \r\n");
				break;
			}
			if(f_eof(&file)){break;}
		}
	}


	f_close(&file);
	f_mount(0, NULL);

	return No_Error;
}


int seuv_data_dump(){
	f_mount(0, &fs);
	char fileName[]="0:/seuv.bin";
	int count=0;
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK){
		printf("\r\n f_open() fail .. \r\n");

	}
	else{
		while(1){


			res = f_read(&file,&buffer,seuv_length,&br);

			if(res == FR_OK){
				printf("SEUV Data Dump packet '%d' \r\n",count);
				hex_dump(&buffer[0],seuv_length);
				count++;
			}
			else{
				printf("f_read() fail .. \r\n");
				break;
			}
			if(f_eof(&file)){break;}
		}
	}	
	memcpy(&seuvFrame.packettime,&buffer[0],seuv_length);
	seuvFrame.packettime=csp_ntoh32(seuvFrame.packettime);

	printf("SEUV seuvFrame.packettime = %u \r\n",(unsigned int)seuvFrame.packettime);
	printf("SEUV seuvFrame.samples = %u \r\n",(unsigned int)seuvFrame.samples);
	printf("CH1 AVG = %f \r\n",seuvFrame.ch1AVG);
	printf("CH1 STD = %f \r\n",seuvFrame.ch1STD);
	printf("CH2 AVG = %f \r\n",seuvFrame.ch2AVG);
	printf("CH2 STD = %f \r\n",seuvFrame.ch2STD);
	printf("CH3 AVG = %f \r\n",seuvFrame.ch3AVG);
	printf("CH3 STD = %f \r\n",seuvFrame.ch3STD);
	printf("CH4 AVG = %f \r\n",seuvFrame.ch4AVG);
	printf("CH4 STD = %f \r\n",seuvFrame.ch4STD);

	f_close(&file);
	f_mount(0, NULL);



	return No_Error;
}

int wod_data_dump(){
	f_mount(0, &fs);
	char fileName[]="0:/wod.bin";
	int count=0;
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");

	}
	else{
		while(1){


			res = f_read(&file,&buffer,wod_length,&br);

			if(res == FR_OK){
				printf("WOD Data Dump packet '%d' \r\n",count);
				hex_dump(&buffer[0],wod_length);
				count++;
			}
			else{
				printf("f_read() fail .. \r\n");
				break;
			}
			if(f_eof(&file)){break;}
		}
	}

	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}

int hk_data_dump(){
	f_mount(0, &fs);
	char fileName[]="0:/hk.bin";
	int count=0;
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");

	}
	else{
		while(1){


			res = f_read(&file,&buffer,hk_length,&br);

			if(res == FR_OK){
				printf("HK Data Dump packet '%d' \r\n",count);
				hex_dump(&buffer[0],hk_length);
				count++;
			}
			else{
				printf("f_read() fail .. \r\n");
				break;
			}
			if(f_eof(&file)){break;}
		}
	}


	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}
int thermal_data_dump(){
	f_mount(0, &fs);
	char fileName[]="0:/t_obc.bin";
	int count=0;
	res = f_open(&file,fileName,FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");

	}
	else{
		while(1){


			res = f_read(&file,&ThurmalFrame.packet_number,(int)sizeof(thurmal_frame_t),&br);

			if(res == FR_OK){
				printf("Thermal Data Dump packet '%d' \r\n",count);
				printf("NUM = %d ,T1= %04X ,T2 %04X ,T3= %04X ,T4= %04X ,T5= %04X \n",(int)ThurmalFrame.packet_number,ThurmalFrame.T1,ThurmalFrame.T2,ThurmalFrame.T3,ThurmalFrame.T4,ThurmalFrame.T5);
				printf("T6= %04X ,T7 %04X ,T8= %04X ,T9= %04X ,T10= %04X \n",ThurmalFrame.T6,ThurmalFrame.T7,ThurmalFrame.T8,ThurmalFrame.T9,ThurmalFrame.T10);
				printf("==================================== \r\n");
				count++;
			}
			else{
				printf("f_read() fail .. \r\n");
				break;
			}
			if(f_eof(&file)){break;}
		}
	}


	f_close(&file);
	f_mount(0, NULL);
	return No_Error;
}

int thurmal_1_w(){   // serial =1~N
	f_mount(0, &fs);
	char fileName[]="0:/t_obc.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );

	if(res!=FR_OK)
	{
		printf("\r\n @@ %d @@ \r\n",res);
		printf("\r\n f_open() fail .. \r\n");
	}
	else{

		//printf("\r\n f_open() success .. \r\n");
	}

	f_lseek(&file,file.fsize);
	res = f_write(&file,&ThurmalFrame.packet_number,(int)sizeof(thurmal_frame_t),&br);

	if(res != FR_OK){
		printf("\r\n thurmal_1_w write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}

	return No_Error;
}

int thurmal_2_w(){   // serial =1~N
	f_mount(0, &fs);
	char fileName[]="0:/t_inms.bin";

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n @@ %d @@ \r\n",res);
		printf("\r\n f_open() fail .. \r\n");
	}
	else{

		//printf("\r\n f_open() success .. \r\n");
	}
	f_lseek(&file,file.fsize);
	res = f_write(&file,&Tdata[0],178,&br);

	if(res != FR_OK){
		printf("\r\n thurmal_2_w write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return Error;}
	else{
		f_close(&file);
		f_mount(0, NULL);
		return No_Error;
	}

	return No_Error;
}

int T_data_d(){   // serial =1~N

	f_mount(0, &fs);
	char fileName[]="0:/t_obc.bin";
	res = f_unlink(fileName);	  //先刪除

	if(res!=FR_OK)
	{
		printf("\r\n t_obc.bin f_unlink() fail .. \r\n");

	}
	else{
		printf("\r\n t_obc.bin f_unlink() success .. \r\n");

	}


	f_mount(0, &fs);
	char fileName2[]="0:/t_inms.bin";
	res = f_unlink(fileName2);	  //先刪除

	if(res!=FR_OK)
	{
		printf("\r\n t_inms.bin f_unlink() fail .. \r\n");

	}
	else{
		printf("\r\n t_inms.bin f_unlink() success .. \r\n");

	}

	return No_Error;

}

