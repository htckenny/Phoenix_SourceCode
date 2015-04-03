#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fat_sd/ff.h>
#include "fs.h"
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <util/hexdump.h>
#include "parameter.h"
#define No_Error		0
#define inms_data_length 196
#define wod_length 232
#define seuv_length 77
#define hk_length 64

FATFS fs;
FRESULT res;
FIL file;
UINT br,bw;
uint8_t buffer[300];
FILINFO *fno;

int inms_data_write(uint8_t frameCont[] )
{

	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/inms/inms_";
	strcpy(fileName,s);

	int head = inms_store_count/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");
	printf("start to store data in %s\n",fileName);///////////////////////

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}

	f_lseek(&file,inms_store_count*inms_data_length);
	res = f_write(&file,frameCont,inms_data_length,&bw);

	if(res!=FR_OK){
		printf("\r\n inms_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;
	}
	else{
		printf("\r\n inms_write() success .. \r\n");
		f_close(&file);
		inms_store_count++;
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
		res = f_read(&file,buffer,inms_data_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;}
	else{
		memcpy(txbuf,&buffer,inms_data_length);
		f_close(&file);
		f_mount(0, NULL);
		return 0;
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

	if(res!=FR_OK)
	{
		printf("\r\n f_unlink() fail .. \r\n");
	}else{
		printf("\r\n f_unlink() success .. \r\n");
	}

	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	

	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}

	//將pointer指向文件最後面
	res = f_lseek(&file,file.fsize);

	hex_dump(scriptCont,157);
	res = f_write(&file,scriptCont,(int)scriptCont[0],&bw);
	if(res!=FR_OK)
	{
		printf("\r\n f_write() fail .. \r\n");

	}else{
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

	if(res!=FR_OK)
	{
		printf("f_open() fail .. \r\n");
	}else{
		printf("f_open() success .. \r\n");
	}

	while(1){
		res = f_read(&file,buffer,packlength,&br); //20->packlength
	   
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

	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
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
		}else{
			printf("\r\n f_read() fail .. \r\n");
		}
	
	printf("packlength : %d\n",packlength);
	f_close(&file);
	f_mount(0, NULL);
	return packlength;
}


int wod_write(uint8_t frameCont[] )
{
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/wod/wod_";
	strcpy(fileName,s);
	int head = wod_store_count/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");
	printf("start to store data in %s\n",fileName);


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}

	f_lseek(&file,wod_store_count*wod_length);
	res = f_write(&file,frameCont,wod_length,&bw);

	if(res!=FR_OK){
		printf("\r\n wod_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;
	}
	else{
		printf("\r\n wod_write() success .. \r\n");
		wod_store_count++;
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
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
        int tmp = (serial-1) % 100;
	    f_lseek(&file,tmp*wod_length);
		res = f_read(&file,buffer,wod_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;}
	else{
		memcpy(txbuf,&buffer,wod_length);
		f_close(&file);
		f_mount(0, NULL);
		return 0;
		}
}

int seuv_write(uint8_t frameCont[] )
{
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/seuv/seuv_";
	strcpy(fileName,s);
	int head = seuv_store_count/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");
	printf("start to store data in %s\n",fileName);


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}

	f_lseek(&file,seuv_store_count*seuv_length);
	res = f_write(&file,frameCont,seuv_length,&bw);

	if(res!=FR_OK){
		printf("\r\n wod_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;
	}
	else{
		printf("\r\n wod_write() success .. \r\n");
		f_close(&file);
		seuv_store_count++;
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
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
        int tmp = (serial-1) % 100;
	    f_lseek(&file,tmp*seuv_length);
		res = f_read(&file,buffer,seuv_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;}
	else{
		memcpy(txbuf,&buffer,seuv_length);
		f_close(&file);
		f_mount(0, NULL);
		return 0;
		}
}


int hk_write(uint8_t frameCont[] )
{
	f_mount(0, &fs);
	char fileName[20];
	char s[]="0:/hk/hk_";
	strcpy(fileName,s);
	int head = hk_store_count/100;
	char num[5];
	sprintf(num,"%d", head);
	strcat(fileName,num);
	strcat(fileName,".bin");
	printf("start to store data in %s\n",fileName);


	res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}

	f_lseek(&file,hk_store_count*hk_length);
	res = f_write(&file,frameCont,hk_length,&bw);

	if(res!=FR_OK){
		printf("\r\n wod_write() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;
	}
	else{
		printf("\r\n wod_write() success .. \r\n");
		f_close(&file);
		hk_store_count++;
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
	if(res!=FR_OK)
	{
		printf("\r\n f_open() fail .. \r\n");
	}else{
		printf("\r\n f_open() success .. \r\n");
	}
        int tmp = (serial-1) % 100;
	    f_lseek(&file,tmp*hk_length);
		res = f_read(&file,buffer,hk_length,&br);

	if(res != FR_OK){
		printf("\r\n wod_read() fail .. \r\n");
		f_close(&file);
		f_mount(0, NULL);
		return -1;}
	else{
		memcpy(txbuf,&buffer,hk_length);
		f_close(&file);
		f_mount(0, NULL);
		return 0;
		}
}



int shutdown_flag_set(uint8_t flag){
	f_mount(0, &fs);
	char fileName[100];
	strcpy(fileName,"0:/shut.bin");


		res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
		res = f_write(&file,flag,1,&bw);
		if(res!=FR_OK)
			{
				printf("\r\n f_write() fail .. \r\n");
				f_close(&file);
				f_mount(0, NULL);
				return -1;
			}else{
				printf("\r\n f_write() success .. \r\n");
			}
			f_close(&file);
			f_mount(0, NULL);
			return No_Error;
}
void shutdown_flag_read(void * txbuf){
	f_mount(0, &fs);
	char fileName[100];
	strcpy(fileName,"0:/shut.bin");

	if(res!=FR_OK)
		{
			printf("\r\n f_unlink() fail .. \r\n");
		}else{
			printf("\r\n f_unlink() success .. \r\n");
		}

		res = f_open(&file,fileName,FA_OPEN_ALWAYS|FA_READ|FA_WRITE );



			res = f_read(&file,buffer,1,&br);

			if(res == FR_OK)
			{
				printf("\r\n f_read() success .. \r\n");

			}else{
				printf("\r\n f_read() fail .. \r\n");
			}

		memcpy(txbuf,&buffer,1);
		printf("%d",txbuf);
		f_close(&file);
		f_mount(0, NULL);
}
